#!/usr/bin/env python3
"""从同一套几何和关节数据生成 URDF、SDF，保持坐标与轮距一致。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

PKG = Path(__file__).resolve().parents[1]
P = json.loads((PKG/'config/robot_parameters.json').read_text(encoding='utf-8'))
AXLE_Z = P['wheel']['diameter_m']/2


def el(parent, tag, text=None, **attrs):
    """建立 XML 节点，数值统一采用米、千克、弧度。"""
    node = ET.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})
    if text is not None:
        node.text = str(text)
    return node


def vector(values):
    return ' '.join(f'{v:.10g}' for v in values)


def save_xml(root, path):
    ET.indent(root, space='  ')
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def read_stl(path):
    """读取现有二进制 STL；不依赖 SolidWorks 或第三方网格库。"""
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84+50*count:
        raise ValueError(f'Not a binary STL: {path}')
    return [[struct.unpack_from('<3f', raw, 84+i*50+j) for j in (12,24,36)] for i in range(count)]


def bounds(triangles):
    verts = [v for tri in triangles for v in tri]
    return [[min(v[k] for v in verts) for k in range(3)], [max(v[k] for v in verts) for k in range(3)]]


def write_stl(path, triangles):
    """重算法向，输出以米为数值单位的仿真专用网格。"""
    with path.open('wb') as stream:
        stream.write(b'SmartCar simulation mesh, metres; derived from V7 CAD'.ljust(80,b' '))
        stream.write(struct.pack('<I',len(triangles)))
        for a,b,c in triangles:
            u=[b[i]-a[i] for i in range(3)]; v=[c[i]-a[i] for i in range(3)]
            n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(t*t for t in n))
            n=[t/length for t in n] if length else [0,0,0]
            stream.write(struct.pack('<12fH',*(n+list(a)+list(b)+list(c)),0))


def transform(t, point):
    return [sum(t[j*3+i]*point[j] for j in range(3))+t[9+i] for i in range(3)]


def cad_to_ros(point):
    """右手旋转：CAD +Y 为车前，映射 ROS +X；原点位于主动轮轴中心。"""
    return [point[1]-.04, -point[0], point[2]-AXLE_Z]


def import_cad(repo):
    """归档本次实际使用的网格与坐标，不修改任何原始 CAD 文件。"""
    entries=json.loads((repo/'cad/ros_car_v7/assembly_placements.json').read_text(encoding='utf-8-sig'))
    origin_mm={'01_main_base':[-110,0,-4],'02_motor_bracket':[-22,-17,0],
               '05_battery_stop':[-12,-18,0],'06_upper_deck':[-105,5,0],
               '07_lidar_plate':[-50,83,0],'08_camera_adapter':[-50,190,0],
               'power_module_box_v6_tray':[0,0,-68],'power_module_box_v6_lid':[0,0,-68]}
    color={'01_main_base':[.08,.35,.42,1],'02_motor_bracket':[.8,.32,.06,1],
           '05_battery_stop':[.8,.32,.06,1],'06_upper_deck':[.40,.56,.60,1],
           '07_lidar_plate':[.8,.32,.06,1],'08_camera_adapter':[.8,.32,.06,1]}
    items=[]
    folder=PKG/'cad_source/parts';folder.mkdir(parents=True,exist_ok=True)
    for entry in entries:
        part=Path(entry['path']).stem
        if part not in origin_mm: continue
        parent=Path(entry['path']).parent
        source=parent/(part+'.STL')
        if part.startswith('power_module_box'):
            source=parent/(part+'_solidworks.STL')
        dest=folder/(part+'.stl')
        shutil.copyfile(source,dest)
        items.append({'name':entry['name'],'part':part,'source':f'parts/{part}.stl',
                      'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                      'original_cad_min_mm':origin_mm[part], 'transform':entry['transform'],
                      'color':color.get(part,[.75,.75,.78,1])})
    (PKG/'cad_source/placements.json').write_text(json.dumps(items,indent=2),encoding='utf-8')


def cad_visuals():
    """恢复打印 STL 导出时的平移，再转换到统一 ROS 坐标系。"""
    result=[]; manifest=[]
    for entry in json.loads((PKG/'cad_source/placements.json').read_text()):
        source=PKG/'cad_source'/entry['source']; tris=read_stl(source); lo,_=bounds(tris)
        original=entry['original_cad_min_mm']; t=entry['transform']
        converted=[[cad_to_ros(transform(t,[(v[i]-lo[i]+original[i])/1000 for i in range(3)])) for v in tri] for tri in tris]
        filename=entry['name'].lower()+'.stl'
        write_stl(PKG/'models/smartcar/meshes'/filename,converted)
        result.append({'kind':'mesh','size':filename,'xyz':[0,0,0],'rpy':[0,0,0],'color':entry['color']})
        manifest.append({'name':entry['name'],'source_sha256':entry['sha256'],'mesh':filename,
                         'triangles':len(converted),'bounds_relative_to_axle_m':bounds(converted)})
    (PKG/'cad_source/mesh_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return result


def shape(kind,size,xyz=(0,0,0),rpy=(0,0,0),color=(.15,.18,.2,1)):
    return {'kind':kind,'size':size,'xyz':list(xyz),'rpy':list(rpy),'color':list(color)}


def diagonal_inertia(mass,size,kind='box'):
    if kind=='wheel':
        radius,width=size; a=mass*(3*radius*radius+width*width)/12
        return [a,mass*radius*radius/2,a]
    x,y,z=size
    return [mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12]


def make_links():
    """固定件保留外观；主动轮、两个万向轮具有各自的真实关节拓扑。"""
    frame=cad_visuals()
    # 下列器件为包络，位置沿用 V7，不能作为厂家安装孔模板。
    boxes=[('battery',[-.0365,.022,.064],[.073,.130,.044],[.08,.18,.45,1]),
           ('computer',[-.098,.090,.138],[.056,.085,.030],[.07,.35,.16,1]),
           ('stm32',[-.025,.108,.138],[.050,.050,.016],[.07,.35,.16,1]),
           ('esp32',[.063,.123,.138],[.030,.055,.012],[.08,.16,.50,1]),
           ('esc1',[-.103,.112,.064],[.035,.050,.020],[.08,.09,.10,1]),
           ('esc2',[.068,.112,.064],[.035,.050,.020],[.08,.09,.10,1])]
    for _,lo,size,color in boxes:
        frame.append(shape('box',[size[1],size[0],size[2]],cad_to_ros([lo[i]+size[i]/2 for i in range(3)]),color=color))
    for x in [-.095,.095]:
        for y in [.018,.212]: frame.append(shape('cylinder',[.004,.065],cad_to_ros([x,y,.0965]),color=[.55,.58,.60,1]))
    for x in [-.030,.030]:
        for y in [.103,.163]: frame.append(shape('cylinder',[.004,.05],cad_to_ros([x,y,.158]),color=[.55,.58,.60,1]))
    for x in [-.080,.080]: frame.append(shape('cylinder',[.0122,.0478],cad_to_ros([x,.040,.0325]),[math.pi/2,0,0],[.55,.58,.60,1]))
    for y in [.049,.125]:
        frame.append(shape('box',[.020,.0855,.0015],cad_to_ros([0,y,.1095]),color=[.04,.04,.04,1]))
        for x in [-.042,.042]: frame.append(shape('box',[.020,.0015,.0525],cad_to_ros([x,y,.08425]),color=[.04,.04,.04,1]))
    mass=P['chassis']['mass_kg_ESTIMATED']
    links={'base_link':{'xyz':[0,0,AXLE_Z],'visuals':frame,
          'collisions':[shape('box',[.240,.220,.009],[.080,0,.027]),
                        shape('box',[.230,.210,.004],[.080,0,.0985]),
                        shape('box',[.130,.073,.044],[.047,0,.0535])],
          'mass':mass,'com':P['chassis']['com_relative_to_axle_m_ESTIMATED'],
          'inertia':diagonal_inertia(mass,P['chassis']['inertia_box_m_ESTIMATED'])}}
    joints=[]
    wheel=P['wheel'];radius=wheel['diameter_m']/2;width=wheel['width_m']
    for side,sign in [('left',1),('right',-1)]:
        name=side+'_wheel_link'; geo=shape('cylinder',[radius,width],rpy=[math.pi/2,0,0],color=[.035,.035,.04,1])
        links[name]={'xyz':[0,sign*wheel['track_m']/2,AXLE_Z],'visuals':[geo,shape('cylinder',[.018,width+.0004],rpy=[math.pi/2,0,0],color=[.28,.30,.33,1]),shape('box',[.045,width+.0008,.003],color=[.7,.7,.7,1])],
                     'collisions':[geo],'mass':wheel['mass_kg_ESTIMATED'],'com':[0,0,0],
                     'inertia':diagonal_inertia(wheel['mass_kg_ESTIMATED'],[radius,width],'wheel'),'friction':wheel['friction_ESTIMATED']}
        joints.append({'name':side+'_wheel_joint','parent':'base_link','child':name,'type':'continuous','axis':[0,1,0],'damping':.001})
    caster=P['caster'];cr=caster['wheel_diameter_m']/2;cw=caster['wheel_width_m'];pivot=caster['pivot_height_m_ESTIMATED'];trail=caster['trail_m_INFERRED']
    for side,sign in [('left',1),('right',-1)]:
        fork=side+'_caster_swivel';name=side+'_caster_wheel'; y=sign*caster['pivot_y_m']; x=caster['pivot_x_m']
        fork_vis=[shape('cylinder',[.011,.012],[0,0,.004],color=[.5,.52,.54,1])]
        for sy in [-1,1]:fork_vis.append(shape('box',[.025,.0018,.027],[-trail/2,sy*(cw/2+.002),-.012],color=[.5,.52,.54,1]))
        links[fork]={'xyz':[x,y,pivot],'visuals':fork_vis,'collisions':[], 'mass':caster['fork_mass_kg_ESTIMATED'],'com':[0,0,-.01],'inertia':diagonal_inertia(caster['fork_mass_kg_ESTIMATED'],[.03,.03,.04])}
        geo=shape('cylinder',[cr,cw],rpy=[math.pi/2,0,0],color=[.20,.20,.22,1])
        links[name]={'xyz':[x-trail,y,cr],'visuals':[geo],'collisions':[geo],'mass':caster['wheel_mass_kg_ESTIMATED'],'com':[0,0,0],
                     'inertia':diagonal_inertia(caster['wheel_mass_kg_ESTIMATED'],[cr,cw],'wheel'),'friction':caster['friction_ESTIMATED']}
        joints.extend([{'name':side+'_caster_swivel_joint','parent':'base_link','child':fork,'type':'continuous','axis':[0,0,1],'damping':.0005},
                       {'name':side+'_caster_roll_joint','parent':fork,'child':name,'type':'continuous','axis':[0,1,0],'damping':.0001}])
    lidar=P['lidar'];lx,ly,lz=lidar['mount_xyz_relative_to_axle_m'];size=lidar['case_size_m_ESTIMATED']
    links['lidar_link']={'xyz':[lx,ly,lz+AXLE_Z],'mass':lidar['mass_kg_ESTIMATED'],'com':[0,0,size[2]/2], 'inertia':diagonal_inertia(lidar['mass_kg_ESTIMATED'],size),
                         'visuals':[shape('cylinder',[size[0]/2,size[2]],[0,0,size[2]/2],color=[.09,.10,.11,1])],'collisions':[shape('box',size,[0,0,size[2]/2])]}
    links['laser_frame']={'xyz':[lx,ly,lz+AXLE_Z+lidar['scan_height_above_mount_m_ESTIMATED']],'mass':0,'visuals':[],'collisions':[]}
    joints.extend([{'name':'lidar_mount_joint','parent':'base_link','child':'lidar_link','type':'fixed'}, {'name':'laser_frame_joint','parent':'lidar_link','child':'laser_frame','type':'fixed'}])
    camera=P['camera'];cx,cy,cz=camera['optical_center_xyz_relative_to_axle_m']
    camera_vis=[shape('box',[.011,.036,.036],[-.0145,0,0],color=[.06,.07,.08,1]),shape('cylinder',[.009,.009],[-.0045,0,0],[0,math.pi/2,0],[.18,.25,.3,1]),shape('box',[.025,.038,.0015],[-.014,0,-.02625],color=[.3,.32,.34,1])]
    for sy in [-1,1]:camera_vis.append(shape('box',[.010,.002,.033],[-.014,sy*.018,-.009],color=[.3,.32,.34,1]))
    links['camera_link']={'xyz':[cx,cy,cz+AXLE_Z],'mass':camera['mass_kg_ESTIMATED'],'com':[-.0145,0,0], 'inertia':diagonal_inertia(camera['mass_kg_ESTIMATED'],[.025,.038,.045]), 'visuals':camera_vis,'collisions':[shape('box',[.025,.038,.045],[-.0125,0,-.0045])]}
    links['camera_optical_frame']={'xyz':[cx,cy,cz+AXLE_Z],'mass':0,'visuals':[],'collisions':[], 'rpy':[-math.pi/2,0,-math.pi/2]}
    joints.extend([{'name':'camera_mount_joint','parent':'base_link','child':'camera_link','type':'fixed'}, {'name':'camera_optical_joint','parent':'camera_link','child':'camera_optical_frame','type':'fixed','rpy':[-math.pi/2,0,-math.pi/2]}])
    return links,joints


def geometry(parent,g,sdf=False):
    geo=el(parent,'geometry');kind=g['kind'];size=g['size']
    if kind=='box':
        node=el(geo,'box');el(node,'size',vector(size)) if sdf else node.set('size',vector(size))
    elif kind=='cylinder':
        node=el(geo,'cylinder')
        if sdf:el(node,'radius',size[0]);el(node,'length',size[1])
        else:node.set('radius',str(size[0]));node.set('length',str(size[1]))
    elif kind=='mesh':
        node=el(geo,'mesh')
        if sdf:el(node,'uri','model://smartcar/meshes/'+size)
        else:node.set('filename','package://smartcar_sim/models/smartcar/meshes/'+size)


def build_urdf(links,joints):
    root=ET.Element('robot',name='smartcar')
    el(root,'link',name='base_footprint')
    joint=el(root,'joint',name='base_footprint_joint',type='fixed');el(joint,'parent',link='base_footprint');el(joint,'child',link='base_link');el(joint,'origin',xyz=vector([0,0,AXLE_Z]),rpy='0 0 0')
    for name,data in links.items():
        node=el(root,'link',name=name)
        if data['mass']:
            it=el(node,'inertial');el(it,'origin',xyz=vector(data['com']));el(it,'mass',value=data['mass']);ix,iy,iz=data['inertia'];el(it,'inertia',ixx=ix,iyy=iy,izz=iz,ixy=0,ixz=0,iyz=0)
        for tag,key in [('visual','visuals'),('collision','collisions')]:
            for i,g in enumerate(data[key]):
                shape_node=el(node,tag,name=f'{name}_{tag}_{i}');el(shape_node,'origin',xyz=vector(g['xyz']),rpy=vector(g['rpy']));geometry(shape_node,g)
                if tag=='visual':el(el(shape_node,'material',name=f'{name}_color_{i}'),'color',rgba=vector(g['color']))
    for joint in joints:
        node=el(root,'joint',name=joint['name'],type=joint['type']);el(node,'parent',link=joint['parent']);el(node,'child',link=joint['child'])
        a=links[joint['parent']]['xyz'];b=links[joint['child']]['xyz'];el(node,'origin',xyz=vector([b[i]-a[i] for i in range(3)]),rpy=vector(joint.get('rpy',[0,0,0])))
        if joint['type']=='continuous':el(node,'axis',xyz=vector(joint['axis']));el(node,'limit',effort=5,velocity=20);el(node,'dynamics',damping=joint['damping'],friction=0)
    save_xml(root,PKG/'urdf/smartcar.urdf')


def build_sdf(links,joints,lidar_enabled=True,camera_enabled=False,preview=True):
    root=ET.Element('sdf',version='1.8');model=el(root,'model',name='smartcar',canonical_link='base_link');el(model,'self_collide','false')
    for name,data in links.items():
        if not data['mass']:
            frame=el(model,'frame',name=name,attached_to='lidar_link' if name=='laser_frame' else 'camera_link');el(frame,'pose',vector(data['xyz']+data.get('rpy',[0,0,0])),relative_to='__model__');continue
        node=el(model,'link',name=name);el(node,'pose',vector(data['xyz']+[0,0,0]))
        it=el(node,'inertial');el(it,'pose',vector(data['com']+[0,0,0]));el(it,'mass',data['mass']);inertia=el(it,'inertia')
        for k,v in zip(['ixx','iyy','izz'],data['inertia']):el(inertia,k,v)
        for k in ['ixy','ixz','iyz']:el(inertia,k,0)
        for tag,key in [('visual','visuals'),('collision','collisions')]:
            for i,g in enumerate(data[key]):
                item=el(node,tag,name=f'{name}_{tag}_{i}');el(item,'pose',vector(g['xyz']+g['rpy']));geometry(item,g,True)
                if tag=='visual':
                    mat=el(item,'material');el(mat,'ambient',vector(g['color']));el(mat,'diffuse',vector(g['color']))
                elif 'friction' in data:
                    ode=el(el(el(item,'surface'),'friction'),'ode');el(ode,'mu',data['friction']);el(ode,'mu2',data['friction'])
        if name=='lidar_link' and lidar_enabled:add_lidar(node)
        if name=='camera_link' and camera_enabled:add_camera(node,preview)
    for joint in joints:
        if not links[joint['child']]['mass']:continue
        node=el(model,'joint',name=joint['name'],type='revolute' if joint['type']=='continuous' else 'fixed');el(node,'parent',joint['parent']);el(node,'child',joint['child'])
        if joint['type']=='continuous':
            axis=el(node,'axis');el(axis,'xyz',vector(joint['axis']));lim=el(axis,'limit');el(lim,'lower',-1e16);el(lim,'upper',1e16);el(lim,'effort',5);el(lim,'velocity',20);el(el(axis,'dynamics'),'damping',joint['damping'])
    plugin=el(model,'plugin',filename='ignition-gazebo-diff-drive-system',name='ignition::gazebo::systems::DiffDrive')
    params={'left_joint':'left_wheel_joint','right_joint':'right_wheel_joint','wheel_separation':P['wheel']['track_m'],'wheel_radius':AXLE_Z,'topic':'/model/smartcar/cmd_vel','odom_topic':'/model/smartcar/odometry','tf_topic':'/model/smartcar/tf','frame_id':'odom','child_frame_id':'base_footprint','odom_publish_frequency':50,'max_linear_velocity':.25,'min_linear_velocity':-.25,'max_angular_velocity':1.5,'min_angular_velocity':-1.5,'max_linear_acceleration':.5,'min_linear_acceleration':-.5,'max_angular_acceleration':2,'min_angular_acceleration':-2}
    control=P['control']
    params.update({'max_linear_velocity':control['max_linear_mps'],'min_linear_velocity':-control['max_linear_mps'],'max_angular_velocity':control['max_angular_radps'],'min_angular_velocity':-control['max_angular_radps']})
    for key,value in params.items():el(plugin,key,value)
    state=el(model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher');el(state,'topic','/model/smartcar/joint_states')
    return root


def add_lidar(node):
    p=P['lidar'];sensor=el(node,'sensor',name='c1_scan',type='gpu_lidar');el(sensor,'pose',vector([0,0,p['scan_height_above_mount_m_ESTIMATED'],0,0,0]));el(sensor,'topic','/sim/scan');el(sensor,'update_rate',p['scan_hz']);el(sensor,'always_on','true');el(sensor,'visualize','false')
    ray=el(sensor,'lidar');horizontal=el(el(ray,'scan'),'horizontal')
    # 500 条独立射线覆盖 360 度，末端不重复首条射线。
    for key,value in {'samples':p['samples_per_scan'],'resolution':1,'min_angle':-math.pi,'max_angle':math.pi-2*math.pi/p['samples_per_scan']}.items():el(horizontal,key,value)
    ran=el(ray,'range');el(ran,'min',p['range_min_m']);el(ran,'max',p['range_max_m_white_target']);el(ran,'resolution',.015)
    noise=el(ray,'noise',type='gaussian');el(noise,'mean',0);el(noise,'stddev',p['noise_stddev_m_ESTIMATED'])


def add_camera(node,preview):
    p=P['camera'];sensor=el(node,'sensor',name='uvc_camera',type='camera');el(sensor,'topic','/sim/camera/image_raw');el(sensor,'update_rate',p['fps_ESTIMATED']);el(sensor,'always_on','true');el(sensor,'visualize','false')
    camera=el(sensor,'camera');el(camera,'horizontal_fov',p['horizontal_fov_rad_ESTIMATED']);image=el(camera,'image');el(image,'width',640 if preview else p['width']);el(image,'height',360 if preview else p['height']);el(image,'format','R8G8B8');clip=el(camera,'clip');el(clip,'near',p['near_m']);el(clip,'far',p['far_m']);el(camera,'camera_info_topic','/sim/camera/camera_info')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--import-cad',type=Path);args=parser.parse_args()
    for directory in ['models/smartcar/meshes','urdf','worlds','config','rviz','cad_source']: (PKG/directory).mkdir(parents=True,exist_ok=True)
    if args.import_cad:import_cad(args.import_cad.resolve())
    links,joints=make_links();build_urdf(links,joints)
    save_xml(build_sdf(links,joints,True,False),PKG/'models/smartcar/model.sdf')
    for name,laser,camera,preview in [('physics',False,False,True),('lidar',True,False,True),('sensors_preview',True,True,True),('sensors_1080p',True,True,False)]:
        save_xml(build_sdf(links,joints,laser,camera,preview),PKG/f'models/smartcar/model_{name}.sdf')
    (PKG/'cad_source/link_layout.json').write_text(json.dumps({'links':{n:{k:v for k,v in data.items() if k not in ['visuals','collisions']} for n,data in links.items()},'joints':joints},indent=2),encoding='utf-8')
    print(f'Generated {len(links)} links, {len(joints)} joints; wheel track {P["wheel"]["track_m"]}m')


if __name__=='__main__':main()
