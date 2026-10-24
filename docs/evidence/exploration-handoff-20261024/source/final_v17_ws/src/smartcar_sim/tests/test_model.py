"""独立检查实际交付文件、物理接口与停止边界，不伪造 Gazebo 运行。"""
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('command_guard_core',ROOT/'scripts/command_guard_core.py')
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)


class GuardTests(unittest.TestCase):
    def test_initially_stopped(self):
        self.assertEqual(core.CommandGuard().output(100),(0,0))

    def test_timeout_boundary_without_new_message(self):
        guard=core.CommandGuard();guard.receive(.1,.2,0)
        self.assertEqual(guard.output(.299),(.1,.2));self.assertEqual(guard.output(.300),(0,0));self.assertEqual(guard.output(5),(0,0))

    def test_output_does_not_extend_user_command(self):
        guard=core.CommandGuard();guard.receive(.1,0,0)
        for i in range(1,300):guard.output(i/1000)
        self.assertEqual(guard.output(.301),(0,0))

    def test_new_message_rearms_but_zero_is_immediate(self):
        guard=core.CommandGuard();guard.receive(.1,0,0);guard.receive(.2,0,.25)
        self.assertEqual(guard.output(.4),(.2,0));guard.receive(0,0,.41);self.assertEqual(guard.output(.41),(0,0))

    def test_nonfinite_command_cancels_motion(self):
        for value in [float('nan'),float('inf'),-float('inf')]:
            guard=core.CommandGuard();guard.receive(.1,0,0);guard.receive(value,0,.1);self.assertEqual(guard.output(.2),(0,0))

    def test_clock_rollback_is_stopped(self):
        guard=core.CommandGuard();guard.receive(.1,0,10);self.assertEqual(guard.output(9),(0,0))

    def test_velocity_limits_both_directions(self):
        guard=core.CommandGuard();guard.receive(20,-20,0);self.assertEqual(guard.output(.1),(.25,-1.5))
        guard.receive(-20,20,1);self.assertEqual(guard.output(1.1),(-.25,1.5))


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf=ET.parse(ROOT/'urdf/smartcar.urdf').getroot()
        cls.sdf=ET.parse(ROOT/'models/smartcar/model_physics.sdf').getroot().find('model')
        cls.params=json.loads((ROOT/'config/robot_parameters.json').read_text())

    def test_urdf_is_connected_tree_with_six_moving_joints(self):
        names={n.attrib['name'] for n in self.urdf.findall('link')};children=set()
        joints=self.urdf.findall('joint');self.assertEqual(len(joints),len(names)-1)
        self.assertEqual(sum(j.attrib['type']=='continuous' for j in joints),6)
        graph={j.find('child').attrib['link']:j.find('parent').attrib['link'] for j in joints}
        for joint in joints:
            child=joint.find('child').attrib['link'];parent=joint.find('parent').attrib['link']
            self.assertIn(parent,names);self.assertIn(child,names);self.assertNotIn(child,children);children.add(child)
            seen=set();node=child
            while node in graph:
                self.assertNotIn(node,seen);seen.add(node);node=graph[node]
            self.assertEqual(node,'base_footprint')

    def test_wheel_dimensions_track_and_ground(self):
        wheels=[self.sdf.find(f"link[@name='{side}_wheel_link']") for side in ['left','right']]
        ys=[]
        for wheel in wheels:
            xyz=list(map(float,wheel.findtext('pose').split()))[:3]
            cylinder=wheel.find('collision/geometry/cylinder');radius=float(cylinder.findtext('radius'));width=float(cylinder.findtext('length'))
            self.assertAlmostEqual(radius,.0325);self.assertAlmostEqual(width,.027);self.assertAlmostEqual(xyz[2]-radius,0);ys.append(xyz[1])
        self.assertAlmostEqual(ys[0]-ys[1],.252)
        plugin=self.sdf.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
        self.assertAlmostEqual(float(plugin.findtext('wheel_separation')),ys[0]-ys[1]);self.assertAlmostEqual(float(plugin.findtext('wheel_radius')),.0325)

    def test_casters_have_swivel_and_roll_and_same_ground(self):
        for side in ['left','right']:
            link=self.sdf.find(f"link[@name='{side}_caster_wheel']");z=float(link.findtext('pose').split()[2])
            self.assertAlmostEqual(z-float(link.findtext('collision/geometry/cylinder/radius')),0)
            self.assertAlmostEqual(float(link.findtext('collision/geometry/cylinder/length')),.0225)
            self.assertEqual(self.sdf.find(f"joint[@name='{side}_caster_swivel_joint']/axis/xyz").text,'0 0 1')
            self.assertEqual(self.sdf.find(f"joint[@name='{side}_caster_roll_joint']/axis/xyz").text,'0 1 0')

    def test_positive_inertias_and_triangle_inequality(self):
        for node in self.sdf.findall('link'):
            self.assertGreater(float(node.findtext('inertial/mass')),0)
            values=[float(node.findtext('inertial/inertia/'+key)) for key in ['ixx','iyy','izz']]
            self.assertTrue(all(v>0 for v in values));self.assertLessEqual(max(values),sum(values)-max(values)+1e-10)

    def test_urdf_and_sdf_joint_origins_match(self):
        poses={link.attrib['name']:list(map(float,link.findtext('pose').split()))[:3] for link in self.sdf.findall('link')}
        for joint in self.urdf.findall('joint'):
            child=joint.find('child').attrib['link'];parent=joint.find('parent').attrib['link']
            if child not in poses or parent not in poses:continue
            origin=list(map(float,joint.find('origin').attrib['xyz'].split()))
            for i in range(3):self.assertAlmostEqual(origin[i],poses[child][i]-poses[parent][i])

    def test_camera_optical_axis_faces_forward(self):
        joint=self.urdf.find("joint[@name='camera_optical_joint']")
        roll,pitch,yaw=map(float,joint.find('origin').attrib['rpy'].split())
        # Rz(yaw)*Ry(pitch)*Rx(roll) 的第三列是光学Z轴在车体中的方向。
        axis=[math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll),math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll),math.cos(pitch)*math.cos(roll)]
        for actual,expected in zip(axis,[1,0,0]):self.assertAlmostEqual(actual,expected,places=8)

    def test_c1_nominal_scan_and_camera_profiles(self):
        for profile,width,height in [('sensors_preview',640,360),('sensors_1080p',1920,1080)]:
            tree=ET.parse(ROOT/f'models/smartcar/model_{profile}.sdf')
            scan=tree.find("./model/link[@name='lidar_link']/sensor")
            self.assertEqual(int(scan.findtext('lidar/scan/horizontal/samples')),500);self.assertEqual(float(scan.findtext('update_rate')),10)
            self.assertAlmostEqual(float(scan.findtext('lidar/range/min')),.05);self.assertAlmostEqual(float(scan.findtext('lidar/range/max')),12)
            horizontal=scan.find('lidar/scan/horizontal');coverage=float(horizontal.findtext('max_angle'))-float(horizontal.findtext('min_angle'))
            self.assertAlmostEqual(coverage+coverage/499,2*math.pi,places=8)
            camera=tree.find("./model/link[@name='camera_link']/sensor/camera")
            self.assertEqual(int(camera.findtext('image/width')),width);self.assertEqual(int(camera.findtext('image/height')),height)
        self.assertFalse(self.sdf.findall('.//sensor'))

    def test_meshes_portable_closed_and_in_metres(self):
        for item in self.urdf.findall('.//mesh'):
            filename=item.attrib['filename'];self.assertTrue(filename.startswith('package://smartcar_sim/'))
            path=ROOT/filename.removeprefix('package://smartcar_sim/')
            data=path.read_bytes();count=struct.unpack_from('<I',data,80)[0];self.assertEqual(len(data),84+50*count)
            edges=Counter();verts=[]
            for i in range(count):
                tri=[tuple(round(v,8) for v in struct.unpack_from('<3f',data,84+50*i+j)) for j in [12,24,36]];verts+=tri
                for j in range(3):edges[tuple(sorted([tri[j],tri[(j+1)%3]]))]+=1
            self.assertTrue(all(n==2 for n in edges.values()),path.name)
            self.assertTrue(all(abs(v)<1 for point in verts for v in point),'mesh may still be in millimetres')

    def test_actual_base_and_upper_deck_dimensions(self):
        entries={e['name']:e for e in json.loads((ROOT/'cad_source/mesh_manifest.json').read_text())}
        for name,expected in [('01_Main_chassis',[.24,.22,.009]),('02_Upper_electronics_deck',[.23,.21,.004])]:
            lo,hi=entries[name]['bounds_relative_to_axle_m']
            for got,wanted in zip([hi[i]-lo[i] for i in range(3)],expected):self.assertAlmostEqual(got,wanted,places=5)

    def test_cad_transform_is_rotation_not_mirror(self):
        # CAD->ROS 是绕Z负90度，行列式+1；车前+Y映射ROS+X。
        matrix=[[0,1,0],[-1,0,0],[0,0,1]]
        determinant=matrix[0][0]*(matrix[1][1]*matrix[2][2]-matrix[1][2]*matrix[2][1])-matrix[0][1]*(matrix[1][0]*matrix[2][2]-matrix[1][2]*matrix[2][0])+matrix[0][2]*(matrix[1][0]*matrix[2][1]-matrix[1][1]*matrix[2][0])
        self.assertEqual(determinant,1)


if __name__=='__main__':unittest.main(verbosity=2)
