"""生成本轮全新非对称实验室物理场景，不生成任何SLAM先验地图。"""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET

root = Path.cwd()
sdf = ET.Element('sdf', version='1.8')
world = ET.SubElement(sdf, 'world', name='discovery_lab_world')
# 只复用仿真引擎、光照和地面基础设施；全部墙体和障碍布局在本轮重新定义。
base = ET.parse(root/'ros2_ws/src/smartcar_sim/worlds/smartcar.sdf').find('world')
for tag in ['physics', 'plugin', 'scene', 'light']:
    for item in base.findall(tag):
        world.append(copy.deepcopy(item))
floor = copy.deepcopy(base.find("model[@name='floor']"))
for size in floor.findall('.//plane/size'):
    size.text = '14 12'
world.append(floor)
include = ET.SubElement(world, 'include')
ET.SubElement(include, 'uri').text = '__MODEL_SDF__'
ET.SubElement(include, 'name').text = 'smartcar'
ET.SubElement(include, 'pose').text = '0 0 0.003 0 0 0'


def solid(name, x, y, shape, dimensions, color, yaw=0):
    """静态模型的显示与碰撞使用同一几何，避免只看得见却不能挡车。"""
    height = dimensions[-1]
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {height/2} 0 0 {yaw}'
    link = ET.SubElement(model, 'link', name='link')
    for kind in ['collision', 'visual']:
        item = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(item, 'geometry')
        element = ET.SubElement(geometry, shape)
        if shape == 'box':
            ET.SubElement(element, 'size').text = ' '.join(map(str, dimensions))
        else:
            ET.SubElement(element, 'radius').text = str(dimensions[0])
            ET.SubElement(element, 'length').text = str(height)
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            for field in ['ambient', 'diffuse']:
                ET.SubElement(material, field).text = color


# 外墙中心跨度10×8米，起点为(0,0)，内墙设置错位门洞和遮挡区。
wall = '0.72 0.76 0.8 1'
for name, x, y, sx, sy in [
    ('boundary_west', -5, 0, .15, 8.15),
    ('boundary_east', 5, 0, .15, 8.15),
    ('boundary_north', 0, 4, 10.15, .15),
    ('boundary_south', 0, -4, 10.15, .15),
    ('west_divider_north', -1, 2.45, .15, 3.1),
    ('west_divider_south', -1, -2.55, .15, 2.9),
    ('west_room_return', -4, 1, 2, .15),
    ('east_divider_north', 2.2, 1.9, .15, 4.2),
    ('east_divider_south', 2.2, -3.1, .15, 1.8),
    ('east_room_return', 2.75, 1.1, 1.1, .15),
]:
    solid(name, x, y, 'box', (sx, sy, 1.2), wall)
solid('west_lab_bench', -3.9, 2.7, 'box', (1.1, .7, .8), '0.22 0.55 0.48 1', .15)
solid('west_storage_crate', -3.8, -2.8, 'box', (.8, .9, .7), '0.72 0.43 0.18 1', -.22)
solid('west_round_pillar', -2.7, -1.8, 'cylinder', (.25, 1.1), '0.32 0.48 0.75 1')
solid('central_rack', .5, 2.4, 'box', (.8, 1, .9), '0.68 0.32 0.36 1')
solid('south_equipment', .7, -2.7, 'box', (.7, .65, .65), '0.55 0.39 0.7 1', .32)
solid('east_round_pillar', 4, 2.7, 'cylinder', (.3, 1.1), '0.28 0.58 0.72 1')
solid('east_workbench', 3.8, -3, 'box', (.9, .65, .8), '0.75 0.64 0.25 1')
world.insert(0, ET.Comment(' 2026-10-21全新discovery_lab布局；仅物理场景，不是预构建导航地图。 '))
ET.indent(sdf, space='  ')
path = root/'ros2_ws/src/smartcar_sim/worlds/discovery_lab.sdf'
if path.exists():
    raise FileExistsError(path)
ET.ElementTree(sdf).write(path, encoding='utf-8', xml_declaration=True)
print(path)
