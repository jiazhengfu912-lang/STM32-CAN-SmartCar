#!/usr/bin/env python3
"""运行离线检查并保存真实结果；Gazebo/Ubuntu运行状态单独记录。"""
import ast
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def main():
    """将单包静态结果写入新目录，避免覆盖原始历史证据。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='尚不存在的静态检查结果目录')
    args=parser.parse_args()
    folder=args.output.expanduser().resolve();folder.mkdir(parents=True,exist_ok=False)
    report={'scope':'MODEL_AND_OFFLINE_SOFTWARE_CHECKS_ONLY','ubuntu_humble_build':'NOT_CHECKED_BY_THIS_TOOL','gazebo_physics':'NOT_CHECKED_BY_THIS_TOOL','gazebo_gui':'NOT_CHECKED_BY_THIS_TOOL','camera_and_lidar_messages':'NOT_CHECKED_BY_THIS_TOOL','physical_vehicle':'NOT_CHECKED_BY_THIS_TOOL','checks':{}}
    errors=[]
    for path in sorted(ROOT.rglob('*.py')):
        try:ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
        except Exception as exc:errors.append(f'{path.relative_to(ROOT)}: {exc}')
    report['checks']['python_syntax']={'passed':not errors,'errors':errors}
    xml_errors=[]
    for pattern in ['*.xml','*.urdf','*.sdf','*.config']:
        for path in ROOT.rglob(pattern):
            try:
                if path == ROOT/'config/gazebo_gui.config':
                    # Fortress 官方 GUI 配置允许 dialog/window/plugin 多个顶层元素。
                    content=path.read_text(encoding='utf-8').split('?>',1)[-1]
                    ET.fromstring('<gazebo_gui_config>'+content+'</gazebo_gui_config>')
                else:ET.parse(path)
            except Exception as exc:xml_errors.append(f'{path.relative_to(ROOT)}: {exc}')
    report['checks']['xml_well_formed']={'passed':not xml_errors,'errors':xml_errors,'note':'XML syntax plus unit-test semantic checks; Fortress GUI config is an XML fragment with multiple top-level elements. Not a Fortress schema check.'}
    import yaml
    bridge=yaml.safe_load((ROOT/'config/bridge.yaml').read_text(encoding='utf-8'))
    expected={'/clock','/sim/drive_cmd_vel','/sim/odom','/tf','/joint_states'}
    valid={item['ros_topic_name'] for item in bridge}==expected and all(item['gz_type_name'].startswith('ignition.msgs.') for item in bridge)
    report['checks']['fortress_bridge_yaml']={'passed':valid}
    for path in (ROOT/'rviz').glob('*.rviz'):yaml.safe_load(path.read_text(encoding='utf-8'))
    report['checks']['rviz_yaml']={'passed':True}
    stream=io.StringIO();suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    report['checks']['unit_and_geometry_tests']={'passed':result.wasSuccessful(),'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors)}
    report['checks']['linux_file_newlines']={'passed':all(b'\r\n' not in p.read_bytes() and not p.read_bytes().startswith(b'\xef\xbb\xbf') for pattern in ['*.py','*.bash'] for p in ROOT.rglob(pattern))}
    source_ok=True
    for entry in json.loads((ROOT/'cad_source/placements.json').read_text()):
        actual=hashlib.sha256((ROOT/'cad_source'/entry['source']).read_bytes()).hexdigest()
        source_ok=source_ok and actual==entry['sha256']
    report['checks']['archived_cad_mesh_hashes']={'passed':source_ok}
    report['result']='PASS' if all(c['passed'] for c in report['checks'].values()) else 'FAIL'
    (folder/'tests.log').write_text(stream.getvalue(),encoding='utf-8')
    (folder/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(stream.getvalue());print(json.dumps(report,indent=2))
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':sys.exit(main())
