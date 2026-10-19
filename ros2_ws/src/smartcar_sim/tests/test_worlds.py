"""Scene selection failures and office collision geometry; not a runtime verdict."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sim_launch', ROOT/'launch/sim.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class WorldTests(unittest.TestCase):
    def test_bundled_and_absolute_selection(self):
        for selection in ['smartcar', 'office_loop.sdf', str(ROOT/'worlds/office_loop.sdf')]:
            path, tree, uri = launch.load_world(ROOT/'worlds', selection)
            self.assertTrue(path.is_absolute())
            self.assertEqual(uri.text, '__MODEL_SDF__')
            self.assertEqual(tree.findtext('./world/include/name'), 'smartcar')

    def test_missing_world_and_wrong_template_fail_early(self):
        with self.assertRaisesRegex(ValueError, 'World not found'):
            launch.load_world(ROOT/'worlds', 'missing_scene')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'wrong.sdf'
            for xml in ['<sdf><world/></sdf>', '<sdf><world/><world/></sdf>']:
                path.write_text(xml)
                with self.assertRaises(ValueError):
                    launch.load_world(ROOT/'worlds', str(path))

    def test_collision_and_visual_match_and_doors_are_clear(self):
        world = ET.parse(ROOT/'worlds/office_loop.sdf').find('world')
        models = world.findall('model')
        self.assertEqual(len({m.get('name') for m in models}), len(models))
        obstacles = []
        for model in models:
            self.assertEqual(model.findtext('static'), 'true')
            if model.get('name') == 'floor':
                continue
            self.assertEqual(ET.tostring(model.find('./link/collision/geometry')).strip(),
                             ET.tostring(model.find('./link/visual/geometry')).strip())
            pose = [float(v) for v in model.findtext('pose').split()]
            geometry = model.find('./link/collision/geometry')
            if geometry.find('box') is not None:
                sx, sy, h = map(float, geometry.findtext('box/size').split())
                obstacles.append(('box', pose[0], pose[1], sx/2, sy/2))
            else:
                r = float(geometry.findtext('cylinder/radius'))
                obstacles.append(('circle', pose[0], pose[1], r, r))
        # Conservative 0.25 m car radius, including turns; doors must also allow +/-0.30 m offsets.
        points = [(0, 0)] + [(x, y) for x in [-3, 3] for y in [-1, 1]] + [(0, -3), (0, 3)]
        for x, y in points:
            for _, cx, cy, hx, hy in obstacles:
                dx, dy = max(abs(x-cx)-hx, 0), max(abs(y-cy)-hy, 0)
                self.assertGreater((dx*dx+dy*dy)**.5, .55)


if __name__ == '__main__':
    unittest.main()
