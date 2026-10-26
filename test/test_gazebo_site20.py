"""Checks for metric DEM placement and the rover's physical contact geometry.

Run with system ROS Python: python3 -m pytest test/test_gazebo_site20.py
No Gazebo server or downloaded DEM is needed.
"""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
import yaml

xacro = pytest.importorskip('xacro')
PACKAGE = Path(__file__).resolve().parents[1]/'assets/USD_Assets/lumos1_rockerless_URDF_description/lumos1_rockerless_URDF_description'
spec = importlib.util.spec_from_file_location('prepare_site20', PACKAGE/'scripts/prepare_site20.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


@pytest.fixture
def dem(tmp_path):
    # Distinct X/Y gradients detect flips and transpositions. Even size needs padding.
    rows, cols = np.indices((10, 10))
    values = (6000 + cols*0.5 - rows*0.25).astype(np.float32)
    path = tmp_path/'dem.npy'
    np.save(path, values)
    path.with_suffix('.yaml').write_text(yaml.safe_dump({'size':[10,10], 'pixel_size':[5.,-5.]}))
    return path, values


def test_full_map_preserves_metric_samples_and_origin(dem, tmp_path):
    path, data = dem
    output = tmp_path/'world'
    result = prepare.prepare(path, PACKAGE, output)
    pixels = np.asarray(Image.open(output/'site20_height.png'))
    assert pixels.shape == (17,17)
    top, left = result['padding_top_left']
    heights = pixels*(float(data.max())-float(data.min()))/65535+float(data.min())
    np.testing.assert_allclose(heights[top:top+10,left:left+10], data,
                               atol=result['elevation_quantization_m']/2+1e-7, rtol=0)
    assert heights[8,8] == pytest.approx(result['elevation_origin_m'], abs=1e-4)
    world = ET.parse(output/'site20.sdf').getroot()
    model = world.find('.//model')
    collision = model.find('link/collision/geometry/heightmap')
    visual = model.find('link/visual/geometry/heightmap')
    assert collision.findtext('uri') == visual.findtext('uri')
    assert float(visual.findtext('size').split()[0]) == 80
    # DART's N sample cells span N-1 intervals; Ogre2 uses N-1 directly.
    collision_span = float(collision.findtext('size').split()[0])*16/17
    assert collision_span == float(visual.findtext('size').split()[0])
    # DART uses model pose, Ogre2 uses heightmap/pos. Both must reach the same Z.
    assert float(model.findtext('pose').split()[2]) == float(visual.findtext('pos').split()[2])
    assert collision.findtext('pos') == '0 0 0'


def test_cropped_spawn_follows_positive_x_and_y_slopes(dem, tmp_path):
    path, _ = dem
    center = prepare.prepare(path, PACKAGE, tmp_path/'a', samples=9)
    east = prepare.prepare(path, PACKAGE, tmp_path/'b', samples=9, x=5)
    north = prepare.prepare(path, PACKAGE, tmp_path/'c', samples=9, y=5)
    assert east['z']-center['z'] == pytest.approx(.5, abs=1e-4)
    assert north['z']-center['z'] == pytest.approx(.25, abs=1e-4)


@pytest.mark.parametrize('options', [{'samples':4}, {'samples':-1}, {'samples':33}, {'x':100}, {'y':float('nan')}])
def test_invalid_spawn_and_resolution_rejected(dem, tmp_path, options):
    with pytest.raises(ValueError):
        prepare.prepare(dem[0], PACKAGE, tmp_path/'out', **options)


def test_missing_elevations_rejected(dem, tmp_path):
    path, data = dem
    data[0,0] = -32768
    np.save(path, data)
    with pytest.raises(ValueError, match='missing elevations'):
        prepare.prepare(path, PACKAGE, tmp_path/'out')


def test_rover_wheels_have_consistent_axes_and_ground_contact():
    robot = ET.fromstring(xacro.process_file(str(PACKAGE/'urdf/lumos1_rockerless_URDF.xacro')).toxml())
    for suffix in ('lf','lb','rf','rb'):
        joint = robot.find(f"joint[@name='joint_{suffix}']")
        assert joint.find('axis').get('xyz') == '1.0 0.0 0.0'
        assert float(joint.find('limit').get('effort')) > 0
        wheel = robot.find(f"link[@name='tire_{suffix}_1']/collision/geometry/cylinder")
        assert float(wheel.get('radius')) == .08
    drive = robot.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert len(drive.findall('left_joint')) == len(drive.findall('right_joint')) == 2
    assert robot.find("gazebo/plugin[@filename='libgazebo_ros_control.so']") is None
