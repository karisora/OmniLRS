"""Check that the Gazebo map preserves OmniLRS Lunaryard's metric terrain."""

import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest

pytest.importorskip('xacro')
ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'assets/USD_Assets/lumos1_rockerless_URDF_description/lumos1_rockerless_URDF_description'
sys.path.insert(0, str(PACKAGE/'scripts'))
spec = importlib.util.spec_from_file_location('prepare_lunaryard', PACKAGE/'scripts/prepare_lunaryard.py')
yard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(yard)
sys.path.pop(0)
CONFIG = ROOT/'cfg/environment/lunaryard_40m.yaml'


@pytest.fixture(scope='module')
def exported(tmp_path_factory):
    output = tmp_path_factory.mktemp('lunaryard')
    metadata = yard.prepare(ROOT, CONFIG, PACKAGE, output)
    return output, metadata


def test_native_lunaryard_samples_keep_two_centimeter_spacing(exported):
    output, metadata = exported
    source = np.load(output/'lunaryard_dem.npy')
    assert source.shape == (2000, 2000)
    assert metadata['seed'] == 42
    assert metadata['generator'].endswith('GenerateProceduralMoonYard')
    assert metadata['pixel_size_m'] == .02
    assert metadata['crater_count'] > 0
    pixels = np.asarray(Image.open(output/'lunaryard_height.png'))
    assert pixels.shape == (2049, 2049)
    top, left = metadata['padding_top_left']
    world = ET.parse(output/'lunaryard.sdf').getroot().find('world')
    assert world.get('name') == 'lunaryard'
    model = world.find('model')
    position = np.fromstring(model.findtext('pose'), sep=' ')[:3]
    collision = model.find('link/collision/geometry/heightmap')
    visual = model.find('link/visual/geometry/heightmap')
    collision_size = np.fromstring(collision.findtext('size'), sep=' ')
    visual_size = np.fromstring(visual.findtext('size'), sep=' ')
    assert visual_size[0]/2048 == pytest.approx(.02)
    assert collision_size[0]/2049 == pytest.approx(.02)
    np.testing.assert_allclose(np.fromstring(visual.findtext('pos'), sep=' '), position)
    decoded = position[2]+pixels.astype(float)*visual_size[2]/65535
    np.testing.assert_allclose(decoded[top:top+2000, left:left+2000], source,
                               atol=metadata['elevation_quantization_m']/2+1e-7, rtol=0)
    # Native TerrainManager flips DEM rows onto a grid from Y=0 upward.
    for row, col in ((0, 0), (123, 456), (1999, 1999)):
        wx = position[0]+(left+col-1024)*.02
        wy = position[1]+(1024-top-row)*.02
        assert wx == pytest.approx(col*.02)
        assert wy == pytest.approx((1999-row)*.02)
    assert collision.findtext('uri') == visual.findtext('uri')


def test_export_matches_the_unmodified_native_generator(exported):
    output, metadata = exported
    dem, mask, craters, settings, seed = yard.generate(ROOT, CONFIG, None)
    np.testing.assert_array_equal(np.load(output/'lunaryard_dem.npy'), dem)
    np.testing.assert_array_equal(np.load(output/'lunaryard_mask.npy'), mask)
    assert len(craters) == metadata['crater_count']
    assert metadata['deformation_enabled'] is False
    assert settings['resolution'] == .02 and seed == 42
    assert metadata['x'] == metadata['y'] == 20
    assert metadata['z'] > dem[999:1001, 999:1001].max()
    manifest = json.loads((output/'spawn.json').read_text())
    material = manifest['surface_material']
    assert material['name'] == 'Basalt'
    assert material['material_prim'] == '/Lunaryard/Looks/Basalt'
    assert Path(material['diffuse_source']) == ROOT/'assets/Textures/GravelStones/gravel_stones_diff_4k.png'
    assert Path(material['normal_source']) == ROOT/'assets/Textures/GravelStones/gravel_stones_nor_gl_4k.exr'
    assert manifest['texture_size_m'] == pytest.approx(.2)
    visual = ET.parse(output/'lunaryard.sdf').getroot().find('.//visual/geometry/heightmap')
    assert visual.findtext('texture/diffuse') == Path(material['diffuse_source']).as_uri()
    assert Path(material['normal_uri'].removeprefix('file://')).is_file()


def test_default_gazebo_launcher_selects_lunaryard(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_log'))
    pytest.importorskip('launch_ros')
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext
    import runpy
    module = runpy.run_path(str(PACKAGE/'launch/lunaryard.launch.py'))
    description = module['generate_launch_description']()
    defaults = {action.name: ''.join(part.perform(LaunchContext()) for part in action.default_value)
                for action in description.entities if isinstance(action, DeclareLaunchArgument)}
    assert defaults['map'] == 'lunaryard'
    assert defaults['x'] == defaults['y'] == '20'
    assert defaults['output'].endswith('build/gazebo_lunaryard')
    assert defaults['texture_size'] == 'auto'
    aliases = (PACKAGE/'launch/gazebo.launch.py').read_text()
    assert 'launch/lunaryard.launch.py' in aliases


def test_invalid_spawn_is_rejected_before_generation(tmp_path):
    with pytest.raises(ValueError, match='finite'):
        yard.prepare(ROOT, CONFIG, PACKAGE, tmp_path, x=float('nan'))
    assert not (tmp_path/'lunaryard.sdf').exists()
