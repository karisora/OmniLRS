"""Checks for metric DEM placement and the rover's physical contact geometry.

Run with system ROS Python: python3 -m pytest test/test_gazebo_site20.py
No Gazebo server or downloaded DEM is needed.
"""

import importlib.util
import hashlib
import json
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


def test_velocity_bridge_reaches_the_four_wheel_controller():
    robot = ET.fromstring(xacro.process_file(str(PACKAGE/'urdf/lumos1_rockerless_URDF.xacro')).toxml())
    drive = robot.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    bridges = yaml.safe_load((PACKAGE/'config/bridge.yaml').read_text())
    command = next(item for item in bridges if item['ros_topic_name'] == '/cmd_vel')
    assert command['direction'] == 'ROS_TO_GZ'
    assert command['ros_type_name'] == 'geometry_msgs/msg/Twist'
    assert command['gz_topic_name'] == drive.findtext('topic')


def test_stereo_optical_frames_and_calibration_match_physical_baseline():
    robot = ET.fromstring(xacro.process_file(str(PACKAGE/'urdf/lumos1_rockerless_URDF.xacro')).toxml())
    bridges = yaml.safe_load((PACKAGE/'config/bridge.yaml').read_text())

    def transform(link):
        if link == 'base_footprint':
            return np.eye(4)
        joint = next(j for j in robot.findall('joint') if j.find('child').get('link') == link)
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        cr, cp, cy = np.cos([roll, pitch, yaw])
        sr, sp, sy = np.sin([roll, pitch, yaw])
        rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])
        local = np.eye(4)
        local[:3, :3], local[:3, 3] = rotation, xyz
        return transform(joint.find('parent').get('link')) @ local

    left = transform('stereo_left_optical_frame')
    right = transform('stereo_right_optical_frame')
    baseline = (np.linalg.inv(left) @ right)[0, 3]
    assert baseline == pytest.approx(.12)
    # Both look forward; optical right/down map to rover -Y/-Z.
    for frame in (left, right):
        np.testing.assert_allclose(frame[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-10)
    for side in ('left', 'right'):
        sensor = robot.find(f"gazebo/sensor[@name='stereo_{side}']")
        camera = sensor.find('camera')
        frame_id = f'stereo_{side}_optical_frame'
        assert sensor.findtext('gz_frame_id') == camera.findtext('optical_frame_id') == frame_id
        assert sensor.findtext('update_rate') == '15'
        assert camera.findtext('image/width') == '640'
        assert camera.findtext('image/height') == '480'
        fx = float(camera.findtext('lens/intrinsics/fx'))
        assert float(camera.findtext('lens/projection/p_fx')) == fx
        assert float(camera.findtext('lens/projection/tx')) == pytest.approx(-fx*baseline if side == 'right' else 0)
        for topic, ros_type, gz_type in ((sensor.findtext('topic'), 'Image', 'Image'),
                                        (camera.findtext('camera_info_topic'), 'CameraInfo', 'CameraInfo')):
            bridge = next(item for item in bridges if item['gz_topic_name'] == topic)
            assert bridge['ros_topic_name'] == topic
            assert bridge['direction'] == 'GZ_TO_ROS'
            assert bridge['qos_profile'] == 'SENSOR_DATA'
            assert bridge['ros_type_name'] == f'sensor_msgs/msg/{ros_type}'
            assert bridge['gz_type_name'] == f'gz.msgs.{gz_type}'


def test_world_enables_camera_rendering(dem, tmp_path):
    prepare.prepare(dem[0], PACKAGE, tmp_path/'world')
    world = ET.parse(tmp_path/'world/site20.sdf').getroot()
    plugin = world.find("world/plugin[@name='gz::sim::systems::Sensors']")
    assert plugin.findtext('render_engine') == 'ogre2'


def test_crater_profile_excavates_bowl_and_raises_rim():
    radius = 40
    profile = prepare.crater_profile(np.array([0,20,40,66,80]),radius,8,2)
    assert profile[0] == pytest.approx(-8)
    assert profile[1] < 0
    assert profile[2] == pytest.approx(2)
    np.testing.assert_array_equal(profile[3:],0)


def test_seeded_craters_change_collision_terrain_without_touching_spawn(tmp_path):
    # A flat reference makes crater relief independent of the measured terrain slope.
    source = np.full((257,257),6000.,dtype=np.float32)
    path = tmp_path/'dem.npy'
    np.save(path,source)
    path.with_suffix('.yaml').write_text(yaml.safe_dump({'size':[257,257],'pixel_size':[5.,-5.]}))
    a = prepare.prepare(path,PACKAGE,tmp_path/'a',crater_count=12,crater_seed=42)
    b = prepare.prepare(path,PACKAGE,tmp_path/'b',crater_count=12,crater_seed=42)
    c = prepare.prepare(path,PACKAGE,tmp_path/'c',crater_count=12,crater_seed=43)
    pixels = np.asarray(Image.open(tmp_path/'a/site20_height.png'))
    np.testing.assert_array_equal(pixels,np.asarray(Image.open(tmp_path/'b/site20_height.png')))
    assert not np.array_equal(pixels,np.asarray(Image.open(tmp_path/'c/site20_height.png')))
    assert a['z'] == b['z'] == pytest.approx(.12,abs=a['elevation_quantization_m'])
    assert np.ptp(pixels) > 0
    np.testing.assert_array_equal(np.load(path),source)
    craters = json.loads((tmp_path/'a/craters.json').read_text())
    assert len(craters) == a['crater_count'] == 12
    for crater in craters:
        assert np.hypot(crater['x'],crater['y']) >= crater['radius_m']*1.65+12
        assert crater['radius_m'] >= 20


def test_low_sun_direction_and_packaged_normal_map(dem,tmp_path):
    prepare.prepare(dem[0],PACKAGE,tmp_path/'world',sun_elevation=5,sun_azimuth=90)
    world = ET.parse(tmp_path/'world/site20.sdf').getroot()
    light = world.find('world/light')
    direction = np.fromstring(light.findtext('direction'),sep=' ')
    assert np.linalg.norm(direction) == pytest.approx(1)
    assert direction[0] == pytest.approx(0,abs=1e-10)
    assert direction[1] < 0
    assert direction[2] == pytest.approx(-np.sin(np.deg2rad(5)))
    assert light.findtext('cast_shadows') == 'true'
    visual = world.find('.//visual/geometry/heightmap')
    assert float(visual.findtext('texture/size')) == .375
    assert visual.findtext('texture/diffuse').endswith('/polar_lht_diffuse.png')
    normal = np.asarray(Image.open(PACKAGE/'materials/textures/polar_lht_normal.png'))
    assert normal.shape == (2048,2048,3)
    assert np.std(normal[:,:,0]) > 1


def test_polar_material_source_and_highlands_appearance_are_preserved(dem, tmp_path):
    textures = PACKAGE/'materials/textures'
    metadata = json.loads((textures/'polar_lht.json').read_text())
    source = PACKAGE/'materials/sources/polar_lht.png'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == metadata['source_sha256']
    assert metadata['member'].startswith('Terrain13_LHT/')
    assert 'laboratory' in metadata['source_kind']
    diffuse = np.asarray(Image.open(textures/'polar_lht_diffuse.png'))
    np.testing.assert_array_equal(diffuse[:, :, 0], diffuse[:, :, 1])
    np.testing.assert_array_equal(diffuse[:, :, 0], diffuse[:, :, 2])
    srgb = diffuse[:, :, 0].astype(float)/255
    linear = np.where(srgb <= .04045, srgb/12.92, ((srgb+.055)/1.055)**2.4)
    assert linear.mean() == pytest.approx(.30, abs=.005)
    assert linear.std() < .025
    normal = np.asarray(Image.open(textures/'polar_lht_normal.png')).astype(float)/127.5-1
    assert np.sqrt(np.mean(normal[:, :, 0]**2+normal[:, :, 1]**2)) < .035
    np.testing.assert_allclose(np.linalg.norm(normal, axis=-1), 1, atol=.008)
    result = prepare.prepare(dem[0], PACKAGE, tmp_path/'world')
    assert result['surface_material'] == metadata
    assert result['texture_size_m'] == metadata['tile_size_m']
    assert result['sun_intensity'] == 10


@pytest.mark.parametrize('options', [
    {'crater_count':-1},{'crater_count':2001},{'crater_seed':-2},
    {'sun_elevation':0},{'sun_elevation':91},{'sun_azimuth':float('nan')},
    {'sun_intensity':0},{'texture_size':0},
])
def test_invalid_environment_options_rejected(dem,tmp_path,options):
    with pytest.raises(ValueError):
        prepare.prepare(dem[0],PACKAGE,tmp_path/'out',**options)


def test_matte_heightmap_override_preserves_installed_shaders(tmp_path):
    source = tmp_path/'vendor'
    relative = Path('ogre2/media/Hlms/Terra/Any/800.PixelShader_piece_ps.any')
    shader = source/relative
    shader.parent.mkdir(parents=True)
    shader.write_bytes(b'@else\r\nfloat roughness@n = 0;\r\n@end\r\n')
    output = prepare.matte_terrain_resources(source,tmp_path/'local')
    assert b'roughness@n = 0.95f;' in (output/relative).read_bytes()
    assert b'roughness@n = 0;' in shader.read_bytes()
    # If upstream stops forcing perfectly smooth terrain, leave its resources alone.
    shader.write_text('float roughness@n = 0.8f;')
    assert prepare.matte_terrain_resources(source,tmp_path/'unused') == source
    assert not (tmp_path/'unused').exists()
