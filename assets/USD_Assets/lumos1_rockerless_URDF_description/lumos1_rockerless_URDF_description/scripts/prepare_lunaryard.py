#!/usr/bin/env python3
"""Export OmniLRS's native Lunaryard generator to a metric Gazebo heightmap."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
from string import Template
import sys
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image
import xacro
import yaml

from prepare_site20 import matte_terrain_resources


def native_lighting(config_path, elevation=None, azimuth=None, intensity=None):
    """Translate the preset's initial SunConf and create_sun() orientation."""
    sun = yaml.safe_load(config_path.read_text())['sun_settings']
    elevation = float(sun['elevation'] if elevation is None else elevation)
    azimuth = float(sun['azimuth'] if azimuth is None else azimuth)
    # UsdLux intensity and SDF relative light power are different quantities.
    # Preserve the preset's intensity relative to the standard SunConf value;
    # explicit sun_intensity remains a Gazebo multiplier, as before.
    power = float(sun['intensity'])/1750. if intensity is None else float(intensity)
    if not all(math.isfinite(v) for v in (elevation, azimuth, power)):
        raise ValueError('Sun settings must be finite')
    if not 0 <= elevation <= 90 or not 0 < power <= 100:
        raise ValueError('Invalid sun elevation or relative intensity')
    e, a = math.radians(elevation), math.radians(azimuth)
    # UsdLux emits along -Z. OmniLRS first applies its fixed local quaternion,
    # then Euler xyz(0, elevation, azimuth-90) on the parent Xform.
    direction = (math.cos(e)*math.sin(a), -math.cos(e)*math.cos(a), -math.sin(e))
    color = np.asarray(sun['color'], dtype=float)
    if color.shape != (3,) or not np.isfinite(color).all() or np.any((color < 0) | (color > 1)):
        raise ValueError('Invalid native sun color')
    diffuse = np.append(color*float(sun['diffuse_multiplier']), 1.)
    specular = np.append(color*float(sun['specular_multiplier']), 1.)
    return dict(source='sun_settings in native Lunaryard YAML', elevation_deg=elevation,
                azimuth_deg=azimuth, direction=list(direction), color=color.tolist(),
                diffuse=diffuse.tolist(), specular=specular.tolist(),
                native_intensity=float(sun['intensity']), gazebo_intensity=power,
                intensity_conversion='Relative to OmniLRS SunConf intensity 1750; not a photometric calibration',
                temperature_kelvin=float(sun['temperature']), angular_diameter_deg=float(sun['angle']),
                ambient_rgb=[0., 0., 0.], mode='Fixed initial sun_settings; no stellar-engine time updates')


def native_material(root, config_path, output):
    """Use the bundled maps and UV scale selected by the Lunaryard preset."""
    config = yaml.safe_load(config_path.read_text())
    material_prim = config['terrain_manager']['texture_path']
    name = material_prim.rsplit('/', 1)[-1]
    # These are the same registrations as LunaryardController.build_scene().
    definitions = {'Basalt': 'GravelStones.mdl', 'Sand': 'Sand.mdl',
                   'LunarRegolith8k': 'LunarRegolith8k.mdl'}
    if name not in definitions:
        raise ValueError(f'Unsupported native Lunaryard material: {material_prim}')
    mdl = root/'assets/Textures'/definitions[name]
    text = mdl.read_text()

    def texture(parameter):
        match = re.search(r'^\s*'+parameter+r':\s*texture_2d\("([^"]+)"', text, re.MULTILINE)
        if not match:
            raise ValueError(f'Missing {parameter} in {mdl}')
        path = (mdl.parent/match.group(1)).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        return path

    diffuse = texture('diffuse_texture')
    normal_source = texture('normalmap_texture')
    # Gazebo uses OpenGL normals. Prefer the original GL companion map where
    # OmniLRS's MDL uses a DirectX map with flip_tangent_v enabled.
    directx = '_dx_' in normal_source.name
    gl_source = normal_source.with_name(normal_source.name.replace('_dx_', '_gl_'))
    if directx and gl_source.is_file():
        normal_source, directx = gl_source, False
    normal = normal_source
    if normal.suffix.lower() == '.exr' or directx:
        os.environ.setdefault('OPENCV_IO_ENABLE_OPENEXR', '1')
        import cv2
        values = cv2.imread(str(normal_source), cv2.IMREAD_UNCHANGED)
        if values is None:
            raise ValueError(f'Cannot read native normal map: {normal_source}')
        integer_max = np.iinfo(values.dtype).max if np.issubdtype(values.dtype, np.integer) else None
        values = values[:, :, :3][:, :, ::-1].astype(np.float32)
        if integer_max is not None:
            values /= integer_max
        if directx:
            values[:, :, 1] = 1-values[:, :, 1]
        target = output/'materials'
        target.mkdir(parents=True, exist_ok=True)
        normal = target/f'{name}_normal_gl.png'
        Image.fromarray(np.rint(np.clip(values, 0, 1)*255).astype(np.uint8)).save(normal)
    scale = re.search(r'^\s*texture_scale:\s*float2\(([-+\deE.]+)f?\)', text, re.MULTILINE)
    if not scale or float(scale.group(1)) <= 0:
        raise ValueError(f'Expected a positive isotropic texture_scale in {mdl}')
    return dict(source_kind='OmniLRS bundled Lunaryard material', name=name,
                material_prim=material_prim, mdl=str(mdl),
                diffuse_source=str(diffuse), normal_source=str(normal_source),
                diffuse_uri=diffuse.as_uri(), normal_uri=normal.as_uri(),
                tile_size_m=1/float(scale.group(1)),
                conversion='Original diffuse pixels; EXR normals converted to linear PNG when needed')


def generate(root, config_path, seed):
    """Use the actual OmniLRS classes and YAML, without starting Isaac Sim."""
    sys.path.insert(0, str(root))
    from src.configurations.procedural_terrain_confs import MoonYardConf
    from src.terrain_management.terrain_generation import GenerateProceduralMoonYard

    config = yaml.safe_load(config_path.read_text())
    yard = config['lunaryard_settings']
    seed = config['seed'] if seed is None else seed
    if not isinstance(seed, int) or seed < 0:
        raise ValueError('Lunaryard seed must be a nonnegative integer')
    # Resolve the four Hydra references used in the native moon_yard section.
    # All numeric generation parameters continue to come from the source YAML.
    references = {'${....seed}': seed,
                  '${....lunaryard_settings.resolution}': yard['resolution'],
                  '${....lunaryard_settings.lab_length}': yard['lab_length'],
                  '${....lunaryard_settings.lab_width}': yard['lab_width']}

    def resolve(value):
        if isinstance(value, dict):
            return {key: resolve(item) for key, item in value.items()}
        if isinstance(value, list):
            return [resolve(item) for item in value]
        if isinstance(value, str) and value.startswith('${'):
            if value not in references:
                raise ValueError(f'Unsupported Lunaryard configuration reference: {value}')
            return references[value]
        return value

    settings = resolve(config['terrain_manager']['moon_yard'])
    settings['crater_generator']['profiles_path'] = str(root/settings['crater_generator']['profiles_path'])
    # The exported map is static, as in the native non-deformable preset.
    settings['deformation_engine']['enable'] = False
    generator = GenerateProceduralMoonYard(MoonYardConf(**settings))
    dem, mask, craters = generator.randomize()
    return dem, mask, craters, yard, seed


def prepare(root, config_path, package, output, seed=None, x=20., y=20., yaw=0.,
            sun_elevation=None, sun_azimuth=None, sun_intensity=None, texture_size=None,
            rendering_resources=None):
    root, config_path, package, output = (Path(p).expanduser().resolve() for p in (root, config_path, package, output))
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError('Spawn settings must be finite')
    if texture_size is not None and (not math.isfinite(texture_size) or texture_size <= 0):
        raise ValueError('Invalid texture size')
    lighting = native_lighting(config_path, sun_elevation, sun_azimuth, sun_intensity)
    dem, mask, craters, yard, seed = generate(root, config_path, seed)
    spacing = float(yard['resolution'])
    rows, cols = dem.shape
    if rows != cols or spacing <= 0 or not np.isfinite(dem).all():
        raise ValueError('Gazebo Lunaryard requires a finite square DEM with positive resolution')
    side = 2**math.ceil(math.log2(rows-1))+1
    top = left = side//2-rows//2
    low, high = float(dem.min()), float(dem.max())
    height_range = max(high-low, 1.)
    pixels = np.rint((dem-low)*65535/height_range).astype(np.uint16)
    pixels = np.pad(pixels, ((top, side-rows-top), (left, side-cols-left)), mode='edge')
    # Native TerrainManager places np.flip(DEM, 0) on its +X/+Y grid.
    # Keep those world coordinates and every 2 cm height sample unchanged.
    origin_x, origin_y = cols//2*spacing, (rows-1-rows//2)*spacing

    def height_at(px, py):
        col, row = px/spacing, rows-1-py/spacing
        if not (0 <= col < cols-1 and 0 <= row < rows-1):
            raise ValueError('Spawn footprint lies outside the Lunaryard terrain')
        i, j = int(row)+top, int(col)+left
        u, v = col-int(col), row-int(row)
        value = (1-v)*((1-u)*float(pixels[i,j])+u*float(pixels[i,j+1]))
        value += v*((1-u)*float(pixels[i+1,j])+u*float(pixels[i+1,j+1]))
        return low+value*height_range/65535

    z = max(height_at(x+dx, y+dy) for dx in (-.3, 0, .3) for dy in (-.3, 0, .3))+.12
    output.mkdir(parents=True, exist_ok=True)
    heightmap = output/'lunaryard_height.png'
    Image.fromarray(pixels).save(heightmap)
    np.save(output/'lunaryard_dem.npy', dem)
    np.save(output/'lunaryard_mask.npy', mask)
    material = native_material(root, config_path, output)
    texture_size = material['tile_size_m'] if texture_size is None else texture_size
    world = Template((package/'worlds/site20.sdf.in').read_text()).substitute(
        world_name='lunaryard', world_title='OmniLRS Lunaryard', terrain_name='lunaryard_terrain',
        terrain_x=origin_x, terrain_y=origin_y, terrain_z=low,
        heightmap_uri=escape(heightmap.as_uri()), extent=(side-1)*spacing,
        collision_extent=side*spacing, height_range=height_range,
        texture_uri=escape(material['diffuse_uri']),
        normal_uri=escape(material['normal_uri']), texture_size=texture_size,
        sun_direction=' '.join(str(v) for v in lighting['direction']), sun_intensity=lighting['gazebo_intensity'],
        sun_diffuse=' '.join(str(v) for v in lighting['diffuse']),
        sun_specular=' '.join(str(v) for v in lighting['specular']),
        ambient_rgb=' '.join(str(v) for v in lighting['ambient_rgb']),
        camera_pose=f'{x+1.2} {y-1.2} {z+.9} 0 0.48 2.356194490192345')
    (output/'lunaryard.sdf').write_text(world)
    urdf = xacro.process_file(str(package/'urdf/lumos1_rockerless_URDF.xacro'),
                             mappings={'mesh_directory': (package/'meshes').as_uri()}).toxml()
    (output/'lumos.urdf').write_text(urdf)
    crater_metadata = [dict(coord=list(c.coord), diameter_pixels=c.size, profile_id=int(c.crater_profile_id)) for c in craters]
    (output/'craters.json').write_text(json.dumps(crater_metadata, indent=2)+'\n')
    metadata = dict(world='lunaryard', model='lumos', x=x, y=y, z=z, yaw=yaw,
                    config=str(config_path), config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                    generator='src.terrain_management.terrain_generation.GenerateProceduralMoonYard',
                    seed=seed, pixel_size_m=spacing, source_shape=[rows, cols], heightmap_samples=side,
                    padding_top_left=[top, left], heightmap_origin_xy=[origin_x, origin_y],
                    source_bounds_xy=[0., (cols-1)*spacing, 0., (rows-1)*spacing],
                    elevation_quantization_m=height_range/65535, crater_count=len(craters),
                    deformation_enabled=False, sun_elevation_deg=lighting['elevation_deg'],
                    sun_azimuth_deg=lighting['azimuth_deg'], sun_intensity=lighting['gazebo_intensity'],
                    lighting=lighting, texture_size_m=texture_size, surface_material=material,
                    urdf=str(output/'lumos.urdf'))
    if rendering_resources is not None:
        metadata['rendering_resources'] = str(matte_terrain_resources(rendering_resources, output/'rendering'))
    (output/'spawn.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--rendering-resources', type=Path)
    parser.add_argument('--texture-size', type=lambda value: None if value == 'auto' else float(value),
                        default=None, help='Tile size in meters; auto uses the native MDL UV scale')
    for name in ('sun-elevation', 'sun-azimuth', 'sun-intensity'):
        parser.add_argument('--'+name, type=lambda value: None if value == 'auto' else float(value),
                            default=None, help='auto uses the native sun_settings preset')
    for name, default in (('x', 20.), ('y', 20.), ('yaw', 0.)):
        parser.add_argument('--'+name, type=float, default=default)
    args = parser.parse_args()
    try:
        result = prepare(args.root, args.config, args.package, args.output, args.seed,
                         args.x, args.y, args.yaw, args.sun_elevation, args.sun_azimuth,
                         args.sun_intensity, args.texture_size, args.rendering_resources)
    except (ValueError, FileNotFoundError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
