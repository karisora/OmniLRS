#!/usr/bin/env python3
"""Generate a metric Gazebo heightmap without resampling Site20's 5 m grid."""

import argparse
import json
import math
from pathlib import Path
import secrets
import shutil
from string import Template
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image
import xacro
import yaml


def matte_terrain_resources(source, output):
    """Work around Ogre2's zero roughness for SDF heightmaps without roughness maps.

    GZ_RENDERING_RESOURCE_PATH scopes the copy to this simulator process. Never
    edit installed Gazebo resources. A fixed upstream shader needs no override.
    """
    source, output = Path(source).resolve(), Path(output).resolve()
    relative = Path('ogre2/media/Hlms/Terra/Any/800.PixelShader_piece_ps.any')
    shader = source/relative
    contents = shader.read_bytes()
    old = b'float roughness@n = 0;'
    if old not in contents:
        return source
    shutil.copytree(source/'ogre2/media',output/'ogre2/media',dirs_exist_ok=True)
    (output/relative).write_bytes(contents.replace(old,b'float roughness@n = 0.95f;'))
    return output


def crater_profile(distance, radius, depth, rim_height):
    """A smooth bowl, raised rim and tapered ejecta, in meters (synthetic)."""
    t = np.asarray(distance) / radius
    bowl = -depth * np.maximum(1-t*t, 0)**2
    rim = rim_height * np.exp(-0.5*((t-1)/0.13)**2)
    taper = np.clip((1.65-t)/0.3, 0, 1)
    taper = taper*taper*(3-2*taper)
    return (bowl+rim)*taper


def add_craters(data, r0, c0, source_shape, spawn_x, spawn_y, count, seed):
    """Modify a DEM copy, keeping every crater outside the spawn exclusion zone."""
    if count == 0:
        return []
    rng = np.random.default_rng(seed)
    xmin = (c0-source_shape[1]//2)*5
    xmax = xmin+(data.shape[1]-1)*5
    ymax = (source_shape[0]//2-r0)*5
    ymin = ymax-(data.shape[0]-1)*5
    if min(xmax-xmin, ymax-ymin) < 100:
        raise ValueError('Craters require a map at least 100 m wide; use samples>=33 or crater_count:=0')
    result = []
    local_count = max(1, count//3)
    for _ in range(count*200):
        if len(result) == count:
            return result
        # >= 40 m diameter, resolved by at least eight intervals in the 5 m DEM.
        radius = float(np.exp(rng.uniform(np.log(20), np.log(80))))
        margin = radius*1.65
        if len(result) < local_count:
            angle = rng.uniform(0, 2*np.pi)
            distance = rng.uniform(margin+15, max(margin+16, 240))
            cx, cy = spawn_x+distance*np.cos(angle), spawn_y+distance*np.sin(angle)
        else:
            if xmin+margin >= xmax-margin or ymin+margin >= ymax-margin:
                continue
            cx, cy = rng.uniform(xmin+margin,xmax-margin), rng.uniform(ymin+margin,ymax-margin)
        if not (xmin+margin < cx < xmax-margin and ymin+margin < cy < ymax-margin):
            continue
        # Includes the ejecta apron, not just the bowl. Preserve the original start area.
        if math.hypot(cx-spawn_x,cy-spawn_y) < margin+12:
            continue
        depth = radius*float(rng.uniform(.12,.22))
        rim = radius*float(rng.uniform(.025,.055))
        i0 = max(0,math.floor((ymax-cy-margin)/5))
        i1 = min(data.shape[0],math.ceil((ymax-cy+margin)/5)+1)
        j0 = max(0,math.floor((cx-margin-xmin)/5))
        j1 = min(data.shape[1],math.ceil((cx+margin-xmin)/5)+1)
        xx = xmin+np.arange(j0,j1)*5-cx
        yy = ymax-np.arange(i0,i1)*5-cy
        distance = np.hypot(yy[:,None],xx[None,:])
        data[i0:i1,j0:j1] += crater_profile(distance,radius,depth,rim)
        result.append(dict(x=float(cx),y=float(cy),radius_m=radius,depth_m=depth,rim_height_m=rim))
    raise ValueError('Could not place all craters; reduce crater_count or use a larger map')


def prepare(dem_path, package_path, output_path, samples=0, x=0.0, y=0.0, yaw=0.0,
            crater_count=0, crater_seed=20, sun_elevation=5.0, sun_azimuth=135.0,
            sun_intensity=10.0, texture_size=0.375, rendering_resources=None):
    dem_path, package_path, output_path = map(Path, (dem_path, package_path, output_path))
    dem_path = dem_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("Spawn coordinates must be finite")
    if not isinstance(crater_count, int) or not 0 <= crater_count <= 2000:
        raise ValueError('crater_count must be an integer between 0 and 2000')
    if not isinstance(crater_seed, int) or crater_seed < -1:
        raise ValueError('crater_seed must be nonnegative, or -1 for a new random seed')
    if not all(math.isfinite(v) for v in (sun_elevation,sun_azimuth,sun_intensity,texture_size)):
        raise ValueError('Sun and texture settings must be finite')
    if not 0 < sun_elevation <= 90 or not 0 < sun_intensity <= 100 or texture_size <= 0:
        raise ValueError('Sun elevation must be in (0,90], intensity in (0,100], and texture_size positive')
    seed = secrets.randbits(32) if crater_seed == -1 else crater_seed
    if not dem_path.is_file():
        raise FileNotFoundError(f"Site20 DEM not found: {dem_path}; supply dem:=/path/to/dem.npy")
    with dem_path.with_suffix('.yaml').open() as stream:
        info = yaml.safe_load(stream)
    if info['pixel_size'] != [5.0, -5.0]:
        raise ValueError("Expected a north-up DEM with pixel_size: [5.0, -5.0]")
    source = np.load(dem_path, mmap_mode='r', allow_pickle=False)
    if source.ndim != 2 or list(source.shape[::-1]) != info['size']:
        raise ValueError("DEM dimensions do not match dem.yaml")
    rows, cols = source.shape
    if samples:
        if samples < 3 or (samples - 1) & (samples - 2) or samples > min(rows, cols):
            raise ValueError("samples must be 0 (whole map) or 2^n+1 fitting inside the DEM")
        r0, c0 = rows // 2 - samples // 2, cols // 2 - samples // 2
        data = np.array(source[r0:r0+samples, c0:c0+samples], dtype=np.float64, copy=True)
        top = left = 0
        side = samples
    else:
        r0 = c0 = 0
        data = np.array(source, dtype=np.float64, copy=True)
        side = 2 ** math.ceil(math.log2(max(rows, cols) - 1)) + 1
        top, left = side // 2 - rows // 2, side // 2 - cols // 2
    if not np.isfinite(data).all() or np.any(data <= -9999):
        raise ValueError("DEM contains missing elevations; repair them before generating terrain")
    craters = add_craters(data,r0,c0,source.shape,x,y,crater_count,seed)
    low, high = float(data.min()), float(data.max())
    height_range = max(high - low, 1.0)
    # Quantize after deformation so rendering, collision and spawn height agree.
    pixels = np.rint((data - low) * (65535 / height_range)).astype(np.uint16)
    if samples == 0:
        pixels = np.pad(pixels, ((top, side-rows-top), (left, side-cols-left)), mode='edge')
    origin_elevation = float(source[rows//2, cols//2])
    terrain_z = low - origin_elevation

    def height_at(px, py):
        # Image row zero is north (+Y); column zero is west (-X).
        col, row = side//2 + px/5, side//2 - py/5
        sr, sc = row-top+r0, col-left+c0
        if not (0 <= sr < rows-1 and 0 <= sc < cols-1 and 0 <= row < side-1 and 0 <= col < side-1):
            raise ValueError("Spawn footprint lies outside the real DEM (padding is not lunar data)")
        i, j = int(row), int(col)
        u, v = col-j, row-i
        value = ((1-v)*((1-u)*float(pixels[i,j])+u*float(pixels[i,j+1]))
                 + v*((1-u)*float(pixels[i+1,j])+u*float(pixels[i+1,j+1])))
        return terrain_z + value * height_range/65535

    # Allow for slope across the whole rover and the heightfield's triangular interpolation.
    z = max(height_at(x+dx, y+dy) for dx in (-0.3, 0, 0.3) for dy in (-0.3, 0, 0.3)) + 0.12
    output_path.mkdir(parents=True, exist_ok=True)
    heightmap = output_path/'site20_height.png'
    Image.fromarray(pixels).save(heightmap)
    textures = package_path.resolve()/'materials/textures'
    for name in ('polar_lht_diffuse.png','polar_lht_normal.png','polar_lht.json'):
        if not (textures/name).is_file():
            raise FileNotFoundError(f'Missing packaged lunar texture: {textures/name}')
    elevation, azimuth = math.radians(sun_elevation), math.radians(sun_azimuth)
    sun_direction = (-math.cos(elevation)*math.cos(azimuth),
                     -math.cos(elevation)*math.sin(azimuth),-math.sin(elevation))
    world = Template((package_path/'worlds/site20.sdf.in').read_text()).substitute(
        world_name='site20', world_title='Site20', terrain_name='site20_5m_terrain', terrain_x=0, terrain_y=0,
        terrain_z=terrain_z, heightmap_uri=escape(heightmap.as_uri()),
        extent=(side-1)*5, collision_extent=side*5, height_range=height_range,
        texture_uri=escape((textures/'polar_lht_diffuse.png').as_uri()),
        normal_uri=escape((textures/'polar_lht_normal.png').as_uri()),texture_size=texture_size,
        sun_direction=' '.join(str(v) for v in sun_direction),sun_intensity=sun_intensity,
        camera_pose=f'{x+1.2} {y-1.2} {z+0.9} 0 0.48 2.356194490192345')
    (output_path/'site20.sdf').write_text(world)
    urdf = xacro.process_file(str(package_path/'urdf/lumos1_rockerless_URDF.xacro'),
                              mappings={'mesh_directory': (package_path.resolve()/'meshes').as_uri()}).toxml()
    (output_path/'lumos.urdf').write_text(urdf)
    (output_path/'craters.json').write_text(json.dumps(craters,indent=2)+'\n')
    metadata = dict(world='site20', model='lumos', x=x, y=y, z=z, yaw=yaw,
                    dem=str(dem_path), pixel_size_m=5.0, source_shape=list(source.shape),
                    heightmap_samples=side, source_window=[r0,c0,*data.shape],
                    padding_top_left=[top,left], elevation_origin_m=origin_elevation,
                    elevation_quantization_m=height_range/65535,
                    crater_count=len(craters),crater_seed=seed,craters_file=str(output_path/'craters.json'),
                    sun_elevation_deg=sun_elevation,sun_azimuth_deg=sun_azimuth,
                    sun_intensity=sun_intensity,texture_size_m=texture_size,
                    surface_material=json.loads((textures/'polar_lht.json').read_text()),
                    urdf=str(output_path/'lumos.urdf'))
    if rendering_resources is not None:
        metadata['rendering_resources'] = str(matte_terrain_resources(rendering_resources,output_path/'rendering'))
    (output_path/'spawn.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dem', required=True, type=Path)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--samples', type=int, default=0)
    parser.add_argument('--crater-count',type=int,default=64)
    parser.add_argument('--crater-seed',type=int,default=20)
    parser.add_argument('--sun-elevation',type=float,default=5.0)
    parser.add_argument('--sun-azimuth',type=float,default=135.0)
    parser.add_argument('--sun-intensity',type=float,default=10.0)
    parser.add_argument('--texture-size',type=float,default=0.375)
    parser.add_argument('--rendering-resources',type=Path)
    for name in ('x', 'y', 'yaw'):
        parser.add_argument('--'+name, type=float, default=0.0)
    args = parser.parse_args()
    try:
        result = prepare(args.dem,args.package,args.output,args.samples,args.x,args.y,args.yaw,
                         args.crater_count,args.crater_seed,args.sun_elevation,args.sun_azimuth,
                         args.sun_intensity,args.texture_size,args.rendering_resources)
    except (ValueError, FileNotFoundError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
