#!/usr/bin/env python3
"""Generate a metric Gazebo heightmap without resampling Site20's 5 m grid."""

import argparse
import json
import math
from pathlib import Path
from string import Template
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image
import xacro
import yaml


def prepare(dem_path, package_path, output_path, samples=0, x=0.0, y=0.0, yaw=0.0):
    dem_path, package_path, output_path = map(Path, (dem_path, package_path, output_path))
    dem_path = dem_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("Spawn coordinates must be finite")
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
        data = np.asarray(source[r0:r0+samples, c0:c0+samples], dtype=np.float64)
        top = left = 0
        side = samples
    else:
        r0 = c0 = 0
        data = np.asarray(source, dtype=np.float64)
        side = 2 ** math.ceil(math.log2(max(rows, cols) - 1)) + 1
        top, left = side // 2 - rows // 2, side // 2 - cols // 2
    if not np.isfinite(data).all() or np.any(data <= -9999):
        raise ValueError("DEM contains missing elevations; repair them before generating terrain")
    low, high = float(data.min()), float(data.max())
    height_range = max(high - low, 1.0)
    # 16-bit elevations: <= 2.8 cm error over the full Site20 height range.
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
    Image.new('RGB', (8, 8), (110, 108, 104)).save(output_path/'regolith.png')
    Image.new('RGB', (8, 8), (128, 128, 255)).save(output_path/'regolith_normal.png')
    world = Template((package_path/'worlds/site20.sdf.in').read_text()).substitute(
        terrain_z=terrain_z, heightmap_uri=escape(heightmap.as_uri()),
        extent=(side-1)*5, collision_extent=side*5, height_range=height_range,
        texture_uri=escape((output_path/'regolith.png').as_uri()),
        normal_uri=escape((output_path/'regolith_normal.png').as_uri()),
        camera_pose=f'{x+1.2} {y-1.2} {z+0.9} 0 0.48 2.356194490192345')
    (output_path/'site20.sdf').write_text(world)
    urdf = xacro.process_file(str(package_path/'urdf/lumos1_rockerless_URDF.xacro'),
                              mappings={'mesh_directory': (package_path.resolve()/'meshes').as_uri()}).toxml()
    (output_path/'lumos.urdf').write_text(urdf)
    metadata = dict(world='site20', model='lumos', x=x, y=y, z=z, yaw=yaw,
                    dem=str(dem_path), pixel_size_m=5.0, source_shape=list(source.shape),
                    heightmap_samples=side, source_window=[r0,c0,*data.shape],
                    padding_top_left=[top,left], elevation_origin_m=origin_elevation,
                    elevation_quantization_m=height_range/65535,
                    urdf=str(output_path/'lumos.urdf'))
    (output_path/'spawn.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dem', required=True, type=Path)
    parser.add_argument('--package', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--samples', type=int, default=0)
    for name in ('x', 'y', 'yaw'):
        parser.add_argument('--'+name, type=float, default=0.0)
    args = parser.parse_args()
    try:
        result = prepare(args.dem, args.package, args.output, args.samples, args.x, args.y, args.yaw)
    except (ValueError, FileNotFoundError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
