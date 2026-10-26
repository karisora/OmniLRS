#!/usr/bin/env python3
"""Build a neutral highlands material from NASA POLAR Terrain13 NU-LHT data.

The source is a laboratory analog photograph, not an orbital albedo map. Its
fine contrast is retained after removing broad illumination; normal detail is
an artistic shading proxy, not a recovered terrain measurement.
"""

import argparse
import hashlib
from io import BytesIO
import json
from pathlib import Path
import struct
import urllib.request
from zipfile import ZipFile
import zlib

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
MATERIALS = ROOT/'assets/USD_Assets/lumos1_rockerless_URDF_description/lumos1_rockerless_URDF_description/materials'
DATASET = 'https://ti.arc.nasa.gov/dataset/IRG_PolarDB/'
ARCHIVE = DATASET+'PolarDB_download/Terrain13_LHT.zip'
MEMBER = 'Terrain13_LHT/PosA_1500_Loff/Sun_30/CamL_287297164_0128.png'
SOURCE = MATERIALS/'sources/polar_lht.png'


def download_source(destination):
    """Fetch one image with HTTP ranges rather than the full 207 MB archive."""
    def request(byte_range):
        req = urllib.request.Request(ARCHIVE, headers={'Range': 'bytes='+byte_range})
        with urllib.request.urlopen(req, timeout=60) as response:
            if response.status != 206:
                raise RuntimeError('NASA server must support partial ZIP downloads (HTTP 206)')
            return response.read(), response.headers['Content-Range']

    tail, content_range = request('-65536')
    total_size = int(content_range.rsplit('/', 1)[1])
    # ZipFile can read this central directory; offsets are relative to the tail.
    entry = ZipFile(BytesIO(tail)).getinfo(MEMBER)
    offset = entry.header_offset + total_size - len(tail)
    data, _ = request(f'{offset}-{offset+entry.compress_size+1023}')
    if data[:4] != b'PK\x03\x04' or entry.compress_type != 8:
        raise RuntimeError('Unexpected NASA ZIP member format')
    name_size, extra_size = struct.unpack_from('<HH', data, 26)
    start = 30 + name_size + extra_size
    raw = zlib.decompress(data[start:start+entry.compress_size], -15)
    if len(raw) != entry.file_size or zlib.crc32(raw) != entry.CRC:
        raise RuntimeError('NASA image failed its ZIP size / CRC check')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(raw)


def blur_periodic(values, sigma):
    fy = np.fft.fftfreq(values.shape[0])[:, None]
    fx = np.fft.fftfreq(values.shape[1])[None, :]
    kernel = np.exp(-2*np.pi**2*sigma**2*(fx*fx+fy*fy))
    return np.fft.ifft2(np.fft.fft2(values)*kernel).real


def periodic_component(values):
    """Remove the smooth boundary discontinuity using a periodic Poisson solve."""
    boundary = np.zeros_like(values)
    boundary[0, :] = values[-1, :]-values[0, :]
    boundary[-1, :] = -boundary[0, :]
    boundary[:, 0] += values[:, -1]-values[:, 0]
    boundary[:, -1] -= values[:, -1]-values[:, 0]
    fy = np.fft.fftfreq(values.shape[0])[:, None]
    fx = np.fft.fftfreq(values.shape[1])[None, :]
    denominator = 2*np.cos(2*np.pi*fx)+2*np.cos(2*np.pi*fy)-4
    denominator[0, 0] = 1
    smooth = np.fft.fft2(boundary)/denominator
    smooth[0, 0] = 0
    return values-np.fft.ifft2(smooth).real


def build_material(source, output):
    raw = np.asarray(Image.open(source))
    if raw.shape != (1216, 1936) or raw.max() > 4095:
        raise ValueError('Expected the original 1936 x 1216, 12-bit POLAR Terrain13 image')
    # Approximate perspective rectification of the 1 m square testbed. Select
    # the upper central/right soil patch, excluding all three rocks and edges.
    corners = (819, 432, 785, 818, 1560, 818, 1433, 432)  # TL, BL, BR, TR
    rectified = Image.fromarray(raw.astype(np.float32)).transform(
        (1024, 1024), Image.Transform.QUAD, corners, Image.Resampling.BICUBIC)
    crop = (480, 64, 864, 448)
    patch = np.asarray(rectified.crop(crop)).astype(np.float64)
    if patch.min() <= 0 or np.any(patch >= 4090):
        raise ValueError('Selected soil contains invalid or saturated samples')
    # Work with log intensity so broad multiplicative lighting divides out.
    detail = np.log(patch)
    detail -= blur_periodic(detail, 14)
    detail = periodic_component(detail)
    detail = blur_periodic(detail, .6)
    detail = np.clip(detail/(detail.std()+1e-12), -2.5, 2.5)
    detail -= detail.mean()
    # 0.30 is the highlands reference albedo described in the POLAR README.
    # It is a reference material setting, not a calibrated estimate of this photo.
    albedo = np.clip(.30*(1+.065*detail), .22, .38)
    srgb = np.where(albedo <= .0031308, 12.92*albedo, 1.055*albedo**(1/2.4)-.055)
    diffuse = np.repeat(np.rint(srgb*255).astype(np.uint8)[:, :, None], 3, axis=2)
    # Keep RMS slopes small: a strong normal map incorrectly blackens powder
    # at the 5 degree solar elevation. No laboratory cast shadows are baked in.
    smooth_detail = blur_periodic(detail, 1.2)
    dx = (np.roll(smooth_detail, -1, 1)-np.roll(smooth_detail, 1, 1))*.5
    dy = (np.roll(smooth_detail, -1, 0)-np.roll(smooth_detail, 1, 0))*.5
    scale = .025/np.sqrt(np.mean(dx*dx+dy*dy))
    normal = np.stack((-dx*scale, dy*scale, np.ones_like(dx)), axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    output.mkdir(parents=True, exist_ok=True)
    Image.fromarray(diffuse).resize((2048, 2048), Image.Resampling.BICUBIC).save(output/'polar_lht_diffuse.png')
    # Resize float vectors before quantizing, retaining unit normals.
    enlarged = np.stack([np.asarray(Image.fromarray(normal[:, :, i].astype(np.float32)).resize(
        (2048, 2048), Image.Resampling.BICUBIC)) for i in range(3)], axis=-1)
    enlarged /= np.linalg.norm(enlarged, axis=-1, keepdims=True)
    Image.fromarray(np.rint((enlarged*.5+.5)*255).astype(np.uint8)).save(output/'polar_lht_normal.png')
    metadata = dict(
        dataset=DATASET, documentation=DATASET+'PolarDB_Readme.pdf', archive=ARCHIVE, member=MEMBER,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_kind='NASA Ames laboratory NU-LHT highlands regolith analog; not lunar orbital imagery',
        source_size=[1936, 1216], source_bits=12, rectification_quad=list(corners),
        rectified_size=[1024, 1024], crop=list(crop), output_size=[2048, 2048],
        tile_size_m=.375, tile_scale='Approximate from the 1 m testbed; not a surveyed surface scale',
        reference_linear_albedo=.30, albedo_detail_fraction=.065, normal_rms_slope=.025,
        processing='Perspective rectification, log illumination removal, periodic boundary correction, neutral sRGB albedo',
        normal_note='Low amplitude image-derived shading proxy; not measured height or recovered normals',
        citation='Wong et al., Polar Optical Lunar Analog Reconstruction (POLAR) Stereo Dataset, NASA Ames, May 2017')
    (output/'polar_lht.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--output', type=Path, default=MATERIALS/'textures')
    parser.add_argument('--download', action='store_true', help='Fetch the original image from NASA first')
    args = parser.parse_args()
    if args.download:
        download_source(args.source)
    result = build_material(args.source, args.output)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
