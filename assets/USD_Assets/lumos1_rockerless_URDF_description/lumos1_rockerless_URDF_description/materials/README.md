# NASA POLAR highlands material

The default Site20 ground material uses the light, neutral **NU-LHT highlands
regolith simulant** from [NASA Ames POLAR Terrain13](https://ti.arc.nasa.gov/dataset/IRG_PolarDB/).
This is a laboratory analog photograph, not a photograph taken on the Moon or a
georeferenced Site20 albedo map. Site20's measured terrain heights still come
from [NASA PGDA / LOLA product 78](https://pgda.gsfc.nasa.gov/products/78), which
provides elevation and uncertainty products rather than a surface color texture.

The original image is included as `sources/polar_lht.png` so regeneration works
offline. It is the unchanged, 1936 × 1216, 12-bit image from this archive member:

```text
https://ti.arc.nasa.gov/dataset/IRG_PolarDB/PolarDB_download/Terrain13_LHT.zip
Terrain13_LHT/PosA_1500_Loff/Sun_30/CamL_287297164_0128.png
SHA-256: b0bd3b31da65d91e20a9ecf9f7c1663af3c67782d293b47d1a1eec3663ced80f
```

`textures/polar_lht.json` records the source, checksum, crop, approximate
perspective correction, material settings and processing. The source has no
saturated samples in the selected soil patch; the rocks, sandbox rim and their
cast shadows are excluded. Broad illumination is divided out in log intensity,
and a periodic boundary correction reduces visible tiling seams. Fine contrast
is preserved at low amplitude rather than baking the laboratory lighting into
the ground. No image generation is used.

The 2048 × 2048 RGB diffuse map is neutral gray, with a mean linear reference
albedo of 0.30, matching the highlands reference discussed in the
[POLAR documentation, p. 5](https://ti.arc.nasa.gov/dataset/IRG_PolarDB/PolarDB_Readme.pdf).
This is a reference material value, not a photometric measurement recovered from
the source image. The image is resampled from a smaller crop; 2048 pixels do not
represent new measured detail. A 0.375 m tile size is estimated from the 1 m
testbed and approximate rectification, and is not a surveyed scale.

The OpenGL normal map supplies only subtle visual detail, with RMS slope 0.025
(about 1.4°). It is inferred from image contrast, not measured surface normals or
height. These details do not alter the DEM or wheel collision geometry. Large
terrain and crater shadows are produced by Gazebo lighting. The solar elevation
remains 5°; the default `sun_intensity=10` is a display / photographic brightness
setting. This material is an approximation using Gazebo's standard terrain
shading, not a calibrated lunar Hapke reflectance model.

From the repository root, rebuild the packaged textures:

```bash
python3 scripts/prepare_lunar_material.py
# Re-fetch only this image using HTTP byte ranges, then rebuild:
python3 scripts/prepare_lunar_material.py --download
```

Dependencies: Python 3, NumPy and Pillow. Runtime simulation has no network
dependency. The older `regolith_diffuse.png` and `regolith_normal.png` remain as
legacy copies derived from OmniLRS's `assets/Textures/LunarRegolith8k/`; the
default world now uses `polar_lht_diffuse.png` and `polar_lht_normal.png`.

Attribution: Uland Wong, Ara Nefian, Larry Edwards, Xavier Buoyssounouse,
P. Michael Furlong, Matt Deans, and Terry Fong. *Polar Optical Lunar Analog
Reconstruction (POLAR) Stereo Dataset*. NASA Ames Research Center, May 2017.
The dataset and source documentation remain the authoritative attribution and
usage references.
