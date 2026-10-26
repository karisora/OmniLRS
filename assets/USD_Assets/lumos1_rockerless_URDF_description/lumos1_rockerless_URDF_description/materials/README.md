The 2048 × 2048 textures in `textures/` are derived from the existing OmniLRS assets:

- `assets/Textures/LunarRegolith8k/LunarRegolith8k_diff.png`
- `assets/Textures/LunarRegolith8k/LunarRegolith8k_nor_gl.png`

The RGB diffuse texture is resized with Lanczos filtering. The OpenGL normal map
is resized and its vectors are normalized again. Color values are preserved;
the conversion adds no illumination or cast shadows. Original asset
attribution and terms remain applicable. The reduced copies allow the ROS package
to be installed without loading the original 8K textures (over 650 MB combined).
