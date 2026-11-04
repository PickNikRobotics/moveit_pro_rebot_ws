# Converted meshes

MuJoCo cannot load COLLADA, so these STL files are conversions of
the original `rebot_bench_base_config` COLLADA visuals, now in
`picknik_boston_rebot_b601_rs_hw`, made with trimesh (pycollada):
each file's scene graph is flattened into the DAE's own frame, in metres, and
split into one binary STL per embedded material. Geometry is otherwise
unchanged; the base package's files stay the source.

| Files | Source in `picknik_boston_rebot_b601_rs_hw/description/assets/` | Licence |
| --- | --- | --- |
| `printed/<part>_purple.stl`, `printed/<part>_white.stl` | `printed/<part>.dae` | CERN-OHL-W-2.0, exact text in `LICENSE`; editable sources and source/change notice in the Boston hardware description |
| `realsense/d435.stl` | `realsense/d435.dae` | Apache-2.0, exact text and notices in `realsense/LICENSE` and `realsense/UPSTREAM_NOTICE.txt` |

The conversion source is pinned to example workspace
`6bf6c0ed37d580141b2ef72bd67bc351e0b89c37`, package `src/rebot_bench_sim`.
All converted mesh bytes are unchanged; package URIs were relocated. See
[model provenance](../source-provenance.md) and
[editable source attribution](../../../picknik_boston_rebot_b601_rs_hw/description/NOTICE.md).

`_purple` takes the embedded colour sRGB (91, 44, 143), `_white` sRGB
(242, 242, 242); `rebot_assets.xml` applies them as MuJoCo materials.
