# Selected Code Samples — Spatial Polyurethane Foam 3D Printing

**Project:** Beyond the Layer — Spatial 3D Printing, ETH Zürich

## Purpose
These code samples illustrate how geometric and fabrication constraints were encoded in a computational workflow for spatial robotic printing with reactive polyurethane foam. They are selected, not independently runnable scripts or a complete robotic control package.

## Sample 01 — 01_spatial_frame_orientation_excerpt.py

- `align_frames_z_to_segments`: align robot tool direction to path segments, with special treatment near the printbed.
- `orient_frames_down`: enforce downward pointing tool orientations with a minimum inclination to the XY plane.
- `interpolate_frames`: insert targets where frame rotation or point distance exceeds configured thresholds; uses quaternion interpolation.


## Sample 02 — 02_velocity_assignment_excerpt.py

- `assign_velocity_by_movement`: distinguish upward, downward, and near-horizontal moves; map inclination to print velocities.
- `adjust_velocity_by_cantilever`: reduce velocity where a point lacks a sufficiently nearby support point in an earlier layer.
- `_support_distance_xy`: approximate support using pointwise XY proximity one nominal layer lower.


## Integration and dependencies
The excerpts are member methods of classes composed into `PrintPath` in the full workflow. They require objects with `layers`, `printpoints`, `point`, `frame`, `point_type`, and other attributes. The original project relies on **COMPAS** and **Rhino / Grasshopper** for portions of the workflow. `PrintPath`, `PrintLayer`, and `PrintPoint` manage trajectory data; other modules handle metrics and export.

**These files are intended for source-code review, not for direct execution.** 