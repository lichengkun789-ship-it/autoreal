---
name: object-localization
description: Localize visible objects in a calibrated Piper RGB-D frame without moving either arm.
---

# Piper RGB-D object localization

Call `piper_observe` and choose a camera where the object is clearly visible. The
returned preview may be downscaled: convert preview pixels to full-resolution pixels
using each camera's reported width/height and preview_width/preview_height.

Call `piper_pixel_to_position` with the fresh `frame_id`, calibrated camera name, and
one or more full-resolution pixels. Use a small radius so the tool can take the median
of nearby valid depth values. Inspect `valid_fraction`; try nearby pixels or another
camera when depth is missing or the point lies on an object edge.

Read the returned `frame`; it is explicitly `piper_left_base` or `piper_right_base`.
`position_m` is expressed in that frame. It is geometric evidence, not permission to
move. Re-observe after the scene or either arm moves, and never reuse an eye-in-hand
frame after its mounted arm changes pose.
