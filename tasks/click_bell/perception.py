"""Deterministic yellow press-cap localization from a saved Piper RGB-D frame."""

from __future__ import annotations

from collections import deque
import json
from pathlib import Path

import numpy as np
from PIL import Image

from services.perception.geometry import deproject_pixel, transform_point


def _components(
    mask: np.ndarray, *, min_area: int, include_pixels: bool = False
) -> list[dict]:
    height, width = mask.shape
    remaining = {tuple(item) for item in np.argwhere(mask)}
    components = []
    while remaining:
        start = remaining.pop()
        queue = deque([start])
        area = sum_x = sum_y = 0
        pixels = []
        min_x = max_x = int(start[1])
        min_y = max_y = int(start[0])
        while queue:
            raw_y, raw_x = queue.popleft()
            y, x = int(raw_y), int(raw_x)
            area += 1
            sum_x += x
            sum_y += y
            if include_pixels:
                pixels.append((y, x))
            min_x, max_x = min(min_x, x), max(max_x, x)
            min_y, max_y = min(min_y, y), max(max_y, y)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == dy == 0:
                        continue
                    neighbor = (y + dy, x + dx)
                    if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width:
                        if neighbor in remaining:
                            remaining.remove(neighbor)
                            queue.append(neighbor)
        if area >= min_area:
            component = {
                "area_px": area,
                "bbox_xyxy": [min_x, min_y, max_x + 1, max_y + 1],
                "centroid_px": [sum_x / area, sum_y / area],
            }
            if include_pixels:
                component["_pixels_yx"] = pixels
            components.append(component)
    return sorted(components, key=lambda item: item["area_px"], reverse=True)


def _detect_press_cap(
    image: Image.Image | str | Path, *, include_pixels: bool = False
) -> tuple[dict, np.ndarray | None]:
    if not isinstance(image, Image.Image):
        image = Image.open(image)
    rgb = image.convert("RGB")
    hsv = np.asarray(rgb.convert("HSV"), dtype=np.uint8)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    # PIL hue is 0..255. The narrower blue range excludes the green backdrop.
    blue_mask = (
        (hue >= 115)
        & (hue <= 150)
        & (saturation >= 45)
        & (value >= 50)
    )
    yellow_mask = (
        (hue >= 25)
        & (hue <= 55)
        & (saturation >= 45)
        & (value >= 80)
    )
    blue_components = _components(blue_mask, min_area=80)
    yellow_components = _components(
        yellow_mask, min_area=20, include_pixels=include_pixels
    )
    if not blue_components:
        raise ValueError("no blue bell component detected")

    bell = blue_components[0]
    bx0, by0, bx1, by1 = bell["bbox_xyxy"]
    bell_width = max(1, bx1 - bx0)
    bell_height = max(1, by1 - by0)
    bell_x, bell_y = bell["centroid_px"]
    candidates = []
    for component in yellow_components:
        x, y = component["centroid_px"]
        if not bx0 - 0.15 * bell_width <= x <= bx1 + 0.15 * bell_width:
            continue
        if not by0 - 0.35 * bell_height <= y < bell_y:
            continue
        x_offset = abs(x - bell_x) / bell_width
        top_offset = abs(y - by0) / bell_height
        component = {**component, "score": x_offset + 0.5 * top_offset}
        candidates.append(component)
    if not candidates:
        raise ValueError("blue bell found, but no yellow press cap was found above it")

    cap = min(candidates, key=lambda item: item["score"])
    pixels_yx = None
    if include_pixels:
        pixels_yx = np.asarray(cap["_pixels_yx"], dtype=np.int64)
    public_cap = {key: value for key, value in cap.items() if not key.startswith("_")}
    cap_x, cap_y = cap["centroid_px"]
    confidence = (
        "high"
        if cap["area_px"] >= 80
        and abs(cap_x - bell_x) <= 0.2 * bell_width
        and cap_y < bell_y
        else "medium"
    )
    result = {
        "image_size": [rgb.width, rgb.height],
        "press_pixel": [int(round(cap_x)), int(round(cap_y))],
        "confidence": confidence,
        "bell": bell,
        "press_cap": public_cap,
        "yellow_components_considered": len(yellow_components),
    }
    return result, pixels_yx


def detect_press_cap(image: Image.Image | str | Path) -> dict:
    """Find the yellow cap above the largest cyan-blue bell component."""
    result, _ = _detect_press_cap(image)
    return result


def _interior_pixels(pixels_yx: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Remove one boundary pixel so bell sides and depth edges do not dominate."""
    mask = np.zeros(shape, dtype=bool)
    mask[pixels_yx[:, 0], pixels_yx[:, 1]] = True
    padded = np.pad(mask, 1, constant_values=False)
    interior = mask.copy()
    for dy in range(3):
        for dx in range(3):
            interior &= padded[dy : dy + shape[0], dx : dx + shape[1]]
    selected = np.argwhere(interior)
    minimum = max(12, int(round(0.25 * len(pixels_yx))))
    return selected if len(selected) >= minimum else pixels_yx


def _fit_press_surface(
    depth_mm: np.ndarray,
    pixels_yx: np.ndarray,
    intrinsics: dict,
    T_base_camera: np.ndarray,
) -> dict:
    selected = _interior_pixels(pixels_yx, depth_mm.shape)
    samples = []
    for y, x in selected:
        raw_depth = int(depth_mm[int(y), int(x)])
        if raw_depth <= 0:
            continue
        samples.append(
            deproject_pixel(float(x), float(y), raw_depth / 1000.0, intrinsics)
        )
    if len(samples) < 12:
        raise ValueError(
            "press-cap surface has fewer than 12 valid interior depth samples"
        )

    camera_points = np.asarray(samples, dtype=np.float64)
    depth_center = float(np.median(camera_points[:, 2]))
    depth_mad = float(np.median(np.abs(camera_points[:, 2] - depth_center)))
    coarse_threshold_m = max(0.015, 4.0 * 1.4826 * depth_mad)
    coarse_inliers = np.abs(camera_points[:, 2] - depth_center) <= coarse_threshold_m
    if int(coarse_inliers.sum()) < 12:
        raise ValueError(
            "press-cap surface has fewer than 12 depth-consistent samples"
        )
    plane_points = camera_points[coarse_inliers]

    def fit_plane(points):
        center = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - center, full_matrices=False)
        normal = vh[-1]
        return center, normal / np.linalg.norm(normal)

    center_camera, normal_camera = fit_plane(plane_points)
    residuals = np.abs((plane_points - center_camera) @ normal_camera)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    threshold_m = max(0.0025, median + 3.0 * 1.4826 * mad)
    inliers = residuals <= threshold_m
    if int(inliers.sum()) < 12:
        raise ValueError("press-cap surface plane has fewer than 12 inlier depth samples")

    inlier_points = plane_points[inliers]
    center_camera, normal_camera = fit_plane(inlier_points)
    residuals = np.abs((inlier_points - center_camera) @ normal_camera)
    if float(normal_camera @ -center_camera) < 0:
        normal_camera = -normal_camera

    matrix = np.asarray(T_base_camera, dtype=np.float64)
    center_base = transform_point(matrix, center_camera)
    normal_base = matrix[:3, :3] @ normal_camera
    rmse_mm = float(np.sqrt(np.mean(np.square(residuals))) * 1000.0)
    depth_mad_mm = float(
        np.median(
            np.abs(inlier_points[:, 2] - np.median(inlier_points[:, 2]))
        )
        * 1000.0
    )
    fit_95_mm = 1.96 * max(rmse_mm, 1.4826 * depth_mad_mm)
    return {
        "status": "passed",
        "method": "eroded_yellow_component_robust_plane",
        "position_m": np.round(center_base, 5).tolist(),
        "normal_base": np.round(normal_base, 6).tolist(),
        "normal_convention": "points from the surface toward the observing camera",
        "component_pixels": int(len(pixels_yx)),
        "interior_pixels": int(len(selected)),
        "valid_depth_samples": int(len(camera_points)),
        "inlier_depth_samples": int(len(inlier_points)),
        "valid_depth_fraction": round(float(len(camera_points) / len(selected)), 3),
        "plane_rmse_mm": round(rmse_mm, 3),
        "depth_mad_mm": round(depth_mad_mm, 3),
        "depth_fit_spread_95_mm": round(fit_95_mm, 3),
        "uncertainty_scope": "approximate_depth_and_plane_fit_spread_only; excludes_camera_extrinsic_error",
    }


def localize_saved_frame(
    frame_directory: str | Path,
    camera: str,
    *,
    radius: int = 3,
) -> dict:
    """Detect and back-project the press cap using an existing frame's evidence."""
    if not 0 <= radius <= 15:
        raise ValueError("radius must be in [0, 15]")
    directory = Path(frame_directory)
    metadata = json.loads((directory / "meta.json").read_text())
    entry = metadata.get("cameras", {}).get(camera)
    if entry is None:
        raise ValueError(f"frame has no camera {camera!r}")
    if not entry.get("rgb") or not entry.get("depth"):
        raise ValueError(f"camera {camera!r} needs saved RGB and aligned depth")
    if "T_base_camera" not in entry or "base_frame" not in entry:
        raise ValueError(f"camera {camera!r} has no base-frame calibration")

    detection, pixels_yx = _detect_press_cap(
        directory / entry["rgb"], include_pixels=True
    )
    assert pixels_yx is not None
    u, v = detection["press_pixel"]
    depth = np.asarray(Image.open(directory / entry["depth"]), dtype=np.uint16)
    window = depth[
        max(0, v - radius) : min(depth.shape[0], v + radius + 1),
        max(0, u - radius) : min(depth.shape[1], u + radius + 1),
    ]
    valid = window[window > 0]
    if valid.size == 0:
        raise ValueError("press-cap pixel has no valid aligned depth")
    depth_m = float(np.median(valid)) / 1000.0
    point_camera = deproject_pixel(u, v, depth_m, entry["intrinsics"])
    point_base = transform_point(entry["T_base_camera"], point_camera)
    surface = _fit_press_surface(
        depth,
        pixels_yx,
        entry["intrinsics"],
        np.asarray(entry["T_base_camera"], dtype=np.float64),
    )
    return {
        "schema_version": 1,
        "kind": "click_bell_press_cap_localization",
        "status": "passed",
        "frame_id": metadata.get("frame_id"),
        "camera": camera,
        "frame": entry["base_frame"],
        **detection,
        "depth_m": round(depth_m, 5),
        "valid_fraction": round(float(valid.size / window.size), 3),
        "position_m": surface["position_m"],
        "legacy_center_position_m": np.round(point_base, 5).tolist(),
        "surface": surface,
    }
