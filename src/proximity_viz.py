#!/usr/bin/env python3
"""
proximity_viz_node.py
=====================
Drop-in software replacement for haptic_proximity_node when bHaptics hardware
is unavailable.  Instead of driving a vest, it renders a top-down Tesla-style
proximity display and publishes it as a sensor_msgs/CompressedImage topic.

WHAT IT LOOKS LIKE
------------------
A dark circular viewport centred on the robot icon.  Obstacles detected by
the cylindrical density grid are drawn as smooth, rounded proximity arcs whose
distance from the robot body, colour, and opacity encode closeness:

  • Far obstacles   → thin, dim cyan arc near the outer ring
  • Close obstacles → thick, bright amber/red arc close to the robot body
  • Approach rate   → arc pulses (opacity oscillation) just like a real ADAS
                       display; faster approach = faster pulse

The same four-stage smoothing pipeline from haptic_proximity_node is preserved
verbatim so behaviour is numerically identical — only the output sink changes.

HOW IT WORKS
------------
1.  Reads /cylindrical_grid/density_cloud exactly as the haptic node does.
2.  Casts one ray per angular sector (same geometry, controllable via params).
3.  Each ray feeds the same TactorDistanceTracker pipeline.
4.  A Matplotlib / Cairo-free pure-numpy + cv2 renderer draws the arcs.
5.  The finished frame is JPEG-compressed and published to
    /proximity_viz/compressed at ~publish_rate Hz.

No bHaptics, no Matplotlib, no external dependencies beyond what a standard
ROS Noetic desktop install provides (rospy, cv2, numpy, sensor_msgs).

SUBSCRIBED TOPICS
-----------------
    /cylindrical_grid/density_cloud  (sensor_msgs/PointCloud2)

PUBLISHED TOPICS
----------------
    /proximity_viz/compressed        (sensor_msgs/CompressedImage)
    /proximity_viz/ray_markers       (visualization_msgs/MarkerArray)  [optional]

PARAMETERS  (all ~-namespaced, identical defaults to haptic node)
-----------------------------------------------------------------
Rendering:
    ~image_size           int    512     px  (square output image)
    ~display_radius_m     float  4.0     m   world metres shown in image radius
    ~jpeg_quality         int    85
    ~robot_icon_radius_px int    32          drawn robot footprint
    ~arc_base_width_px    float  4.0         minimum arc stroke width
    ~arc_max_width_px     float  18.0        maximum arc stroke width at dist=0
    ~arc_angular_width_deg float 8.5         half-width of each arc segment
    ~n_angular            int    36          ray sectors (match grid_viz_node)

Smoothing (identical to haptic node):
    ~dist_ema_alpha               float  0.25
    ~dist_outlier_zscore          float  2.0
    ~rate_attack_alpha            float  0.6
    ~rate_release_alpha           float  0.10
    ~approach_rate_threshold_on   float  0.07  m/s
    ~approach_rate_threshold_off  float  0.03  m/s
    ~approach_rate_max            float  1.0   m/s
    ~distance_window_secs         float  0.3   s

Grid geometry (must match grid_viz_node):
    ~grid_r_min           float  0.15
    ~grid_r_max           float  8.0
    ~grid_n_radial        int    8
    ~grid_log_radial      bool   true

Sensing:
    ~ray_range            float  4.0   m
    ~density_threshold    float  0.05
    ~publish_rate         float  20.0  Hz
    ~z_margin             float  0.05  m
    ~vest_z_offset        float  0.0   m
    ~vest_height          float  0.5   m
    ~frame_id             str    "base_link"
    ~debug_rays           bool   false
"""

import json
import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# ─────────────────────────────────────────────────────────────────────────────
#  Smoothing pipeline  (identical to haptic_proximity_node)
# ─────────────────────────────────────────────────────────────────────────────

class TactorDistanceTracker:
    """Four-stage smoothing pipeline — see module docstring for full details."""

    _MAX_SAMPLES = 128

    def __init__(self, window_secs, dist_ema_alpha, dist_outlier_zscore,
                 rate_attack_alpha, rate_release_alpha,
                 approach_rate_threshold_on, approach_rate_threshold_off,
                 approach_rate_max, ray_range):
        self.window_secs                 = window_secs
        self.dist_ema_alpha              = dist_ema_alpha
        self.dist_outlier_zscore         = dist_outlier_zscore
        self.rate_attack_alpha           = rate_attack_alpha
        self.rate_release_alpha          = rate_release_alpha
        self.approach_rate_threshold_on  = approach_rate_threshold_on
        self.approach_rate_threshold_off = approach_rate_threshold_off
        self.approach_rate_max           = approach_rate_max
        self.ray_range                   = ray_range

        self._dist_smooth   = None          # type: Optional[float]
        self._smoothed_rate = 0.0           # type: float
        self._gate_open     = False         # type: bool
        self._history       = deque(maxlen=self._MAX_SAMPLES)  # type: Deque[Tuple[float, float]]
        self._prev_raw_dist = None          # type: Optional[float]

    def update(self, raw_distance, timestamp):
        # type: (float, float) -> float
        """Return intensity in [0, 1]; 0 at steady-state or receding."""
        prev_raw = self._prev_raw_dist
        self._prev_raw_dist = raw_distance

        # Sentinel-transition guard
        if (prev_raw is not None
                and prev_raw >= self.ray_range
                and raw_distance < self.ray_range):
            self._dist_smooth   = raw_distance
            self._smoothed_rate = 0.0
            self._gate_open     = False
            self._history.clear()

        # Stage 1: Distance EMA
        if self._dist_smooth is None:
            self._dist_smooth = raw_distance
        else:
            a = self.dist_ema_alpha
            self._dist_smooth = a * raw_distance + (1.0 - a) * self._dist_smooth

        self._history.append((timestamp, self._dist_smooth))
        cutoff = timestamp - self.window_secs
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        # Stage 2 + OLS derivative
        raw_rate = self._estimate_approach_rate()

        # Stage 3: Asymmetric rate EMA
        a = self.rate_attack_alpha if raw_rate > self._smoothed_rate else self.rate_release_alpha
        self._smoothed_rate = a * raw_rate + (1.0 - a) * self._smoothed_rate
        if self._smoothed_rate < 0.0:
            self._smoothed_rate = 0.0

        # Stage 4: Hysteretic gate
        if self._gate_open:
            if self._smoothed_rate < self.approach_rate_threshold_off:
                self._gate_open = False
        else:
            if self._smoothed_rate >= self.approach_rate_threshold_on:
                self._gate_open = True

        if not self._gate_open:
            return 0.0

        intensity = self._smoothed_rate / max(self.approach_rate_max, 1e-9)
        return max(0.0, min(1.0, intensity))

    def get_smoothed_distance(self):
        # type: () -> Optional[float]
        return self._dist_smooth

    def _estimate_approach_rate(self):
        # type: () -> float
        n = len(self._history)
        if n < 2:
            return 0.0
        if n == 2:
            t0, d0 = self._history[0]
            t1, d1 = self._history[1]
            dt = t1 - t0
            return 0.0 if dt < 1e-6 else -(d1 - d0) / dt

        ts = np.array([s[0] for s in self._history], dtype=np.float64)
        ds = np.array([s[1] for s in self._history], dtype=np.float64)

        if self.dist_outlier_zscore > 0.0:
            ds = self._reject_outliers(ds, self.dist_outlier_zscore)

        t_mean = ts.mean()
        ts_c   = ts - t_mean
        denom  = float(np.dot(ts_c, ts_c))
        if denom < 1e-12:
            return 0.0
        return -float(np.dot(ts_c, ds)) / denom

    @staticmethod
    def _reject_outliers(ds, zscore):
        # type: (np.ndarray, float) -> np.ndarray
        ds     = ds.copy()
        median = np.median(ds)
        q75, q25 = np.percentile(ds, [75, 25])
        iqr    = q75 - q25
        if iqr < 1e-9:
            return ds
        lo, hi = median - zscore * iqr, median + zscore * iqr
        ds[(ds < lo) | (ds > hi)] = median
        return ds


# ─────────────────────────────────────────────────────────────────────────────
#  PointCloud2 reader  (identical to haptic node)
# ─────────────────────────────────────────────────────────────────────────────

def _get_field_offset(cloud, name):
    # type: (PointCloud2, str) -> int
    for f in cloud.fields:
        if f.name == name:
            return f.offset
    raise KeyError("Field '{}' not found in PointCloud2".format(name))


def read_density_cloud(cloud):
    # type: (PointCloud2) -> np.ndarray
    """Returns (N, 4) float32 array: [x, y, z, intensity]."""
    if cloud.width == 0:
        return np.zeros((0, 4), dtype=np.float32)
    ox = _get_field_offset(cloud, "x")
    oy = _get_field_offset(cloud, "y")
    oz = _get_field_offset(cloud, "z")
    oi = _get_field_offset(cloud, "intensity")
    n   = cloud.width * cloud.height
    buf = np.frombuffer(cloud.data, dtype=np.uint8)[:n * cloud.point_step].reshape(n, cloud.point_step)
    return np.column_stack([
        buf[:, ox:ox+4].view(np.float32).flatten(),
        buf[:, oy:oy+4].view(np.float32).flatten(),
        buf[:, oz:oz+4].view(np.float32).flatten(),
        buf[:, oi:oi+4].view(np.float32).flatten(),
    ])


# ─────────────────────────────────────────────────────────────────────────────
#  Density ray caster  (identical geometry to haptic node)
# ─────────────────────────────────────────────────────────────────────────────

class DensityRayCaster:

    def __init__(self, r_min, r_max, n_radial, n_angular, log_radial):
        self.n_radial  = n_radial
        self.n_angular = n_angular

        if log_radial and r_min > 0:
            log_min = math.log(r_min)
            log_max = math.log(r_max)
            self.r_bounds = [
                math.exp(log_min + i * (log_max - log_min) / n_radial)
                for i in range(n_radial + 1)
            ]
        else:
            dr = (r_max - r_min) / n_radial
            self.r_bounds = [r_min + i * dr for i in range(n_radial + 1)]

        self.r_centres = [
            0.5 * (self.r_bounds[i] + self.r_bounds[i + 1])
            for i in range(n_radial)
        ]
        self._pts = np.zeros((0, 4), dtype=np.float32)  # type: np.ndarray

    def update(self, cloud):
        # type: (PointCloud2) -> None
        self._pts = read_density_cloud(cloud)

    def _r_bin(self, r):
        # type: (float) -> int
        if r < self.r_bounds[0] or r >= self.r_bounds[-1]:
            return -1
        lo, hi = 0, self.n_radial - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if r < self.r_bounds[mid + 1]:
                hi = mid
            else:
                lo = mid + 1
        return lo

    def cast_ray_at_z(self, angle_rad, tactor_z, z_margin,
                      ray_range, density_threshold):
        # type: (float, float, float, float, float) -> Tuple[float, float]
        pts = self._pts
        if pts.shape[0] == 0:
            return 0.0, ray_range

        mask = (pts[:, 2] >= tactor_z - z_margin) & \
               (pts[:, 2] <= tactor_z + z_margin)
        band = pts[mask]
        if band.shape[0] == 0:
            return 0.0, ray_range

        xs  = band[:, 0].astype(np.float64)
        ys  = band[:, 1].astype(np.float64)
        ins = band[:, 3]

        thetas = np.arctan2(ys, xs)
        thetas[thetas < 0] += 2.0 * math.pi

        two_pi_inv = self.n_angular / (2.0 * math.pi)
        dmap = {}  # type: Dict[Tuple[int, int], float]
        for k in range(band.shape[0]):
            r  = math.sqrt(float(xs[k])**2 + float(ys[k])**2)
            ir = self._r_bin(r)
            if ir < 0:
                continue
            ith = min(int(thetas[k] * two_pi_inv), self.n_angular - 1)
            key = (ir, ith)
            val = float(ins[k])
            if key not in dmap or dmap[key] < val:
                dmap[key] = val

        ith_ray = int((angle_rad % (2.0 * math.pi)) * two_pi_inv) % self.n_angular
        r_end   = min(ray_range, self.r_bounds[-1])

        for ir in range(self.n_radial):
            if self.r_centres[ir] > r_end:
                break
            intensity = dmap.get((ir, ith_ray), 0.0)
            if intensity >= density_threshold:
                return intensity, self.r_centres[ir]

        return 0.0, ray_range


# ─────────────────────────────────────────────────────────────────────────────
#  Tesla-style proximity renderer  (OpenCV, no Matplotlib)
# ─────────────────────────────────────────────────────────────────────────────

class ProximityRenderer:
    """
    Renders a top-down obstacle proximity display similar to Tesla ADAS UI.

    The circular viewport shows the robot at centre.  Each angular sector with
    a detected obstacle is drawn as a rounded arc whose radial position maps
    linearly from the robot body (close obstacle) to the viewport edge (far).
    Arc colour and stroke width further encode distance; approach-rate intensity
    drives an opacity pulse for actively closing obstacles.
    """

    # Colour stops: (distance_fraction, BGR)
    #   distance_fraction = smoothed_distance / display_radius_m
    _COLOUR_STOPS = [
        (0.00, (0,   60,  255)),   # red-orange  — very close
        (0.20, (0,  165,  255)),   # amber
        (0.45, (0,  230,  200)),   # warm cyan
        (1.00, (80, 200,  120)),   # muted green — far
    ]

    def __init__(self, image_size, display_radius_m,
                 robot_icon_radius_px,
                 arc_base_width_px, arc_max_width_px,
                 arc_angular_width_deg):
        # type: (int, float, int, float, float, float) -> None
        self.sz          = image_size
        self.cx          = image_size // 2
        self.cy          = image_size // 2
        self.px_per_m    = (image_size // 2 - robot_icon_radius_px - 8) / display_radius_m
        self.robot_r     = robot_icon_radius_px
        self.w_min       = arc_base_width_px
        self.w_max       = arc_max_width_px
        self.half_arc_deg = arc_angular_width_deg
        self.display_r_m = display_radius_m
        self._t          = 0.0   # phase clock for pulse animations

    # ── Colour interpolation ─────────────────────────────────────────────────

    def _distance_colour(self, dist_m, alpha=1.0):
        # type: (float, float) -> Tuple[int, int, int, int]
        """Return a BGRA tuple for a given distance in metres."""
        frac  = max(0.0, min(1.0, dist_m / self.display_r_m))
        stops = self._COLOUR_STOPS

        bgr = stops[-1][1]
        for i in range(len(stops) - 1):
            f0, c0 = stops[i]
            f1, c1 = stops[i + 1]
            if frac <= f1:
                t = (frac - f0) / max(f1 - f0, 1e-9)
                bgr = tuple(int(c0[j] + t * (c1[j] - c0[j])) for j in range(3))
                break

        a = max(0, min(255, int(alpha * 255)))
        return (bgr[0], bgr[1], bgr[2], a)

    # ── Arc drawing ─────────────────────────────────────────────────────────

    def _draw_arc(self, canvas, angle_deg, dist_m, intensity):
        # type: (np.ndarray, float, float, float) -> None
        """
        Draw one smooth arc segment on `canvas` (BGRA float32, range 0-255).

        The arc is drawn at the pixel radius corresponding to dist_m, spanning
        +/-half_arc_deg around angle_deg.  Width and brightness scale inversely
        with distance.  When intensity > 0 (obstacle approaching) the arc
        pulses using the renderer's internal phase clock.
        """
        dist_m = max(0.01, min(dist_m, self.display_r_m))
        frac   = dist_m / self.display_r_m

        # Radial pixel position
        robot_outer = self.robot_r + 4   # keep arcs outside body outline
        max_r_px    = self.sz // 2 - 10
        r_px        = robot_outer + (max_r_px - robot_outer) * frac

        # Stroke width: thick when close, thin when far
        w_px = int(self.w_max + (self.w_min - self.w_max) * frac)
        w_px = max(2, w_px)

        # Opacity: base + pulse when approaching
        base_alpha = 0.65 + (1.0 - frac) * 0.35
        if intensity > 0.05:
            pulse_speed = 1.5 + intensity * 6.0
            pulse       = 0.5 + 0.5 * math.sin(self._t * pulse_speed)
            base_alpha  = min(1.0, base_alpha + intensity * pulse * 0.35)

        colour = self._distance_colour(dist_m, base_alpha)

        # OpenCV ellipse: angles are measured CW from 3 o'clock in image space.
        # ROS uses CCW from 12 o'clock (forward = +x).  Convert:
        #   image_angle = 90 - ros_angle_deg   (flip y, rotate)
        img_centre_deg = 90.0 - angle_deg
        start_deg = img_centre_deg - self.half_arc_deg
        end_deg   = img_centre_deg + self.half_arc_deg

        # Draw on a temporary per-arc layer so we can use alpha blending
        layer = np.zeros_like(canvas)
        cv2.ellipse(
            layer,
            (self.cx, self.cy),
            (int(r_px), int(r_px)),
            0,                    # rotation of the ellipse axes
            start_deg, end_deg,   # arc span
            colour,
            thickness=w_px,
            lineType=cv2.LINE_AA,
        )
        # Alpha blend: out = canvas + layer  (layer already has pre-multiplied alpha baked via colour)
        mask = layer[:, :, 3:4] / 255.0
        canvas[:] = canvas * (1.0 - mask) + layer * mask

    # ── Obstacle "blob" connector lines ──────────────────────────────────────

    def _draw_connectors(self, canvas, sectors):
        # type: (np.ndarray, List[Tuple[float, float, float]]) -> None
        """
        For groups of adjacent active sectors, draw a soft spline connecting
        the arc midpoints, simulating Tesla's continuous obstacle silhouette.

        sectors: list of (angle_deg, dist_m, intensity) for active sectors only.
        Sorted by angle for adjacency detection.
        """
        if len(sectors) < 2:
            return

        # Sort by angle
        sectors = sorted(sectors, key=lambda s: s[0])

        # Group adjacent sectors (within 2 x sector_width)
        gap_thresh = 2.2 * (360.0 / 36)   # ~two sector widths at 36 sectors

        groups = []
        group  = [sectors[0]]
        for prev, curr in zip(sectors, sectors[1:]):
            diff = (curr[0] - prev[0]) % 360.0
            if diff < gap_thresh:
                group.append(curr)
            else:
                groups.append(group)
                group = [curr]
        groups.append(group)

        # Wrap-around check
        if (len(groups) > 1
                and ((sectors[0][0] + 360.0 - sectors[-1][0]) % 360.0) < gap_thresh):
            groups[0] = groups[-1] + groups[0]
            groups.pop(-1)

        for group in groups:
            if len(group) < 2:
                continue
            pts = []
            for (ang, dist, _inten) in group:
                r_px = self._dist_to_px(dist)
                rad  = math.radians(90.0 - ang)   # image frame
                px   = self.cx + r_px * math.cos(rad)
                py   = self.cy - r_px * math.sin(rad)
                pts.append((int(px), int(py)))

            # Draw as a polyline with rounded corners using sub-segment smoothing
            for i in range(len(pts) - 1):
                dist_avg = 0.5 * (group[i][1] + group[i + 1][1])
                col = self._distance_colour(dist_avg, alpha=0.45)
                cv2.line(canvas, pts[i], pts[i + 1], col,
                         thickness=2, lineType=cv2.LINE_AA)

    def _dist_to_px(self, dist_m):
        # type: (float) -> float
        frac = max(0.0, min(1.0, dist_m / self.display_r_m))
        robot_outer = self.robot_r + 4
        max_r_px    = self.sz // 2 - 10
        return robot_outer + (max_r_px - robot_outer) * frac

    # ── Background / HUD ─────────────────────────────────────────────────────

    def _draw_background(self, canvas):
        # type: (np.ndarray) -> None
        """Draw concentric range rings, compass ticks, and viewport circle."""
        # Radial range rings at 1 m intervals
        for r_m in np.arange(1.0, self.display_r_m + 0.01, 1.0):
            r_px = int(self._dist_to_px(r_m))
            cv2.circle(canvas, (self.cx, self.cy), r_px,
                       (50, 50, 50, 180), 1, lineType=cv2.LINE_AA)

            # Range label (top of ring)
            label_x = self.cx
            label_y = self.cy - r_px - 4
            if 0 < label_y < self.sz:
                cv2.putText(canvas, "{:.0f}m".format(r_m),
                            (label_x - 12, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28,
                            (70, 70, 70, 200), 1, cv2.LINE_AA)

        # Viewport boundary circle
        cv2.circle(canvas, (self.cx, self.cy), self.sz // 2 - 2,
                   (60, 60, 60, 255), 2, lineType=cv2.LINE_AA)

        # Cardinal direction ticks
        for deg, label in [(90, "F"), (0, "R"), (270, "B"), (180, "L")]:
            rad  = math.radians(deg)
            r_px = self.sz // 2 - 14
            tx   = self.cx + int(r_px * math.cos(rad))
            ty   = self.cy - int(r_px * math.sin(rad))
            cv2.putText(canvas, label,
                        (tx - 5, ty + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (90, 90, 90, 255), 1, cv2.LINE_AA)

    def _draw_robot(self, canvas):
        # type: (np.ndarray) -> None
        """Draw a simple robot footprint icon at centre."""
        r = self.robot_r
        # Body fill
        cv2.circle(canvas, (self.cx, self.cy), r,
                   (35, 35, 35, 255), -1, lineType=cv2.LINE_AA)
        # Body outline
        cv2.circle(canvas, (self.cx, self.cy), r,
                   (120, 120, 120, 255), 2, lineType=cv2.LINE_AA)
        # Forward indicator arrow
        fw_x = self.cx
        fw_y = self.cy - r + 6
        cv2.arrowedLine(canvas,
                        (self.cx, self.cy + r // 3),
                        (fw_x, fw_y),
                        (200, 200, 200, 255), 2,
                        tipLength=0.4, line_type=cv2.LINE_AA)

    # ── Public render entry point ────────────────────────────────────────────

    def render(self, sector_data, dt):
        # type: (List[Tuple[float, float, float]], float) -> np.ndarray
        """
        Produce one BGR frame.

        Parameters
        ----------
        sector_data : list of (angle_deg, dist_m, intensity)
            angle_deg  — sector centre direction in degrees (0=forward/+X, CCW)
            dist_m     — smoothed hit distance in metres
            intensity  — approach-rate intensity in [0, 1] (0 = silent/steady)
        dt : float
            Elapsed time since last render (seconds), used to advance pulse clock.

        Returns
        -------
        np.ndarray (H, W, 3) uint8, BGR
        """
        self._t += dt

        # Work in BGRA float32 for alpha compositing
        canvas = np.zeros((self.sz, self.sz, 4), dtype=np.float32)

        # Circular clip mask
        cv2.circle(canvas, (self.cx, self.cy),
                   self.sz // 2 - 2,
                   (18, 18, 18, 255), -1, lineType=cv2.LINE_AA)

        self._draw_background(canvas)

        # Draw arcs for all sectors with an obstacle inside display radius
        active_sectors = []
        for (ang_deg, dist_m, intensity) in sector_data:
            if dist_m < self.display_r_m:
                self._draw_arc(canvas, ang_deg, dist_m, intensity)
                if dist_m < self.display_r_m * 0.9:
                    active_sectors.append((ang_deg, dist_m, intensity))

        # Draw soft connector lines for adjacent obstacle clusters
        self._draw_connectors(canvas, active_sectors)

        self._draw_robot(canvas)

        # Status overlay (top-left corner)
        active_count = sum(1 for (_, d, _) in sector_data if d < self.display_r_m)
        cv2.putText(canvas, "PROXIMITY  {} sectors".format(active_count),
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (100, 100, 100, 220), 1, cv2.LINE_AA)

        # Convert BGRA float -> BGR uint8
        bgra_u8 = np.clip(canvas, 0, 255).astype(np.uint8)
        return cv2.cvtColor(bgra_u8, cv2.COLOR_BGRA2BGR)


# ─────────────────────────────────────────────────────────────────────────────
#  Main node
# ─────────────────────────────────────────────────────────────────────────────

class ProximityVizNode:

    def __init__(self):
        rospy.init_node("proximity_viz_node", anonymous=False)

        # ── Rendering params ─────────────────────────────────────────────────
        image_size            = rospy.get_param("~image_size",            512)
        display_radius_m      = rospy.get_param("~display_radius_m",      4.0)
        jpeg_quality          = rospy.get_param("~jpeg_quality",          85)
        robot_icon_radius_px  = rospy.get_param("~robot_icon_radius_px",  32)
        arc_base_width_px     = rospy.get_param("~arc_base_width_px",     4.0)
        arc_max_width_px      = rospy.get_param("~arc_max_width_px",      18.0)
        arc_angular_width_deg = rospy.get_param("~arc_angular_width_deg", 8.5)

        # ── Sensing / grid params ────────────────────────────────────────────
        self.frame_id          = rospy.get_param("~frame_id",          "base_link")
        self.publish_rate      = rospy.get_param("~publish_rate",      20.0)
        self.ray_range         = rospy.get_param("~ray_range",         display_radius_m)
        self.density_threshold = rospy.get_param("~density_threshold", 0.05)
        self.z_margin          = rospy.get_param("~z_margin",          0.05)
        self.debug_rays        = rospy.get_param("~debug_rays",        False)

        vest_z_offset = rospy.get_param("~vest_z_offset", 0.0)
        vest_height   = rospy.get_param("~vest_height",   0.5)

        r_min       = rospy.get_param("~grid_r_min",      0.15)
        r_max       = rospy.get_param("~grid_r_max",      8.0)
        n_radial    = rospy.get_param("~grid_n_radial",   8)
        n_angular   = rospy.get_param("~n_angular",       36)
        log_radial  = rospy.get_param("~grid_log_radial", True)

        # ── Smoothing params (identical to haptic node) ──────────────────────
        dist_ema_alpha              = rospy.get_param("~dist_ema_alpha",              0.25)
        dist_outlier_zscore         = rospy.get_param("~dist_outlier_zscore",         2.0)
        rate_attack_alpha           = rospy.get_param("~rate_attack_alpha",           0.6)
        rate_release_alpha          = rospy.get_param("~rate_release_alpha",          0.10)
        approach_rate_threshold_on  = rospy.get_param("~approach_rate_threshold_on",  0.07)
        approach_rate_threshold_off = rospy.get_param("~approach_rate_threshold_off", 0.03)
        approach_rate_max           = rospy.get_param("~approach_rate_max",           1.0)
        distance_window_secs        = rospy.get_param("~distance_window_secs",        0.3)

        if approach_rate_threshold_off >= approach_rate_threshold_on:
            rospy.logwarn("[ProximityViz] threshold_off >= threshold_on — clamping")
            approach_rate_threshold_off = approach_rate_threshold_on * 0.4

        # ── Build per-sector ray + tracker ───────────────────────────────────
        # One ray per angular sector, placed at mid-height of the vest
        self._n_angular = n_angular
        self._sensor_z  = vest_z_offset + vest_height * 0.5
        self._sectors   = []  # type: List[float]

        for i in range(n_angular):
            angle_rad = i * 2.0 * math.pi / n_angular
            self._sectors.append(angle_rad)

        self._trackers = [
            TactorDistanceTracker(
                window_secs                  = distance_window_secs,
                dist_ema_alpha               = dist_ema_alpha,
                dist_outlier_zscore          = dist_outlier_zscore,
                rate_attack_alpha            = rate_attack_alpha,
                rate_release_alpha           = rate_release_alpha,
                approach_rate_threshold_on   = approach_rate_threshold_on,
                approach_rate_threshold_off  = approach_rate_threshold_off,
                approach_rate_max            = approach_rate_max,
                ray_range                    = self.ray_range,
            )
            for _ in self._sectors
        ]

        # ── Ray caster ───────────────────────────────────────────────────────
        self.ray_caster = DensityRayCaster(
            r_min, r_max, n_radial, n_angular, log_radial)

        # ── Renderer ─────────────────────────────────────────────────────────
        self.renderer = ProximityRenderer(
            image_size            = image_size,
            display_radius_m      = display_radius_m,
            robot_icon_radius_px  = robot_icon_radius_px,
            arc_base_width_px     = arc_base_width_px,
            arc_max_width_px      = arc_max_width_px,
            arc_angular_width_deg = arc_angular_width_deg,
        )
        self._jpeg_quality  = jpeg_quality
        self._jpeg_params   = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        self._last_cloud    = None   # type: Optional[PointCloud2]
        self._last_time     = rospy.Time.now().to_sec()

        # ── Publishers ───────────────────────────────────────────────────────
        self._img_pub = rospy.Publisher(
            "/proximity_viz/compressed", CompressedImage, queue_size=2)

        if self.debug_rays:
            self._ray_pub = rospy.Publisher(
                "/proximity_viz/ray_markers", MarkerArray, queue_size=2)

        rospy.Subscriber(
            "/cylindrical_grid/density_cloud", PointCloud2,
            self._on_cloud, queue_size=2, buff_size=2**20)

        rospy.loginfo(
            "[ProximityViz] Ready  |  {} sectors  |  "
            "display_r={}m  |  image={}px  |  rate={}Hz".format(
                n_angular, display_radius_m, image_size, self.publish_rate)
        )

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _on_cloud(self, msg):
        # type: (PointCloud2) -> None
        self._last_cloud = msg
        self.ray_caster.update(msg)

    # ── Main loop ────────────────────────────────────────────────────────────

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self._last_cloud is not None:
                self._update()
            rate.sleep()

    def _update(self):
        now             = rospy.Time.now().to_sec()
        dt              = now - self._last_time
        self._last_time = now

        sector_data = []  # type: List[Tuple[float, float, float]]

        for idx, angle_rad in enumerate(self._sectors):
            _density, hit_dist = self.ray_caster.cast_ray_at_z(
                angle_rad         = angle_rad,
                tactor_z          = self._sensor_z,
                z_margin          = self.z_margin,
                ray_range         = self.ray_range,
                density_threshold = self.density_threshold,
            )
            intensity = self._trackers[idx].update(hit_dist, now)

            # Use the EMA-smoothed distance for rendering (more stable position)
            smooth_dist = self._trackers[idx].get_smoothed_distance()
            if smooth_dist is None:
                smooth_dist = hit_dist

            # Convert angle_rad (ROS: CCW from +X / forward) to degrees
            angle_deg = math.degrees(angle_rad)
            sector_data.append((angle_deg, smooth_dist, intensity))

        # Render frame
        bgr_frame = self.renderer.render(sector_data, dt)

        # Compress and publish
        ok, buf = cv2.imencode(".jpg", bgr_frame, self._jpeg_params)
        if not ok:
            rospy.logwarn_throttle(5.0, "[ProximityViz] JPEG encode failed")
            return

        msg                 = CompressedImage()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.format          = "jpeg"
        msg.data            = buf.tobytes()
        self._img_pub.publish(msg)

        # Optional Rviz ray markers
        if self.debug_rays and hasattr(self, "_ray_pub"):
            self._ray_pub.publish(
                self._build_ray_markers(sector_data))

    # ── Debug markers ────────────────────────────────────────────────────────

    def _build_ray_markers(self, sector_data):
        # type: (List[Tuple[float, float, float]]) -> MarkerArray
        ma  = MarkerArray()
        mk  = Marker()
        now = rospy.Time.now()
        mk.header.frame_id    = self.frame_id
        mk.header.stamp       = now
        mk.ns                 = "proximity_viz_rays"
        mk.id                 = 0
        mk.type               = Marker.LINE_LIST
        mk.action             = Marker.ADD
        mk.scale.x            = 0.015
        mk.pose.orientation.w = 1.0
        mk.lifetime           = rospy.Duration(0.15)

        for (ang_deg, dist_m, intensity) in sector_data:
            angle_rad = math.radians(ang_deg)
            end_r     = dist_m
            t         = max(0.0, min(1.0, 1.0 - dist_m / self.ray_range))
            col       = ColorRGBA(
                r=min(1.0, 2.0 * t),
                g=min(1.0, 2.0 * (1.0 - t)),
                b=0.0, a=0.7)
            p0 = Point(x=0.0, y=0.0, z=self._sensor_z)
            p1 = Point(x=end_r * math.cos(angle_rad),
                       y=end_r * math.sin(angle_rad),
                       z=self._sensor_z)
            mk.points += [p0, p1]
            mk.colors  += [col, col]

        ma.markers.append(mk)
        return ma


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        node = ProximityVizNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass