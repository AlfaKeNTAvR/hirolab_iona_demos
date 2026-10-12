#!/usr/bin/env python3
"""
haptic_proximity_node.py
========================
Reads the cylindrical density grid (sensor_msgs/PointCloud2 with intensity)
published by grid_viz_node and drives a bHaptics TactSuit Pro via the
bhaptics_ros_bridge to give the wearer spatial obstacle proximity feedback.

HOW IT WORKS
------------
1.  The bHaptics TactSuit Pro vest is modelled as a cylinder of tactors
    floating at `vest_z_offset` above base_link, with radius `vest_radius`.

2.  Each tactor has a known (angle, height) position on the vest body.
    A sense ray is projected outward from every tactor's 3-D position in
    the horizontal direction the tactor faces (radially away from the body),
    extended to `ray_range` metres.

3.  Vibration is driven exclusively by the APPROACH RATE (−dD/dt), so the
    vest is silent at steady-state and when objects recede, and buzzes harder
    the faster an obstacle closes in.

4.  Each ray is evaluated against only the voxels within a z-band centred on
    that tactor's own z position (±z_margin), giving independent per-row
    feedback.

5.  Intensities are published to `/bhaptics/play_dot` at `~publish_rate` Hz.

═══════════════════════════════════════════════════════════════════════════════
SMOOTHING PIPELINE  (per tactor, applied in order)
═══════════════════════════════════════════════════════════════════════════════

The raw distance signal from the cylindrical grid is inherently jittery: the
discrete shell structure means a stationary obstacle can appear to jump between
adjacent shells on consecutive frames.  Without pre-processing this produces
large spurious derivatives and phantom vibration at steady state.

Four independently tunable stages are applied before the derivative is
committed to the haptic output.

  ┌─────────────────────────────────────────────────────────────────────────┐
  │ Stage 1 — Distance EMA  (spatial / temporal pre-filter)                 │
  │                                                                         │
  │   d_smooth[t] = α_d · d_raw[t]  +  (1−α_d) · d_smooth[t−1]            │
  │                                                                         │
  │   Applied BEFORE any derivative is computed.  Because a stationary      │
  │   obstacle at the boundary between two shells will appear to oscillate  │
  │   by one shell width, this is the single most effective stage.          │
  │                                                                         │
  │   α_d = ~dist_ema_alpha   (0 = very heavy smooth, 1 = passthrough)     │
  │   Typical: 0.15–0.35.  Lower values → better steady-state silence at   │
  │   the cost of slightly slower distance tracking.                        │
  └─────────────────────────────────────────────────────────────────────────┘
         │
         ▼  d_smooth fed into the sliding window
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ Stage 2 — Outlier rejection inside the derivative window                │
  │                                                                         │
  │   Before the least-squares slope is computed, any sample whose          │
  │   distance deviates from the window median by more than                 │
  │   `dist_outlier_zscore × IQR` is replaced with the median.             │
  │                                                                         │
  │   This kills single-frame glitch samples that slip through Stage 1      │
  │   (e.g. momentary drop-out to ray_range and back in one tick).          │
  │                                                                         │
  │   dist_outlier_zscore = ~dist_outlier_zscore                           │
  │   0 = disabled (no replacement);  1.5 = moderate;  3.0 = conservative  │
  └─────────────────────────────────────────────────────────────────────────┘
         │
         ▼  slope → raw_approach_rate = −dD/dt
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ Stage 3 — Asymmetric attack / release EMA on the rate estimate          │
  │                                                                         │
  │   The smoothed rate uses different coefficients when the raw rate is    │
  │   rising (fast attack → quick response to genuine approaches) versus    │
  │   falling (slow release → gradual fade-out rather than abrupt cut).     │
  │                                                                         │
  │   if raw_rate > smoothed_rate:                                          │
  │       smoothed_rate = α_atk · raw_rate + (1−α_atk) · smoothed_rate     │
  │   else:                                                                 │
  │       smoothed_rate = α_rel · raw_rate + (1−α_rel) · smoothed_rate     │
  │                                                                         │
  │   α_atk = ~rate_attack_alpha   (typical 0.5–0.8, fast)                 │
  │   α_rel = ~rate_release_alpha  (typical 0.05–0.2, slow)                │
  │                                                                         │
  │   A slow release prevents the common artefact where the buzzer cuts     │
  │   out for one frame mid-approach because the derivative estimate dips   │
  │   transiently below threshold.                                          │
  └─────────────────────────────────────────────────────────────────────────┘
         │
         ▼  smoothed_rate compared to hysteretic thresholds
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ Stage 4 — Hysteretic noise gate (Schmitt trigger)                       │
  │                                                                         │
  │   Two thresholds replace the single threshold from the previous         │
  │   implementation:                                                       │
  │                                                                         │
  │     • threshold_on   — smoothed_rate must exceed this to START buzzing  │
  │     • threshold_off  — buzzing continues until rate drops BELOW this    │
  │                                                                         │
  │   threshold_off < threshold_on prevents chattering when the rate        │
  │   hovers exactly at the boundary (classic Schmitt-trigger behaviour).   │
  │                                                                         │
  │   ~approach_rate_threshold_on   (default 0.07 m/s)                     │
  │   ~approach_rate_threshold_off  (default 0.03 m/s)                     │
  └─────────────────────────────────────────────────────────────────────────┘
         │
         ▼  intensity = clamp(smoothed_rate / approach_rate_max, 0, 1)

PARAMETER QUICK-REFERENCE
--------------------------
Geometry / sensing (unchanged):
    ~vest_z_offset        float  0.0    m
    ~vest_height          float  0.5    m
    ~vest_radius          float  0.2    m
    ~ray_range            float  2.0    m
    ~density_threshold    float  0.05
    ~publish_rate         float  20.0   Hz
    ~min_intensity        float  0.05
    ~intensity_scale      float  1.0
    ~debug_rays           bool   true
    ~z_margin             float  0.05   m
    ~frame_id             str    "base_link"

Grid geometry (must match grid_viz_node):
    ~grid_r_min           float  0.15
    ~grid_r_max           float  8.0
    ~grid_n_radial        int    8
    ~grid_n_angular       int    36
    ~grid_log_radial      bool   true

Approach-rate core:
    ~approach_rate_max       float  1.0    m/s  → intensity 1.0
    ~distance_window_secs    float  0.3    s    OLS fit window

Stage 1 — Distance pre-filter:
    ~dist_ema_alpha          float  0.25        0=heavy smooth, 1=passthrough

Stage 2 — Outlier rejection:
    ~dist_outlier_zscore     float  2.0         IQR multiplier; 0=disabled

Stage 3 — Asymmetric rate EMA:
    ~rate_attack_alpha       float  0.6         fast rise coefficient
    ~rate_release_alpha      float  0.10        slow fall coefficient

Stage 4 — Hysteretic gate:
    ~approach_rate_threshold_on   float  0.07  m/s  gate open threshold
    ~approach_rate_threshold_off  float  0.03  m/s  gate close threshold

TUNING GUIDE
------------
Too much false buzz at steady state:
    → Lower ~dist_ema_alpha (try 0.10–0.15)
    → Raise ~approach_rate_threshold_on (try 0.08–0.12)
    → Lower ~dist_outlier_zscore to 1.5 (more aggressive outlier removal)

Buzz cuts out mid-approach (choppy sensation):
    → Lower ~rate_release_alpha (try 0.05–0.08)
    → Lower ~approach_rate_threshold_off (try 0.01–0.02)

Response too sluggish to genuine approaches:
    → Raise ~dist_ema_alpha toward 0.4
    → Raise ~rate_attack_alpha (try 0.7–0.9)
    → Lower ~distance_window_secs (try 0.15–0.2)

Buzz doesn't start until obstacle is very close:
    → Lower ~approach_rate_max (try 0.5)
    → Lower ~approach_rate_threshold_on (try 0.04)

SUBSCRIBED TOPICS
-----------------
    /cylindrical_grid/density_cloud  (sensor_msgs/PointCloud2)

PUBLISHED TOPICS
----------------
    /bhaptics/play_dot            (std_msgs/String)
    /bhaptics/stop_all            (std_msgs/Empty)
    /haptic_proximity/ray_markers (visualization_msgs/MarkerArray)
"""

import json
import math
from collections import deque

import rospy
import numpy as np
from std_msgs.msg import String, Empty
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# ──────────────────────────────────────────────────────────────────────────────
#  bHaptics constants
# ──────────────────────────────────────────────────────────────────────────────
VEST_FRONT = 201
VEST_BACK  = 202
PANEL_COLS = 4
PANEL_ROWS = 5
PANEL_SIZE = PANEL_COLS * PANEL_ROWS   # 20

COL_ANGLE_OFFSETS = np.array([3*math.pi/8,  math.pi/8,
                               -math.pi/8, -3*math.pi/8])


# ──────────────────────────────────────────────────────────────────────────────
#  Tactor layout
# ──────────────────────────────────────────────────────────────────────────────

def build_tactor_layout(vest_z_offset: float,
                        vest_height: float,
                        vest_radius: float) -> list:
    tactors = []
    panels  = [
        (VEST_FRONT, 0.0,     "front"),
        (VEST_BACK,  math.pi, "back"),
    ]
    for (position, centre_angle, panel_name) in panels:
        for row in range(PANEL_ROWS):
            z = vest_z_offset + vest_height * (PANEL_ROWS - 1 - row) / max(PANEL_ROWS - 1, 1)
            for col in range(PANEL_COLS):
                angle = centre_angle + COL_ANGLE_OFFSETS[col]
                if position == VEST_FRONT:
                    if math.cos(angle) <= 0:
                        rospy.logwarn_once(
                            f"[HapticProximity] Front tactor col={col} "
                            f"angle={math.degrees(angle):.1f}° outside forward hemisphere — skipping.")
                        continue
                else:
                    if math.cos(angle) >= 0:
                        rospy.logwarn_once(
                            f"[HapticProximity] Back tactor col={col} "
                            f"angle={math.degrees(angle):.1f}° outside rear hemisphere — skipping.")
                        continue
                tactors.append({
                    "position":  position,
                    "panel":     panel_name,
                    "panel_idx": row * PANEL_COLS + col,
                    "angle_rad": angle,
                    "x": vest_radius * math.cos(angle),
                    "y": vest_radius * math.sin(angle),
                    "z": z,
                })
    return tactors


# ──────────────────────────────────────────────────────────────────────────────
#  Four-stage smoothing pipeline (one instance per tactor)
# ──────────────────────────────────────────────────────────────────────────────

class TactorDistanceTracker:
    """
    Processes raw hit-distance samples through four successive smoothing stages
    before committing an approach-rate intensity to the haptic output.

    See module docstring for a full description of each stage.

    Stage 1 — Distance EMA
    ~~~~~~~~~~~~~~~~~~~~~~
    The raw distance is exponentially smoothed *before* entering the derivative
    window.  This is the primary defence against quantisation noise from the
    discrete grid shells: a stationary obstacle near a shell boundary toggles
    between two shell radii every frame, which without pre-smoothing produces a
    spurious velocity equal to one shell width / sample period.

    dist_ema_alpha : 0 = heavy smooth (large time constant), 1 = no smoothing.

    Stage 2 — Outlier rejection inside the OLS window
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Any smoothed-distance sample within the derivative window that deviates
    from the window median by more than `dist_outlier_zscore × IQR` is clamped
    to the median before the least-squares slope is computed.  The IQR-based
    spread estimator is itself robust to the outliers, so one spike does not
    inflate the rejection threshold.  Set dist_outlier_zscore = 0 to disable.

    Stage 3 — Asymmetric attack / release EMA
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    The rate EMA uses a fast coefficient (rate_attack_alpha) when the raw rate
    is rising and a slow coefficient (rate_release_alpha) when it is falling.
    This combination allows quick onset response to genuine approaches while
    preventing the buzzer from cutting out abruptly when the derivative
    estimate temporarily dips between frames.

    Stage 4 — Hysteretic noise gate (Schmitt trigger)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    A single threshold creates on/off chatter when the smoothed rate hovers at
    the boundary.  Two thresholds eliminate this: the gate opens only when the
    rate exceeds threshold_on, and stays open until it drops below threshold_off.
    threshold_off must be strictly less than threshold_on.
    """

    _MAX_SAMPLES = 128

    def __init__(self,
                 window_secs:                 float,
                 dist_ema_alpha:              float,
                 dist_outlier_zscore:         float,
                 rate_attack_alpha:           float,
                 rate_release_alpha:          float,
                 approach_rate_threshold_on:  float,
                 approach_rate_threshold_off: float,
                 approach_rate_max:           float,
                 ray_range:                   float):

        self.window_secs                 = window_secs
        self.dist_ema_alpha              = dist_ema_alpha
        self.dist_outlier_zscore         = dist_outlier_zscore
        self.rate_attack_alpha           = rate_attack_alpha
        self.rate_release_alpha          = rate_release_alpha
        self.approach_rate_threshold_on  = approach_rate_threshold_on
        self.approach_rate_threshold_off = approach_rate_threshold_off
        self.approach_rate_max           = approach_rate_max
        self.ray_range                   = ray_range

        # Stage 1 state
        self._dist_smooth: float | None = None

        # Stage 3 state
        self._smoothed_rate: float = 0.0

        # Stage 4 state  (True = gate is currently open / buzzing)
        self._gate_open: bool = False

        # Derivative window: stores (timestamp_s, smoothed_distance_m) pairs
        self._history: deque = deque(maxlen=self._MAX_SAMPLES)

        # Previous raw distance for sentinel-transition detection
        self._prev_raw_dist: float | None = None

    # ── Public API ───────────────────────────────────────────────────────────

    def update(self, raw_distance: float, timestamp: float) -> float:
        """
        Ingest one raw hit-distance sample and return intensity ∈ [0, 1].

        Returns > 0 only when an obstacle is actively and consistently
        approaching faster than the hysteretic gate threshold.  Returns 0 at
        steady state and when the obstacle is receding.

        Parameters
        ----------
        raw_distance : float
            Hit distance in metres.  Equals ray_range when no obstacle is
            within sensing range.
        timestamp : float
            Current ROS time in seconds.
        """
        # ── Sentinel-transition guard ────────────────────────────────────────
        # A ray_range → real_hit transition means the obstacle was previously
        # outside sensing range.  The apparent jump would produce a massive
        # spurious approach rate.  Reset all pipeline state and seed the EMA
        # at the actual hit distance so the next frames start cleanly.
        prev_raw = self._prev_raw_dist
        self._prev_raw_dist = raw_distance

        if (prev_raw is not None
                and prev_raw >= self.ray_range
                and raw_distance < self.ray_range):
            self._dist_smooth   = raw_distance
            self._smoothed_rate = 0.0
            self._gate_open     = False
            self._history.clear()

        # ── Stage 1: Distance EMA ────────────────────────────────────────────
        # Smooth raw distance before computing any derivative.  This collapses
        # shell-boundary oscillations into a near-constant value at steady state,
        # which makes dD/dt ≈ 0 as desired.
        if self._dist_smooth is None:
            self._dist_smooth = raw_distance
        else:
            a = self.dist_ema_alpha
            self._dist_smooth = a * raw_distance + (1.0 - a) * self._dist_smooth

        # Record the *smoothed* value in the derivative window
        self._history.append((timestamp, self._dist_smooth))

        # Prune samples older than the window
        cutoff = timestamp - self.window_secs
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        # ── Stage 2 + derivative estimation ─────────────────────────────────
        # Outlier rejection is applied inside _estimate_approach_rate.
        raw_rate = self._estimate_approach_rate()

        # ── Stage 3: Asymmetric attack / release EMA ─────────────────────────
        if raw_rate > self._smoothed_rate:
            a = self.rate_attack_alpha     # fast: respond quickly to real closing
        else:
            a = self.rate_release_alpha    # slow: don't cut out abruptly
        self._smoothed_rate = a * raw_rate + (1.0 - a) * self._smoothed_rate

        # Clamp to non-negative: a slowly decaying negative value would delay
        # the gate from opening on the next genuine approach.
        if self._smoothed_rate < 0.0:
            self._smoothed_rate = 0.0

        # ── Stage 4: Hysteretic noise gate ───────────────────────────────────
        if self._gate_open:
            # Gate stays open until rate drops below the lower threshold
            if self._smoothed_rate < self.approach_rate_threshold_off:
                self._gate_open = False
        else:
            # Gate opens only when rate exceeds the higher threshold
            if self._smoothed_rate >= self.approach_rate_threshold_on:
                self._gate_open = True

        if not self._gate_open:
            return 0.0

        # ── Output scaling ───────────────────────────────────────────────────
        intensity = self._smoothed_rate / max(self.approach_rate_max, 1e-9)
        return max(0.0, min(1.0, intensity))

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _estimate_approach_rate(self) -> float:
        """
        Estimate approach rate (m/s, positive = obstacle closing in) from the
        current derivative window via OLS linear regression on (t, d) samples.

        Stage 2 outlier rejection is applied here before the OLS fit.
        Falls back to a two-point finite difference with fewer than 3 samples.
        """
        n = len(self._history)

        if n < 2:
            return 0.0

        if n == 2:
            t0, d0 = self._history[0]
            t1, d1 = self._history[1]
            dt = t1 - t0
            if dt < 1e-6:
                return 0.0
            return -(d1 - d0) / dt   # positive when distance is decreasing

        ts = np.array([s[0] for s in self._history], dtype=np.float64)
        ds = np.array([s[1] for s in self._history], dtype=np.float64)

        # Stage 2: replace outliers with the window median before fitting
        if self.dist_outlier_zscore > 0.0:
            ds = self._reject_outliers(ds, self.dist_outlier_zscore)

        # OLS on mean-centred time axis for numerical stability
        t_mean = ts.mean()
        ts_c   = ts - t_mean
        denom  = float(np.dot(ts_c, ts_c))
        if denom < 1e-12:
            return 0.0

        slope = float(np.dot(ts_c, ds)) / denom   # dD/dt  (negative = approaching)
        return -slope                              # positive = approaching

    @staticmethod
    def _reject_outliers(ds: np.ndarray, zscore: float) -> np.ndarray:
        """
        Replace values more than `zscore × IQR` from the median with the median.

        Using IQR instead of σ keeps the spread estimate robust when outliers
        are present — a single spike does not inflate the rejection band.
        Operates on a copy; the history deque is not modified.
        """
        ds     = ds.copy()
        median = np.median(ds)
        q75, q25 = np.percentile(ds, [75, 25])
        iqr    = q75 - q25
        if iqr < 1e-9:
            return ds   # all values are identical — nothing to reject
        lo = median - zscore * iqr
        hi = median + zscore * iqr
        mask = (ds < lo) | (ds > hi)
        ds[mask] = median
        return ds


# ──────────────────────────────────────────────────────────────────────────────
#  PointCloud2 fast reader
# ──────────────────────────────────────────────────────────────────────────────

def _get_field_offset(cloud: PointCloud2, name: str) -> int:
    for f in cloud.fields:
        if f.name == name:
            return f.offset
    raise KeyError(f"Field '{name}' not found in PointCloud2")


def read_density_cloud(cloud: PointCloud2) -> np.ndarray:
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


# ──────────────────────────────────────────────────────────────────────────────
#  Ray caster (geometry unchanged from v1)
# ──────────────────────────────────────────────────────────────────────────────

class DensityRayCaster:
    """
    Caches the latest density cloud and provides per-tactor z-band ray casting.
    Smoothing is handled entirely by TactorDistanceTracker; this class only
    performs the geometric hit test and returns a raw distance.
    """

    def __init__(self, r_min: float, r_max: float, n_radial: int,
                 n_angular: int, log_radial: bool):
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
        self._pts: np.ndarray = np.zeros((0, 4), dtype=np.float32)

    def update(self, cloud: PointCloud2):
        self._pts = read_density_cloud(cloud)

    def _r_bin(self, r: float) -> int:
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

    def cast_ray_at_z(self, angle_rad: float, tactor_z: float,
                      z_margin: float, ray_range: float,
                      density_threshold: float) -> tuple:
        """Returns (hit_intensity, hit_distance); hit_distance = ray_range on miss."""
        pts = self._pts
        if pts.shape[0] == 0:
            return 0.0, ray_range

        mask = (pts[:, 2] >= tactor_z - z_margin) & (pts[:, 2] <= tactor_z + z_margin)
        band = pts[mask]
        if band.shape[0] == 0:
            return 0.0, ray_range

        xs  = band[:, 0].astype(np.float64)
        ys  = band[:, 1].astype(np.float64)
        ins = band[:, 3]

        thetas = np.arctan2(ys, xs)
        thetas[thetas < 0] += 2.0 * math.pi

        two_pi_inv = self.n_angular / (2.0 * math.pi)
        dmap: dict = {}
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


# ──────────────────────────────────────────────────────────────────────────────
#  Foxglove marker builders
# ──────────────────────────────────────────────────────────────────────────────

def _lerp_color(t: float) -> ColorRGBA:
    """Green (silent / receding) → Yellow → Red (fast approach). t ∈ [0, 1]."""
    c = ColorRGBA()
    c.r = min(1.0, 2.0 * t)
    c.g = min(1.0, 2.0 * (1.0 - t))
    c.b = 0.0
    c.a = 0.7
    return c


def build_ray_markers(tactors: list, intensities: list, hit_distances: list,
                      ray_range: float, frame_id: str,
                      vest_radius: float) -> MarkerArray:
    ma  = MarkerArray()
    now = rospy.Time.now()

    for panel_name, position in [("front", VEST_FRONT), ("back", VEST_BACK)]:
        mk = Marker()
        mk.header.frame_id    = frame_id
        mk.header.stamp       = now
        mk.ns                 = f"haptic_rays_{panel_name}"
        mk.id                 = 0 if position == VEST_FRONT else 1
        mk.type               = Marker.LINE_LIST
        mk.action             = Marker.ADD
        mk.scale.x            = 0.015
        mk.pose.orientation.w = 1.0
        mk.lifetime           = rospy.Duration(0.15)

        for i, t in enumerate(tactors):
            if t["position"] != position:
                continue
            intensity = intensities[i]
            end_r     = hit_distances[i] if intensity > 0 else ray_range
            angle     = t["angle_rad"]
            p0 = Point(x=t["x"], y=t["y"], z=t["z"])
            p1 = Point(x=t["x"] + end_r * math.cos(angle),
                       y=t["y"] + end_r * math.sin(angle),
                       z=t["z"])
            col = _lerp_color(intensity)
            mk.points += [p0, p1]
            mk.colors  += [col, col]

        ma.markers.append(mk)

    # Vest cylinder outline
    vest_mk = Marker()
    vest_mk.header.frame_id    = frame_id
    vest_mk.header.stamp       = now
    vest_mk.ns                 = "vest_outline"
    vest_mk.id                 = 0
    vest_mk.type               = Marker.LINE_LIST
    vest_mk.action             = Marker.ADD
    vest_mk.scale.x            = 0.008
    vest_mk.color              = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.4)
    vest_mk.pose.orientation.w = 1.0
    vest_mk.lifetime           = rospy.Duration(0.15)

    z_levels = sorted(set(round(t["z"], 3) for t in tactors))
    N_CIRCLE = 24
    for z in z_levels:
        for j in range(N_CIRCLE):
            a0 = 2 * math.pi * j       / N_CIRCLE
            a1 = 2 * math.pi * (j + 1) / N_CIRCLE
            vest_mk.points.append(Point(x=vest_radius * math.cos(a0),
                                        y=vest_radius * math.sin(a0), z=z))
            vest_mk.points.append(Point(x=vest_radius * math.cos(a1),
                                        y=vest_radius * math.sin(a1), z=z))
    ma.markers.append(vest_mk)
    return ma


# ──────────────────────────────────────────────────────────────────────────────
#  Main node
# ──────────────────────────────────────────────────────────────────────────────

class HapticProximityNode:

    def __init__(self):
        rospy.init_node("haptic_proximity_node", anonymous=False)

        # ── Geometry / sensing ───────────────────────────────────────────────
        self.frame_id          = rospy.get_param("~frame_id",          "base_link")
        self.vest_z_offset     = rospy.get_param("~vest_z_offset",     0.0)
        self.vest_height       = rospy.get_param("~vest_height",       0.5)
        self.vest_radius       = rospy.get_param("~vest_radius",       0.2)
        self.ray_range         = rospy.get_param("~ray_range",         2.0)
        self.density_threshold = rospy.get_param("~density_threshold", 0.05)
        self.publish_rate      = rospy.get_param("~publish_rate",      20.0)
        self.min_intensity     = rospy.get_param("~min_intensity",     0.05)
        self.intensity_scale   = rospy.get_param("~intensity_scale",   1.0)
        self.debug_rays        = rospy.get_param("~debug_rays",        True)
        self.z_margin          = rospy.get_param("~z_margin",          0.05)

        # ── Grid geometry ────────────────────────────────────────────────────
        r_min      = rospy.get_param("~grid_r_min",      0.15)
        r_max      = rospy.get_param("~grid_r_max",      8.0)
        n_radial   = rospy.get_param("~grid_n_radial",   8)
        n_angular  = rospy.get_param("~grid_n_angular",  36)
        log_radial = rospy.get_param("~grid_log_radial", True)

        # ── Approach-rate core ────────────────────────────────────────────────
        approach_rate_max    = rospy.get_param("~approach_rate_max",    1.0)
        distance_window_secs = rospy.get_param("~distance_window_secs", 0.3)

        # ── Stage 1: Distance EMA ────────────────────────────────────────────
        dist_ema_alpha = rospy.get_param("~dist_ema_alpha", 0.25)

        # ── Stage 2: Outlier rejection ───────────────────────────────────────
        dist_outlier_zscore = rospy.get_param("~dist_outlier_zscore", 2.0)

        # ── Stage 3: Asymmetric rate EMA ─────────────────────────────────────
        rate_attack_alpha  = rospy.get_param("~rate_attack_alpha",  0.6)
        rate_release_alpha = rospy.get_param("~rate_release_alpha", 0.10)

        # ── Stage 4: Hysteretic gate ─────────────────────────────────────────
        approach_rate_threshold_on  = rospy.get_param("~approach_rate_threshold_on",  0.07)
        approach_rate_threshold_off = rospy.get_param("~approach_rate_threshold_off", 0.03)

        if approach_rate_threshold_off >= approach_rate_threshold_on:
            rospy.logwarn(
                "[HapticProximity] approach_rate_threshold_off must be strictly "
                "less than approach_rate_threshold_on — clamping to on × 0.4")
            approach_rate_threshold_off = approach_rate_threshold_on * 0.4

        # ── Tactor layout ────────────────────────────────────────────────────
        self.tactors = build_tactor_layout(
            self.vest_z_offset, self.vest_height, self.vest_radius)

        rospy.loginfo(
            f"[HapticProximity] {len(self.tactors)} tactors\n"
            f"  Stage 1 dist_ema_alpha      = {dist_ema_alpha:.3f}\n"
            f"  Stage 2 dist_outlier_zscore = {dist_outlier_zscore:.2f}\n"
            f"  Stage 3 rate_attack_alpha   = {rate_attack_alpha:.3f}\n"
            f"          rate_release_alpha  = {rate_release_alpha:.3f}\n"
            f"  Stage 4 gate_on             = {approach_rate_threshold_on:.4f} m/s\n"
            f"          gate_off            = {approach_rate_threshold_off:.4f} m/s\n"
            f"  window = {distance_window_secs:.2f}s   "
            f"rate_max = {approach_rate_max:.2f} m/s"
        )

        # ── Ray caster ───────────────────────────────────────────────────────
        self.ray_caster = DensityRayCaster(
            r_min, r_max, n_radial, n_angular, log_radial)

        # ── Per-tactor distance trackers ─────────────────────────────────────
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
            for _ in self.tactors
        ]

        self._last_cloud: PointCloud2 | None = None

        # ── Publishers ───────────────────────────────────────────────────────
        self.haptic_pub = rospy.Publisher("/bhaptics/play_dot", String, queue_size=5)
        self.stop_pub   = rospy.Publisher("/bhaptics/stop_all", Empty,  queue_size=1)
        if self.debug_rays:
            self.ray_marker_pub = rospy.Publisher(
                "/haptic_proximity/ray_markers", MarkerArray, queue_size=5)

        rospy.Subscriber(
            "/cylindrical_grid/density_cloud", PointCloud2,
            self._on_density_cloud, queue_size=2, buff_size=2**20)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("[HapticProximity] Ready — waiting for density cloud…")

    # ── Callbacks / loop ─────────────────────────────────────────────────────

    def _on_density_cloud(self, msg: PointCloud2):
        self._last_cloud = msg
        self.ray_caster.update(msg)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self._last_cloud is not None:
                self._update()
            rate.sleep()

    def _update(self):
        """
        Per-cycle pipeline:
          1. Cast ray            → raw hit_distance
          2. Four-stage tracker  → approach-rate intensity ∈ [0,1]
          3. Global scale + gate → final intensity
          4. Publish haptics + markers
        """
        now_secs    = rospy.Time.now().to_sec()
        intensities = []
        hit_dists   = []

        for idx, tactor in enumerate(self.tactors):
            _raw_density, hit_dist = self.ray_caster.cast_ray_at_z(
                angle_rad         = tactor["angle_rad"],
                tactor_z          = tactor["z"],
                z_margin          = self.z_margin,
                ray_range         = self.ray_range,
                density_threshold = self.density_threshold,
            )

            approach_intensity = self._trackers[idx].update(hit_dist, now_secs)

            scaled = approach_intensity * self.intensity_scale
            if scaled < self.min_intensity:
                scaled = 0.0

            intensities.append(scaled)
            hit_dists.append(hit_dist)

        self._publish_haptics(intensities)
        if self.debug_rays and hasattr(self, "ray_marker_pub"):
            self.ray_marker_pub.publish(
                build_ray_markers(self.tactors, intensities, hit_dists,
                                  self.ray_range, self.frame_id, self.vest_radius))

    def _publish_haptics(self, intensities: list):
        front_vals = [0] * PANEL_SIZE
        back_vals  = [0] * PANEL_SIZE

        for i, tactor in enumerate(self.tactors):
            val = max(0, min(100, int(round(intensities[i] * 100.0))))
            if tactor["position"] == VEST_FRONT:
                front_vals[tactor["panel_idx"]] = val
            else:
                back_vals[tactor["panel_idx"]]  = val

        duration_ms = int(1000.0 / self.publish_rate * 1.5)

        for position, vals in [(VEST_FRONT, front_vals), (VEST_BACK, back_vals)]:
            if any(v > 0 for v in vals):
                self.haptic_pub.publish(String(data=json.dumps({
                    "position":     position,
                    "duration_ms":  duration_ms,
                    "values":       vals,
                    "device_index": -1,
                })))

    def _on_shutdown(self):
        rospy.loginfo("[HapticProximity] Shutdown — stopping all haptics")
        try:
            self.stop_pub.publish(Empty())
        except Exception:
            pass


# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    try:
        node = HapticProximityNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass