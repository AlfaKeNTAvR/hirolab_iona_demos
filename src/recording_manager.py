#!/usr/bin/env python3
"""
recording_manager_node.py  (optimized)
=======================================
Manages rosbag recording lifecycle for the IONA system.

Key optimizations vs. original:
  - Unlimited write buffer (--buffsize 0) eliminates dropped messages
  - Large chunk size (--chunksize 8192) reduces small-write pressure on external drives
  - Automatic bag splitting (--split --size) prevents total loss on crash/full disk
  - --min-space guard stops recording before the drive fills up
  - --tcpnodelay lowers subscriber latency for high-frequency topics
  - Optional ionice priority boosts rosbag I/O above other processes (no root needed)
  - Optional automatic substitution of raw image topics with compressed equivalents
  - Pre-flight checks: drive mount detection + free-space threshold
  - rosbag stdout/stderr captured to a per-bag .log file for post-hoc diagnosis
  - Process watchdog detects unexpected rosbag exits and clears state
  - Graceful shutdown uses SIGINT with a long flush timeout before escalating

Subscribes:
    ~cmd_topic   (std_msgs/String, default /recording/cmd)
        JSON commands:
            {"action":"start","experiment":"fitts","participant":"P01",
             "conditions":[{"key":"camera","value":"Front"}, ...],
             "topics":["/tf","/joint_states", ...]}
            {"action":"stop"}

Publishes:
    ~topics_topic  (std_msgs/String, default /recording/topics, latched)
        {"topics":[{"name":"/tf","type":"tf2_msgs/TFMessage"}, ...]}
    ~status_topic  (std_msgs/String, default /recording/status)
        {"recording":true, "filename":"...", "startedAt":"...", "freeSpaceMb":...}
        {"recording":false,"lastFilename":"...","durationSeconds":12.3}
        {"recording":<bool>,"error":"..."}

Params:
    ~bag_path              (str)   Directory for .bag files.         Default: ~/iona_bags
    ~cache_file            (str)   Path to JSON config cache.        Default: <bag_path>/.last_recording_config.json
    ~topics_poll_rate      (float) Hz to republish topic list.       Default: 0.5
    ~buf_mb                (int)   rosbag write buffer in MB.        Default: 0  (unlimited)
    ~chunk_kb              (int)   rosbag chunk size in KB.          Default: 8192  (8 MB)
    ~split_mb              (int)   Split bags every N MB.            Default: 4096  (4 GB)
    ~min_space_mb          (int)   rosbag --min-space threshold MB.  Default: 10240 (10 GB)
    ~min_space_start_mb    (int)   Free space required to START.     Default: 20480 (20 GB)
    ~stop_timeout          (float) Seconds to wait for SIGINT flush. Default: 60.0
    ~prefer_compressed     (bool)  Auto-swap raw image → compressed. Default: true
    ~lz4                   (bool)  Enable lz4 compression.           Default: false
    ~use_ionice            (bool)  Wrap cmd with ionice priority.    Default: true
    ~warn_raw_images       (bool)  Warn when raw image topics found. Default: true
"""

import json
import os
import re
import shutil
import signal
import subprocess
import time
from datetime import datetime

import rospy
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Image topic suffixes that have a compressed counterpart
_RAW_SUFFIXES = (
    "/image_raw",
    "/image_color",
    "/image_rect",
    "/image_rect_color",
    "/image_rect_raw",
)


def sanitize(s):
    """Strip a string to filesystem-safe characters."""
    if s is None:
        return ""
    s = re.sub(r"[^A-Za-z0-9_\-]+", "_", str(s).strip())
    return s.strip("_") or "x"


def to_compressed(topic):
    """Return the /compressed variant of a raw image topic, or topic unchanged."""
    for suffix in _RAW_SUFFIXES:
        if topic.endswith(suffix):
            return topic + "/compressed"
    return topic


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class RecordingManagerNode:

    def __init__(self):
        # ── Paths ────────────────────────────────────────────────────────
        self.bag_path = os.path.expanduser(
            rospy.get_param("~bag_path", "~/iona_bags"))
        os.makedirs(self.bag_path, exist_ok=True)

        default_cache = os.path.join(self.bag_path, ".last_recording_config.json")
        self.cache_file = os.path.expanduser(
            rospy.get_param("~cache_file", default_cache))

        # ── Tuning params ────────────────────────────────────────────────
        self.buf_mb             = int(rospy.get_param("~buf_mb",             0))
        self.chunk_kb           = int(rospy.get_param("~chunk_kb",           8192))
        self.split_mb           = int(rospy.get_param("~split_mb",           4096))
        self.min_space_mb       = int(rospy.get_param("~min_space_mb",       10240))
        self.min_space_start_mb = int(rospy.get_param("~min_space_start_mb", 20480))
        self.stop_timeout       = float(rospy.get_param("~stop_timeout",     60.0))
        self.prefer_compressed  = bool(rospy.get_param("~prefer_compressed", True))
        self.lz4                = bool(rospy.get_param("~lz4",               False))
        self.use_ionice         = bool(rospy.get_param("~use_ionice",        True))
        self.warn_raw_images    = bool(rospy.get_param("~warn_raw_images",   True))

        # ── ROS topics / timers ──────────────────────────────────────────
        cmd_topic    = rospy.get_param("~cmd_topic",    "/recording/cmd")
        topics_topic = rospy.get_param("~topics_topic", "/recording/topics")
        status_topic = rospy.get_param("~status_topic", "/recording/status")
        poll_rate    = float(rospy.get_param("~topics_poll_rate", 0.5))

        self.topics_pub = rospy.Publisher(topics_topic, String, queue_size=1,  latch=True)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=10)
        self.cmd_sub    = rospy.Subscriber(cmd_topic,   String, self._on_cmd, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / poll_rate), self._publish_topics)
        rospy.Timer(rospy.Duration(5.0),             self._watchdog)
        rospy.on_shutdown(self._on_shutdown)

        # ── State ────────────────────────────────────────────────────────
        self.proc             = None   # subprocess.Popen for active rosbag
        self._log_file        = None   # open file handle for rosbag output
        self.current_filename = None
        self.start_time       = None
        self.cached_config    = self._load_cache()

        rospy.loginfo(
            "[recording_manager] ready — bag_path=%s  buf=%s MB  chunk=%d KB  "
            "split=%d MB  min_space=%d MB",
            self.bag_path,
            "unlimited" if self.buf_mb == 0 else str(self.buf_mb),
            self.chunk_kb, self.split_mb, self.min_space_mb,
        )

    # ── Cache ────────────────────────────────────────────────────────────

    def _load_cache(self):
        if os.path.isfile(self.cache_file):
            try:
                with open(self.cache_file) as f:
                    return json.load(f)
            except Exception as e:
                rospy.logwarn("[recording_manager] cache read failed: %s", e)
        return {"experiment": "", "participant": "", "conditions": [], "topics": []}

    def _save_cache(self):
        try:
            with open(self.cache_file, "w") as f:
                json.dump(self.cached_config, f, indent=2)
        except Exception as e:
            rospy.logwarn("[recording_manager] cache write failed: %s", e)

    def _merge_with_cache(self, msg):
        c = dict(self.cached_config)
        return {
            "experiment":  msg.get("experiment")  or c.get("experiment",  ""),
            "participant": msg.get("participant") or c.get("participant", ""),
            "conditions":  msg.get("conditions")  or c.get("conditions",  []),
            "topics":      msg.get("topics")      or c.get("topics",      []),
        }

    # ── Pre-flight checks ────────────────────────────────────────────────

    def _is_external_drive(self):
        """True if bag_path is on a different device than / (i.e. a mounted drive)."""
        try:
            return os.stat(self.bag_path).st_dev != os.stat("/").st_dev
        except Exception as e:
            rospy.logwarn("[recording_manager] mount check failed: %s", e)
            return True  # non-blocking if we can't determine

    def _free_space_mb(self):
        try:
            return shutil.disk_usage(self.bag_path).free // (1024 * 1024)
        except Exception as e:
            rospy.logwarn("[recording_manager] disk usage check failed: %s", e)
            return float("inf")

    # ── Topic list ───────────────────────────────────────────────────────

    def _publish_topics(self, _event):
        try:
            pub = rospy.get_published_topics()
        except Exception as e:
            rospy.logwarn("[recording_manager] get_published_topics failed: %s", e)
            return
        topics = sorted(
            ({"name": n, "type": t} for n, t in pub),
            key=lambda x: x["name"],
        )
        self.topics_pub.publish(String(data=json.dumps({"topics": topics})))

    # ── Process watchdog ─────────────────────────────────────────────────

    def _watchdog(self, _event):
        """Detect unexpected rosbag exits and reset state."""
        if self.proc is None:
            return
        rc = self.proc.poll()
        if rc is None:
            return  # still running

        rospy.logerr(
            "[recording_manager] rosbag exited unexpectedly (rc=%d) — "
            "bag may be incomplete: %s", rc, self.current_filename,
        )
        self._cleanup_proc()
        self._publish_status(
            recording=False,
            error="rosbag exited unexpectedly (rc=%d)" % rc,
            lastFilename=self.current_filename or "",
        )

    # ── Command dispatch ─────────────────────────────────────────────────

    def _on_cmd(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[recording_manager] bad JSON: %s", e)
            self._publish_status(error="bad JSON: %s" % e)
            return

        action = data.get("action")
        if action == "start":
            self._handle_start(data)
        elif action == "stop":
            self._handle_stop()
        else:
            rospy.logwarn("[recording_manager] unknown action: %s", action)
            self._publish_status(error="unknown action: %s" % action)

    # ── Start recording ──────────────────────────────────────────────────

    def _handle_start(self, data):
        if self.proc is not None and self.proc.poll() is None:
            rospy.logwarn("[recording_manager] start ignored — already recording")
            self._publish_status(recording=True, filename=self.current_filename,
                                  error="already recording")
            return

        merged = self._merge_with_cache(data)
        topics = merged.get("topics") or []

        if not topics:
            rospy.logwarn("[recording_manager] no topics — refusing to start")
            self._publish_status(recording=False, error="no topics selected")
            return

        # ── Pre-flight: drive mount ──────────────────────────────────────
        if not self._is_external_drive():
            rospy.logwarn(
                "[recording_manager] bag_path=%s appears to be on the root filesystem. "
                "External drive may not be mounted. Recording anyway, but verify.",
                self.bag_path,
            )

        # ── Pre-flight: disk space ───────────────────────────────────────
        free_mb = self._free_space_mb()
        if free_mb < self.min_space_start_mb:
            msg = ("insufficient disk space: %.1f GB free, need %.1f GB to start"
                   % (free_mb / 1024.0, self.min_space_start_mb / 1024.0))
            rospy.logerr("[recording_manager] %s", msg)
            self._publish_status(recording=False, error=msg, freeSpaceMb=free_mb)
            return

        # ── Warn about raw image topics ──────────────────────────────────
        if self.warn_raw_images:
            raw = [t for t in topics if any(t.endswith(s) for s in _RAW_SUFFIXES)]
            if raw:
                rospy.logwarn(
                    "[recording_manager] %d raw image topic(s) detected — very high "
                    "bandwidth. %s. Topics: %s",
                    len(raw),
                    "Switching to /compressed variants" if self.prefer_compressed
                    else "Consider ~prefer_compressed:=true or record compressed topics",
                    raw,
                )

        # ── Optionally swap raw → compressed ────────────────────────────
        if self.prefer_compressed:
            topics = [to_compressed(t) for t in topics]

        # ── Build output path ────────────────────────────────────────────
        self.cached_config = merged
        self._save_cache()

        experiment  = sanitize(merged.get("experiment")  or "experiment")
        participant = sanitize(merged.get("participant") or "unknown")
        cond_parts  = [sanitize(c.get("value", ""))
                       for c in (merged.get("conditions") or [])
                       if c.get("value")]
        timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")

        base     = "-".join(p for p in [experiment, participant] + cond_parts + [timestamp] if p)
        bag_path = os.path.join(self.bag_path, base + ".bag")
        log_path = os.path.join(self.bag_path, base + "_rosbag.log")

        # ── Build rosbag command ─────────────────────────────────────────
        # Note: with --split, rosbag appends _NNN to the base, yielding
        # base_0.bag, base_1.bag, etc.
        cmd = [
            "rosbag", "record",
            "-O", bag_path,
            "--buffsize",  str(self.buf_mb),    # 0 = unlimited in-memory buffer
            "--chunksize", str(self.chunk_kb),  # KB per disk flush
            "--split", "--size", str(self.split_mb),  # split every N MB
            "--min-space", str(self.min_space_mb),    # stop before drive fills
            "--tcpnodelay",                     # disable Nagle for subscribers
        ]
        if self.lz4:
            cmd.append("--lz4")
        cmd += list(topics) + ["__name:=iona_recording_manager_bag"]

        # ionice: best-effort class (-c 2), highest level (-n 0); no root needed
        if self.use_ionice and shutil.which("ionice"):
            cmd = ["ionice", "-c", "2", "-n", "0"] + cmd

        rospy.loginfo("[recording_manager] cmd: %s", " ".join(cmd))
        rospy.loginfo("[recording_manager] rosbag log -> %s", log_path)

        # ── Launch ───────────────────────────────────────────────────────
        try:
            log_f = open(log_path, "w")
            self.proc = subprocess.Popen(cmd, stdout=log_f, stderr=log_f)
            self._log_file = log_f
        except Exception as e:
            rospy.logerr("[recording_manager] failed to launch rosbag: %s", e)
            self._publish_status(recording=False, error="failed to start: %s" % e)
            self.proc = None
            return

        self.current_filename = base + ".bag"
        self.start_time = time.time()
        rospy.loginfo("[recording_manager] recording started -> %s  (%.1f GB free)",
                      bag_path, free_mb / 1024.0)
        self._publish_status(
            recording=True,
            filename=self.current_filename,
            startedAt=timestamp,
            freeSpaceMb=free_mb,
            preferCompressed=self.prefer_compressed,
        )

    # ── Stop recording ───────────────────────────────────────────────────

    def _handle_stop(self):
        if self.proc is None or self.proc.poll() is not None:
            rospy.logwarn("[recording_manager] stop ignored — not recording")
            self._publish_status(recording=False, error="not recording")
            self.proc = None
            return

        rospy.loginfo(
            "[recording_manager] stopping — sending SIGINT, waiting up to %.0fs for flush…",
            self.stop_timeout,
        )
        try:
            self.proc.send_signal(signal.SIGINT)
            self.proc.wait(timeout=self.stop_timeout)
        except subprocess.TimeoutExpired:
            rospy.logwarn(
                "[recording_manager] rosbag did not flush in %.0fs — sending SIGTERM. "
                "Bag may be incomplete.", self.stop_timeout,
            )
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                rospy.logerr(
                    "[recording_manager] SIGTERM ignored — sending SIGKILL. "
                    "Bag is likely CORRUPT: %s", self.current_filename,
                )
                self.proc.kill()
                self.proc.wait()
        except Exception as e:
            rospy.logwarn("[recording_manager] error stopping rosbag: %s", e)

        duration      = time.time() - self.start_time if self.start_time else 0.0
        last_filename = self.current_filename
        self._cleanup_proc()

        rospy.loginfo("[recording_manager] stopped -> %s (%.1fs)", last_filename, duration)
        self._publish_status(
            recording=False,
            lastFilename=last_filename,
            durationSeconds=round(duration, 2),
        )

    # ── Internals ────────────────────────────────────────────────────────

    def _cleanup_proc(self):
        """Close log file and clear process state."""
        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None
        self.proc             = None
        self.current_filename = None
        self.start_time       = None

    def _publish_status(self, **kwargs):
        self.status_pub.publish(String(data=json.dumps(kwargs)))

    def _on_shutdown(self):
        if self.proc is not None and self.proc.poll() is None:
            rospy.loginfo("[recording_manager] node shutting down — stopping active recording")
            self._handle_stop()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rospy.init_node("recording_manager", anonymous=False)
    RecordingManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
