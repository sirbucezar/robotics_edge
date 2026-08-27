#!/usr/bin/env python3
"""Interactive A/B capture rig for the camera-height experiment.

Walks you through a fixed list of positions one at a time. For each: prints
where to put the robot, waits for you to press Enter, counts down so your
hands are out of frame, then grabs N distinct frames and writes them as JPEG.

Saves frames directly rather than recording bags -- a bag would need a second
extraction pass on the Mac, and this way you see immediately whether the shot
is usable (the sharpness/brightness readout catches motion blur and a dark
room before you have moved the robot away).

Runs standalone: no build, no launch file, nothing added to the graph beyond
one subscriber. Deliberately does NOT depend on cv_bridge -- same reason as
imgmsg_to_bgr in yolo_detector_node.py (numpy ABI breakage on Foxy).

    python3 ab_capture.py                  # full run
    python3 ab_capture.py --start 3        # resume at position 3
    python3 ab_capture.py --only 5 6       # redo just those two
"""

import argparse
import datetime
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# Height is the thing under test, so it is spelled out in the banner every
# time -- getting position 4 at the wrong height silently ruins the pair.
LOW_CM = 18
HIGH_CM = 75

POSITIONS = [
    dict(n=1, slug="pos1_mouth_low",
         where="Corridor, at a row MOUTH",
         aim="down the gap",
         height="%d cm - ON THE FLOOR" % LOW_CM),
    dict(n=2, slug="pos2_mouth_high",
         where="SAME spot as position 1 (do not move the robot on the floor)",
         aim="down the gap",
         height="~%d cm - ON A CHAIR / BOX" % HIGH_CM),
    dict(n=3, slug="pos3_ingap_low",
         where="~2 m INTO the gap",
         aim="down the gap",
         height="%d cm - ON THE FLOOR" % LOW_CM),
    dict(n=4, slug="pos4_ingap_high",
         where="SAME spot as position 3",
         aim="down the gap",
         height="~%d cm - ON A CHAIR / BOX" % HIGH_CM),
    dict(n=5, slug="pos5_person_low",
         where="~1 m from a SEATED person",
         aim="at them",
         height="%d cm - ON THE FLOOR" % LOW_CM),
    dict(n=6, slug="pos6_person_high",
         where="SAME spot as position 5",
         aim="at them",
         height="~%d cm - ON A CHAIR / BOX" % HIGH_CM),
    dict(n=7, slug="pos7_empty_low",
         where="An EMPTY gap - NO PEOPLE IN FRAME AT ALL",
         aim="down the gap",
         height="%d cm - ON THE FLOOR" % LOW_CM,
         note="Hard negatives. This is the clutter floor: chair legs, table "
              "legs, bags. If a person wanders into frame the shot is void."),
]

BANNER = "=" * 72


def imgmsg_to_bgr(msg):
    """cv_bridge-free conversion. Same encodings as yolo_detector_node."""
    import cv2
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return buf.reshape(h, w, 3)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
    raise ValueError("unsupported image encoding %r" % msg.encoding)


class FrameGrabber(Node):
    """Holds only the newest frame. We never want a queued stale one -- the
    robot may have been picked up and moved since it was published."""

    def __init__(self, topic, qos_reliability):
        super().__init__("ab_capture")
        rel = (ReliabilityPolicy.BEST_EFFORT if qos_reliability == "best_effort"
               else ReliabilityPolicy.RELIABLE)
        qos = QoSProfile(depth=1, reliability=rel,
                         history=HistoryPolicy.KEEP_LAST)
        self._lock = threading.Lock()
        self._msg = None
        self._count = 0
        self.create_subscription(Image, topic, self._cb, qos)

    def _cb(self, msg):
        with self._lock:
            self._msg = msg
            self._count += 1

    def latest(self):
        with self._lock:
            return self._msg, self._count

    def wait_for_stream(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg, _ = self.latest()
            if msg is not None:
                return msg
            time.sleep(0.1)
        return None


def stamp_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def sharpness(bgr):
    """Variance of Laplacian. Low value = motion blur or out of focus. Only
    meaningful compared against the other shots in this same run."""
    import cv2
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def ask(prompt):
    """input() that treats a closed stdin as 'quit' rather than a traceback,
    so a piped or Ctrl-D'd run still writes its manifest."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print("\n(stdin closed - quitting)")
        return "q"


def countdown(seconds):
    for i in range(seconds, 0, -1):
        sys.stdout.write("\r  starting in %d ... (hands out of frame)   " % i)
        sys.stdout.flush()
        time.sleep(1.0)
    sys.stdout.write("\r  CAPTURING                                     \n")
    sys.stdout.flush()


def capture_position(grabber, pos, outdir, n_frames, duration, delay):
    """Grab n_frames distinct frames spread over `duration` seconds."""
    import cv2

    countdown(delay)

    # Everything published before this instant is stale: it may show your
    # hands, or the robot mid-placement. The grabber keeps only the newest
    # frame, but "newest" can still predate the countdown if the camera has
    # stalled -- so require the frame counter to have advanced past here.
    _, baseline = grabber.latest()

    frames = []
    seen = set()
    # Spread the grabs across the FULL duration, first at t=0 and last at
    # t=duration, so the samples actually span the window.
    interval = duration / float(n_frames - 1) if n_frames > 1 else 0.0
    t_start = time.time()
    for i in range(n_frames):
        target = t_start + i * interval
        while time.time() < target:
            time.sleep(0.01)
        # Wait for a frame we have not already saved, so a stalled publisher
        # cannot give us N copies of one image.
        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg, count = grabber.latest()
            if (msg is not None and count > baseline
                    and stamp_key(msg) not in seen):
                seen.add(stamp_key(msg))
                frames.append(msg)
                break
            time.sleep(0.01)
        else:
            print("  ! no fresh frame for shot %d - camera stalled?" % (i + 1))

    if not frames:
        return None
    if len(frames) < n_frames:
        print("  ! only got %d of %d frames - camera is dropping."
              % (len(frames), n_frames))

    posdir = os.path.join(outdir, pos["slug"])
    # Clear first: a redo with fewer frames would otherwise leave higher-
    # numbered JPEGs from the previous take sitting next to the new ones.
    if os.path.isdir(posdir):
        for stale in os.listdir(posdir):
            if stale.endswith(".jpg"):
                os.remove(os.path.join(posdir, stale))
    os.makedirs(posdir, exist_ok=True)

    written = []
    for i, msg in enumerate(frames):
        bgr = imgmsg_to_bgr(msg)
        path = os.path.join(posdir, "%s_%02d.jpg" % (pos["slug"], i))
        cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        written.append(dict(
            path=path,
            sharpness=round(sharpness(bgr), 1),
            mean_brightness=round(float(bgr.mean()), 1),
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            width=msg.width, height=msg.height, encoding=msg.encoding,
        ))

    print("\n  saved %d frames -> %s" % (len(written), posdir))
    print("  %-28s %10s %10s" % ("file", "sharpness", "brightness"))
    for w in written:
        print("  %-28s %10.1f %10.1f"
              % (os.path.basename(w["path"]), w["sharpness"],
                 w["mean_brightness"]))

    sharps = [w["sharpness"] for w in written]
    bright = [w["mean_brightness"] for w in written]
    if max(sharps) < 40:
        print("  ! WARNING: all frames look SOFT (blur or out of focus).")
        print("    Hold still, make sure the robot is not rocking, redo with r.")
    if max(bright) < 45:
        print("  ! WARNING: very DARK. Lights on? Redo with r.")
    elif min(bright) > 225:
        print("  ! WARNING: likely BLOWN OUT (window behind subject?).")

    return dict(position=pos["n"], slug=pos["slug"], where=pos["where"],
                aim=pos["aim"], height=pos["height"], frames=written,
                captured_at=datetime.datetime.now().isoformat(timespec="seconds"))


def show_banner(pos, idx, total):
    print("\n" + BANNER)
    print("POSITION %d of %d   [%s]" % (idx, total, pos["slug"]))
    print(BANNER)
    print("  WHERE : %s" % pos["where"])
    print("  AIM   : %s" % pos["aim"])
    print("  HEIGHT: %s" % pos["height"])
    if pos.get("note"):
        print("  NOTE  : %s" % pos["note"])
    if pos["n"] != 7:
        print("  PEOPLE: 2-3 seated, one further down the row.")
    print(BANNER)


def main():
    ap = argparse.ArgumentParser(description="A/B camera-height capture rig")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--qos", default="best_effort",
                    choices=["best_effort", "reliable"])
    ap.add_argument("--out", default=os.path.expanduser(
        "~/limo_project/data/ab_test"))
    ap.add_argument("--frames", type=int, default=4,
                    help="frames saved per position (default 4, pick the best later)")
    ap.add_argument("--secs", type=float, default=5.0,
                    help="seconds to spread the frames over (default 5)")
    ap.add_argument("--delay", type=int, default=3,
                    help="countdown before capture, to clear your hands (default 3)")
    ap.add_argument("--start", type=int, default=1,
                    help="start at this position number")
    ap.add_argument("--only", type=int, nargs="+",
                    help="run only these position numbers")
    args = ap.parse_args()

    if args.only:
        todo = [p for p in POSITIONS if p["n"] in args.only]
    else:
        todo = [p for p in POSITIONS if p["n"] >= args.start]
    if not todo:
        print("nothing to do")
        return 1

    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    grabber = FrameGrabber(args.topic, args.qos)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(grabber,), daemon=True)
    spin_thread.start()

    print("\nwaiting for frames on %s (%s) ..." % (args.topic, args.qos))
    first = grabber.wait_for_stream(timeout=10.0)
    if first is None:
        print("\nNO FRAMES on %s after 10 s." % args.topic)
        print("Start the camera first:")
        print("  ros2 run limo_perception color_camera_node")
        rclpy.shutdown()
        return 1
    print("stream live: %dx%d %s\n" % (first.width, first.height, first.encoding))

    results = []
    i = 0
    try:
        while i < len(todo):
            pos = todo[i]
            show_banner(pos, i + 1, len(todo))
            ans = ask("  [Enter]=capture  s=skip  q=quit : ")
            if ans == "q":
                break
            if ans == "s":
                i += 1
                continue

            rec = capture_position(grabber, pos, args.out,
                                   args.frames, args.secs, args.delay)
            if rec is None:
                print("  ! captured nothing. Camera alive?")
                continue

            nxt = ask("\n  [Enter]=next  r=redo this one  q=quit : ")
            if nxt == "r":
                continue          # same index, shoot it again (overwrites)
            results.append(rec)
            if nxt == "q":
                break
            i += 1
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        manifest = os.path.join(args.out, "manifest.json")
        old = []
        if os.path.exists(manifest):
            try:
                with open(manifest) as fh:
                    old = json.load(fh).get("positions", [])
            except ValueError:
                old = []
        # Later runs of the same position win, so --only re-shoots cleanly.
        merged = dict((r["slug"], r) for r in old)
        for r in results:
            merged[r["slug"]] = r
        with open(manifest, "w") as fh:
            json.dump(dict(
                topic=args.topic,
                frames_per_position=args.frames,
                positions=[merged[k] for k in sorted(merged)],
            ), fh, indent=2)

        print("\n" + BANNER)
        print("captured %d position(s) this run" % len(results))
        print("manifest: %s" % manifest)
        done = sorted(merged)
        print("have on disk: %s" % (", ".join(done) if done else "nothing"))
        missing = [p["slug"] for p in POSITIONS if p["slug"] not in merged]
        if missing:
            print("STILL MISSING: %s" % ", ".join(missing))
        print(BANNER)

        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
