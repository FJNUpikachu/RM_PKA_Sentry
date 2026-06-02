#!/usr/bin/env python3
import argparse
import os
import sys
from dataclasses import dataclass
from typing import Optional, Tuple


def _require(module: str, hint: str):
    try:
        return __import__(module)
    except Exception as e:
        raise RuntimeError(f"Missing python module '{module}'. {hint}\nOriginal: {e}") from e


cv2 = _require("cv2", "Install OpenCV (e.g. apt: python3-opencv, or pip: opencv-python).")
np = _require("numpy", "Install numpy (apt: python3-numpy, or pip: numpy).")

rosbag2_py = _require("rosbag2_py", "Make sure you sourced ROS 2 and installed rosbag2_py.")
rclpy_serialization = _require("rclpy.serialization", "Make sure you have rclpy installed.")
rosidl_runtime_py = _require("rosidl_runtime_py.utilities", "Make sure rosidl_runtime_py is installed.")


@dataclass
class Frame:
    ts_ns: int
    img: "np.ndarray"


def decode_image(msg) -> "np.ndarray":
    # sensor_msgs/msg/Image
    if hasattr(msg, "encoding") and hasattr(msg, "data"):
        enc = (msg.encoding or "").lower()
        h = int(msg.height)
        w = int(msg.width)
        if h <= 0 or w <= 0:
            raise ValueError(f"Invalid image size: {w}x{h}")

        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            img = buf.reshape((h, w, 3))
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img
        if enc in ("mono8",):
            img = buf.reshape((h, w))
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if enc in ("rgba8", "bgra8"):
            img = buf.reshape((h, w, 4))
            if enc == "rgba8":
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            else:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            return img

        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    # sensor_msgs/msg/CompressedImage
    if hasattr(msg, "format") and hasattr(msg, "data"):
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"Failed to decode compressed image (format={getattr(msg, 'format', '')})")
        return img

    raise ValueError(f"Unsupported message type: {type(msg)}")


def open_reader(bag_dir: str, storage_id: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                                    output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def infer_fps_from_timestamps(ts_ns_list) -> Optional[float]:
    if len(ts_ns_list) < 5:
        return None
    deltas = []
    last = ts_ns_list[0]
    for t in ts_ns_list[1:]:
        d = t - last
        if d > 0:
            deltas.append(d)
        last = t
    if not deltas:
        return None
    deltas.sort()
    median = deltas[len(deltas) // 2]
    if median <= 0:
        return None
    return 1e9 / float(median)


def main():
    ap = argparse.ArgumentParser(description="Convert a rosbag2 topic to MP4.")
    ap.add_argument("--bag", required=True, help="rosbag2 folder path (contains metadata.yaml)")
    ap.add_argument("--topic", required=True, help="Image topic (Image or CompressedImage)")
    ap.add_argument("--out", required=True, help="Output mp4 path")
    ap.add_argument("--storage", default="sqlite3", help="Storage id: sqlite3 (db3) or mcap")
    ap.add_argument("--fps", type=float, default=0.0, help="Output FPS. 0 = infer from timestamps")
    ap.add_argument("--fourcc", default="mp4v", help="FourCC codec, e.g. mp4v or avc1")
    ap.add_argument("--max_frames", type=int, default=0, help="Limit frames (0 = no limit)")
    args = ap.parse_args()

    bag_dir = os.path.expanduser(args.bag)
    out_path = os.path.expanduser(args.out)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    reader = open_reader(bag_dir, args.storage)

    # Find topic type
    topic_type = None
    for t in reader.get_all_topics_and_types():
        if t.name == args.topic:
            topic_type = t.type
            break
    if topic_type is None:
        available = [t.name for t in reader.get_all_topics_and_types()]
        raise RuntimeError(f"Topic not found: {args.topic}\nAvailable topics: {available}")

    msg_cls = rosidl_runtime_py.utilities.get_message(topic_type)

    frames_written = 0
    writer = None
    out_fps = float(args.fps)
    buffered_ts = []
    buffered_frames = []

    def ensure_writer(img_bgr, fps):
        nonlocal writer
        if writer is not None:
            return
        h, w = img_bgr.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
        writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h), True)
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter: {out_path} (fourcc={args.fourcc}, fps={fps})")

    # Buffer a short window to infer fps (if needed) and lock video size
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic != args.topic:
            continue

        msg = rclpy_serialization.deserialize_message(data, msg_cls)
        img = decode_image(msg)
        buffered_ts.append(int(ts_ns))
        buffered_frames.append(Frame(ts_ns=int(ts_ns), img=img))

        if len(buffered_frames) >= 30:
            break

    if not buffered_frames:
        raise RuntimeError(f"No messages read from topic: {args.topic}")

    if out_fps <= 0.0:
        inferred = infer_fps_from_timestamps(buffered_ts)
        out_fps = inferred if inferred and inferred > 0 else 30.0

    first_img = buffered_frames[0].img
    ensure_writer(first_img, out_fps)

    base_h, base_w = first_img.shape[:2]

    def write_frame(img):
        nonlocal frames_written
        if img.shape[0] != base_h or img.shape[1] != base_w:
            img = cv2.resize(img, (base_w, base_h), interpolation=cv2.INTER_AREA)
        writer.write(img)
        frames_written += 1

    for fr in buffered_frames:
        write_frame(fr.img)
        if args.max_frames and frames_written >= args.max_frames:
            break

    # Continue streaming
    while reader.has_next():
        if args.max_frames and frames_written >= args.max_frames:
            break
        topic, data, ts_ns = reader.read_next()
        if topic != args.topic:
            continue
        msg = rclpy_serialization.deserialize_message(data, msg_cls)
        img = decode_image(msg)
        write_frame(img)

    if writer is not None:
        writer.release()

    print(f"[bag_to_mp4] wrote {frames_written} frames -> {out_path} (fps={out_fps:.2f})")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise
    except Exception as e:
        print(f"[bag_to_mp4] ERROR: {e}", file=sys.stderr)
        sys.exit(1)

