#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from datetime import datetime
import csv
import os

RESULT_FILE = "/home/hzq/桌面/测试结果/latest_detection.csv"
CONTEXT_FILE = "/home/hzq/桌面/文字稿/latest_context.csv"

RESULT_HEADER = ["timestamp", "detection_1", "detection_2", "detection_3"]
CONTEXT_HEADER = ["timestamp", "scene_class", "random_role", "speed", "distance", "location"]


# ================= 工具函数 =================
def ensure_dir(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)


def append_csv(path, header, row):
    ensure_dir(path)
    file_exists = os.path.isfile(path)

    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        writer.writerow(row)


# ================= 回调函数 =================
def result_cb(msg):
    text = msg.data
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    parts = text.split("/")

    if len(parts) != 8:
        rospy.logwarn(
            f"[RecklessListener] invalid format "
            f"(expect 7 fields, got {len(parts)}): {text}"
        )
        return

    # -------- 前 3 个 → RESULT_FILE --------
    detection_1, detection_2, detection_3 = parts[0:3]

    append_csv(
        RESULT_FILE,
        RESULT_HEADER,
        [timestamp, detection_1, detection_2, detection_3]
    )

    # -------- 后 4 个 → CONTEXT_FILE --------
    scene_class, random_role, speed, distance, location = parts[3:8]

    append_csv(
        CONTEXT_FILE,
        CONTEXT_HEADER,
        [timestamp, scene_class, random_role, speed, distance, location]
    )

    rospy.loginfo(
        f"[RecklessListener] "
        f"result=({detection_1},{detection_2},{detection_3})"
    )


# ================= 主函数 =================
def main():
    rospy.init_node("reckless_detection_listener", anonymous=True)

    rospy.Subscriber("/reckless_detection/result", String, result_cb)

    rospy.loginfo("[RecklessListener] Started.")
    rospy.loginfo(f"  - result  -> {RESULT_FILE}")
    rospy.spin()


if __name__ == "__main__":
    main()
