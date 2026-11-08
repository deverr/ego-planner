#!/usr/bin/env python3
"""按固定时延转发消息，模拟链路时延。

支持两类：
  --type cloud   sensor_msgs/PointCloud2（感知时延）
  --type odom    nav_msgs/Odometry（状态估计时延）

消息保留原始 header.stamp —— 也就是「数据晚到但时间戳是对的」，这正是真实
传感器/状态估计链路的延迟形态。delay=0 时直接透传，用作基线。
"""
import argparse
import heapq
import random
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

TYPES = {"cloud": PointCloud2, "odom": Odometry}


class DelayRelay(object):
    def __init__(self, args):
        cls = TYPES[args.type]
        self.delay = args.delay_ms / 1000.0
        self.jitter = args.jitter_ms / 1000.0
        self.heap = []          # (release_time, seq, msg)
        self.seq = 0
        self.last_release = 0.0
        self.pub = rospy.Publisher(args.out, cls, queue_size=50)
        rospy.Subscriber(args.inp, cls, self.cb, queue_size=100)
        if self.delay > 0.0 or self.jitter > 0.0:
            rospy.Timer(rospy.Duration(0.002), self.flush)
        rospy.loginfo("%s delay relay: %s -> %s, %.0f ms +- %.0f ms",
                      args.type, args.inp, args.out, args.delay_ms, args.jitter_ms)
        if self.jitter > self.delay:
            eff = (self.delay + self.jitter) ** 2 / (2.0 * self.jitter) \
                if self.jitter > 0 else self.delay
            rospy.logwarn(
                "jitter(%.0f ms) > delay(%.0f ms): negative samples are clamped "
                "to 0, so the effective mean delay is ~%.1f ms, not %.0f ms",
                args.jitter_ms, args.delay_ms, eff * 1000.0, args.delay_ms)

    def cb(self, msg):
        if self.delay <= 0.0 and self.jitter <= 0.0:
            self.pub.publish(msg)
            return
        d = self.delay
        if self.jitter > 0.0:
            # 每条消息独立抽样：这才是真实链路的形态（固定时延是它的特例）
            d = max(0.0, d + random.uniform(-self.jitter, self.jitter))
        self.seq += 1
        release = rospy.Time.now().to_sec() + d
        # 保持发布顺序单调：下游按时间戳做插值，乱序会让指标算错
        release = max(release, self.last_release)
        self.last_release = release
        heapq.heappush(self.heap, (release, self.seq, msg))

    def flush(self, _event):
        now = rospy.Time.now().to_sec()
        while self.heap and self.heap[0][0] <= now:
            self.pub.publish(heapq.heappop(self.heap)[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", choices=sorted(TYPES), default="cloud")
    ap.add_argument("--in", dest="inp", default="/pcl_render_node/cloud")
    ap.add_argument("--out", dest="out", default="/delayed/msg")
    ap.add_argument("--delay-ms", type=float, default=0.0)
    ap.add_argument("--jitter-ms", type=float, default=0.0,
                    help="每条消息在 base 上下的均匀抖动幅度")
    args, _ = ap.parse_known_args()

    rospy.init_node("delay_relay_" + args.type)
    DelayRelay(args)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
