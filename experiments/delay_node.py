#!/usr/bin/env python3
"""按固定时延转发消息，模拟链路时延。

支持两类：
  --type cloud   sensor_msgs/PointCloud2（感知时延）
  --type odom    nav_msgs/Odometry（状态估计时延）

消息保留原始 header.stamp —— 也就是「数据晚到但时间戳是对的」，这正是真实
传感器/状态估计链路的延迟形态。delay=0 时直接透传，用作基线。
"""
import argparse
import collections
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

TYPES = {"cloud": PointCloud2, "odom": Odometry}


class DelayRelay(object):
    def __init__(self, args):
        cls = TYPES[args.type]
        self.delay = args.delay_ms / 1000.0
        self.buf = collections.deque()
        self.pub = rospy.Publisher(args.out, cls, queue_size=50)
        rospy.Subscriber(args.inp, cls, self.cb, queue_size=100)
        if self.delay > 0.0:
            rospy.Timer(rospy.Duration(0.002), self.flush)
        rospy.loginfo("%s delay relay: %s -> %s, %.0f ms",
                      args.type, args.inp, args.out, args.delay_ms)

    def cb(self, msg):
        if self.delay <= 0.0:
            self.pub.publish(msg)
            return
        self.buf.append((rospy.Time.now(), msg))

    def flush(self, _event):
        now = rospy.Time.now()
        while self.buf:
            t, msg = self.buf[0]
            if (now - t).to_sec() < self.delay:
                break
            self.buf.popleft()
            self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", choices=sorted(TYPES), default="cloud")
    ap.add_argument("--in", dest="inp", default="/pcl_render_node/cloud")
    ap.add_argument("--out", dest="out", default="/delayed/msg")
    ap.add_argument("--delay-ms", type=float, default=0.0)
    args, _ = ap.parse_known_args()

    rospy.init_node("delay_relay_" + args.type)
    DelayRelay(args)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
