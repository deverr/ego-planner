#!/usr/bin/env python3
"""单次实验的记录器 + 完成判定。

订阅里程计并落盘 CSV，订阅全局点云（地面真值障碍）并落盘 npy；
当无人机「访问过中间点 后抵达终点」时退出 0，超时退出 2。

用法：
  python3 monitor.py --out raw/fill0.20_seed3.csv --cloud raw/fill0.20_seed3.npy \
      --goal -15,0,1 --p2 0,15,1 --tol 0.5 --timeout 120
"""
import argparse
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def xyz(s):
    parts = [float(v) for v in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("need x,y,z")
    return tuple(parts)


class Monitor(object):
    def __init__(self, args):
        self.args = args
        self.n = 0
        self.cum = 0.0          # 累计航程，用来区分「起点就在终点附近」
        self.prev = None
        self.result = "timeout"
        self.cloud_saved = False

        self.fh = open(args.out, "w", buffering=1)
        self.fh.write("t,x,y,z,vx,vy,vz\n")

        rospy.Subscriber("/visual_slam/odom", Odometry, self.cb_odom, queue_size=200)
        rospy.Subscriber("/map_generator/global_cloud", PointCloud2, self.cb_cloud, queue_size=1)

    def cb_cloud(self, msg):
        """只存一次：地图在单次实验内是静态的。"""
        if self.cloud_saved:
            return
        self.cloud_saved = True
        try:
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.asarray(list(pts), dtype=np.float64)
            if arr.size == 0:
                return
            if arr.shape[0] > 600000:
                idx = np.random.choice(arr.shape[0], 600000, replace=False)
                arr = arr[idx]
            np.save(self.args.cloud, arr)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("cloud save failed: %s", exc)

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        t = msg.header.stamp.to_sec()
        self.fh.write("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"
                      % (t, p.x, p.y, p.z, v.x, v.y, v.z))
        self.n += 1

        # 累计航程：起点与终点很近，必须先用航程排除「刚起飞就被判定到点」
        if self.prev is not None:
            self.cum += float(np.linalg.norm(
                [p.x - self.prev[0], p.y - self.prev[1], p.z - self.prev[2]]))
        self.prev = (p.x, p.y, p.z)

        if self.cum >= self.args.min_path:
            dg = np.linalg.norm([p.x - self.args.goal[0],
                                 p.y - self.args.goal[1],
                                 p.z - self.args.goal[2]])
            if dg < self.args.tol:
                self.result = "success"
                rospy.signal_shutdown("reached goal")

    def close(self):
        try:
            self.fh.close()
        except Exception:  # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--goal", type=xyz, required=True)
    ap.add_argument("--min-path", type=float, default=40.0,
                    help="累计航程超过该值后才接受终点判定 (m)")
    ap.add_argument("--tol", type=float, default=0.5)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    rospy.init_node("trial_monitor", disable_signals=True)
    mon = Monitor(args)

    t0 = time.time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and (time.time() - t0) < args.timeout:
        rate.sleep()

    mon.close()
    with open(args.out.replace(".csv", ".status"), "w") as fh:
        fh.write("%s %d\n" % (mon.result, mon.n))
    print("%s samples=%d" % (mon.result, mon.n))
    sys.exit(0 if mon.result == "success" else 2)


if __name__ == "__main__":
    main()
