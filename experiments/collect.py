#!/usr/bin/env python3
"""把单次实验的原始数据折算成一行指标，追加到 summary.csv。

指标：
  success        是否在超时前抵达终点（且已访问中间点）
  flight_time    从起飞到抵达终点的秒数（失败记为 NaN）
  path_len       实际飞行路径长度 (m)
  max_speed      最大速度 (m/s)
  min_clearance  与地面真值障碍点云的最近距离 (m)，用于近似穿障判定
  replans        重规划次数（解析 roslaunch 日志）
"""
import argparse
import csv
import os
import re
import sys

import numpy as np

VOXEL = 0.25         # 点云降采样栅格，加速最近距离计算
CHUNK = 100          # 每批查询的位姿数


def load_odom(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        return None
    if data.shape == ():
        data = data.reshape(1)
    return data


def downsample(cloud):
    if cloud is None or cloud.size == 0:
        return None
    key = np.round(cloud / VOXEL).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return cloud[idx]


def min_clearance(cloud_ds, xyz, stride=10):
    """轨迹按 10 帧下采样后暴力求最近障碍距离。

    不依赖 scipy：块大小控制到 100 个位姿，单块内存 ~40MB。
    """
    if cloud_ds is None or xyz.shape[0] == 0:
        return float("nan")
    xyz = xyz[::stride]
    best = float("inf")
    for i in range(0, xyz.shape[0], CHUNK):
        block = xyz[i:i + CHUNK]
        d = np.linalg.norm(block[:, None, :] - cloud_ds[None, :, :], axis=2)
        best = min(best, float(d.min()))
    return best


def count_replans(log_path):
    if not os.path.exists(log_path):
        return 0
    with open(log_path, "r", errors="ignore") as fh:
        return len(re.findall(r"to REPLAN_TRAJ", fh.read()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--status", required=True)
    ap.add_argument("--goal", required=True)
    ap.add_argument("--min-path", type=float, default=40.0)
    ap.add_argument("--tol", type=float, default=0.5)
    ap.add_argument("--fill", type=float, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--max-vel", type=float, required=True)
    ap.add_argument("--max-acc", type=float, required=True)
    ap.add_argument("--summary", required=True)
    args = ap.parse_args()

    goal = np.array([float(v) for v in args.goal.split(",")])

    status = "timeout"
    if os.path.exists(args.status):
        status = open(args.status).read().split()[0]

    row = dict(id=args.id, fill=args.fill, seed=args.seed,
               max_vel=args.max_vel, max_acc=args.max_acc,
               success=int(status == "success"),
               flight_time=float("nan"), path_len=float("nan"),
               max_speed=float("nan"), min_clearance=float("nan"),
               replans=count_replans(args.log))

    data = load_odom(args.odom) if os.path.exists(args.odom) else None
    if data is not None and data.size > 1:
        xyz = np.column_stack([data["x"], data["y"], data["z"]])
        t = data["t"] - data["t"][0]
        speed = np.sqrt(data["vx"] ** 2 + data["vy"] ** 2 + data["vz"] ** 2)

        row["path_len"] = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
        row["max_speed"] = float(speed.max())

        cloud = None
        if os.path.exists(args.cloud):
            try:
                cloud = np.load(args.cloud)
            except Exception:  # noqa: BLE001
                cloud = None
        row["min_clearance"] = min_clearance(downsample(cloud), xyz)

        if status == "success":
            cum = np.concatenate([[0.0], np.cumsum(
                np.linalg.norm(np.diff(xyz, axis=0), axis=1))])
            near_goal = np.linalg.norm(xyz - goal, axis=1) < args.tol
            ok = near_goal & (cum >= args.min_path)
            if ok.any():
                row["flight_time"] = float(t[int(np.argmax(ok))])

    fields = ["id", "fill", "seed", "max_vel", "max_acc", "success",
              "flight_time", "path_len", "max_speed", "min_clearance", "replans"]
    new_file = not os.path.exists(args.summary)
    with open(args.summary, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        if new_file:
            w.writeheader()
        w.writerow({k: row[k] for k in fields})

    print("  %-22s success=%d  t=%6.1fs  clear=%.2fm  replans=%d"
          % (args.id, row["success"],
             row["flight_time"] if row["flight_time"] == row["flight_time"] else -1,
             row["min_clearance"] if row["min_clearance"] == row["min_clearance"] else -1,
             row["replans"]))


if __name__ == "__main__":
    sys.exit(main())
