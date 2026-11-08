#!/usr/bin/env python3
"""独立审计：不复用 collect.py 的任何计算，直接从原始文件重算并逐行比对。

按 codex 的 review 收紧过：
- 重算不出来的指标，不允许因为「summary 里是有限值」而蒙混过关
- nan 显式处理，不能靠 nan 比较恒 False 静默跳过
- 地图不变量违规计入失败，而不是只打印
- 容差按数值精度定，不用相对百分比
"""
import csv
import hashlib
import math
import pathlib
import re
import sys
from collections import defaultdict

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
RAW = ROOT / "results" / "raw"
LOGS = ROOT / "logs"
SUMMARY = ROOT / "results" / "summary.csv"

GOAL = np.array([-15.0, 0.0, 1.0])
TOL = 0.5
MIN_PATH = 40.0

ATOL = 1e-6          # 同一份原始数据、不同求和顺序带来的差异
RTOL = 1e-9
PARAM_TOL = 0.01     # 参数以整数毫秒写进日志

RELAY = re.compile(
    r"odom delay relay: \S+ -> \S+, ([\d.]+) ms(?: \+?-? ?([\d.]+) ms)?")

METRICS = ("path_len", "max_speed", "track_med", "omega_p95", "flight_time")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def finite(x):
    return x == x and not math.isinf(x)


def load_odom(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    if d.size == 0:
        return None
    if d.shape == ():
        d = d.reshape(1)
    return d


def recompute(tag):
    """返回 {指标: 值}；算不出来的指标不出现在结果里，由调用方判定是否可接受。"""
    out = {}
    odom = RAW / (tag + ".csv")
    if not odom.exists():
        return out
    d = load_odom(odom)
    if d is None or d.size < 2:
        return out

    names = d.dtype.names or ()
    xyz = np.column_stack([d["x"], d["y"], d["z"]])

    out["path_len"] = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
    out["max_speed"] = float(np.sqrt(d["vx"] ** 2 + d["vy"] ** 2 + d["vz"] ** 2).max())

    if all(k in names for k in ("wx", "wy", "wz")):
        w = np.sqrt(d["wx"] ** 2 + d["wy"] ** 2 + d["wz"] ** 2)
        out["omega_p95"] = float(np.percentile(w, 95))

    lift = int(np.argmax(xyz[:, 2] > 0.3)) if (xyz[:, 2] > 0.3).any() else 0
    t = d["t"] - d["t"][lift]
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))])
    hit = (np.linalg.norm(xyz - GOAL, axis=1) < TOL) & (cum >= MIN_PATH)
    if hit.any():
        out["flight_time"] = float(t[int(np.argmax(hit))])

    cmd = RAW / (tag + "_cmd.csv")
    if cmd.exists():
        c = np.genfromtxt(cmd, delimiter=",", names=True)
        if c.size >= 10 and c.shape != ():
            # 时间重叠必须充分，否则 np.interp 会把越界样本钳到端点，算出假误差
            o_lo, o_hi = float(d["t"].min()), float(d["t"].max())
            inside = (c["t"] >= o_lo) & (c["t"] <= o_hi)
            if float(inside.mean()) > 0.9:
                ax = np.interp(c["t"], d["t"], d["x"])
                ay = np.interp(c["t"], d["t"], d["y"])
                az = np.interp(c["t"], d["t"], d["z"])
                m = az > 0.3
                if m.sum() >= 10:
                    e = np.sqrt((c["x"] - ax) ** 2 + (c["y"] - ay) ** 2 + (c["z"] - az) ** 2)
                    out["track_med"] = float(np.median(e[m]))
    return out


def log_delay(tag):
    p = LOGS / (tag + ".log")
    if not p.exists():
        return None
    m = RELAY.search(p.read_text(errors="ignore"))
    if not m:
        return None
    return fnum(m.group(1)), (fnum(m.group(2)) if m.group(2) else 0.0)


def fingerprint(tag):
    p = RAW / (tag + ".npy")
    return hashlib.md5(p.read_bytes()).hexdigest()[:12] if p.exists() else None


def close(a, b):
    return abs(a - b) <= ATOL + RTOL * abs(b)


def main():
    rows = list(csv.DictReader(SUMMARY.open()))
    problems = defaultdict(list)
    print("summary 行数:", len(rows))

    for r in rows:
        tag = r["id"]

        # --- 1. 原始文件必须齐全 ---
        for path, label in ((RAW / (tag + ".csv"), "odom"),
                            (RAW / (tag + ".status"), "status"),
                            (LOGS / (tag + ".log"), "log")):
            if not path.exists():
                problems["缺原始文件"].append("%s (%s)" % (tag, label))

        # --- 2. 参数必须有限，且与日志一致 ---
        d_csv = fnum(r.get("odom_delay") or 0)
        j_csv = fnum(r.get("odom_jitter") or 0)
        if not (finite(d_csv) and finite(j_csv)):
            problems["参数非有限值"].append(
                "%s %s/%s" % (tag, r.get("odom_delay"), r.get("odom_jitter")))
        got = log_delay(tag)
        if got is None:
            if finite(d_csv) and (d_csv > 0 or j_csv > 0):
                problems["注入了时延但日志无 relay 记录"].append(tag)
        else:
            d_log, j_log = got
            if not (finite(d_log) and abs(d_log - d_csv) <= PARAM_TOL):
                problems["时延不符"].append("%s log=%s csv=%s" % (tag, d_log, d_csv))
            if not (finite(j_log) and abs(j_log - j_csv) <= PARAM_TOL):
                problems["抖动不符"].append("%s log=%s csv=%s" % (tag, j_log, j_csv))

        # --- 3. 指标：重算不出来的不许有有限值 ---
        rc = recompute(tag)
        for key in METRICS:
            claimed = fnum(r.get(key))
            have = rc.get(key)
            if have is None:
                if finite(claimed):
                    problems["无原始数据支撑"].append(
                        "%s.%s=%s 但不可重算" % (tag, key, claimed))
                continue
            if not finite(claimed):
                problems["summary 缺失但可重算"].append(
                    "%s.%s csv=%s 重算=%.6f" % (tag, key, r.get(key), have))
                continue
            if not close(claimed, have):
                problems["重算不符:" + key].append(
                    "%s csv=%.6f 重算=%.6f" % (tag, claimed, have))

        # --- 4. 成功标记 ---
        st = RAW / (tag + ".status")
        if st.exists():
            status = st.read_text().split()[0]
            if int(r["success"]) != int(status == "success"):
                problems["成功标记不符"].append(
                    "%s csv=%s status=%s" % (tag, r["success"], status))

    # --- 5. 地图不变量（违规计入失败）---
    st_ids = {p.stem for p in RAW.glob("*.status")}
    row_ids = {r["id"] for r in rows}
    skipped_file = ROOT / "results" / "skipped.txt"
    skipped = set()
    if skipped_file.exists():
        skipped = {l.strip() for l in skipped_file.read_text().splitlines() if l.strip()}
    for t in st_ids - row_ids:
        # 被 runner 判定为 unhealthy 并记入 skipped.txt 的，属于预期排除
        if t in skipped:
            continue
        problems["有 status 但无 summary 行"].append(t)
    for t in row_ids - st_ids:
        problems["有 summary 行但无 status"].append(t)

    by_group = defaultdict(set)
    no_map = 0
    for r in rows:
        fp = fingerprint(r["id"])
        if fp is None:
            no_map += 1
            continue
        by_group[(r["fill"], r["seed"])].add(fp)
    if no_map:
        problems["缺地图指纹"].append("%d 行" % no_map)
    for k, v in by_group.items():
        if len(v) > 1:
            problems["同一 (fill,seed) 地图不一致"].append("%s: %d 种" % (k, len(v)))

    fp_to_groups = defaultdict(set)
    for k, v in by_group.items():
        for f in v:
            fp_to_groups[f].add(k)
    shared = {f: g for f, g in fp_to_groups.items() if len(g) > 1}
    if shared:
        problems["不同 seed 复用了同一张地图"].append("%d 张" % len(shared))

    if skipped:
        print("预期排除（unhealthy，见 results/skipped.txt）:", len(skipped), sorted(skipped))
    print("不同 (fill,seed) 组数:", len(by_group),
          " 地图指纹数:", len({f for v in by_group.values() for f in v}))

    print()
    if not problems:
        print("✅ 全部检查通过（%d 行）" % len(rows))
        return 0
    for k, v in problems.items():
        print("❌ %s: %d 处" % (k, len(v)))
        for item in v[:5]:
            print("    ", item)
    return 1


if __name__ == "__main__":
    sys.exit(main())
