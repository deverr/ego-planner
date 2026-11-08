#!/usr/bin/env python3
"""抖动 vs 固定时延、加速度上限、体角速度三张表。"""
import argparse
import csv
import os
import statistics as st


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in ("cloud_delay", "odom_delay", "odom_jitter"):
            if not r.get(k):
                r[k] = "0"
    return rows


def stats(rs):
    tr = [fnum(r["track_med"]) for r in rs
          if fnum(r["track_med"]) == fnum(r["track_med"]) and fnum(r["track_med"]) > 1e-3]
    om = [fnum(r["omega_p95"]) for r in rs if fnum(r["omega_p95"]) == fnum(r["omega_p95"])]
    ok = sum(int(r["success"]) for r in rs)
    return (len(rs), ok,
            st.mean(tr) if tr else float("nan"),
            st.mean(om) if om else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", required=True)
    args = ap.parse_args()

    rows = [r for r in load(args.summary)
            if fnum(r["fill"]) == 0.12 and fnum(r["cloud_delay"]) == 0]

    def sel(vel, delay, jitter, acc=3.0):
        return [r for r in rows
                if fnum(r["max_vel"]) == vel and fnum(r["odom_delay"]) == delay
                and fnum(r["odom_jitter"]) == jitter and fnum(r["max_acc"]) == acc]

    out = ["", "### 抖动 vs 固定时延（同为 4 m/s 与 2 m/s）", "",
           "`抖动 base0 ±20` 因负样本截断到 0，实际均值约 5 ms；"
           "`base20 ±20` 无截断，均值 20 ms。", "",
           "| 工况 | 固定 0 ms | 抖动 均值≈5 ms | 固定 20 ms | 抖动 均值 20 ms |",
           "|---|---|---|---|---|"]
    for v in (2.0, 4.0):
        cells = []
        for d, j in ((0, 0), (0, 20), (20, 0), (20, 20)):
            n, ok, tr, _ = stats(sel(v, d, j))
            cells.append("%d/%d, %.3f" % (ok, n, tr) if n and tr == tr else "—")
        out.append("| %.0f m/s | %s |" % (v, " | ".join(cells)))
    text = "\n".join(out)

    out2 = ["", "### 体角速度与加速度上限（4 m/s）", "",
            "| max_acc | 时延 | n | 成功 | 跟踪误差 (m) | omega_p95 (rad/s) |",
            "|---|---|---|---|---|---|"]
    for a in (3.0, 6.0, 9.0):
        for d in (0, 20):
            rs = sel(4.0, d, 0, a)
            if not rs:
                continue
            n, ok, tr, om = stats(rs)
            out2.append("| %.1f | %d ms | %d | %d/%d | %s | %s |"
                        % (a, d, n, ok, n,
                           "%.3f" % tr if tr == tr else "—",
                           "%.2f" % om if om == om else "—"))
    text += "\n".join(out2)

    print(text)
    path = os.path.join(os.path.dirname(args.summary), "table_jitter.md")
    with open(path, "w") as fh:
        fh.write(text + "\n")
    print("\nwritten:", path)


if __name__ == "__main__":
    main()
