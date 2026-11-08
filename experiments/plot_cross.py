#!/usr/bin/env python3
"""速度 × 状态估计时延的交叉表与折线图。"""
import argparse
import csv
import os
import statistics as st

VELS = (2.0, 3.0, 4.0)
DELAYS = (0, 10, 20, 50)


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", required=True)
    args = ap.parse_args()

    rows = [r for r in load(args.summary)
            if fnum(r.get("fill")) == 0.12 and fnum(r.get("cloud_delay") or 0) == 0]
    for r in rows:
        if not r.get("odom_delay"):
            r["odom_delay"] = "0"

    cells = {}
    for r in rows:
        v, d = fnum(r["max_vel"]), fnum(r["odom_delay"])
        if v in VELS and d in DELAYS:
            cells.setdefault((v, d), []).append(r)

    lines = ["", "### 跟踪误差 × 速度 × 状态估计时延",
             "",
             "单元格：`成功数/总数，中位跟踪误差`",
             "",
             "| max_vel | " + " | ".join("%d ms" % d for d in DELAYS) + " |",
             "|---|" + "---|" * len(DELAYS)]
    series = {v: [] for v in VELS}
    for v in VELS:
        out = []
        for d in DELAYS:
            rs = cells.get((v, d), [])
            if not rs:
                out.append("—")
                series[v].append(float("nan"))
                continue
            ok = sum(int(r["success"]) for r in rs)
            # track_med == 0 表示那次试验的指令流是空的（失败且未起飞），不是真的零误差
            tr = [fnum(r["track_med"]) for r in rs
                  if fnum(r["track_med"]) == fnum(r["track_med"]) and fnum(r["track_med"]) > 1e-3]
            mean = st.mean(tr) if tr else float("nan")
            series[v].append(mean)
            out.append("%d/%d, %s" % (ok, len(rs), "%.3f" % mean if tr else "—"))
        lines.append("| %.1f | %s |" % (v, " | ".join(out)))

    text = "\n".join(lines)
    print(text)
    out_md = os.path.join(os.path.dirname(args.summary), "table_cross.md")
    with open(out_md, "w") as fh:
        fh.write(text + "\n")
    print("\nwritten:", out_md)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for v in VELS:
        xs = [d for d, y in zip(DELAYS, series[v]) if y == y]
        ys = [y for y in series[v] if y == y]
        if len(xs) >= 2:
            ax.plot(xs, ys, marker="o", label="%.0f m/s" % v)
    ax.axhline(0.15, ls="--", lw=1, color="crimson")
    ax.text(2, 0.157, "Aerobatic Planner: position tracking error <= 0.15 m",
            fontsize=7.5, color="crimson")
    ax.set_xlabel("state-estimate latency (ms)")
    ax.set_ylabel("median tracking error (m)")
    ax.set_title("Tracking error vs latency, by speed")
    ax.set_xticks(DELAYS)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(os.path.dirname(args.summary), "fig_latency_speed.png")
    fig.savefig(path, dpi=140)
    print("written:", path)


if __name__ == "__main__":
    main()
