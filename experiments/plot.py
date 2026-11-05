#!/usr/bin/env python3
"""从 summary.csv 生成 Markdown 结果表和（可选）图表。"""
import argparse
import csv
import os
import statistics as st


def load(path):
    with open(path) as fh:
        return [r for r in csv.DictReader(fh)]


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def agg(rows):
    succ = sum(int(r["success"]) for r in rows)
    n = len(rows)
    times = [fnum(r["flight_time"]) for r in rows if fnum(r["flight_time"]) == fnum(r["flight_time"])]
    clears = [fnum(r["min_clearance"]) for r in rows if fnum(r["min_clearance"]) == fnum(r["min_clearance"])]
    reps = [fnum(r["replans"]) for r in rows if fnum(r["replans"]) == fnum(r["replans"])]
    res = {
        "n": n,
        "success": succ,
        "rate": 100.0 * succ / n if n else 0.0,
        "time": (st.mean(times), st.stdev(times)) if len(times) > 1 else (float("nan"), float("nan")),
        "clear": (min(clears) if clears else float("nan"),
                  st.mean(clears) if clears else float("nan")),
        "replans": st.mean(reps) if reps else float("nan"),
    }
    return res


def table(rows, key, label):
    groups = {}
    for r in rows:
        groups.setdefault(fnum(r[key]), []).append(r)
    out = ["", "### %s" % label, "",
           "| %s | trials | success | flight time (s) | min clearance (m) | replans |" % key,
           "|---|---|---|---|---|---|"]
    for k in sorted(groups):
        a = agg(groups[k])
        t, sd = a["time"]
        ts = "%.1f ± %.1f" % (t, sd) if t == t else "—"
        cs = "%.2f / %.2f" % (a["clear"][0], a["clear"][1]) if a["clear"][0] == a["clear"][0] else "—"
        out.append("| %.2f | %d | %d/%d (%.0f%%) | %s | %s | %.1f |"
                   % (k, a["n"], a["success"], a["n"], a["rate"], ts, cs, a["replans"]))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", required=True)
    ap.add_argument("--png", default=None)
    args = ap.parse_args()

    rows = load(args.summary)
    if not rows:
        print("no rows")
        return

    # 密度对比只能取同一速度下的样本，否则速度组的行会混进来
    base = [r for r in rows if fnum(r["max_vel"]) == 2.0]
    speed = [r for r in rows if fnum(r["fill"]) == 0.12]

    md = [table(base, "fill", "障碍密度扫描（fill，max_vel 固定 2.0）")]
    if len({fnum(r["max_vel"]) for r in speed}) > 1:
        md.append(table(speed, "max_vel", "速度上限扫描（max_vel，fill 固定 0.12）"))
    text = "\n".join(md)
    print(text)

    out_md = os.path.join(os.path.dirname(args.summary), "table.md")
    with open(out_md, "w") as fh:
        fh.write(text + "\n")
    print("\nwritten:", out_md)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    outdir = os.path.dirname(args.summary)
    for subset, key, xlabel, title, fname in [
        (base, "fill", "obstacle fill ratio",
         "Flight time vs obstacle density", "fig_fill.png"),
        (speed, "max_vel", "max velocity (m/s)",
         "Flight time vs velocity limit", "fig_speed.png"),
    ]:
        groups = {}
        for r in subset:
            v = fnum(r["flight_time"])
            if v == v:
                groups.setdefault(fnum(r[key]), []).append(v)
        if len(groups) < 2:
            continue
        keys = sorted(groups)
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.boxplot([groups[k] for k in keys], labels=["%.2f" % k for k in keys])
        ax.set_xlabel(xlabel)
        ax.set_ylabel("flight time (s)")
        ax.set_title(title)
        fig.tight_layout()
        path = os.path.join(outdir, fname)
        fig.savefig(path, dpi=140)
        print("written:", path)


if __name__ == "__main__":
    main()
