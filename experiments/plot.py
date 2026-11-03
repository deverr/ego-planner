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

    md = [table(rows, "fill", "障碍密度扫描（fill）")]
    if any(fnum(r["max_vel"]) != 2.0 for r in rows):
        md.append(table(rows, "max_vel", "速度上限扫描（max_vel）"))
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

    png = args.png or os.path.join(os.path.dirname(args.summary), "fig_fill.png")
    fills = sorted({fnum(r["fill"]) for r in rows})
    data = [[fnum(r["flight_time"]) for r in rows
             if fnum(r["fill"]) == f and fnum(r["flight_time"]) == fnum(r["flight_time"])]
            for f in fills]
    data = [d for d in data if d]
    if data:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.boxplot(data, labels=["%.2f" % f for f in fills[:len(data)]])
        ax.set_xlabel("obstacle fill ratio")
        ax.set_ylabel("flight time (s)")
        ax.set_title("EGO-Planner flight time vs obstacle density")
        fig.tight_layout()
        fig.savefig(png, dpi=140)
        print("written:", png)


if __name__ == "__main__":
    main()
