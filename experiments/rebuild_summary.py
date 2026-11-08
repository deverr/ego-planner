#!/usr/bin/env python3
"""按 results/raw/ 里的原始数据重建 summary.csv（新增 track_rmse / cloud_delay 列后需要）。"""
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
RAW = REPO / "results" / "raw"

PATTERN = re.compile(
    r"^fill(?P<fill>[\d.]+)_seed(?P<seed>\d+)_v(?P<vel>[\d.]+)"
    r"(?:_d(?P<delay>[\d.]+))?(?:_o(?P<odelay>[\d.]+))?$")


def main():
    summary = REPO / "results" / "summary.csv"
    if summary.exists():
        summary.unlink()

    ok = 0
    for status in sorted(RAW.glob("*.status")):
        tag = status.stem
        if status.read_text().split()[0] not in ("success", "timeout"):
            print("skip (unhealthy):", tag, file=sys.stderr)
            continue
        m = PATTERN.match(tag)
        if not m:
            print("skip (unparsable):", tag, file=sys.stderr)
            continue
        csv = RAW / (tag + ".csv")
        cloud = RAW / (tag + ".npy")
        log = REPO / "logs" / (tag + ".log")
        if not csv.exists():
            print("skip (no odom):", tag, file=sys.stderr)
            continue
        cmd = [
            sys.executable, str(HERE / "collect.py"),
            "--id", tag, "--odom", str(csv), "--cloud", str(cloud),
            "--log", str(log), "--status", str(status),
            "--goal=-15,0,1", "--min-path", "40", "--tol", "0.5",
            "--fill", m.group("fill"), "--seed", m.group("seed"),
            "--max-vel", m.group("vel"), "--max-acc", "3.0",
            "--cloud-delay", m.group("delay") or "0",
            "--odom-delay", m.group("odelay") or "0",
            "--summary", str(summary),
        ]
        if subprocess.call(cmd) == 0:
            ok += 1

    print("rebuilt %d rows -> %s" % (ok, summary))


if __name__ == "__main__":
    main()
