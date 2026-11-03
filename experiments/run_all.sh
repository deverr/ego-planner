#!/bin/bash
# 批量实验：障碍密度 / 速度上限 两组扫描
#
# 快速冒烟测试：
#   ./run_all.sh --quick              # 2 个 seed，1 组参数，约 4 分钟
# 完整跑：
#   ./run_all.sh                     # 3 组密度 × 20 seed + 3 组速度 × 10 seed
# 自定义：
#   FILLS="0.12 0.20" SEEDS="1 2 3" ./run_all.sh

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
cd "$REPO" || exit 1

# ROS 的 setup.bash 引用了未定义变量，先关掉 -u 再开
set +u
source /opt/ros/noetic/setup.bash
if [ -f /ws/devel/setup.bash ]; then source /ws/devel/setup.bash; fi
if [ -f devel/setup.bash ]; then source devel/setup.bash; fi

# pcl_render_node 需要 OpenGL 上下文，必须给虚拟显示，否则会崩
export DISPLAY=${DISPLAY:-:1}
export LIBGL_ALWAYS_SOFTWARE=1
export DISABLE_ROS1_EOL_WARNINGS=1

GOAL="-15,0,1"          # 与 exp.launch 的 point4 一致
TOL=0.5
TIMEOUT=${TIMEOUT:-120}

RAW="$REPO/results/raw"
LOGS="$REPO/logs"
SUMMARY="$REPO/results/summary.csv"
mkdir -p "$RAW" "$LOGS" "$REPO/results"

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

if [ "$QUICK" = "1" ]; then
    SEEDS=${SEEDS:-"1 2"}
    FILLS=${FILLS:-"0.12"}
    VELS=${VELS:-""}
else
    SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20"}
    FILLS=${FILLS:-"0.12 0.20 0.28"}
    VELS=${VELS:-"2.0 3.0 4.0"}
fi

kill_all() {
    # 用 -x 精确匹配进程名，避免 pkill -f 匹配到自己这条命令行
    for p in rviz ego_planner_node mockamap_node quadrotor_simulator_so3 \
             nodelet pcl_render_node traj_server waypoint_generator \
             odom_visualization rosout; do
        pkill -9 -x "$p" >/dev/null 2>&1
    done
    pkill -9 -f "/opt/ros/noetic/bin/roslaunch" >/dev/null 2>&1
    pkill -9 -f "/opt/ros/noetic/bin/rosmaster" >/dev/null 2>&1
    pkill -9 -f "experiments/monitor.py"        >/dev/null 2>&1
    for _ in $(seq 1 20); do
        pgrep -f "/opt/ros/noetic/bin/rosmaster" >/dev/null 2>&1 || break
        sleep 0.3
    done
    sleep 0.5
}

run_one() {   # $1=fill $2=seed $3=max_vel
    local fill=$1 seed=$2 vel=$3
    local id="fill${fill}_seed${seed}_v${vel}"
    local csv="$RAW/${id}.csv" cloud="$RAW/${id}.npy" status="$RAW/${id}.status"
    local log="$LOGS/${id}.log"

    echo ">>> $id"
    kill_all
    roslaunch ego_planner_experiments exp.launch \
        seed:="$seed" fill:="$fill" max_vel:="$vel" \
        > "$log" 2>&1 &
    local lpid=$!

    # 等 master + 里程计话题就绪
    for _ in $(seq 1 60); do
        if rostopic list 2>/dev/null | grep -q "^/visual_slam/odom$"; then break; fi
        sleep 0.5
    done

    timeout $((TIMEOUT + 20)) python3 "$HERE/monitor.py" \
        --out "$csv" --cloud "$cloud" \
        --goal="$GOAL" --min-path 40 --tol "$TOL" --timeout "$TIMEOUT"
    local rc=$?

    kill -INT $lpid >/dev/null 2>&1
    sleep 1
    kill_all

    python3 "$HERE/collect.py" \
        --id "$id" --odom "$csv" --cloud "$cloud" --log "$log" \
        --status "$status" --goal="$GOAL" --min-path 40 --tol "$TOL" \
        --fill "$fill" --seed "$seed" --max-vel "$vel" --max-acc 3.0 \
        --summary "$SUMMARY" || true

    return $rc
}

# ---- A 组：障碍密度（fill） ----
for fill in $FILLS; do
    for seed in $SEEDS; do
        run_one "$fill" "$seed" 2.0
    done
done

# ---- B 组：速度上限（fill 固定 0.12） ----
if [ -n "$VELS" ]; then
    for vel in $VELS; do
        for seed in $SEEDS; do
            run_one "0.12" "$seed" "$vel"
        done
    done
fi

echo
echo "=== 完成，汇总：$SUMMARY ==="
python3 "$HERE/plot.py" --summary "$SUMMARY"
