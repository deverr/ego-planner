#!/bin/bash
# 批量实验：障碍密度（fill）/ 速度上限（max_vel）两组扫描
#
# 冒烟测试：
#   ./run_all.sh --quick
# 完整跑（默认 3 组密度 × 12 seed + 2 组速度 × 8 seed）：
#   ./run_all.sh
# 自定义：
#   FILLS="0.12 0.20" SEEDS="1 2 3" VELS="" ./run_all.sh

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
cd "$REPO" || exit 1

# 同一时间只允许一个实例，否则两个 sim 会同时往 /visual_slam/odom 发数据
exec 9>/tmp/ego_exp.lock
if ! flock -n 9; then
    echo "另一个 run_all.sh 正在运行，退出。"
    exit 1
fi

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
MIN_PATH=40             # 累计航程阈值，避免「刚起飞就判到点」
TIMEOUT=${TIMEOUT:-110}
ATTEMPTS=${ATTEMPTS:-3}

RAW="$REPO/results/raw"
LOGS="$REPO/logs"
SUMMARY="$REPO/results/summary.csv"
SKIPPED="$REPO/results/skipped.txt"
mkdir -p "$RAW" "$LOGS" "$REPO/results"

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

if [ "$QUICK" = "1" ]; then
    SEEDS=${SEEDS-"1 2"}
    FILLS=${FILLS-"0.12"}
    VELS=${VELS-""}
    BSEEDS=${BSEEDS-"1 2"}
    DELAYS=${DELAYS-""}
else
    SEEDS=${SEEDS-"1 2 3 4 5 6 7 8 9 10 11 12"}
    FILLS=${FILLS-"0.12 0.20 0.28"}
    VELS=${VELS-"3.0 4.0"}
    BSEEDS=${BSEEDS-"1 2 3 4 5 6 7 8"}
    DELAYS=${DELAYS-"10 20 50 100"}
fi

SIM_BIN="/ws/devel/lib/so3_quadrotor_simulator/quadrotor_simulator_so3"

kill_all() {
    # 注意：pkill -x 不可用。Linux 进程名（comm）被截断到 15 字符，
    # quadrotor_simulator_so3 / ego_planner_node / waypoint_generator
    # 都匹配不上，会残留上一轮的仿真器。
    # 改用二进制路径做 -f 匹配，既唯一又不会匹配到本脚本自身。
    pkill -9 -f "/ws/devel/lib/"                 >/dev/null 2>&1
    pkill -9 -f "/opt/ros/noetic/lib/rviz"       >/dev/null 2>&1
    pkill -9 -f "/opt/ros/noetic/lib/rosout"     >/dev/null 2>&1
    pkill -9 -f "/opt/ros/noetic/bin/roslaunch"  >/dev/null 2>&1
    pkill -9 -f "/opt/ros/noetic/bin/rosmaster"  >/dev/null 2>&1
    pkill -9 -f "experiments/monitor.py"         >/dev/null 2>&1

    for _ in $(seq 1 40); do
        pgrep -f "/ws/devel/lib/" >/dev/null 2>&1 || break
        sleep 0.25
    done
    sleep 0.4
}

sim_count() {
    pgrep -f "$SIM_BIN" 2>/dev/null | wc -l | tr -d ' '
}

# 起一次仿真并跑到结束。成功返回 0/2，起不来返回 3
attempt_once() {   # $1=fill $2=seed $3=max_vel $4=tag $5=cloud_delay $6=odom_delay
    local fill=$1 seed=$2 vel=$3 tag=$4 delay=$5 odelay=${6:-0}
    local csv="$RAW/${tag}.csv" cloud="$RAW/${tag}.npy" status="$RAW/${tag}.status"
    local log="$LOGS/${tag}.log"

    kill_all
    roslaunch ego_planner_experiments exp.launch \
        seed:="$seed" fill:="$fill" max_vel:="$vel" \
        cloud_delay_ms:="$delay" odom_delay_ms:="$odelay" \
        > "$log" 2>&1 9>&- &
    local lpid=$!

    for _ in $(seq 1 80); do
        if rostopic list 2>/dev/null | grep -q "^/visual_slam/odom$"; then break; fi
        sleep 0.5
    done

    if [ "$(sim_count)" != "1" ]; then
        kill -INT $lpid >/dev/null 2>&1
        sleep 1
        kill_all
        return 3
    fi

    timeout $((TIMEOUT + 45)) python3 "$HERE/monitor.py" \
        --out "$csv" --cloud "$cloud" \
        --goal="$GOAL" --min-path "$MIN_PATH" --tol "$TOL" \
        --timeout "$TIMEOUT" --ready-z 0.5 --ready-timeout 30 9>&-
    local rc=$?

    kill -INT $lpid >/dev/null 2>&1
    sleep 1
    kill_all

    if [ "$rc" = "3" ]; then
        return 3
    fi

    python3 "$HERE/collect.py" \
        --id "$tag" --odom "$csv" --cloud "$cloud" --log "$log" \
        --status "$status" --goal="$GOAL" --min-path "$MIN_PATH" --tol "$TOL" \
        --fill "$fill" --seed "$seed" --max-vel "$vel" --max-acc 3.0 \
        --cloud-delay "$delay" --odom-delay "${odelay:-0}" \
        --summary "$SUMMARY" || true

    return $rc
}

run_one() {   # $1=fill $2=seed $3=max_vel $4=cloud_delay $5=odom_delay
    local fill=$1 seed=$2 vel=$3 delay=${4:-0} odelay=${5:-0}
    local id="fill${fill}_seed${seed}_v${vel}"
    [ "$delay" != "0" ] && id="${id}_d${delay}"
    [ "$odelay" != "0" ] && id="${id}_o${odelay}"

    if grep -qx "$id" "$SUMMARY" 2>/dev/null; then
        echo "--- $id 已完成，跳过"
        return 0
    fi

    local n=1
    while [ "$n" -le "$ATTEMPTS" ]; do
        echo ">>> $id (attempt $n/$ATTEMPTS)"
        attempt_once "$fill" "$seed" "$vel" "$id" "$delay" "$odelay"
        local rc=$?
        if [ "$rc" != "3" ]; then
            return $rc
        fi
        echo "    !!! 仿真未正常启动（无人机没起飞或多实例冲突），重试"
        n=$((n + 1))
    done

    echo "    !!! $id 连续失败，记为无效并跳过"
    echo "$id" >> "$SKIPPED"
    return 1
}

# ---- A 组：障碍密度（max_vel 固定 2.0） ----
for fill in $FILLS; do
    for seed in $SEEDS; do
        run_one "$fill" "$seed" 2.0 0 0
    done
done

# ---- B 组：速度上限（fill 固定 0.12，2.0 已在 A 组） ----
for vel in $VELS; do
    for seed in $BSEEDS; do
        run_one "0.12" "$seed" "$vel" 0 0
    done
done

# ---- C 组：状态估计时延注入（fill=0.12, max_vel=2.0） ----
for o in $ODELAYS; do
    for seed in ${CSEEDS-"1 2 3 4 5 6"}; do
        run_one "0.12" "$seed" 2.0 0 "$o"
    done
done

echo
echo "=== 完成 ==="
if [ -s "$SKIPPED" ]; then
    echo "跳过的无效试验："
    cat "$SKIPPED"
fi
python3 "$HERE/plot.py" --summary "$SUMMARY"
