#!/bin/bash
#
# EKF2 (PX4) vs IEKF (KF-GINS) 快速启动脚本
#
# 使用方法:
#   ./quick_comparison_start.sh [scenario] [duration]
#
# 示例:
#   ./quick_comparison_start.sh open_field 600      # 开阔地，10分钟
#   ./quick_comparison_start.sh urban_canyon 1200   # 城市峡谷，20分钟
#

set -e  # 遇到错误退出

# 配置
SCENARIO=${1:-"open_field"}
DURATION=${2:-"600"}  # 秒
WS_ROOT="${HOME}/kf_gins_ws"
DATA_DIR="${WS_ROOT}/data/comparison_results"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}EKF vs IEKF 对比测试快速启动${NC}"
echo -e "${BLUE}================================${NC}\n"

# 验证环境
echo -e "${YELLOW}[1/5] 验证环境...${NC}"
if [ ! -d "$WS_ROOT" ]; then
    echo -e "${RED}❌ 工作空间不存在: $WS_ROOT${NC}"
    exit 1
fi

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo -e "${RED}❌ ROS 安装不完整，请先编译: colcon build${NC}"
    exit 1
fi

source "$WS_ROOT/install/setup.bash"
echo -e "${GREEN}✓ 环境已加载${NC}"

# 验证场景
echo -e "\n${YELLOW}[2/5] 配置参数...${NC}"
echo -e "  场景: ${BLUE}$SCENARIO${NC}"
echo -e "  时长: ${BLUE}$DURATION${NC}s"

# 创建数据目录
mkdir -p "$DATA_DIR/$SCENARIO"
echo -e "${GREEN}✓ 数据目录: $DATA_DIR/$SCENARIO${NC}"

# 启动对比测试
echo -e "\n${YELLOW}[3/5] 启动对比测试...${NC}"

# 后台启动主 launcher
nohup bash -c "
    source $WS_ROOT/install/setup.bash
    ros2 launch kf_gins_ros2_native compare_ekf_iekf.launch.py \
        use_sim_time:=true \
        scenario:=$SCENARIO \
        record_bag:=true \
        start_rviz:=true \
        enable_real_time_comparison:=true
" > "$DATA_DIR/$SCENARIO/launcher.log" 2>&1 &

LAUNCHER_PID=$!
echo -e "${GREEN}✓ Launcher 已启动 (PID: $LAUNCHER_PID)${NC}"

# 等待 ROS 节点启动
echo -e "\n${YELLOW}[4/5] 等待节点启动...${NC}"
sleep 5

# 检查关键节点
for i in {1..30}; do
    if ros2 node list 2>/dev/null | grep -q "kf_gins_node"; then
        echo -e "${GREEN}✓ KF-GINS 节点已启动${NC}"
        break
    fi
    if [ $i -eq 30 ]; then
        echo -e "${RED}✗ 等待超时，请检查 launcher.log${NC}"
        exit 1
    fi
    echo "  等待中... ($i/30)"
    sleep 1
done

# 在新窗口启动实时监控
echo -e "\n${YELLOW}[5/5] 启动实时监控...${NC}"

# 启动 rqt_plot
if command -v rqt_plot &> /dev/null; then
    nohup rqt_plot \
        /comparison/metrics[0] \
        /comparison/metrics[1] \
        /comparison/metrics[2] \
        2>/dev/null > /dev/null &
    echo -e "${GREEN}✓ rqt_plot 已启动${NC}"
fi

# 启动对比脚本 (如果不在 launcher 中)
if ! grep -q "real_time_comparison" "$DATA_DIR/$SCENARIO/launcher.log"; then
    nohup python3 "$WS_ROOT/src/kf_gins_ros2_native/scripts/real_time_comparison.py" \
        > "$DATA_DIR/$SCENARIO/real_time_comparison.log" 2>&1 &
    echo -e "${GREEN}✓ 实时对比脚本已启动${NC}"
fi

# 显示启动摘要
echo -e "\n${BLUE}================================${NC}"
echo -e "${GREEN}✅ 启动完成！${NC}"
echo -e "${BLUE}================================${NC}\n"

echo "📊 监控信息:"
echo -e "  - RViz: 应该自动打开，显示 EKF2 (蓝) vs IEKF (红) 轨迹"
echo -e "  - rqt_plot: 实时显示位置/姿态/速度误差"
echo -e "  - 日志: $DATA_DIR/$SCENARIO/launcher.log"
echo -e "  - Bag: /tmp/kf_gins_comparison_YYYYmmdd_HHMMSS (自动生成)\n"

echo "🚁 启动无人机:"
echo -e "  在 PX4 pxh 命令行执行："
echo -e "    ${BLUE}pxh> commander check${NC}"
echo -e "    ${BLUE}pxh> commander arm${NC}"
echo -e "    ${BLUE}pxh> commander takeoff${NC}\n"

echo "⏱️ 测试计时:"
echo -e "  启动时间: $(date '+%H:%M:%S')"
echo -e "  预计运行时长: $DURATION 秒"
echo -e "  预计完成时间: $(date -d "+$DURATION seconds" '+%H:%M:%S')\n"

echo "🛑 停止测试:"
echo -e "  ${YELLOW}kill $LAUNCHER_PID${NC} (停止所有节点)"
echo -e "  或在 PX4 pxh 中: ${YELLOW}commander land${NC}\n"

echo "📈 离线分析:"
echo -e "  测试完成后，运行:"
echo -e "  ${BLUE}python3 \\${NC}"
echo -e "    ${BLUE}~/kf_gins_ws/src/kf_gins_ros2_native/scripts/offline_analysis.py \\${NC}"
echo -e "    ${BLUE}--rosbag /tmp/kf_gins_comparison_YYYYmmdd_HHMMSS \\${NC}"
echo -e "    ${BLUE}--scenario $SCENARIO \\${NC}"
echo -e "    ${BLUE}--output $DATA_DIR/${NC}\n"

echo "📚 更多信息:"
echo -e "  查看完整指南: ${BLUE}cat ~/kf_gins_ws/tools/EKF_IEKF_COMPARISON_GUIDE.md${NC}\n"

# 可选：自动停止计时
	if [ -n "$DURATION" ] && [ "$DURATION" != "0" ]; then
	    echo "⏲️ ${DURATION}秒后自动停止测试..."
	    sleep "$DURATION"
    
    echo -e "\n${YELLOW}正在停止测试...${NC}"
    kill $LAUNCHER_PID 2>/dev/null || true
    
	    echo -e "${GREEN}✓ 测试已停止${NC}"
	    echo -e "\n${YELLOW}开始离线分析...${NC}"

	    BAG_DIR=$(ls -td /tmp/kf_gins_comparison_* 2>/dev/null | head -n 1 || true)
	    if [ -z "$BAG_DIR" ]; then
	        echo -e "${YELLOW}⚠️  未找到 /tmp/kf_gins_comparison_*，跳过离线分析${NC}"
	    else
	        echo -e "${GREEN}✓ 使用 rosbag: $BAG_DIR${NC}"
	        python3 "$WS_ROOT/src/kf_gins_ros2_native/scripts/offline_analysis.py" \
	            --rosbag "$BAG_DIR" \
	            --scenario "$SCENARIO" \
	            --output "$DATA_DIR" || true
	    fi
	    
	    echo -e "\n${GREEN}✅ 完整的对比测试已完成！${NC}"
	    echo -e "结果位置: ${BLUE}$DATA_DIR/$SCENARIO/${NC}\n"
	fi
