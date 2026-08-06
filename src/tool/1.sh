#!/usr/bin/env bash
# ======================================================
# 键盘输入延迟诊断脚本 v3 （无 bc、无整数比较错误）
# 作者: ChatGPT GPT-5
# ======================================================

LOG_DIR="$HOME/keyboard_monitor_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/keyboard_delay_$(date +%F_%H-%M-%S).log"

echo "🧠 启动键盘延迟监控器..."
echo "日志路径: $LOG_FILE"
echo "--------------------------------------------"
echo "按 Ctrl + C 结束监控。"
echo "--------------------------------------------"

# ====== 基本信息 ======
{
    echo "系统信息快照 - $(date)"
    echo "--------------------------------------------"
    uname -a
    echo ""
    echo "GPU 驱动信息:"
    nvidia-smi | head -n 10 2>/dev/null || echo "未检测到 NVIDIA GPU"
    echo "--------------------------------------------"
} >> "$LOG_FILE"

# ====== 主循环 ======
while true; do
    TIMESTAMP=$(date +'%F %T')

    # 获取 CPU 空闲率（兼容中英文）
    CPU_LINE=$(LC_ALL=C top -bn1 | grep "Cpu(s)")
    IDLE=$(echo "$CPU_LINE" | awk '{for(i=1;i<=NF;i++) if($i ~ /id/) print $(i-1)}' | tr -d ',' | tr ',' '.')
    # 若为空，设为 0，防止报错
    [ -z "$IDLE" ] && IDLE=0
    CPU_USAGE=$(awk "BEGIN {print 100 - $IDLE}")

    # 获取内存使用率
    MEM_USAGE=$(free -m | awk '/Mem:/ { printf "%.1f", $3/$2*100 }')
    [ -z "$MEM_USAGE" ] && MEM_USAGE=0

    # 转换为整数（取整以防小数导致错误）
    CPU_INT=$(printf "%.0f" "$CPU_USAGE" 2>/dev/null || echo 0)
    MEM_INT=$(printf "%.0f" "$MEM_USAGE" 2>/dev/null || echo 0)

    # 获取输入法与 GPU 状态
    INPUT_METHOD=$(ps -e | grep -E "ibus|fcitx|fcitx5" | awk '{print $4}' | tr '\n' ' ')
    [ -z "$INPUT_METHOD" ] && INPUT_METHOD="(未检测到输入法进程)"
    GPU_INFO=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits 2>/dev/null)
    [ -z "$GPU_INFO" ] && GPU_INFO="无 GPU 或 nvidia-smi 不可用"

    # 写入实时状态
    echo "[$TIMESTAMP] CPU: ${CPU_USAGE}% | MEM: ${MEM_USAGE}% | 输入法: ${INPUT_METHOD} | GPU: ${GPU_INFO}" >> "$LOG_FILE"

    # 检查 USB 键盘异常
    dmesg | tail -n 15 | grep -Ei "usb.*keyboard|input.*error|reset full-speed" >> "$LOG_FILE"

    # 检测高负载
    if [ "$CPU_INT" -ge 90 ] || [ "$MEM_INT" -ge 90 ]; then
        echo -e "\n⚠️ [$TIMESTAMP] 检测到高负载！保存详细快照..." >> "$LOG_FILE"
        {
            echo "--------------- top ---------------"
            LC_ALL=C top -bn1 | head -n 20
            echo "--------------- ps ---------------"
            ps aux --sort=-%cpu | head -n 15
            echo "--------------- dmesg ---------------"
            dmesg | tail -n 30
            echo "--------------------------------------------"
        } >> "$LOG_FILE"
    fi

    sleep 5
done
