#!/bin/bash
# ==============================================================================
# Script chuyển tiếp khởi chạy Drone HITL Simulator
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/scripts/run_hitl_launcher.sh" ]; then
    exec "$SCRIPT_DIR/scripts/run_hitl_launcher.sh" "$@"
elif [ -f "$SCRIPT_DIR/DroneHITL/run.sh" ]; then
    exec "$SCRIPT_DIR/DroneHITL/run.sh" "$@"
else
    echo "Lỗi: Không tìm thấy script khởi chạy!"
    exit 1
fi

