#!/bin/bash
# ==============================================================================
# Script chuyển tiếp khởi chạy Drone HITL Simulator
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/DroneHITL/run.sh" "$@"

