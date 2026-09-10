#!/usr/bin/env python3
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_flight(csv_path="/home/do/drone_control_cpp/log/latest_flight.csv"):
    if not os.path.exists(csv_path):
        print(f"[!] Không tìm thấy file log: {csv_path}")
        return

    print(f"[*] Đang vẽ đồ thị từ: {csv_path}...")
    df = pd.read_csv(csv_path)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("DỮ LIỆU BAY THỰC TẾ GỬI SANG GAZEBO (TELEMETRY & MOTOR COMMANDS)", fontsize=14, fontweight="bold")

    # 1. Đồ thị Độ cao & Vận tốc thẳng đứng
    ax1 = axes[0]
    ax1.plot(df['time_s'], df['actual_alt_m'], label='Actual Alt (m)', color='blue', linewidth=2)
    if 'actual_vz_mps' in df.columns:
        ax1.plot(df['time_s'], df['actual_vz_mps'], label='Vz (m/s)', color='cyan', linestyle='--', alpha=0.7)
    ax1.set_ylabel("Độ cao (m)")
    ax1.set_title("Độ cao thực tế và vận tốc Z")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper left")

    # 2. Đồ thị Góc nghiêng Roll / Pitch
    ax2 = axes[1]
    ax2.plot(df['time_s'], df['target_roll_deg'], label='Target Roll (deg)', color='red', linestyle=':')
    ax2.plot(df['time_s'], df['actual_roll_deg'], label='Actual Roll (deg)', color='darkred', linewidth=1.5)
    ax2.plot(df['time_s'], df['target_pitch_deg'], label='Target Pitch (deg)', color='green', linestyle=':')
    ax2.plot(df['time_s'], df['actual_pitch_deg'], label='Actual Pitch (deg)', color='darkgreen', linewidth=1.5)
    ax2.set_ylabel("Góc (độ)")
    ax2.set_title("Góc nghiêng Roll & Pitch")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper left")

    # 3. Đồ thị Tốc độ 4 động cơ gửi sang Gazebo (rad/s)
    ax3 = axes[2]
    ax3.plot(df['time_s'], df['w0_rad_s'], label='w0 Front-Right (rad/s)', color='red', alpha=0.8)
    ax3.plot(df['time_s'], df['w1_rad_s'], label='w1 Rear-Left (rad/s)', color='blue', alpha=0.8)
    ax3.plot(df['time_s'], df['w2_rad_s'], label='w2 Front-Left (rad/s)', color='green', alpha=0.8)
    ax3.plot(df['time_s'], df['w3_rad_s'], label='w3 Rear-Right (rad/s)', color='orange', alpha=0.8)
    ax3.set_xlabel("Thời gian (giây)")
    ax3.set_ylabel("Tốc độ quay (rad/s)")
    ax3.set_title("Tốc độ 4 cánh quạt gửi sang Gazebo (/x500/command/motor_speed)")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper left")

    plt.tight_layout()
    output_png = csv_path.replace(".csv", "_plot.png")
    plt.savefig(output_png, dpi=150)
    print(f"[✓] Đã lưu hình đồ thị tại: {output_png}")

if __name__ == "__main__":
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "/home/do/drone_control_cpp/log/latest_flight.csv"
    plot_flight(csv_file)
