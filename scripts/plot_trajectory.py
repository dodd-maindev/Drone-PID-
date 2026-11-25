#!/usr/bin/env python3
"""
scripts/plot_trajectory.py
===============================================================================
Công cụ phân tích và vẽ đồ thị bám quỹ đạo bay tự động (Trajectory Tracking Analysis)
theo tiêu chuẩn bài báo khoa học / đồ án NCKH (IEEE / Springer format).

Đầu vào: File CSV từ TrajectoryLogger (logs/trajectory/latest_trajectory.csv)
Đầu ra:
  1. SVG Publication Plot: 4 hàng x(t), y(t), z(t), yaw(t) so sánh Lý thuyết vs Thực tế
  2. Báo cáo định lượng (Quantitative Metrics): RMSE (x,y,z,3D,yaw), MAE, Max Error
  3. Interactive HTML Dashboard: trajectory_report.html xem trực tiếp trên trình duyệt
  4. PNG (nếu matplotlib khả dụng)
===============================================================================
"""

import sys
import os
import csv
import math
import glob

def find_latest_csv(preferred_path=None):
    if preferred_path and os.path.isfile(preferred_path):
        return preferred_path
    
    candidates = [
        "logs/trajectory/latest_trajectory.csv",
        "DroneHITL/logs/trajectory/latest_trajectory.csv",
        "../logs/trajectory/latest_trajectory.csv",
        "/mnt/c/Users/ASUS/Drone-PID-/logs/trajectory/latest_trajectory.csv",
        "/mnt/c/DroneHITL/logs/trajectory/latest_trajectory.csv"
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    
    # Tìm file traj_*.csv mới nhất
    search_patterns = [
        "logs/trajectory/traj_*.csv",
        "DroneHITL/logs/trajectory/traj_*.csv"
    ]
    found_files = []
    for pat in search_patterns:
        found_files.extend(glob.glob(pat))
    if found_files:
        found_files.sort(key=os.path.getmtime, reverse=True)
        return found_files[0]
    return None

def parse_csv(csv_path):
    data = {
        'time': [],
        'x_ref': [], 'y_ref': [], 'z_ref': [], 'yaw_ref_rad': [], 'yaw_ref_deg': [],
        'x_act': [], 'y_act': [], 'z_act': [], 'yaw_act_rad': [], 'yaw_act_deg': [],
        'err_x': [], 'err_y': [], 'err_z': [], 'err_dist_3d': [], 'err_yaw_deg': [],
        'vx_act': [], 'vy_act': [], 'vz_act': [],
        'wp_seq': [], 'nav_mode': []
    }
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row.get('time_s', 0.0))
                xr = float(row.get('target_x', 0.0))
                yr = float(row.get('target_y', 0.0))
                zr = float(row.get('target_z', 0.0))
                ywr_rad = float(row.get('target_yaw_rad', 0.0))
                ywr_deg = float(row.get('target_yaw_deg', 0.0))

                xa = float(row.get('actual_x', 0.0))
                ya = float(row.get('actual_y', 0.0))
                za = float(row.get('actual_z', 0.0))
                ywa_rad = float(row.get('actual_yaw_rad', 0.0))
                ywa_deg = float(row.get('actual_yaw_deg', 0.0))

                ex = float(row.get('err_x', xa - xr))
                ey = float(row.get('err_y', ya - yr))
                ez = float(row.get('err_z', za - zr))
                edist = float(row.get('err_dist_3d', math.sqrt(ex*ex + ey*ey + ez*ez)))
                eyaw = float(row.get('err_yaw_deg', ywa_deg - ywr_deg))

                data['time'].append(t)
                data['x_ref'].append(xr)
                data['y_ref'].append(yr)
                data['z_ref'].append(zr)
                data['yaw_ref_rad'].append(ywr_rad)
                data['yaw_ref_deg'].append(ywr_deg)

                data['x_act'].append(xa)
                data['y_act'].append(ya)
                data['z_act'].append(za)
                data['yaw_act_rad'].append(ywa_rad)
                data['yaw_act_deg'].append(ywa_deg)

                data['err_x'].append(ex)
                data['err_y'].append(ey)
                data['err_z'].append(ez)
                data['err_dist_3d'].append(edist)
                data['err_yaw_deg'].append(eyaw)

                data['vx_act'].append(float(row.get('actual_vx', 0.0)))
                data['vy_act'].append(float(row.get('actual_vy', 0.0)))
                data['vz_act'].append(float(row.get('actual_vz', 0.0)))
                data['wp_seq'].append(int(row.get('wp_seq', 0)))
                data['nav_mode'].append(int(row.get('nav_mode', 0)))
            except (ValueError, KeyError):
                continue
    return data

def unwrap_angles(angles):
    if not angles:
        return []
    unwrapped = [angles[0]]
    for i in range(1, len(angles)):
        diff = angles[i] - angles[i-1]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + diff)
    return unwrapped

def compute_metrics(data):
    N = len(data['time'])
    if N == 0:
        return {}
    
    # Chuẩn hóa thời gian bắt đầu từ 0
    t0 = data['time'][0]
    data['time'] = [t - t0 for t in data['time']]

    # Unwrap yaw angles để không bị nhảy đứt đoạn ±π trên đồ thị
    data['yaw_ref_rad'] = unwrap_angles(data['yaw_ref_rad'])
    data['yaw_act_rad'] = unwrap_angles(data['yaw_act_rad'])

    def rmse(err_list):
        return math.sqrt(sum(e * e for e in err_list) / len(err_list)) if err_list else 0.0
    
    def mae(err_list):
        return sum(abs(e) for e in err_list) / len(err_list) if err_list else 0.0

    rmse_x = rmse(data['err_x'])
    rmse_y = rmse(data['err_y'])
    rmse_z = rmse(data['err_z'])
    rmse_3d = rmse(data['err_dist_3d'])
    
    # Chuẩn hóa góc yaw delta
    yaw_errs_rad = []
    for a, r in zip(data['yaw_act_rad'], data['yaw_ref_rad']):
        d = a - r
        while d > math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        yaw_errs_rad.append(d)
    
    rmse_yaw_rad = rmse(yaw_errs_rad)
    rmse_yaw_deg = rmse_yaw_rad * 180.0 / math.pi
    
    mae_x = mae(data['err_x'])
    mae_y = mae(data['err_y'])
    mae_z = mae(data['err_z'])
    mae_3d = mae(data['err_dist_3d'])
    max_err_3d = max(data['err_dist_3d']) if data['err_dist_3d'] else 0.0
    flight_time = data['time'][-1] if data['time'] else 0.0

    return {
        'N': N,
        'flight_time_s': flight_time,
        'rmse_x': rmse_x,
        'rmse_y': rmse_y,
        'rmse_z': rmse_z,
        'rmse_3d': rmse_3d,
        'rmse_yaw_rad': rmse_yaw_rad,
        'rmse_yaw_deg': rmse_yaw_deg,
        'mae_x': mae_x,
        'mae_y': mae_y,
        'mae_z': mae_z,
        'mae_3d': mae_3d,
        'max_err_3d': max_err_3d
    }

def generate_svg_plot(data, metrics, out_svg_path):
    """
    Vẽ đồ thị chuẩn bài báo khoa học ra định dạng vector SVG thuần (Zero Dependencies)
    với 4 đồ thị con: X(t), Y(t), Z(t), Yaw(t).
    """
    N = len(data['time'])
    if N < 2:
        return
    
    t_min = data['time'][0]
    t_max = max(data['time'][-1], 0.1)

    width = 960
    height = 1000
    margin_left = 90
    margin_right = 50
    margin_top = 80
    margin_bottom = 50
    plot_w = width - margin_left - margin_right
    sub_h = 180
    spacing = 45

    subplots_info = [
        ('x (m)', data['x_ref'], data['x_act']),
        ('y (m)', data['y_ref'], data['y_act']),
        ('z (m)', data['z_ref'], data['z_act']),
        ('yaw (rad)', data['yaw_ref_rad'], data['yaw_act_rad'])
    ]

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" style="background-color:#ffffff; font-family: -apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Arial, sans-serif;">')

    # Header Title
    svg.append(f'<text x="{width/2}" y="32" font-size="18" font-weight="bold" text-anchor="middle" fill="#111827">ĐỒ THỊ ĐỊNH LƯỢNG BÁM QUỸ ĐẠO BAY TỰ ĐỘNG (TRAJECTORY TRACKING BENCHMARK)</text>')
    
    # Subtitle with Quantitative Metrics
    metrics_str = f"RMSE 3D: {metrics['rmse_3d']:.3f}m | Max Error: {metrics['max_err_3d']:.3f}m | RMSE Z: {metrics['rmse_z']:.3f}m | RMSE Yaw: {metrics['rmse_yaw_rad']:.3f} rad ({metrics['rmse_yaw_deg']:.1f}°) | Thời gian bay: {metrics['flight_time_s']:.1f}s ({metrics['N']} mẫu)"
    svg.append(f'<text x="{width/2}" y="52" font-size="12" font-weight="500" text-anchor="middle" fill="#4B5563">{metrics_str}</text>')

    # Global Legend (Top Right)
    leg_x = width - margin_right - 280
    leg_y = 48
    svg.append(f'<rect x="{leg_x}" y="38" width="270" height="28" rx="4" fill="#F3F4F6" stroke="#D1D5DB" stroke-width="1"/>')
    # Ref: Solid Red
    svg.append(f'<line x1="{leg_x+10}" y1="{leg_y}" x2="{leg_x+35}" y2="{leg_y}" stroke="#DC2626" stroke-width="2.5"/>')
    svg.append(f'<text x="{leg_x+42}" y="{leg_y+4}" font-size="11" font-weight="600" fill="#DC2626">expected trajectory</text>')
    # Act: Dashed Blue
    svg.append(f'<line x1="{leg_x+160}" y1="{leg_y}" x2="{leg_x+185}" y2="{leg_y}" stroke="#2563EB" stroke-width="2.5" stroke-dasharray="6,4"/>')
    svg.append(f'<text x="{leg_x+192}" y="{leg_y+4}" font-size="11" font-weight="600" fill="#2563EB">TExp (PID Actual)</text>')

    for idx, (label, y_ref, y_act) in enumerate(subplots_info):
        plot_y = margin_top + idx * (sub_h + spacing)
        
        # Calculate Y limits with padding
        all_y = y_ref + y_act
        min_val = min(all_y)
        max_val = max(all_y)
        span = max_val - min_val
        if span < 0.2:
            span = 0.2
        pad = span * 0.15
        y_min = min_val - pad
        y_max = max_val + pad

        # Background box
        svg.append(f'<rect x="{margin_left}" y="{plot_y}" width="{plot_w}" height="{sub_h}" fill="#FAFAFA" stroke="#9CA3AF" stroke-width="1.2"/>')

        # Grid lines (5 horizontal, 6 vertical)
        for i in range(5):
            val = y_min + (y_max - y_min) * (i / 4.0)
            py = plot_y + sub_h - (i / 4.0) * sub_h
            svg.append(f'<line x1="{margin_left}" y1="{py}" x2="{margin_left+plot_w}" y2="{py}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
            svg.append(f'<text x="{margin_left-10}" y="{py+4}" font-size="11" text-anchor="end" fill="#4B5563">{val:.2f}</text>')

        for i in range(6):
            t_val = t_min + (t_max - t_min) * (i / 5.0)
            px = margin_left + (i / 5.0) * plot_w
            svg.append(f'<line x1="{px}" y1="{plot_y}" x2="{px}" y2="{plot_y+sub_h}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
            if idx == 3: # Only bottom subplot shows time labels
                svg.append(f'<text x="{px}" y="{plot_y+sub_h+16}" font-size="11" text-anchor="middle" fill="#4B5563">{t_val:.1f}</text>')

        # Y-axis Label
        svg.append(f'<text x="{margin_left-50}" y="{plot_y + sub_h/2}" font-size="13" font-weight="bold" text-anchor="middle" fill="#1F2937" transform="rotate(-90 {margin_left-50} {plot_y + sub_h/2})">{label}</text>')
        
        if idx == 3:
            svg.append(f'<text x="{margin_left + plot_w/2}" y="{plot_y+sub_h+36}" font-size="13" font-weight="bold" text-anchor="middle" fill="#1F2937">t (s)</text>')

        # Downsample points for SVG path if N is huge (> 1500)
        step = max(1, N // 1000)
        indices = list(range(0, N, step))
        if indices[-1] != N - 1:
            indices.append(N - 1)

        def make_path(y_vals):
            pts = []
            for i in indices:
                t = data['time'][i]
                y = y_vals[i]
                px = margin_left + ((t - t_min) / (t_max - t_min)) * plot_w
                py = plot_y + sub_h - ((y - y_min) / (y_max - y_min)) * sub_h
                pts.append(f"{px:.1f},{py:.1f}")
            return "M " + " L ".join(pts)

        path_ref = make_path(y_ref)
        path_act = make_path(y_act)

        # Draw Expected (Red solid)
        svg.append(f'<path d="{path_ref}" fill="none" stroke="#DC2626" stroke-width="2.2" stroke-linejoin="round"/>')
        # Draw Actual (Blue dashed)
        svg.append(f'<path d="{path_act}" fill="none" stroke="#2563EB" stroke-width="2.0" stroke-dasharray="7,4" stroke-linejoin="round"/>')

    svg.append('</svg>')

    with open(out_svg_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(svg))
    print(f"[✓ SVG Plot] Đã xuất đồ thị vector chất lượng cao: {out_svg_path}")

def generate_interactive_html(data, metrics, out_html_path):
    """
    Sinh trang Web Dashboard HTML tương tác có sẵn đồ thị 2D & 3D (dùng Plotly CDN)
    Mở trực tiếp trên mọi trình duyệt Windows / Linux mà không cần cài thêm thư viện!
    """
    N = len(data['time'])
    step = max(1, N // 800)
    idx_list = list(range(0, N, step))
    if idx_list[-1] != N - 1:
        idx_list.append(N - 1)

    t_json = [round(data['time'][i], 2) for i in idx_list]
    xr_json = [round(data['x_ref'][i], 3) for i in idx_list]
    xa_json = [round(data['x_act'][i], 3) for i in idx_list]
    yr_json = [round(data['y_ref'][i], 3) for i in idx_list]
    ya_json = [round(data['y_act'][i], 3) for i in idx_list]
    zr_json = [round(data['z_ref'][i], 3) for i in idx_list]
    za_json = [round(data['z_act'][i], 3) for i in idx_list]
    ywr_json = [round(data['yaw_ref_rad'][i], 3) for i in idx_list]
    ywa_json = [round(data['yaw_act_rad'][i], 3) for i in idx_list]
    err3d_json = [round(data['err_dist_3d'][i], 3) for i in idx_list]

    html_content = f"""<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Báo cáo Bám Quỹ Đạo Bay Drone HITL ESP32</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
            margin: 0;
            padding: 24px;
            background-color: #0f172a;
            color: #f8fafc;
        }}
        .header {{
            text-align: center;
            margin-bottom: 24px;
            padding: 20px;
            background: #1e293b;
            border-radius: 12px;
            border: 1px solid #334155;
        }}
        h1 {{ margin: 0 0 8px 0; font-size: 24px; color: #38bdf8; }}
        p.sub {{ margin: 0; font-size: 14px; color: #94a3b8; }}
        .metrics-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 16px;
            margin-bottom: 24px;
        }}
        .card {{
            background: #1e293b;
            padding: 16px;
            border-radius: 10px;
            border: 1px solid #334155;
            text-align: center;
        }}
        .card .title {{ font-size: 13px; color: #94a3b8; text-transform: uppercase; font-weight: 600; }}
        .card .val {{ font-size: 26px; font-weight: 700; color: #38bdf8; margin: 8px 0 0 0; }}
        .card .unit {{ font-size: 12px; color: #64748b; }}
        .chart-box {{
            background: #1e293b;
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 24px;
            border: 1px solid #334155;
        }}
        .chart-title {{ font-size: 16px; font-weight: 600; color: #e2e8f0; margin-bottom: 12px; }}
        .btn-export {{
            background: #0284c7; color: white; border: none; padding: 10px 18px; border-radius: 6px;
            font-size: 14px; font-weight: 600; cursor: pointer; float: right;
        }}
        .btn-export:hover {{ background: #0369a1; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>📊 BÁO CÁO ĐỊNH LƯỢNG BÁM QUỸ ĐẠO (TRAJECTORY TRACKING REPORT)</h1>
        <p class="sub">Hệ thống điều khiển Quadrotor x500 HITL ESP32 Firmware | So sánh Quỹ đạo Lý thuyết vs Thực tế</p>
    </div>

    <div class="metrics-grid">
        <div class="card">
            <div class="title">Sai số Không gian 3D (RMSE)</div>
            <div class="val">{metrics['rmse_3d']:.3f}</div>
            <div class="unit">mét (m)</div>
        </div>
        <div class="card">
            <div class="title">Sai số 3D Tối Đa (Max Error)</div>
            <div class="val">{metrics['max_err_3d']:.3f}</div>
            <div class="unit">mét (m)</div>
        </div>
        <div class="card">
            <div class="title">Sai số Độ cao Z (RMSE Z)</div>
            <div class="val">{metrics['rmse_z']:.3f}</div>
            <div class="unit">mét (m)</div>
        </div>
        <div class="card">
            <div class="title">Sai số Hướng Yaw (RMSE Yaw)</div>
            <div class="val">{metrics['rmse_yaw_deg']:.1f}°</div>
            <div class="unit">{metrics['rmse_yaw_rad']:.3f} rad</div>
        </div>
        <div class="card">
            <div class="title">Thời gian thực thi</div>
            <div class="val">{metrics['flight_time_s']:.1f}</div>
            <div class="unit">giây ({metrics['N']} mẫu dữ liệu)</div>
        </div>
    </div>

    <div class="chart-box">
        <div class="chart-title">🌐 Quỹ Đạo Không Gian 3D (3D Flight Path vs Expected Waypoints)</div>
        <div id="plot3d" style="height: 540px;"></div>
    </div>

    <div class="chart-box">
        <div class="chart-title">🗺️ Mặt Bằng Quỹ Đạo Mặt Đất 2D (Top-Down 2D Ground Track: X vs Y)</div>
        <div id="plot2d" style="height: 520px;"></div>
    </div>

    <div class="chart-box">
        <div class="chart-title">📈 Phân Tích Định Lượng Từng Trục: X(t), Y(t), Z(t), Yaw(t) & Sai Số 3D(t)</div>
        <div id="plot_sub" style="height: 850px;"></div>
    </div>

    <script>
        const t = {t_json};
        const xr = {xr_json};
        const xa = {xa_json};
        const yr = {yr_json};
        const ya = {ya_json};
        const zr = {zr_json};
        const za = {za_json};
        const ywr = {ywr_json};
        const ywa = {ywa_json};
        const err3d = {err3d_json};

        // 3D Plot
        const traceRef3d = {{
            x: xr, y: yr, z: zr,
            type: 'scatter3d', mode: 'lines',
            name: 'Expected Trajectory',
            line: {{ color: '#ef4444', width: 6 }}
        }};
        const traceAct3d = {{
            x: xa, y: ya, z: za,
            type: 'scatter3d', mode: 'lines',
            name: 'Actual Trajectory (TExp)',
            line: {{ color: '#38bdf8', width: 4, dash: 'dash' }}
        }};
        const layout3d = {{
            paper_bgcolor: '#1e293b',
            plot_bgcolor: '#1e293b',
            font: {{ color: '#f8fafc' }},
            scene: {{
                xaxis: {{ title: 'X (North, m)', gridcolor: '#334155' }},
                yaxis: {{ title: 'Y (East, m)', gridcolor: '#334155' }},
                zaxis: {{ title: 'Z (Alt, m)', gridcolor: '#334155' }},
                camera: {{ eye: {{ x: 1.5, y: 1.5, z: 1.2 }} }}
            }},
            margin: {{ l: 0, r: 0, b: 0, t: 20 }}
        }};
        Plotly.newPlot('plot3d', [traceRef3d, traceAct3d], layout3d);

        // 2D Ground Track (X vs Y) - Aspect ratio 1:1
        const traceRef2d = {{
            x: yr, y: xr,
            type: 'scatter', mode: 'lines+markers',
            name: 'Expected Path (Waypoints)',
            line: {{ color: '#ef4444', width: 3 }},
            marker: {{ size: 7, color: '#ef4444', symbol: 'circle' }}
        }};
        const traceAct2d = {{
            x: ya, y: xa,
            type: 'scatter', mode: 'lines',
            name: 'Actual Flight Track (TExp)',
            line: {{ color: '#38bdf8', width: 2.5, dash: 'dash' }}
        }};
        const layout2d = {{
            paper_bgcolor: '#1e293b',
            plot_bgcolor: '#1e293b',
            font: {{ color: '#f8fafc' }},
            xaxis: {{ title: 'East Y (m)', gridcolor: '#334155', scaleanchor: 'y', scaleratio: 1 }},
            yaxis: {{ title: 'North X (m)', gridcolor: '#334155' }},
            margin: {{ l: 60, r: 30, t: 30, b: 50 }},
            legend: {{ orientation: 'h', y: 1.08 }}
        }};
        Plotly.newPlot('plot2d', [traceRef2d, traceAct2d], layout2d);

        // Subplots
        const traces = [
            // Row 1: X(t)
            {{ x: t, y: xr, xaxis: 'x', yaxis: 'y', name: 'X Expected', line: {{ color: '#ef4444', width: 2.2 }} }},
            {{ x: t, y: xa, xaxis: 'x', yaxis: 'y', name: 'X Actual', line: {{ color: '#38bdf8', width: 2, dash: 'dash' }} }},
            // Row 2: Y(t)
            {{ x: t, y: yr, xaxis: 'x2', yaxis: 'y2', name: 'Y Expected', line: {{ color: '#ef4444', width: 2.2 }} }},
            {{ x: t, y: ya, xaxis: 'x2', yaxis: 'y2', name: 'Y Actual', line: {{ color: '#38bdf8', width: 2, dash: 'dash' }} }},
            // Row 3: Z(t)
            {{ x: t, y: zr, xaxis: 'x3', yaxis: 'y3', name: 'Z Expected', line: {{ color: '#ef4444', width: 2.2 }} }},
            {{ x: t, y: za, xaxis: 'x3', yaxis: 'y3', name: 'Z Actual', line: {{ color: '#38bdf8', width: 2, dash: 'dash' }} }},
            // Row 4: Yaw(t)
            {{ x: t, y: ywr, xaxis: 'x4', yaxis: 'y4', name: 'Yaw Expected (rad)', line: {{ color: '#ef4444', width: 2.2 }} }},
            {{ x: t, y: ywa, xaxis: 'x4', yaxis: 'y4', name: 'Yaw Actual (rad)', line: {{ color: '#38bdf8', width: 2, dash: 'dash' }} }},
            // Row 5: 3D Tracking Error
            {{ x: t, y: err3d, xaxis: 'x5', yaxis: 'y5', name: '3D Error (m)', line: {{ color: '#f59e0b', width: 2 }} }}
        ];

        const layoutSub = {{
            paper_bgcolor: '#1e293b',
            plot_bgcolor: '#1e293b',
            font: {{ color: '#f8fafc' }},
            grid: {{ rows: 5, columns: 1, pattern: 'independent' }},
            yaxis: {{ title: 'X (m)', gridcolor: '#334155' }},
            yaxis2: {{ title: 'Y (m)', gridcolor: '#334155' }},
            yaxis3: {{ title: 'Z (m)', gridcolor: '#334155' }},
            yaxis4: {{ title: 'Yaw (rad)', gridcolor: '#334155' }},
            yaxis5: {{ title: 'Error (m)', gridcolor: '#334155' }},
            xaxis5: {{ title: 'Thời gian t (giây)' }},
            showlegend: true,
            legend: {{ orientation: 'h', y: 1.05 }},
            margin: {{ l: 60, r: 30, t: 30, b: 50 }}
        }};
        Plotly.newPlot('plot_sub', traces, layoutSub);
    </script>
</body>
</html>
"""
    with open(out_html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    print(f"[✓ HTML Dashboard] Đã xuất trang Dashboard tương tác: {out_html_path}")

def try_matplotlib_png(data, metrics, out_png_path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        t = data['time']
        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        
        # Subplot 1: x(t)
        axes[0].plot(t, data['x_ref'], 'r-', label='expected trajectory', linewidth=2.0)
        axes[0].plot(t, data['x_act'], 'b--', label='TExp (PID Actual)', linewidth=1.8)
        axes[0].set_ylabel('x (m)', fontsize=12, fontweight='bold')
        axes[0].grid(True, linestyle='--', alpha=0.6)
        axes[0].legend(loc='upper right', frameon=True)

        # Subplot 2: y(t)
        axes[1].plot(t, data['y_ref'], 'r-', label='expected trajectory', linewidth=2.0)
        axes[1].plot(t, data['y_act'], 'b--', label='TExp (PID Actual)', linewidth=1.8)
        axes[1].set_ylabel('y (m)', fontsize=12, fontweight='bold')
        axes[1].grid(True, linestyle='--', alpha=0.6)

        # Subplot 3: z(t)
        axes[2].plot(t, data['z_ref'], 'r-', label='expected trajectory', linewidth=2.0)
        axes[2].plot(t, data['z_act'], 'b--', label='TExp (PID Actual)', linewidth=1.8)
        axes[2].set_ylabel('z (m)', fontsize=12, fontweight='bold')
        axes[2].grid(True, linestyle='--', alpha=0.6)

        # Subplot 4: yaw(t)
        axes[3].plot(t, data['yaw_ref_rad'], 'r-', label='expected trajectory', linewidth=2.0)
        axes[3].plot(t, data['yaw_act_rad'], 'b--', label='TExp (PID Actual)', linewidth=1.8)
        axes[3].set_ylabel('yaw (rad)', fontsize=12, fontweight='bold')
        axes[3].set_xlabel('t (s)', fontsize=12, fontweight='bold')
        axes[3].grid(True, linestyle='--', alpha=0.6)

        fig.suptitle(f"TRAJECTORY TRACKING PERFORMANCE (RMSE 3D = {metrics['rmse_3d']:.3f}m | Max Err = {metrics['max_err_3d']:.3f}m)",
                     fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(out_png_path, dpi=300)
        plt.close()
        print(f"[✓ PNG Plot] Đã xuất ảnh 300 DPI: {out_png_path}")
    except Exception as e:
        # Nếu chưa có matplotlib, báo qua
        pass

def main():
    target_csv = None
    if len(sys.argv) > 1:
        target_csv = sys.argv[1]
    
    csv_file = find_latest_csv(target_csv)
    if not csv_file:
        print("[!] Không tìm thấy file log quỹ đạo CSV. Hãy chạy bay với cờ --auto trước.")
        sys.exit(1)
    
    print(f"[+] Đang phân tích file log: {csv_file}")
    data = parse_csv(csv_file)
    if len(data['time']) == 0:
        print("[!] File CSV rỗng hoặc không chứa dữ liệu bám quỹ đạo hợp lệ.")
        sys.exit(1)

    metrics = compute_metrics(data)
    
    print("=" * 60)
    print(" KẾT QUẢ ĐỊNH LƯỢNG BÁM QUỸ ĐẠO BAY (QUANTITATIVE BENCHMARK)")
    print("=" * 60)
    print(f"  • Tổng thời gian bay       : {metrics['flight_time_s']:.2f} giây ({metrics['N']} điểm mẫu)")
    print(f"  • Sai số 3D (RMSE 3D)      : {metrics['rmse_3d']:.4f} m")
    print(f"  • Sai số 3D Tối đa (Max Err): {metrics['max_err_3d']:.4f} m")
    print(f"  • Sai số Trung bình (MAE 3D): {metrics['mae_3d']:.4f} m")
    print(f"  • Sai số Trục X (RMSE X)   : {metrics['rmse_x']:.4f} m (MAE: {metrics['mae_x']:.4f} m)")
    print(f"  • Sai số Trục Y (RMSE Y)   : {metrics['rmse_y']:.4f} m (MAE: {metrics['mae_y']:.4f} m)")
    print(f"  • Sai số Độ cao Z (RMSE Z) : {metrics['rmse_z']:.4f} m (MAE: {metrics['mae_z']:.4f} m)")
    print(f"  • Sai số Hướng Yaw (RMSE)  : {metrics['rmse_yaw_rad']:.4f} rad ({metrics['rmse_yaw_deg']:.2f}°)")
    print("=" * 60)

    out_dir = os.path.dirname(csv_file)
    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    out_svg = os.path.join(out_dir, f"{base_name}_plot.svg")
    out_html = os.path.join(out_dir, f"{base_name}_report.html")
    out_png = os.path.join(out_dir, f"{base_name}_plot.png")

    generate_svg_plot(data, metrics, out_svg)
    generate_interactive_html(data, metrics, out_html)
    try_matplotlib_png(data, metrics, out_png)

    # Đồng bộ sang DroneHITL trên Windows đúng thư mục tương ứng để tiện xem
    norm_out_dir = os.path.normpath(out_dir)
    if not norm_out_dir.startswith("DroneHITL") and os.path.isdir("DroneHITL"):
        win_out_dir = os.path.join("DroneHITL", norm_out_dir)
        os.makedirs(win_out_dir, exist_ok=True)
        import shutil
        for src in [out_svg, out_html, out_png]:
            if os.path.isfile(src):
                dst = os.path.join(win_out_dir, os.path.basename(src))
                shutil.copyfile(src, dst)
        print(f"[✓ Đồng bộ] Đã sao chép sang thư mục Windows: {win_out_dir}")

if __name__ == '__main__':
    main()

