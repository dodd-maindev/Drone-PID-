#!/usr/bin/env python3
"""
scripts/compare_trajectories.py
===============================================================================
Vẽ đồ thị so sánh 2 thuật toán (Thuật toán 1: ITSMC vs Thuật toán 2: RL Critic-Only + ESO)
trên CÙNG MỘT ĐỒ THỊ (SVG vector chuẩn bài báo + Interactive HTML Dashboard).

Cách dùng:
  python3 scripts/compare_trajectories.py
  python3 scripts/compare_trajectories.py <csv_algo1> <csv_algo2> [--label1 "ITSMC"] [--label2 "RL Critic+ESO"]
===============================================================================
"""

import argparse
import os
import shutil
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from plot_trajectory import parse_csv, compute_metrics


def find_algo_csv(candidates):
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


def polyline_points(t_list, val_list, t_min, t_max, y_min, y_max, margin_left, plot_y, plot_w, sub_h, step=1):
    pts = []
    t_span = max(t_max - t_min, 1e-6)
    y_span = max(y_max - y_min, 1e-6)
    for i in range(0, len(t_list), step):
        px = margin_left + ((t_list[i] - t_min) / t_span) * plot_w
        py = plot_y + sub_h - ((val_list[i] - y_min) / y_span) * sub_h
        pts.append(f"{px:.1f},{py:.1f}")
    if len(t_list) > 1 and (len(t_list) - 1) % step != 0:
        px = margin_left + ((t_list[-1] - t_min) / t_span) * plot_w
        py = plot_y + sub_h - ((val_list[-1] - y_min) / y_span) * sub_h
        pts.append(f"{px:.1f},{py:.1f}")
    return " ".join(pts)


def generate_compare_svg(d1, m1, label1, d2, m2, label2, out_svg_path):
    t1 = d1['time']
    t2 = d2['time']
    t_min = 0.0
    t_max = max(t1[-1] if t1 else 1.0, t2[-1] if t2 else 1.0, 0.1)

    width = 1040
    margin_left = 90
    margin_right = 50
    margin_top = 105
    plot_w = width - margin_left - margin_right
    sub_h = 165
    spacing = 42

    subplots = [
        ('x (m)',       d1['x_ref'],       d1['x_act'],       d2['x_ref'],       d2['x_act']),
        ('y (m)',       d1['y_ref'],       d1['y_act'],       d2['y_ref'],       d2['y_act']),
        ('z (m)',       d1['z_ref'],       d1['z_act'],       d2['z_ref'],       d2['z_act']),
        ('yaw (rad)',   d1['yaw_ref_rad'], d1['yaw_act_rad'], d2['yaw_ref_rad'], d2['yaw_act_rad']),
        ('err 3D (m)',  [0.0]*len(t1),     d1['err_dist_3d'], [0.0]*len(t2),     d2['err_dist_3d']),
    ]

    table_y = margin_top + len(subplots) * (sub_h + spacing) + 15
    height = table_y + 135

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
        f'style="background-color:#ffffff; font-family: -apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Arial, sans-serif;">'
    ]

    # Title & Subtitle
    svg.append(
        f'<text x="{width/2}" y="30" font-size="18" font-weight="bold" text-anchor="middle" fill="#111827">'
        f'SO SÁNH BÁM QUỸ ĐẠO: {label1} vs {label2}</text>'
    )
    svg.append(
        f'<text x="{width/2}" y="50" font-size="12" font-weight="500" text-anchor="middle" fill="#4B5563">'
        f'{label1}: RMSE 3D = {m1["rmse_3d"]:.3f}m, Max = {m1["max_err_3d"]:.3f}m ({m1["flight_time_s"]:.1f}s)  |  '
        f'{label2}: RMSE 3D = {m2["rmse_3d"]:.3f}m, Max = {m2["max_err_3d"]:.3f}m ({m2["flight_time_s"]:.1f}s)</text>'
    )

    # Legend Box
    leg_w = 640
    leg_x = (width - leg_w) / 2
    leg_y = 75
    svg.append(f'<rect x="{leg_x}" y="60" width="{leg_w}" height="28" rx="6" fill="#F8FAFC" stroke="#CBD5E1" stroke-width="1"/>')
    # Ref
    svg.append(f'<line x1="{leg_x+18}" y1="{leg_y}" x2="{leg_x+50}" y2="{leg_y}" stroke="#DC2626" stroke-width="2.4"/>')
    svg.append(f'<text x="{leg_x+58}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#DC2626">Expected Trajectory</text>')
    # Algo 1
    svg.append(f'<line x1="{leg_x+215}" y1="{leg_y}" x2="{leg_x+247}" y2="{leg_y}" stroke="#2563EB" stroke-width="2.4" stroke-dasharray="7,4"/>')
    svg.append(f'<text x="{leg_x+255}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#2563EB">{label1}</text>')
    # Algo 2
    svg.append(f'<line x1="{leg_x+415}" y1="{leg_y}" x2="{leg_x+447}" y2="{leg_y}" stroke="#059669" stroke-width="2.4" stroke-dasharray="7,4"/>')
    svg.append(f'<text x="{leg_x+455}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#059669">{label2}</text>')

    step1 = max(1, len(t1) // 1200)
    step2 = max(1, len(t2) // 1200)

    for idx, (ylabel, ref1, act1, ref2, act2) in enumerate(subplots):
        plot_y = margin_top + idx * (sub_h + spacing)
        all_vals = list(act1) + list(act2)
        if ref1 is not None:
            all_vals += list(ref1)
        if ref2 is not None:
            all_vals += list(ref2)

        min_val = min(all_vals)
        max_val = max(all_vals)
        span = max(max_val - min_val, 0.2)
        pad = span * 0.14
        y_min = min_val - pad
        y_max = max_val + pad

        svg.append(f'<rect x="{margin_left}" y="{plot_y}" width="{plot_w}" height="{sub_h}" fill="#FAFAFA" stroke="#9CA3AF" stroke-width="1.2"/>')

        # Horizontal grid
        for i in range(5):
            val = y_min + (y_max - y_min) * (i / 4.0)
            py = plot_y + sub_h - (i / 4.0) * sub_h
            svg.append(f'<line x1="{margin_left}" y1="{py:.1f}" x2="{margin_left+plot_w}" y2="{py:.1f}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
            svg.append(f'<text x="{margin_left-8}" y="{py+4:.1f}" font-size="11" text-anchor="end" fill="#4B5563">{val:.2f}</text>')

        # Vertical grid
        for i in range(7):
            tv = t_min + (t_max - t_min) * (i / 6.0)
            px = margin_left + (i / 6.0) * plot_w
            svg.append(f'<line x1="{px:.1f}" y1="{plot_y}" x2="{px:.1f}" y2="{plot_y+sub_h}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
            if idx == len(subplots) - 1:
                svg.append(f'<text x="{px:.1f}" y="{plot_y+sub_h+16}" font-size="11" text-anchor="middle" fill="#4B5563">{tv:.1f}</text>')

        svg.append(
            f'<text x="26" y="{plot_y + sub_h/2}" font-size="13" font-weight="bold" text-anchor="middle" fill="#111827" '
            f'transform="rotate(-90, 26, {plot_y + sub_h/2})">{ylabel}</text>'
        )

        # Reference curve (use longer run or first available)
        if ref1 is not None:
            ref_t = t1 if len(t1) >= len(t2) else t2
            ref_v = ref1 if len(t1) >= len(t2) else ref2
            ref_step = step1 if len(t1) >= len(t2) else step2
            pts_ref = polyline_points(ref_t, ref_v, t_min, t_max, y_min, y_max, margin_left, plot_y, plot_w, sub_h, ref_step)
            svg.append(f'<polyline fill="none" stroke="#DC2626" stroke-width="2.0" opacity="0.85" points="{pts_ref}"/>')

        # Algo 1 (ITSMC - Blue dashed)
        pts_1 = polyline_points(t1, act1, t_min, t_max, y_min, y_max, margin_left, plot_y, plot_w, sub_h, step1)
        svg.append(f'<polyline fill="none" stroke="#2563EB" stroke-width="2.1" stroke-dasharray="7,4" points="{pts_1}"/>')

        # Algo 2 (RL Critic+ESO - Emerald dashed)
        pts_2 = polyline_points(t2, act2, t_min, t_max, y_min, y_max, margin_left, plot_y, plot_w, sub_h, step2)
        svg.append(f'<polyline fill="none" stroke="#059669" stroke-width="2.1" stroke-dasharray="7,4" points="{pts_2}"/>')

    # X-axis label
    last_y = margin_top + (len(subplots) - 1) * (sub_h + spacing) + sub_h + 34
    svg.append(f'<text x="{margin_left + plot_w/2}" y="{last_y}" font-size="13" font-weight="bold" text-anchor="middle" fill="#111827">Thời gian t (s)</text>')

    # Quantitative Summary Table at bottom
    svg.append(f'<rect x="{margin_left}" y="{table_y}" width="{plot_w}" height="105" rx="6" fill="#F8FAFC" stroke="#CBD5E1" stroke-width="1"/>')
    cols = [
        ("Thuật toán", 18),
        ("RMSE X (m)", 230),
        ("RMSE Y (m)", 335),
        ("RMSE Z (m)", 440),
        ("RMSE 3D (m)", 550),
        ("MAE 3D (m)", 665),
        ("Max Err (m)", 775),
        ("RMSE Yaw (°)", 885),
    ]
    for title, dx in cols:
        svg.append(f'<text x="{margin_left + dx}" y="{table_y + 24}" font-size="11.5" font-weight="bold" fill="#334155">{title}</text>')
    svg.append(f'<line x1="{margin_left+12}" y1="{table_y+34}" x2="{margin_left+plot_w-12}" y2="{table_y+34}" stroke="#CBD5E1" stroke-width="1"/>')

    for r_idx, (lbl, m, color) in enumerate([(label1, m1, "#2563EB"), (label2, m2, "#059669")]):
        ry = table_y + 58 + r_idx * 28
        vals = [
            (lbl, 18, "bold", color),
            (f"{m['rmse_x']:.4f}", 230, "normal", "#1E293B"),
            (f"{m['rmse_y']:.4f}", 335, "normal", "#1E293B"),
            (f"{m['rmse_z']:.4f}", 440, "normal", "#1E293B"),
            (f"{m['rmse_3d']:.4f}", 550, "bold", color),
            (f"{m['mae_3d']:.4f}", 665, "normal", "#1E293B"),
            (f"{m['max_err_3d']:.4f}", 775, "normal", "#1E293B"),
            (f"{m['rmse_yaw_deg']:.2f}", 885, "normal", "#1E293B"),
        ]
        for txt, dx, fw, clr in vals:
            svg.append(f'<text x="{margin_left + dx}" y="{ry}" font-size="12" font-weight="{fw}" fill="{clr}">{txt}</text>')

    svg.append('</svg>')

    with open(out_svg_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(svg))
    print(f"[✓ SVG Compare] {out_svg_path}")


def generate_side_by_side_svg(d1, m1, label1, d2, m2, label2, out_svg_path):
    """
    Vẽ 2 cột song song (Trái: Algo 1, Phải: Algo 2) trong CÙNG 1 ảnh SVG.
    Hai bên dùng CHUNG thang trục X (0 -> t_max) và CHUNG thang trục Y ở mỗi hàng
    để đảm bảo so sánh trực quan công bằng, đồng thời mỗi bên hiển thị đúng
    đường Expected Trajectory của riêng lần bay đó.
    """
    t1 = d1['time']
    t2 = d2['time']
    t_min = 0.0
    t_max = max(t1[-1] if t1 else 1.0, t2[-1] if t2 else 1.0, 0.1)

    width = 1440
    margin_left = 80
    margin_right = 35
    col_gap = 65
    col_w = (width - margin_left - margin_right - col_gap) / 2.0
    col_x = [margin_left, margin_left + col_w + col_gap]

    margin_top = 128
    sub_h = 155
    spacing = 40

    subplots = [
        ('x (m)',      d1['x_ref'],       d1['x_act'],       d2['x_ref'],       d2['x_act']),
        ('y (m)',      d1['y_ref'],       d1['y_act'],       d2['y_ref'],       d2['y_act']),
        ('z (m)',      d1['z_ref'],       d1['z_act'],       d2['z_ref'],       d2['z_act']),
        ('yaw (rad)',  d1['yaw_ref_rad'], d1['yaw_act_rad'], d2['yaw_ref_rad'], d2['yaw_act_rad']),
        ('err 3D (m)', [0.0]*len(t1),     d1['err_dist_3d'], [0.0]*len(t2),     d2['err_dist_3d']),
    ]

    table_y = margin_top + len(subplots) * (sub_h + spacing) + 18
    height = table_y + 125

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
        f'style="background-color:#ffffff; font-family: -apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Arial, sans-serif;">'
    ]

    # Global Title
    svg.append(
        f'<text x="{width/2}" y="30" font-size="19" font-weight="bold" text-anchor="middle" fill="#111827">'
        f'SO SÁNH BÁM QUỸ ĐẠO SONG SONG: {label1} vs {label2}</text>'
    )
    svg.append(
        f'<text x="{width/2}" y="50" font-size="12" font-weight="500" text-anchor="middle" fill="#4B5563">'
        f'Cùng thang đo trục X (0 → {t_max:.1f}s) và trục Y trên cả 2 cột — mỗi cột đối chiếu với chính Expected Trajectory của lần bay đó</text>'
    )

    # Legend Box
    leg_w = 680
    leg_x = (width - leg_w) / 2
    leg_y = 75
    svg.append(f'<rect x="{leg_x}" y="60" width="{leg_w}" height="28" rx="6" fill="#F8FAFC" stroke="#CBD5E1" stroke-width="1"/>')
    svg.append(f'<line x1="{leg_x+20}" y1="{leg_y}" x2="{leg_x+52}" y2="{leg_y}" stroke="#DC2626" stroke-width="2.4"/>')
    svg.append(f'<text x="{leg_x+60}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#DC2626">Expected Trajectory</text>')
    svg.append(f'<line x1="{leg_x+225}" y1="{leg_y}" x2="{leg_x+257}" y2="{leg_y}" stroke="#2563EB" stroke-width="2.4" stroke-dasharray="7,4"/>')
    svg.append(f'<text x="{leg_x+265}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#2563EB">{label1}</text>')
    svg.append(f'<line x1="{leg_x+435}" y1="{leg_y}" x2="{leg_x+467}" y2="{leg_y}" stroke="#059669" stroke-width="2.4" stroke-dasharray="7,4"/>')
    svg.append(f'<text x="{leg_x+475}" y="{leg_y+4}" font-size="12" font-weight="600" fill="#059669">{label2}</text>')

    # Column Headers
    col_headers = [
        (f"(a) {label1} — RMSE 3D: {m1['rmse_3d']:.3f}m | Thời gian: {m1['flight_time_s']:.1f}s", "#1D4ED8"),
        (f"(b) {label2} — RMSE 3D: {m2['rmse_3d']:.3f}m | Thời gian: {m2['flight_time_s']:.1f}s", "#047857"),
    ]
    for c_idx, (htxt, hclr) in enumerate(col_headers):
        cx = col_x[c_idx] + col_w / 2.0
        svg.append(f'<text x="{cx:.1f}" y="114" font-size="14" font-weight="bold" text-anchor="middle" fill="{hclr}">{htxt}</text>')

    step1 = max(1, len(t1) // 1000)
    step2 = max(1, len(t2) // 1000)

    for idx, (ylabel, ref1, act1, ref2, act2) in enumerate(subplots):
        plot_y = margin_top + idx * (sub_h + spacing)

        # Shared Y limits across both algorithms
        all_vals = list(act1) + list(act2)
        if ref1 is not None:
            all_vals += list(ref1)
        if ref2 is not None:
            all_vals += list(ref2)
        min_val = min(all_vals)
        max_val = max(all_vals)
        span = max(max_val - min_val, 0.2)
        pad = span * 0.14
        y_min = min_val - pad
        y_max = max_val + pad

        # Shared Y-axis title on far left
        svg.append(
            f'<text x="22" y="{plot_y + sub_h/2:.1f}" font-size="13" font-weight="bold" text-anchor="middle" fill="#111827" '
            f'transform="rotate(-90, 22, {plot_y + sub_h/2:.1f})">{ylabel}</text>'
        )

        cols_data = [
            (0, t1, ref1, act1, step1, "#2563EB", 'stroke-dasharray="7,4"'),
            (1, t2, ref2, act2, step2, "#059669", 'stroke-dasharray="7,4"'),
        ]

        for c_idx, tc, refc, actc, stepc, clr, dash_attr in cols_data:
            px0 = col_x[c_idx]
            svg.append(f'<rect x="{px0:.1f}" y="{plot_y}" width="{col_w:.1f}" height="{sub_h}" fill="#FAFAFA" stroke="#9CA3AF" stroke-width="1.2"/>')

            # Horizontal grid + Y tick labels
            for i in range(5):
                val = y_min + (y_max - y_min) * (i / 4.0)
                py = plot_y + sub_h - (i / 4.0) * sub_h
                svg.append(f'<line x1="{px0:.1f}" y1="{py:.1f}" x2="{px0+col_w:.1f}" y2="{py:.1f}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
                svg.append(f'<text x="{px0-7:.1f}" y="{py+4:.1f}" font-size="10.5" text-anchor="end" fill="#4B5563">{val:.2f}</text>')

            # Vertical grid + X tick labels on bottom row
            for i in range(7):
                tv = t_min + (t_max - t_min) * (i / 6.0)
                gx = px0 + (i / 6.0) * col_w
                svg.append(f'<line x1="{gx:.1f}" y1="{plot_y}" x2="{gx:.1f}" y2="{plot_y+sub_h}" stroke="#E5E7EB" stroke-width="1" stroke-dasharray="2,2"/>')
                if idx == len(subplots) - 1:
                    svg.append(f'<text x="{gx:.1f}" y="{plot_y+sub_h+16}" font-size="10.5" text-anchor="middle" fill="#4B5563">{tv:.1f}</text>')

            if refc is not None:
                pts_ref = polyline_points(tc, refc, t_min, t_max, y_min, y_max, px0, plot_y, col_w, sub_h, stepc)
                svg.append(f'<polyline fill="none" stroke="#DC2626" stroke-width="2.0" opacity="0.9" points="{pts_ref}"/>')

            pts_act = polyline_points(tc, actc, t_min, t_max, y_min, y_max, px0, plot_y, col_w, sub_h, stepc)
            svg.append(f'<polyline fill="none" stroke="{clr}" stroke-width="2.1" {dash_attr} points="{pts_act}"/>')

            if idx == len(subplots) - 1:
                svg.append(f'<text x="{px0 + col_w/2:.1f}" y="{plot_y+sub_h+33}" font-size="12" font-weight="bold" text-anchor="middle" fill="#111827">Thời gian t (s)</text>')

    # Quantitative Summary Table at bottom
    table_w = width - margin_left - margin_right
    svg.append(f'<rect x="{margin_left}" y="{table_y}" width="{table_w}" height="102" rx="6" fill="#F8FAFC" stroke="#CBD5E1" stroke-width="1"/>')
    cols = [
        ("Thuật toán", 20),
        ("Thời gian (s)", 290),
        ("RMSE X (m)", 420),
        ("RMSE Y (m)", 545),
        ("RMSE Z (m)", 670),
        ("RMSE 3D (m)", 800),
        ("MAE 3D (m)", 935),
        ("Max Err (m)", 1065),
        ("RMSE Yaw (°)", 1195),
    ]
    for title, dx in cols:
        svg.append(f'<text x="{margin_left + dx}" y="{table_y + 24}" font-size="12" font-weight="bold" fill="#334155">{title}</text>')
    svg.append(f'<line x1="{margin_left+14}" y1="{table_y+34}" x2="{margin_left+table_w-14}" y2="{table_y+34}" stroke="#CBD5E1" stroke-width="1"/>')

    for r_idx, (lbl, m, color) in enumerate([(label1, m1, "#2563EB"), (label2, m2, "#059669")]):
        ry = table_y + 57 + r_idx * 27
        vals = [
            (lbl, 20, "bold", color),
            (f"{m['flight_time_s']:.2f}", 290, "normal", "#1E293B"),
            (f"{m['rmse_x']:.4f}", 420, "normal", "#1E293B"),
            (f"{m['rmse_y']:.4f}", 545, "normal", "#1E293B"),
            (f"{m['rmse_z']:.4f}", 670, "normal", "#1E293B"),
            (f"{m['rmse_3d']:.4f}", 800, "bold", color),
            (f"{m['mae_3d']:.4f}", 935, "normal", "#1E293B"),
            (f"{m['max_err_3d']:.4f}", 1065, "normal", "#1E293B"),
            (f"{m['rmse_yaw_deg']:.2f}", 1195, "normal", "#1E293B"),
        ]
        for txt, dx, fw, clr in vals:
            svg.append(f'<text x="{margin_left + dx}" y="{ry}" font-size="12" font-weight="{fw}" fill="{clr}">{txt}</text>')

    svg.append('</svg>')

    with open(out_svg_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(svg))
    print(f"[✓ SVG Side-by-Side] {out_svg_path}")


def generate_compare_html(d1, m1, label1, d2, m2, label2, out_html_path):
    step1 = max(1, len(d1['time']) // 1500)
    step2 = max(1, len(d2['time']) // 1500)

    def js_arr(arr, step):
        return "[" + ",".join(f"{x:.4f}" for x in arr[::step]) + "]"

    html = f"""<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <title>So sánh quỹ đạo: {label1} vs {label2}</title>
    <script src="https://cdn.plot.ly/plotly-2.32.0.min.js"></script>
    <style>
        body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Arial, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; }}
        .container {{ max-width: 1400px; margin: 0 auto; }}
        h1 {{ text-align: center; color: #38bdf8; margin-bottom: 6px; }}
        .subtitle {{ text-align: center; color: #94a3b8; margin-bottom: 20px; font-size: 14px; }}
        .table-wrap {{ background: #1e293b; border: 1px solid #334155; border-radius: 10px; padding: 16px; margin-bottom: 20px; overflow-x: auto; }}
        table {{ width: 100%; border-collapse: collapse; font-size: 14px; }}
        th, td {{ padding: 10px 12px; text-align: center; border-bottom: 1px solid #334155; }}
        th {{ color: #94a3b8; font-weight: 600; text-transform: uppercase; font-size: 12px; }}
        .c1 {{ color: #60a5fa; font-weight: bold; }}
        .c2 {{ color: #34d399; font-weight: bold; }}
        .chart-grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 18px; margin-bottom: 18px; }}
        .chart-box {{ background: #ffffff; border-radius: 10px; padding: 10px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }}
        .full-width {{ grid-column: 1 / -1; }}
        @media (max-width: 1000px) {{ .chart-grid {{ grid-template-columns: 1fr; }} }}
    </style>
</head>
<body>
<div class="container">
    <h1>📊 So Sánh Bám Quỹ Đạo: {label1} vs {label2}</h1>
    <p class="subtitle">Đồ thị gộp trực tiếp 2 thuật toán trên cùng hệ quy chiếu thời gian và không gian 2D/3D</p>

    <div class="table-wrap">
        <table>
            <thead>
                <tr>
                    <th>Thuật toán</th><th>Thời gian bay (s)</th><th>RMSE X (m)</th><th>RMSE Y (m)</th>
                    <th>RMSE Z (m)</th><th>RMSE 3D (m)</th><th>MAE 3D (m)</th><th>Max Err 3D (m)</th><th>RMSE Yaw (°)</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td class="c1">{label1}</td><td>{m1['flight_time_s']:.2f}</td>
                    <td>{m1['rmse_x']:.4f}</td><td>{m1['rmse_y']:.4f}</td><td>{m1['rmse_z']:.4f}</td>
                    <td class="c1">{m1['rmse_3d']:.4f}</td><td>{m1['mae_3d']:.4f}</td>
                    <td>{m1['max_err_3d']:.4f}</td><td>{m1['rmse_yaw_deg']:.2f}°</td>
                </tr>
                <tr>
                    <td class="c2">{label2}</td><td>{m2['flight_time_s']:.2f}</td>
                    <td>{m2['rmse_x']:.4f}</td><td>{m2['rmse_y']:.4f}</td><td>{m2['rmse_z']:.4f}</td>
                    <td class="c2">{m2['rmse_3d']:.4f}</td><td>{m2['mae_3d']:.4f}</td>
                    <td>{m2['max_err_3d']:.4f}</td><td>{m2['rmse_yaw_deg']:.2f}°</td>
                </tr>
            </tbody>
        </table>
    </div>

    <div class="chart-grid">
        <div class="chart-box"><div id="plot_xy" style="height: 440px;"></div></div>
        <div class="chart-box"><div id="plot_3d" style="height: 440px;"></div></div>
        <div class="chart-box full-width"><div id="plot_sub" style="height: 920px;"></div></div>
    </div>
</div>

<script>
const t1 = {js_arr(d1['time'], step1)};
const x1_ref = {js_arr(d1['x_ref'], step1)}, y1_ref = {js_arr(d1['y_ref'], step1)}, z1_ref = {js_arr(d1['z_ref'], step1)}, yw1_ref = {js_arr(d1['yaw_ref_rad'], step1)};
const x1_act = {js_arr(d1['x_act'], step1)}, y1_act = {js_arr(d1['y_act'], step1)}, z1_act = {js_arr(d1['z_act'], step1)}, yw1_act = {js_arr(d1['yaw_act_rad'], step1)};
const e1_3d = {js_arr(d1['err_dist_3d'], step1)};

const t2 = {js_arr(d2['time'], step2)};
const x2_ref = {js_arr(d2['x_ref'], step2)}, y2_ref = {js_arr(d2['y_ref'], step2)}, z2_ref = {js_arr(d2['z_ref'], step2)}, yw2_ref = {js_arr(d2['yaw_ref_rad'], step2)};
const x2_act = {js_arr(d2['x_act'], step2)}, y2_act = {js_arr(d2['y_act'], step2)}, z2_act = {js_arr(d2['z_act'], step2)}, yw2_act = {js_arr(d2['yaw_act_rad'], step2)};
const e2_3d = {js_arr(d2['err_dist_3d'], step2)};

const xr = t1.length >= t2.length ? x1_ref : x2_ref;
const yr = t1.length >= t2.length ? y1_ref : y2_ref;
const zr = t1.length >= t2.length ? z1_ref : z2_ref;
const ywr = t1.length >= t2.length ? yw1_ref : yw2_ref;
const tr = t1.length >= t2.length ? t1 : t2;

Plotly.newPlot('plot_xy', [
    {{ x: xr, y: yr, mode: 'lines', name: 'Expected Trajectory', line: {{ color: '#dc2626', width: 2.5 }} }},
    {{ x: x1_act, y: y1_act, mode: 'lines', name: '{label1}', line: {{ color: '#2563eb', width: 2.2, dash: 'dash' }} }},
    {{ x: x2_act, y: y2_act, mode: 'lines', name: '{label2}', line: {{ color: '#059669', width: 2.2, dash: 'dash' }} }}
], {{
    title: 'Quỹ Đạo Mặt Bằng 2D (X - Y)',
    xaxis: {{ title: 'X (m)', scaleanchor: 'y', scaleratio: 1 }},
    yaxis: {{ title: 'Y (m)' }},
    legend: {{ orientation: 'h', y: 1.08 }}
}});

Plotly.newPlot('plot_3d', [
    {{ x: xr, y: yr, z: zr, mode: 'lines', type: 'scatter3d', name: 'Expected Trajectory', line: {{ color: '#dc2626', width: 4 }} }},
    {{ x: x1_act, y: y1_act, z: z1_act, mode: 'lines', type: 'scatter3d', name: '{label1}', line: {{ color: '#2563eb', width: 4, dash: 'dash' }} }},
    {{ x: x2_act, y: y2_act, z: z2_act, mode: 'lines', type: 'scatter3d', name: '{label2}', line: {{ color: '#059669', width: 4, dash: 'dash' }} }}
], {{
    title: 'Quỹ Đạo Không Gian 3D',
    scene: {{ xaxis: {{ title: 'X (m)' }}, yaxis: {{ title: 'Y (m)' }}, zaxis: {{ title: 'Z (m)' }} }},
    margin: {{ l: 0, r: 0, b: 0, t: 40 }}
}});

const traces = [
    {{ x: tr, y: xr, name: 'Expected', line: {{ color: '#dc2626', width: 2 }}, xaxis: 'x1', yaxis: 'y1' }},
    {{ x: t1, y: x1_act, name: '{label1}', line: {{ color: '#2563eb', width: 2, dash: 'dash' }}, xaxis: 'x1', yaxis: 'y1' }},
    {{ x: t2, y: x2_act, name: '{label2}', line: {{ color: '#059669', width: 2, dash: 'dash' }}, xaxis: 'x1', yaxis: 'y1' }},

    {{ x: tr, y: yr, showlegend: false, line: {{ color: '#dc2626', width: 2 }}, xaxis: 'x2', yaxis: 'y2' }},
    {{ x: t1, y: y1_act, showlegend: false, line: {{ color: '#2563eb', width: 2, dash: 'dash' }}, xaxis: 'x2', yaxis: 'y2' }},
    {{ x: t2, y: y2_act, showlegend: false, line: {{ color: '#059669', width: 2, dash: 'dash' }}, xaxis: 'x2', yaxis: 'y2' }},

    {{ x: tr, y: zr, showlegend: false, line: {{ color: '#dc2626', width: 2 }}, xaxis: 'x3', yaxis: 'y3' }},
    {{ x: t1, y: z1_act, showlegend: false, line: {{ color: '#2563eb', width: 2, dash: 'dash' }}, xaxis: 'x3', yaxis: 'y3' }},
    {{ x: t2, y: z2_act, showlegend: false, line: {{ color: '#059669', width: 2, dash: 'dash' }}, xaxis: 'x3', yaxis: 'y3' }},

    {{ x: tr, y: ywr, showlegend: false, line: {{ color: '#dc2626', width: 2 }}, xaxis: 'x4', yaxis: 'y4' }},
    {{ x: t1, y: yw1_act, showlegend: false, line: {{ color: '#2563eb', width: 2, dash: 'dash' }}, xaxis: 'x4', yaxis: 'y4' }},
    {{ x: t2, y: yw2_act, showlegend: false, line: {{ color: '#059669', width: 2, dash: 'dash' }}, xaxis: 'x4', yaxis: 'y4' }},

    {{ x: t1, y: e1_3d, showlegend: false, line: {{ color: '#2563eb', width: 2, dash: 'dash' }}, xaxis: 'x5', yaxis: 'y5' }},
    {{ x: t2, y: e2_3d, showlegend: false, line: {{ color: '#059669', width: 2, dash: 'dash' }}, xaxis: 'x5', yaxis: 'y5' }}
];

Plotly.newPlot('plot_sub', traces, {{
    title: 'So Sánh Chi Tiết Theo Thời Gian: x(t), y(t), z(t), yaw(t), err_3D(t)',
    grid: {{ rows: 5, columns: 1, pattern: 'independent', roworder: 'top to bottom' }},
    yaxis1: {{ title: 'x (m)' }},
    yaxis2: {{ title: 'y (m)' }},
    yaxis3: {{ title: 'z (m)' }},
    yaxis4: {{ title: 'yaw (rad)' }},
    yaxis5: {{ title: 'err 3D (m)' }},
    xaxis5: {{ title: 'Thời gian t (s)' }},
    legend: {{ orientation: 'h', y: 1.04 }},
    margin: {{ l: 60, r: 30, t: 55, b: 50 }}
}});
</script>
</body>
</html>
"""
    with open(out_html_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"[✓ HTML Compare] {out_html_path}")


def main():
    parser = argparse.ArgumentParser(description="So sánh 2 thuật toán trên cùng 1 đồ thị")
    parser.add_argument("csv1", nargs="?", help="File CSV thuật toán 1 (mặc định: ITSMC)")
    parser.add_argument("csv2", nargs="?", help="File CSV thuật toán 2 (mặc định: RL Critic+ESO)")
    parser.add_argument("--label1", default="Algo 1: ITSMC", help="Nhãn thuật toán 1")
    parser.add_argument("--label2", default="Algo 2: RL Critic-Only + ESO", help="Nhãn thuật toán 2")
    parser.add_argument("--out-dir", default="logs/trajectory", help="Thư mục xuất đồ thị so sánh")
    args = parser.parse_args()

    csv1 = args.csv1 or find_algo_csv([
        "logs/itsmc/trajectory/latest_trajectory.csv",
        "DroneHITL/logs/itsmc/trajectory/latest_trajectory.csv",
    ])
    csv2 = args.csv2 or find_algo_csv([
        "logs/critic/trajectory/latest_trajectory.csv",
        "DroneHITL/logs/critic/trajectory/latest_trajectory.csv",
    ])

    if not csv1 or not os.path.isfile(csv1):
        print("[!] Không tìm thấy CSV cho Thuật toán 1 (ITSMC).")
        sys.exit(1)
    if not csv2 or not os.path.isfile(csv2):
        print("[!] Không tìm thấy CSV cho Thuật toán 2 (RL Critic+ESO).")
        sys.exit(1)

    print(f"[+] Algo 1 ({args.label1}): {csv1}")
    print(f"[+] Algo 2 ({args.label2}): {csv2}")

    d1 = parse_csv(csv1)
    d2 = parse_csv(csv2)
    m1 = compute_metrics(d1)
    m2 = compute_metrics(d2)

    os.makedirs(args.out_dir, exist_ok=True)
    out_svg = os.path.join(args.out_dir, "compare_itsmc_vs_critic.svg")
    out_side_svg = os.path.join(args.out_dir, "compare_side_by_side.svg")
    out_html = os.path.join(args.out_dir, "compare_itsmc_vs_critic.html")

    generate_compare_svg(d1, m1, args.label1, d2, m2, args.label2, out_svg)
    generate_side_by_side_svg(d1, m1, args.label1, d2, m2, args.label2, out_side_svg)
    generate_compare_html(d1, m1, args.label1, d2, m2, args.label2, out_html)

    if os.path.isdir("DroneHITL") and not os.path.normpath(args.out_dir).startswith("DroneHITL"):
        win_dir = os.path.join("DroneHITL", os.path.normpath(args.out_dir))
        os.makedirs(win_dir, exist_ok=True)
        for src in [out_svg, out_side_svg, out_html]:
            shutil.copyfile(src, os.path.join(win_dir, os.path.basename(src)))
        print(f"[✓ Đồng bộ] {win_dir}")


if __name__ == "__main__":
    main()
