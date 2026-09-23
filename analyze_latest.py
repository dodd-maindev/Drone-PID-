#!/usr/bin/env python3
import csv
import sys

filename = '/home/do/drone_control_cpp/log/latest_flight.csv'

with open(filename, 'r') as f:
    rows = [r for r in csv.DictReader(f) if r.get('actual_alt_m')]

print(f"Loaded {len(rows)} samples from {filename}")
if not rows:
    sys.exit(0)

# Check hover vs moving
print("\n=== ANALYZING MOVING AND BRAKING PHASES ===")
for i in range(1, len(rows)):
    t = float(rows[i]['time_s'])
    vx = float(rows[i]['actual_vx'])
    vy = float(rows[i]['actual_vy'])
    tp = float(rows[i]['target_pitch_deg'])
    ap = float(rows[i]['actual_pitch_deg'])
    tr = float(rows[i]['target_roll_deg'])
    ar = float(rows[i]['actual_roll_deg'])
    x = float(rows[i]['actual_x'])
    y = float(rows[i]['actual_y'])

    # Look at intervals where velocity changes rapidly
    if 24.0 <= t <= 45.0 and int(t * 100) % 50 == 0:
        print(f"t={t:5.1f}s | x={x:6.2f}m | vx={vx:5.2f}m/s | target_pitch={tp:5.1f}° | act_pitch={ap:5.1f}° | y={y:5.2f}m | vy={vy:5.2f}m/s")
