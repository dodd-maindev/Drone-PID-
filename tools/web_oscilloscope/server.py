#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DRONE HITL - REAL-TIME WEB OSCILLOSCOPE & TELEMETRY SERVER
Chạy bằng Python Standard Library (không cần cài đặt thư viện ngoài).
Nhận UDP JSON từ C++ HITL Bridge (cổng 9098) và phát SSE (Server-Sent Events) tới Web Browser (cổng 8088).
"""

import os
import sys
import json
import time
import socket
import select
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

HOST = '0.0.0.0'
DEFAULT_PORT = 8088
UDP_PORT = 9098
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')

# Bộ đệm dữ liệu toàn cục
MAX_HISTORY = 1500
g_history = []
g_history_lock = threading.Lock()
g_clients = set()
g_clients_lock = threading.Lock()
g_last_telemetry_time = 0
g_packet_count = 0
g_running = True

def udp_telemetry_listener():
    """Lắng nghe dữ liệu Telemetry UDP từ C++ HITL Bridge ở tần số 50-100Hz."""
    global g_last_telemetry_time, g_packet_count, g_running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
        sock.settimeout(0.5)
        print(f"[✓ UDP] Đang lắng nghe gói tin Telemetry tại cổng UDP {UDP_PORT}...")
    except Exception as e:
        print(f"[✗ UDP] Không thể mở cổng UDP {UDP_PORT}: {e}")
        return

    while g_running:
        try:
            data, _ = sock.recvfrom(4096)
            if not data:
                continue
            text = data.decode('utf-8', errors='ignore').strip()
            if not text:
                continue
            
            try:
                pkt = json.loads(text)
            except Exception:
                continue

            g_last_telemetry_time = time.time()
            g_packet_count += 1
            pkt['server_ts'] = g_last_telemetry_time

            # Lưu vào bộ đệm trượt
            with g_history_lock:
                g_history.append(pkt)
                if len(g_history) > MAX_HISTORY:
                    g_history.pop(0)

            # Phát tới tất cả web clients qua SSE
            sse_payload = f"data: {json.dumps(pkt)}\n\n".encode('utf-8')
            with g_clients_lock:
                dead_clients = []
                for client_wfile in g_clients:
                    try:
                        client_wfile.write(sse_payload)
                        client_wfile.flush()
                    except Exception:
                        dead_clients.append(client_wfile)
                for dc in dead_clients:
                    g_clients.discard(dc)

        except socket.timeout:
            continue
        except Exception as e:
            if not g_running:
                break
            time.sleep(0.01)

    sock.close()

class OscilloscopeHttpHandler(BaseHTTPRequestHandler):
    """Handler phục vụ Web giao diện tĩnh và luồng Server-Sent Events."""

    def log_message(self, format, *args):
        # Tắt bớt log HTTP định kỳ để không làm rối terminal
        if '/events' in args[0] or '/api/status' in args[0]:
            return
        super().log_message(format, *args)

    def do_GET(self):
        url = urlparse(self.path)
        path = url.path

        if path == '/events':
            # Kênh SSE phát dữ liệu thời gian thực
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            # Gửi lịch sử gần nhất ngay khi client vừa mở
            with g_history_lock:
                recent = list(g_history[-300:]) if len(g_history) > 300 else list(g_history)
            
            if recent:
                init_msg = json.dumps({"type": "history", "data": recent})
                try:
                    self.wfile.write(f"data: {init_msg}\n\n".encode('utf-8'))
                    self.wfile.flush()
                except Exception:
                    return

            # Đăng ký client vào danh sách phát
            with g_clients_lock:
                g_clients.add(self.wfile)

            # Giữ kết nối mở
            try:
                while g_running:
                    time.sleep(1.0)
                    # Gửi heartbeat định kỳ để giữ kết nối không bị timeout
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
            except Exception:
                pass
            finally:
                with g_clients_lock:
                    g_clients.discard(self.wfile)
            return

        elif path == '/api/status':
            now = time.time()
            is_active = (now - g_last_telemetry_time) < 1.0
            resp = {
                "active": is_active,
                "packet_count": g_packet_count,
                "connected_clients": len(g_clients),
                "last_seen_sec": round(now - g_last_telemetry_time, 2) if g_last_telemetry_time > 0 else -1
            }
            body = json.dumps(resp).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        elif path == '/api/history':
            with g_history_lock:
                body = json.dumps(g_history).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # Phục vụ file tĩnh
        if path == '/' or path == '':
            file_path = os.path.join(STATIC_DIR, 'index.html')
        else:
            rel = path.lstrip('/')
            file_path = os.path.join(STATIC_DIR, rel)

        if not os.path.exists(file_path) or os.path.isdir(file_path):
            file_path = os.path.join(STATIC_DIR, 'index.html')

        ext = os.path.splitext(file_path)[1].lower()
        content_types = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
            '.json': 'application/json; charset=utf-8',
            '.png': 'image/png',
            '.svg': 'image/svg+xml',
            '.ico': 'image/x-icon'
        }
        ctype = content_types.get(ext, 'text/plain')

        try:
            with open(file_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(f"Error reading file: {e}".encode('utf-8'))

def main():
    global g_running
    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            if sys.argv[1] == '--port' and len(sys.argv) > 2:
                port = int(sys.argv[2])

    # Khởi động thread nghe UDP Telemetry
    t_udp = threading.Thread(target=udp_telemetry_listener, daemon=True)
    t_udp.start()

    server = HTTPServer((HOST, port), OscilloscopeHttpHandler)
    print("=" * 60)
    print("  🚁 DRONE HITL DIGITAL OSCILLOSCOPE & TELEMETRY LAB")
    print("=" * 60)
    print(f" [✓] Web Server đang chạy tại: http://localhost:{port}")
    print(f" [✓] Nhận tín hiệu từ C++ Bridge qua UDP cổng {UDP_PORT}")
    print(f" [✓] Mở trình duyệt (Chrome/Edge) tại: http://localhost:{port}")
    print("=" * 60)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] Đang tắt Web Oscilloscope...")
    finally:
        g_running = False
        server.server_close()

if __name__ == '__main__':
    main()
