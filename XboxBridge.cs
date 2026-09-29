using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;

namespace DroneControl {
    [StructLayout(LayoutKind.Sequential)]
    public struct XINPUT_GAMEPAD {
        public ushort wButtons;
        public byte bLeftTrigger;
        public byte bRightTrigger;
        public short sThumbLX;
        public short sThumbLY;
        public short sThumbRX;
        public short sThumbRY;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct XINPUT_STATE {
        public uint dwPacketNumber;
        public XINPUT_GAMEPAD Gamepad;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct XboxUdpPacket {
        public uint magic;          // 0x58424F58 ('XBOX')
        public ushort buttons;      // wButtons
        public byte leftTrigger;    // bLeftTrigger
        public byte rightTrigger;   // bRightTrigger
        public short thumbLX;       // sThumbLX
        public short thumbLY;       // sThumbLY
        public short thumbRX;       // sThumbRX
        public short thumbRY;       // sThumbRY
        public byte isConnected;    // 1 if connected, 0 if not
    }

    class XboxBridge {
        [DllImport("xinput1_4.dll", EntryPoint = "XInputGetState")]
        private static extern int XInputGetState14(int dwUserIndex, ref XINPUT_STATE pState);

        [DllImport("xinput1_3.dll", EntryPoint = "XInputGetState")]
        private static extern int XInputGetState13(int dwUserIndex, ref XINPUT_STATE pState);

        [DllImport("xinput9_1_0.dll", EntryPoint = "XInputGetState")]
        private static extern int XInputGetState91(int dwUserIndex, ref XINPUT_STATE pState);

        private static int GetControllerState(int index, ref XINPUT_STATE state) {
            try { return XInputGetState14(index, ref state); } catch {}
            try { return XInputGetState13(index, ref state); } catch {}
            try { return XInputGetState91(index, ref state); } catch {}
            return -1;
        }

        static byte[] PacketToBytes(XboxUdpPacket pkt) {
            int size = Marshal.SizeOf(pkt);
            byte[] arr = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(pkt, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        static void Log(string message) {
            try {
                string dir = AppDomain.CurrentDomain.BaseDirectory;
                string logFile = Path.Combine(dir, "XboxBridge.log");
                string line = "[" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff") + "] " + message + Environment.NewLine;
                File.AppendAllText(logFile, line);
            } catch {}
        }

        // Tự động tìm IP của máy ảo WSL2 từ Windows
        static string AutoDetectWslIp() {
            string[] wslExeCandidates = new string[] {
                "wsl.exe",
                @"C:\Windows\System32\wsl.exe",
                @"C:\Windows\Sysnative\wsl.exe"
            };

            foreach (var exe in wslExeCandidates) {
                try {
                    ProcessStartInfo psi = new ProcessStartInfo(exe, "hostname -I") {
                        RedirectStandardOutput = true,
                        UseShellExecute = false,
                        CreateNoWindow = true
                    };
                    using (Process p = Process.Start(psi)) {
                        string outStr = p.StandardOutput.ReadToEnd();
                        p.WaitForExit(1500);
                        string[] tokens = outStr.Trim().Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
                        foreach (string t in tokens) {
                            IPAddress testIp;
                            if (IPAddress.TryParse(t, out testIp)) {
                                if (!t.StartsWith("172.17.")) {
                                    Log("AutoDetectWslIp found via " + exe + ": " + t);
                                    return t;
                                }
                            }
                        }
                    }
                } catch (Exception ex) {
                    Log("AutoDetectWslIp with " + exe + " exception: " + ex.Message);
                }
            }

            Log("AutoDetectWslIp fallback to 127.0.0.1");
            return "127.0.0.1";
        }

        // Tìm các địa chỉ broadcast tiềm năng (ví dụ vEthernet WSL)
        static List<IPEndPoint> GetDestinationEndPoints(string primaryIp, int port) {
            List<IPEndPoint> list = new List<IPEndPoint>();
            HashSet<string> added = new HashSet<string>();

            Action<string> addIp = (ipStr) => {
                IPAddress ip;
                if (IPAddress.TryParse(ipStr, out ip) && !added.Contains(ipStr)) {
                    added.Add(ipStr);
                    list.Add(new IPEndPoint(ip, port));
                }
            };

            addIp(primaryIp);
            addIp("127.0.0.1");

            // Quét các card mạng trên Windows, đặc biệt là card ảo WSL
            try {
                foreach (NetworkInterface ni in NetworkInterface.GetAllNetworkInterfaces()) {
                    if (ni.OperationalStatus != OperationalStatus.Up) continue;
                    IPInterfaceProperties props = ni.GetIPProperties();
                    foreach (UnicastIPAddressInformation u in props.UnicastAddresses) {
                        if (u.Address.AddressFamily == AddressFamily.InterNetwork) {
                            byte[] ipBytes = u.Address.GetAddressBytes();
                            byte[] maskBytes = (u.IPv4Mask != null) ? u.IPv4Mask.GetAddressBytes() : null;
                            if (maskBytes != null && maskBytes.Length == 4) {
                                byte[] bcastBytes = new byte[4];
                                for (int i = 0; i < 4; i++) {
                                    bcastBytes[i] = (byte)(ipBytes[i] | ~maskBytes[i]);
                                }
                                string bcastStr = new IPAddress(bcastBytes).ToString();
                                addIp(bcastStr);
                                Log("Network adapter [" + ni.Name + "] IP: " + u.Address + ", Broadcast: " + bcastStr);
                            }
                        }
                    }
                }
            } catch (Exception ex) {
                Log("GetDestinationEndPoints scan exception: " + ex.Message);
            }

            return list;
        }

        static void Main(string[] args) {
            Log("====================================================");
            Log("   XBOX 360 CONTROLLER -> WSL2/DRONE UDP BRIDGE    ");
            Log("====================================================");

            string targetIp = (args.Length > 0 && !string.IsNullOrEmpty(args[0])) ? args[0] : "";
            int port = (args.Length > 1) ? int.Parse(args[1]) : 9099;

            if (string.IsNullOrEmpty(targetIp) || targetIp == "127.0.0.1") {
                string detected = AutoDetectWslIp();
                if (!string.IsNullOrEmpty(detected) && detected != "127.0.0.1") {
                    targetIp = detected;
                } else {
                    targetIp = "127.0.0.1";
                }
            }

            Console.WriteLine("====================================================");
            Console.WriteLine("   XBOX 360 CONTROLLER -> WSL2/DRONE UDP BRIDGE    ");
            Console.WriteLine("====================================================");
            Console.WriteLine(" Target WSL2 IP: " + targetIp + ":" + port);
            Console.WriteLine(" Frequency: 100 Hz (10ms)");
            Console.WriteLine(" Log file: " + Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "XboxBridge.log"));
            Console.WriteLine(" Press Ctrl+C to terminate.");
            Console.WriteLine("----------------------------------------------------\n");

            UdpClient udp = new UdpClient();
            try {
                udp.EnableBroadcast = true;
            } catch {}

            List<IPEndPoint> destinations = GetDestinationEndPoints(targetIp, port);
            foreach (var ep in destinations) {
                Log("Active Target EndPoint: " + ep.ToString());
            }

            XINPUT_STATE state = new XINPUT_STATE();
            uint packetCount = 0;
            DateTime lastPrint = DateTime.Now;
            DateTime lastHeartbeat = DateTime.Now;
            bool lastConnected = false;
            ushort lastButtons = 0;

            while (true) {
                int res = GetControllerState(0, ref state);
                bool connected = (res == 0);

                if (connected != lastConnected) {
                    Log(connected ? ">>> [EVENT] CONTROLLER CONNECTED via XInput <<<" : ">>> [EVENT] CONTROLLER DISCONNECTED <<<");
                    lastConnected = connected;
                }

                XboxUdpPacket pkt = new XboxUdpPacket();
                pkt.magic = 0x58424F58; // 'XBOX' in hex
                pkt.isConnected = (byte)(connected ? 1 : 0);

                if (connected) {
                    pkt.buttons = state.Gamepad.wButtons;
                    pkt.leftTrigger = state.Gamepad.bLeftTrigger;
                    pkt.rightTrigger = state.Gamepad.bRightTrigger;
                    pkt.thumbLX = state.Gamepad.sThumbLX;
                    pkt.thumbLY = state.Gamepad.sThumbLY;
                    pkt.thumbRX = state.Gamepad.sThumbRX;
                    pkt.thumbRY = state.Gamepad.sThumbRY;

                    if (pkt.buttons != lastButtons) {
                        if ((pkt.buttons & 0x8000) != 0 && (lastButtons & 0x8000) == 0) Log("[BUTTON PRESSED] Y (TAKEOFF)");
                        if ((pkt.buttons & 0x1000) != 0 && (lastButtons & 0x1000) == 0) Log("[BUTTON PRESSED] A (LAND)");
                        if ((pkt.buttons & 0x0020) != 0 && (lastButtons & 0x0020) == 0) Log("[BUTTON PRESSED] BACK (DISARM)");
                        if ((pkt.buttons & 0x4000) != 0 && (lastButtons & 0x4000) == 0) Log("[BUTTON PRESSED] X (YAW LEFT)");
                        if ((pkt.buttons & 0x2000) != 0 && (lastButtons & 0x2000) == 0) Log("[BUTTON PRESSED] B (YAW RIGHT)");
                        lastButtons = pkt.buttons;
                    }
                }

                byte[] data = PacketToBytes(pkt);
                for (int i = 0; i < destinations.Count; i++) {
                    try {
                        udp.Send(data, data.Length, destinations[i]);
                    } catch {}
                }
                packetCount++;

                DateTime now = DateTime.Now;
                if ((now - lastHeartbeat).TotalSeconds >= 5) {
                    lastHeartbeat = now;
                    Log("Heartbeat: packets_sent=" + packetCount + ", controller_connected=" + connected);
                }

                if ((now - lastPrint).TotalMilliseconds >= 250) {
                    lastPrint = now;
                    string status = connected ? "[CONNECTED]" : "[NOT DETECTED]";
                    string btnStr = "";
                    if (connected) {
                        if ((pkt.buttons & 0x8000) != 0) btnStr += "Y(Takeoff) ";
                        if ((pkt.buttons & 0x1000) != 0) btnStr += "A(Land) ";
                        if ((pkt.buttons & 0x4000) != 0) btnStr += "X(Yaw-L) ";
                        if ((pkt.buttons & 0x2000) != 0) btnStr += "B(Yaw-R) ";
                        if ((pkt.buttons & 0x0020) != 0) btnStr += "BACK(Disarm) ";
                    }
                    Console.Write("\r" + status + " LY(Alt): " + (pkt.thumbLY / 327.67).ToString("+000;-000; 000") + 
                                  "% | RX(Roll): " + (pkt.thumbRX / 327.67).ToString("+000;-000; 000") + 
                                  "% | RY(Pitch): " + (pkt.thumbRY / 327.67).ToString("+000;-000; 000") + 
                                  "% | Pkts: " + packetCount + " | " + btnStr + "        ");
                }

                Thread.Sleep(10); // 100 Hz
            }
        }
    }
}
