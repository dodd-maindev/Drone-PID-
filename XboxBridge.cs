using System;
using System.Net;
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

        static void Main(string[] args) {
            string targetIp = (args.Length > 0 && !string.IsNullOrEmpty(args[0])) ? args[0] : "127.0.0.1";
            int port = (args.Length > 1) ? int.Parse(args[1]) : 9099;

            Console.WriteLine("====================================================");
            Console.WriteLine("   XBOX 360 CONTROLLER -> WSL2/DRONE UDP BRIDGE    ");
            Console.WriteLine("====================================================");
            Console.WriteLine(" Target Destination: " + targetIp + ":" + port);
            Console.WriteLine(" Frequency: 100 Hz (10ms)");
            Console.WriteLine(" Press Ctrl+C to terminate.");
            Console.WriteLine("----------------------------------------------------\n");

            UdpClient udp = new UdpClient();
            IPEndPoint endPoint = new IPEndPoint(IPAddress.Parse(targetIp), port);
            IPEndPoint loopbackEndPoint = new IPEndPoint(IPAddress.Loopback, port);

            XINPUT_STATE state = new XINPUT_STATE();
            uint packetCount = 0;
            DateTime lastPrint = DateTime.Now;

            while (true) {
                int res = GetControllerState(0, ref state);
                bool connected = (res == 0);

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
                }

                byte[] data = PacketToBytes(pkt);
                try {
                    udp.Send(data, data.Length, endPoint);
                    if (targetIp != "127.0.0.1") {
                        udp.Send(data, data.Length, loopbackEndPoint);
                    }
                    packetCount++;
                } catch (Exception ex) {
                    // ignore network blips
                }

                if ((DateTime.Now - lastPrint).TotalMilliseconds >= 250) {
                    lastPrint = DateTime.Now;
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
