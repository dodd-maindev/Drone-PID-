/**
 * DRONE HITL - DIGITAL OSCILLOSCOPE & TELEMETRY ENGINE
 * Canvas 60 FPS Real-time Waveform Renderer, Signal Metrics, and Artificial Horizon.
 */

(function () {
  'use strict';

  // --- Cấu hình và Trạng thái Toàn cục ---
  const state = {
    isFrozen: false,
    timebase: 5,           // 5 giây cửa sổ trượt
    viewMode: 'quad',      // 'quad' (4 ô sóng riêng biệt) hoặc 'single' (chồng sóng)
    triggerMode: 'auto',   // 'auto', 'norm', 'freeze_step'
    cursor: { active: false, x: 0, y: 0, time: 0 },
    channels: {
      roll: true,
      pitch: true,
      alt: true,
      motors: true,
      rates: false
    },
    // Bộ đệm lưu trữ mẫu dữ liệu (Tối đa 1500 mẫu ~ 30 giây @ 50Hz)
    maxSamples: 1500,
    samples: [],
    lastPacketTime: 0,
    packetCounter: 0,
    fpsCounter: 0,
    lastFpsCalc: performance.now(),
    currentFps: 0,
    flightStartTime: null
  };

  // --- DOM Elements ---
  const canvas = document.getElementById('scopeCanvas');
  const ctx = canvas.getContext('2d');
  const viewportContainer = document.getElementById('viewport-container');

  const horizonCanvas = document.getElementById('horizonCanvas');
  const hzCtx = horizonCanvas.getContext('2d');

  const btnFreeze = document.getElementById('btn-freeze');
  const freezeText = document.getElementById('freeze-text');
  const freezeIcon = document.getElementById('freeze-icon');
  const btnClear = document.getElementById('btn-clear');
  const btnExportCsv = document.getElementById('btn-export-csv');
  const btnScreenshot = document.getElementById('btn-screenshot');

  const valStream = document.getElementById('val-stream');
  const valArm = document.getElementById('val-arm');
  const valFreq = document.getElementById('val-freq');
  const valTxRx = document.getElementById('val-txrx');
  const dotArm = document.getElementById('dot-arm');

  const cursorReadout = document.getElementById('cursor-val');

  // Channel readouts
  const elCh1Act = document.getElementById('ch1-act');
  const elCh1Tgt = document.getElementById('ch1-tgt');
  const elCh1Err = document.getElementById('ch1-err');

  const elCh2Act = document.getElementById('ch2-act');
  const elCh2Tgt = document.getElementById('ch2-tgt');
  const elCh2Err = document.getElementById('ch2-err');

  const elCh3Alt = document.getElementById('ch3-alt');
  const elCh3Tgt = document.getElementById('ch3-tgt');
  const elCh3Vz  = document.getElementById('ch3-vz');

  const elCh4M0  = document.getElementById('ch4-m0');
  const elCh4M1  = document.getElementById('ch4-m1');
  const elCh4M2  = document.getElementById('ch4-m2');
  const elCh4M3  = document.getElementById('ch4-m3');

  const elCh5P   = document.getElementById('ch5-p');
  const elCh5Q   = document.getElementById('ch5-q');
  const elCh5R   = document.getElementById('ch5-r');

  // Horizon & Motors DOM
  const elHzRoll = document.getElementById('hz-roll-disp');
  const elHzPitch = document.getElementById('hz-pitch-disp');
  const elHzYaw = document.getElementById('hz-yaw-disp');

  const mBars = [
    document.getElementById('mbar-0'),
    document.getElementById('mbar-1'),
    document.getElementById('mbar-2'),
    document.getElementById('mbar-3')
  ];
  const mVals = [
    document.getElementById('mval-0'),
    document.getElementById('mval-1'),
    document.getElementById('mval-2'),
    document.getElementById('mval-3')
  ];

  // Metrics DOM
  const elMRollVpp = document.getElementById('m-roll-vpp');
  const elMPitchVpp = document.getElementById('m-pitch-vpp');
  const elMZMax = document.getElementById('m-z-max');
  const elMVzMax = document.getElementById('m-vz-max');
  const elMMotDiff = document.getElementById('m-mot-diff');
  const elMFlightTime = document.getElementById('m-flight-time');

  // Màu sắc Oscilloscope
  const COLORS = {
    ch1: '#00F0FF',       // Roll Actual
    ch1Tgt: 'rgba(0, 240, 255, 0.45)', // Roll Target (Dashed)
    ch2: '#FFB800',       // Pitch Actual
    ch2Tgt: 'rgba(255, 184, 0, 0.45)', // Pitch Target (Dashed)
    ch3: '#00FF88',       // Alt Z Actual
    ch3Tgt: 'rgba(0, 255, 136, 0.45)', // Alt Z Target
    ch3Vz: '#38BDF8',     // Vz
    ch4_0: '#B347FF',     // M0
    ch4_1: '#E879F9',     // M1
    ch4_2: '#A855F7',     // M2
    ch4_3: '#C084FC',     // M3
    ch5: '#FF4766',       // Rate
    grid: 'rgba(255, 255, 255, 0.06)',
    gridSub: 'rgba(255, 255, 255, 0.025)',
    axis: 'rgba(255, 255, 255, 0.18)',
    cursor: '#FFFFFF'
  };

  // --- Resize Canvas ---
  function resizeCanvas() {
    const rect = viewportContainer.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(rect.width * dpr);
    canvas.height = Math.floor(rect.height * dpr);
    ctx.scale(dpr, dpr);
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // --- Server-Sent Events (SSE) Stream ---
  function setupSSE() {
    valStream.textContent = "CONNECTING...";
    valStream.style.color = "#FFB800";

    const evtSource = new EventSource('/events');

    evtSource.onopen = () => {
      valStream.textContent = "LIVE 50Hz";
      valStream.style.color = "#00FF88";
    };

    evtSource.onmessage = (e) => {
      try {
        const payload = JSON.parse(e.data);
        if (payload.type === 'history' && Array.isArray(payload.data)) {
          if (!state.isFrozen) {
            state.samples = payload.data;
          }
          return;
        }

        // Nhận mẫu dữ liệu thời gian thực
        state.packetCounter++;
        state.fpsCounter++;
        state.lastPacketTime = performance.now();

        if (!state.flightStartTime && payload.arm) {
          state.flightStartTime = payload.t;
        }

        if (!state.isFrozen) {
          state.samples.push(payload);
          if (state.samples.length > state.maxSamples) {
            state.samples.shift();
          }
        }

        updateHUD(payload);
      } catch (err) {
        // bỏ qua gói tin không đúng định dạng
      }
    };

    evtSource.onerror = () => {
      valStream.textContent = "OFFLINE";
      valStream.style.color = "#FF4766";
    };
  }

  // --- Cập nhật HUD & Thẻ Giá Trị Tức Thời ---
  function updateHUD(pkt) {
    // Trạng thái ARM
    if (pkt.arm) {
      valArm.textContent = "ARMED (" + (pkt.status || "BAY") + ")";
      valArm.style.color = "#00FF88";
      dotArm.classList.add('active');
    } else {
      valArm.textContent = "DISARMED";
      valArm.style.color = "#FF4766";
      dotArm.classList.remove('active');
    }

    valTxRx.textContent = `${pkt.tx || 0} / ${pkt.rx || 0}`;

    // CH1: Roll
    const rollErr = (pkt.r - (pkt.tr || 0)).toFixed(1);
    elCh1Act.textContent = pkt.r.toFixed(1) + '°';
    elCh1Tgt.textContent = (pkt.tr || 0).toFixed(1) + '°';
    elCh1Err.textContent = (rollErr > 0 ? '+' : '') + rollErr + '°';

    // CH2: Pitch
    const pitchErr = (pkt.p - (pkt.tp || 0)).toFixed(1);
    elCh2Act.textContent = pkt.p.toFixed(1) + '°';
    elCh2Tgt.textContent = (pkt.tp || 0).toFixed(1) + '°';
    elCh2Err.textContent = (pitchErr > 0 ? '+' : '') + pitchErr + '°';

    // CH3: Altitude
    elCh3Alt.textContent = pkt.z.toFixed(2) + 'm';
    elCh3Tgt.textContent = (pkt.tz || 2.0).toFixed(2) + 'm';
    elCh3Vz.textContent = pkt.vz.toFixed(2) + ' m/s';

    // CH4: 4 Motors
    const w = pkt.w || [0, 0, 0, 0];
    elCh4M0.textContent = Math.round(w[0]);
    elCh4M1.textContent = Math.round(w[1]);
    elCh4M2.textContent = Math.round(w[2]);
    elCh4M3.textContent = Math.round(w[3]);

    // Motor Bars (0 - 1000 rad/s)
    for (let i = 0; i < 4; i++) {
      const pct = Math.min(100, Math.max(0, (w[i] / 1000) * 100));
      mBars[i].style.height = pct + '%';
      mVals[i].textContent = Math.round(w[i]);
    }

    // CH5: Rates
    const rates = pkt.rates || [0, 0, 0];
    elCh5P.textContent = rates[0].toFixed(1) + '°/s';
    elCh5Q.textContent = rates[1].toFixed(1) + '°/s';
    elCh5R.textContent = rates[2].toFixed(1) + '°/s';

    // Horizon readout
    elHzRoll.textContent = `R: ${pkt.r.toFixed(1)}°`;
    elHzPitch.textContent = `P: ${pkt.p.toFixed(1)}°`;
    elHzYaw.textContent = `Y: ${pkt.y.toFixed(1)}°`;

    // Tính toán số liệu thống kê nhanh trên bộ đệm
    calcMetrics();

    // Thời gian bay
    if (state.flightStartTime) {
      const dur = Math.max(0, pkt.t - state.flightStartTime);
      const mins = Math.floor(dur / 60);
      const secs = (dur % 60).toFixed(1);
      elMFlightTime.textContent = `${String(mins).padStart(2, '0')}:${String(secs).padStart(4, '0')}`;
    }
  }

  // --- Thống kê DMM Metrics ---
  function calcMetrics() {
    if (state.samples.length < 2) return;
    const n = Math.min(state.samples.length, 250); // Lấy 5 giây gần nhất
    const recent = state.samples.slice(-n);

    let minR = 999, maxR = -999;
    let minP = 999, maxP = -999;
    let maxZ = 0, maxVz = 0;
    let maxM = 0, minM = 9999;

    for (let i = 0; i < recent.length; i++) {
      const s = recent[i];
      if (s.r < minR) minR = s.r;
      if (s.r > maxR) maxR = s.r;
      if (s.p < minP) minP = s.p;
      if (s.p > maxP) maxP = s.p;
      if (s.z > maxZ) maxZ = s.z;
      if (Math.abs(s.vz) > maxVz) maxVz = Math.abs(s.vz);

      if (s.w) {
        for (let j = 0; j < 4; j++) {
          if (s.w[j] > maxM) maxM = s.w[j];
          if (s.w[j] < minM) minM = s.w[j];
        }
      }
    }

    elMRollVpp.textContent = (maxR - minR).toFixed(1) + '°';
    elMPitchVpp.textContent = (maxP - minP).toFixed(1) + '°';
    elMZMax.textContent = maxZ.toFixed(2) + ' m';
    elMVzMax.textContent = maxVz.toFixed(2) + ' m/s';
    elMMotDiff.textContent = Math.round(maxM - minM) + ' rad/s';
  }

  // --- Vẽ Artificial Horizon (Con quay chân trời PFD) ---
  function drawArtificialHorizon() {
    const w = horizonCanvas.width;
    const h = horizonCanvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r = w / 2 - 4;

    hzCtx.clearRect(0, 0, w, h);

    // Lấy góc roll và pitch gần nhất
    let roll = 0, pitch = 0;
    if (state.samples.length > 0) {
      const last = state.samples[state.samples.length - 1];
      roll = last.r * (Math.PI / 180);  // rad
      pitch = last.p;                   // deg
    }

    hzCtx.save();
    // Vùng tròn bo tròn
    hzCtx.beginPath();
    hzCtx.arc(cx, cy, r, 0, Math.PI * 2);
    hzCtx.clip();

    // Dịch chuyển tâm theo Pitch và xoay theo Roll
    hzCtx.translate(cx, cy);
    hzCtx.rotate(-roll);
    const pitchOffset = pitch * 2.2; // 2.2 px mỗi độ
    hzCtx.translate(0, pitchOffset);

    // Bầu trời (Xanh Cyan)
    hzCtx.fillStyle = '#084b6f';
    hzCtx.fillRect(-w, -h * 2, w * 2, h * 2);

    // Mặt đất (Nâu Đất)
    hzCtx.fillStyle = '#422817';
    hzCtx.fillRect(-w, 0, w * 2, h * 2);

    // Đường chân trời (Trắng)
    hzCtx.strokeStyle = '#FFFFFF';
    hzCtx.lineWidth = 2;
    hzCtx.beginPath();
    hzCtx.moveTo(-w, 0);
    hzCtx.lineTo(w, 0);
    hzCtx.stroke();

    // Vạch chia độ Pitch (-20, -10, +10, +20)
    hzCtx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
    hzCtx.fillStyle = 'rgba(255, 255, 255, 0.8)';
    hzCtx.font = '9px JetBrains Mono';
    hzCtx.textAlign = 'center';
    hzCtx.textBaseline = 'middle';

    const pitchMarks = [-30, -20, -10, 10, 20, 30];
    pitchMarks.forEach(deg => {
      const y = -deg * 2.2;
      const lw = deg % 20 === 0 ? 36 : 20;
      hzCtx.beginPath();
      hzCtx.moveTo(-lw / 2, y);
      hzCtx.lineTo(lw / 2, y);
      hzCtx.stroke();
      if (Math.abs(deg) >= 10) {
        hzCtx.fillText(deg.toString(), -lw / 2 - 12, y);
        hzCtx.fillText(deg.toString(), lw / 2 + 12, y);
      }
    });

    hzCtx.restore();

    // Ký hiệu máy bay cố định (Cánh vàng ở giữa)
    hzCtx.save();
    hzCtx.translate(cx, cy);
    hzCtx.strokeStyle = '#FFB800';
    hzCtx.lineWidth = 3;
    hzCtx.fillStyle = '#FFB800';

    // Tâm
    hzCtx.beginPath();
    hzCtx.arc(0, 0, 3, 0, Math.PI * 2);
    hzCtx.fill();

    // Cánh trái
    hzCtx.beginPath();
    hzCtx.moveTo(-35, 0);
    hzCtx.lineTo(-12, 0);
    hzCtx.lineTo(-12, 6);
    hzCtx.stroke();

    // Cánh phải
    hzCtx.beginPath();
    hzCtx.moveTo(35, 0);
    hzCtx.lineTo(12, 0);
    hzCtx.lineTo(12, 6);
    hzCtx.stroke();
    hzCtx.restore();
  }

  // --- VẼ OSCILLOSCOPE CHÍNH TRÊN CANVAS ---
  function drawOscilloscope() {
    const rect = viewportContainer.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;

    ctx.clearRect(0, 0, w, h);

    if (state.samples.length < 2) {
      // Thông báo chờ
      ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
      ctx.font = '14px JetBrains Mono';
      ctx.textAlign = 'center';
      ctx.fillText('ĐANG CHỜ TÍN HIỆU TỪ ESP32 & GAZEBO SIM...', w / 2, h / 2);
      return;
    }

    const latest = state.samples[state.samples.length - 1];
    const tEnd = latest.t;
    const tStart = Math.max(0, tEnd - state.timebase);

    // Lọc mẫu nằm trong cửa sổ timebase
    const visibleSamples = state.samples.filter(s => s.t >= tStart && s.t <= tEnd);
    if (visibleSamples.length < 2) return;

    if (state.viewMode === 'quad') {
      // 4 Ô Sóng độc lập
      const scopeH = h / 4;

      // Scope 1: Roll (° vs Target)
      drawSubScope({
        index: 0,
        yStart: 0,
        height: scopeH,
        width: w,
        title: 'CH1: ROLL ANGLE φ (°)',
        yMin: -25,
        yMax: 25,
        yZero: 0,
        unit: '°',
        tStart, tEnd,
        samples: visibleSamples,
        curves: [
          { key: 'tr', color: COLORS.ch1Tgt, dashed: true, width: 1.5, name: 'Target Roll' },
          { key: 'r',  color: COLORS.ch1,    dashed: false, width: 2.2, name: 'Actual Roll' }
        ]
      });

      // Scope 2: Pitch (° vs Target)
      drawSubScope({
        index: 1,
        yStart: scopeH,
        height: scopeH,
        width: w,
        title: 'CH2: PITCH ANGLE θ (°)',
        yMin: -25,
        yMax: 25,
        yZero: 0,
        unit: '°',
        tStart, tEnd,
        samples: visibleSamples,
        curves: [
          { key: 'tp', color: COLORS.ch2Tgt, dashed: true, width: 1.5, name: 'Target Pitch' },
          { key: 'p',  color: COLORS.ch2,    dashed: false, width: 2.2, name: 'Actual Pitch' }
        ]
      });

      // Scope 3: Altitude Z (m) & Vz (m/s)
      drawSubScope({
        index: 2,
        yStart: scopeH * 2,
        height: scopeH,
        width: w,
        title: 'CH3: ALTITUDE Z (m) & Vz (m/s)',
        yMin: 0,
        yMax: 4.0,
        yZero: null,
        unit: 'm',
        tStart, tEnd,
        samples: visibleSamples,
        curves: [
          { key: 'tz', color: COLORS.ch3Tgt, dashed: true, width: 1.5, name: 'Target Z' },
          { key: 'z',  color: COLORS.ch3,    dashed: false, width: 2.2, name: 'Actual Z' },
          { key: 'vz', color: COLORS.ch3Vz,  dashed: false, width: 1.2, name: 'Vz' }
        ]
      });

      // Scope 4: 4x Motors Speed (rad/s)
      drawSubScope({
        index: 3,
        yStart: scopeH * 3,
        height: scopeH,
        width: w,
        title: 'CH4: 4x MOTORS SPEED ω (rad/s)',
        yMin: 0,
        yMax: 1000,
        yZero: null,
        unit: 'r/s',
        tStart, tEnd,
        samples: visibleSamples,
        curves: [
          { arrayKey: 'w', arrayIdx: 0, color: COLORS.ch4_0, dashed: false, width: 1.8, name: 'M0 (FR)' },
          { arrayKey: 'w', arrayIdx: 1, color: COLORS.ch4_1, dashed: false, width: 1.8, name: 'M1 (RL)' },
          { arrayKey: 'w', arrayIdx: 2, color: COLORS.ch4_2, dashed: false, width: 1.8, name: 'M2 (FL)' },
          { arrayKey: 'w', arrayIdx: 3, color: COLORS.ch4_3, dashed: false, width: 1.8, name: 'M3 (RR)' }
        ]
      });

    } else {
      // SINGLE OVERLAY MODE (Gộp chung toàn màn hình)
      drawSubScope({
        index: 0,
        yStart: 0,
        height: h,
        width: w,
        title: 'MULTI-CHANNEL COMPOSITE OSCILLOSCOPE',
        yMin: -30,
        yMax: 30,
        yZero: 0,
        unit: 'Units',
        tStart, tEnd,
        samples: visibleSamples,
        curves: [
          { key: 'tr', color: COLORS.ch1Tgt, dashed: true, width: 1.5 },
          { key: 'r',  color: COLORS.ch1,    dashed: false, width: 2.2 },
          { key: 'tp', color: COLORS.ch2Tgt, dashed: true, width: 1.5 },
          { key: 'p',  color: COLORS.ch2,    dashed: false, width: 2.2 },
          { key: 'z',  color: COLORS.ch3,    dashed: false, width: 2.0 }
        ]
      });
    }

    // Vẽ con trỏ Cursor nếu người dùng đang hover chuột
    if (state.cursor.active) {
      drawCursor(w, h, tStart, tEnd);
    }
  }

  // --- Hàm vẽ từng ô sóng thành phần ---
  function drawSubScope(cfg) {
    const { yStart, height, width, title, yMin, yMax, yZero, unit, tStart, tEnd, samples, curves } = cfg;

    const padLeft = 55;
    const padRight = 20;
    const padTop = 22;
    const padBottom = 16;

    const plotW = width - padLeft - padRight;
    const plotH = height - padTop - padBottom;
    const plotY0 = yStart + padTop;

    // 1. Nền & Viền ô
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    ctx.strokeRect(padLeft, plotY0, plotW, plotH);

    // Tiêu đề ô sóng
    ctx.fillStyle = 'rgba(255, 255, 255, 0.65)';
    ctx.font = '10px JetBrains Mono';
    ctx.textAlign = 'left';
    ctx.fillText(title, padLeft + 6, yStart + 14);

    // 2. Lưới Graticule (5 vạch ngang, 8 vạch đứng)
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 1;

    // Lưới ngang
    const yDivs = 4;
    for (let i = 0; i <= yDivs; i++) {
      const gy = plotY0 + (plotH / yDivs) * i;
      ctx.beginPath();
      ctx.moveTo(padLeft, gy);
      ctx.lineTo(padLeft + plotW, gy);
      ctx.stroke();

      // Nhãn giá trị Y
      const val = yMax - (yMax - yMin) * (i / yDivs);
      ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
      ctx.textAlign = 'right';
      ctx.fillText(val.toFixed(val % 1 === 0 ? 0 : 1) + unit, padLeft - 6, gy + 3);
    }

    // Đường chuẩn Zero (nếu có)
    if (yZero !== null && yZero >= yMin && yZero <= yMax) {
      const zeroY = plotY0 + plotH * (1 - (yZero - yMin) / (yMax - yMin));
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(padLeft, zeroY);
      ctx.lineTo(padLeft + plotW, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Lưới đứng (Thời gian)
    const tDivs = 6;
    for (let i = 0; i <= tDivs; i++) {
      const gx = padLeft + (plotW / tDivs) * i;
      ctx.beginPath();
      ctx.moveTo(gx, plotY0);
      ctx.lineTo(gx, plotY0 + plotH);
      ctx.stroke();

      if (cfg.index === 3 || state.viewMode === 'single') {
        const tVal = tStart + (tEnd - tStart) * (i / tDivs);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.textAlign = 'center';
        ctx.fillText(tVal.toFixed(1) + 's', gx, plotY0 + plotH + 12);
      }
    }

    // Hàm chuyển đổi tọa độ (t, val) -> (x, y) trên canvas
    function toCanvasX(t) {
      return padLeft + ((t - tStart) / (tEnd - tStart)) * plotW;
    }
    function toCanvasY(val) {
      const norm = (val - yMin) / (yMax - yMin);
      const clamped = Math.max(0, Math.min(1, norm));
      return plotY0 + plotH * (1 - clamped);
    }

    // 3. Vẽ các đường sóng (Waveform curves)
    ctx.save();
    // Giới hạn vẽ bên trong khung plot
    ctx.beginPath();
    ctx.rect(padLeft, plotY0, plotW, plotH);
    ctx.clip();

    curves.forEach(curve => {
      ctx.strokeStyle = curve.color;
      ctx.lineWidth = curve.width || 2;
      if (curve.dashed) {
        ctx.setLineDash([6, 5]);
      } else {
        ctx.setLineDash([]);
      }

      ctx.beginPath();
      let hasStarted = false;

      for (let i = 0; i < samples.length; i++) {
        const s = samples[i];
        let v = 0;
        if (curve.arrayKey && s[curve.arrayKey]) {
          v = s[curve.arrayKey][curve.arrayIdx] || 0;
        } else if (curve.key && s[curve.key] !== undefined) {
          v = s[curve.key];
        }

        const cx = toCanvasX(s.t);
        const cy = toCanvasY(v);

        if (!hasStarted) {
          ctx.moveTo(cx, cy);
          hasStarted = true;
        } else {
          ctx.lineTo(cx, cy);
        }
      }
      ctx.stroke();
    });

    ctx.restore();
  }

  // --- Vẽ đường trỏ đo tín hiệu (Inspector Cursor) ---
  function drawCursor(w, h, tStart, tEnd) {
    const padLeft = 55;
    const plotW = w - padLeft - 20;
    const cx = Math.max(padLeft, Math.min(padLeft + plotW, state.cursor.x));

    // Vẽ vạch đứng trỏ
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Tìm mẫu dữ liệu gần nhất với vị trí trỏ
    const tCursor = tStart + ((cx - padLeft) / plotW) * (tEnd - tStart);
    let nearest = null;
    let minDt = 999;

    for (let i = 0; i < state.samples.length; i++) {
      const dt = Math.abs(state.samples[i].t - tCursor);
      if (dt < minDt) {
        minDt = dt;
        nearest = state.samples[i];
      }
    }

    if (nearest) {
      cursorReadout.textContent = `T=${nearest.t.toFixed(3)}s | Roll=${nearest.r.toFixed(1)}° (Tgt:${(nearest.tr||0).toFixed(1)}°) | Pitch=${nearest.p.toFixed(1)}° | Z=${nearest.z.toFixed(2)}m | Mot=${nearest.w ? Math.round(nearest.w[0]) : 0}`;
    }
  }

  // --- Animation Render Loop ---
  function renderLoop() {
    drawOscilloscope();
    drawArtificialHorizon();

    // Tính FPS
    const now = performance.now();
    if (now - state.lastFpsCalc >= 1000) {
      state.currentFps = state.fpsCounter;
      state.fpsCounter = 0;
      state.lastFpsCalc = now;
      valFreq.textContent = `${state.currentFps} Hz`;
    }

    requestAnimationFrame(renderLoop);
  }

  // --- Xử lý Sự kiện Nút bấm & Tương tác ---

  // 1. Phím Freeze / Run (Tạm dừng dạng sóng)
  function toggleFreeze() {
    state.isFrozen = !state.isFrozen;
    if (state.isFrozen) {
      btnFreeze.classList.add('paused');
      freezeText.textContent = "RUN (RESUME)";
      freezeIcon.textContent = "▶";
    } else {
      btnFreeze.classList.remove('paused');
      freezeText.textContent = "FREEZE (PAUSE)";
      freezeIcon.textContent = "⏸";
    }
  }
  btnFreeze.addEventListener('click', toggleFreeze);
  window.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && e.target.tagName !== 'INPUT') {
      e.preventDefault();
      toggleFreeze();
    }
  });

  // 2. Nút Xóa lịch sử
  btnClear.addEventListener('click', () => {
    state.samples = [];
  });

  // 3. Xuất CSV
  btnExportCsv.addEventListener('click', () => {
    if (state.samples.length === 0) {
      alert("Chưa có dữ liệu để xuất CSV!");
      return;
    }

    let csv = "time_s,roll_deg,target_roll_deg,pitch_deg,target_pitch_deg,yaw_deg,alt_z_m,target_z_m,vz_mps,vx_mps,vy_mps,w0_rads,w1_rads,w2_rads,w3_rads,p_degs,q_degs,r_degs,armed\n";
    state.samples.forEach(s => {
      const w = s.w || [0,0,0,0];
      const rates = s.rates || [0,0,0];
      csv += `${s.t.toFixed(4)},${s.r.toFixed(2)},${(s.tr||0).toFixed(2)},${s.p.toFixed(2)},${(s.tp||0).toFixed(2)},${s.y.toFixed(2)},${s.z.toFixed(4)},${(s.tz||2.0).toFixed(2)},${s.vz.toFixed(3)},${s.vx.toFixed(3)},${s.vy.toFixed(3)},${w[0].toFixed(1)},${w[1].toFixed(1)},${w[2].toFixed(1)},${w[3].toFixed(1)},${rates[0].toFixed(2)},${rates[1].toFixed(2)},${rates[2].toFixed(2)},${s.arm ? 1 : 0}\n`;
    });

    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `drone_oscilloscope_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  });

  // 4. Chụp ảnh Snapshot
  btnScreenshot.addEventListener('click', () => {
    const dataUrl = canvas.toDataURL('image/png');
    const a = document.createElement('a');
    a.href = dataUrl;
    a.download = `oscilloscope_screenshot_${Date.now()}.png`;
    a.click();
  });

  // 5. Timebase Pills (1s, 2s, 5s, 10s, 20s)
  document.querySelectorAll('#timebase-pills .pill').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('#timebase-pills .pill').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      state.timebase = parseFloat(btn.dataset.tb);
    });
  });

  // 6. View Mode Pills (Quad Grid vs Single Overlay)
  document.querySelectorAll('#viewmode-pills .pill').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('#viewmode-pills .pill').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      state.viewMode = btn.dataset.mode;
    });
  });

  // 7. Cursor Hover
  viewportContainer.addEventListener('mousemove', (e) => {
    const rect = viewportContainer.getBoundingClientRect();
    state.cursor.active = true;
    state.cursor.x = e.clientX - rect.left;
    state.cursor.y = e.clientY - rect.top;
  });

  viewportContainer.addEventListener('mouseleave', () => {
    state.cursor.active = false;
    cursorReadout.textContent = "Hover mouse on waveform to inspect";
  });

  // Khởi động
  setupSSE();
  requestAnimationFrame(renderLoop);

})();
