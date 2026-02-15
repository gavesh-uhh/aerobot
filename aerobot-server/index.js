const express = require("express");
const crypto = require("crypto");
const os = require("os");
const multer = require("multer");

const app = express();
app.set("trust proxy", true);
app.use(express.json({ limit: "2mb" }));

const upload = multer({
  storage: multer.memoryStorage(),
  limits: { fileSize: 2 * 1024 * 1024 },
});

const PORT = Number(process.env.PORT || 3000);
const HOST = process.env.HOST || "0.0.0.0";

const BOOT_TS = Date.now();
const isoNow = () => new Date().toISOString();
const uptimeMs = () => Date.now() - BOOT_TS;

const pad = (n, w = 2) => String(n).padStart(w, "0");
const fmtUptime = () => {
  const ms = uptimeMs();
  const s = Math.floor(ms / 1000);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const r = s % 60;
  return `${pad(h)}:${pad(m)}:${pad(r)}`;
};

const getClientIp = (req) =>
  (
    req.headers["x-forwarded-for"]?.toString().split(",")[0] ||
    req.socket.remoteAddress ||
    ""
  ).trim();

const sanitize = (v) => {
  if (v == null) return "";
  const s = String(v);
  return s.length > 500 ? s.slice(0, 500) + "…" : s;
};

const reqId = () => crypto.randomBytes(6).toString("hex");

function logLine(level, msg, meta = {}) {
  const ts = isoNow();
  const up = fmtUptime();
  const parts = [`[${String(level).toUpperCase()}]`, ts, `up=${up}`, msg];
  for (const [k, v] of Object.entries(meta)) {
    if (v === undefined) continue;
    const val = typeof v === "object" ? JSON.stringify(v) : String(v);
    parts.push(`${k}=${val}`);
  }
  console.log(parts.join(" "));
}

function parseConfidence(raw) {
  if (raw == null) return { ok: false, value: null, reason: "missing" };
  const n = Number(raw);
  if (!Number.isFinite(n)) return { ok: false, value: null, reason: "nan" };
  const clamped = Math.max(0, Math.min(1, n));
  return { ok: true, value: clamped, reason: clamped !== n ? "clamped" : "ok" };
}

function safeHeader(req, name) {
  const v = req.headers[name.toLowerCase()];
  return v ? sanitize(v) : undefined;
}

function parseFeedData(raw) {
  if (!raw) return null;
  if (!raw.startsWith("DATA,")) return null;
  const parts = raw.split(",");
  if (parts.length < 13) return null;
  const toNum = (v) => {
    const n = Number(v);
    return Number.isFinite(n) ? n : null;
  };
  return {
    fire_conf: toNum(parts[1]),
    fire_state: toNum(parts[2]),
    locoState: toNum(parts[3]),
    gasState: toNum(parts[4]),
    mq2: toNum(parts[5]),
    mq7: toNum(parts[6]),
    mq135: toNum(parts[7]),
    gasLevel: toNum(parts[8]),
    gasRise: toNum(parts[9]),
    pir: toNum(parts[10]),
    dist: toNum(parts[11]),
    ldr: toNum(parts[12]),
  };
}

function parseUnoLine(raw) {
  if (!raw) return null;
  if (!raw.startsWith("UNO,")) return null;
  const parts = raw.split(",");
  if (parts.length < 12) return null;
  const toNum = (v) => {
    const n = Number(v);
    return Number.isFinite(n) ? n : null;
  };
  return {
    fire_conf: null,
    fire_state: null,
    locoState: toNum(parts[2]),
    gasState: toNum(parts[3]),
    mq2: toNum(parts[4]),
    mq7: toNum(parts[5]),
    mq135: toNum(parts[6]),
    gasLevel: toNum(parts[7]),
    gasRise: toNum(parts[8]),
    pir: toNum(parts[9]),
    dist: toNum(parts[10]),
    ldr: toNum(parts[11]),
  };
}

app.use((req, res, next) => {
  const id = reqId();
  res.locals.reqId = id;
  const start = process.hrtime.bigint();

  res.on("finish", () => {
    const end = process.hrtime.bigint();
    const ms = Number(end - start) / 1e6;
    logLine("info", "http", {
      id,
      method: req.method,
      path: req.originalUrl,
      status: res.statusCode,
      ms: Number(ms.toFixed(2)),
      ip: getClientIp(req),
      ua: safeHeader(req, "user-agent"),
      referer: safeHeader(req, "referer"),
    });
  });

  next();
});

const recent = [];
const RECENT_MAX = 200;
function pushRecent(evt) {
  recent.push(evt);
  if (recent.length > RECENT_MAX) recent.splice(0, recent.length - RECENT_MAX);
}

let fireStats = {
  count: 0,
  lastAt: null,
  lastConfidence: null,
  maxConfidence: 0,
};

let latestTelemetry = null;
const telemetryHistory = [];
const TELEMETRY_MAX = 300;

function recordTelemetry(parsed, meta = {}) {
  latestTelemetry = {
    ts: isoNow(),
    ...parsed,
    ...meta,
  };
  telemetryHistory.push(latestTelemetry);
  if (telemetryHistory.length > TELEMETRY_MAX) {
    telemetryHistory.splice(0, telemetryHistory.length - TELEMETRY_MAX);
  }
}

let latestFireFrame = null;

app.get("/", (req, res) => {
  res.type("text/plain").send(
    [
      "Aerobot ESP32 Listener",
      "",
      "Endpoints:",
      "  GET /selftest",
      "  GET /fireevent?confidence=0.83",
      "  GET /feed?data=DATA,...",
      "  POST /fireframe (multipart: confidence, frame)",
      "  GET /api/latest",
      "  GET /api/history?limit=120",
      "  GET /api/fireframe",
      "  GET /api/fireframe-meta",
      "  GET /health",
      "  GET /status",
      "  GET /recent",
      "  GET /ui",
      "",
    ].join("\n")
  );
});

app.get("/health", (req, res) => {
  res.json({ ok: true, ts: isoNow(), uptime: fmtUptime() });
});

app.get("/status", (req, res) => {
  res.json({
    ok: true,
    ts: isoNow(),
    uptime: fmtUptime(),
    host: os.hostname(),
    pid: process.pid,
    fire: fireStats,
    fireframe: latestFireFrame
      ? {
          ts: latestFireFrame.ts,
          confidence: latestFireFrame.confidence,
          size: latestFireFrame.size,
          ip: latestFireFrame.ip,
          mime: latestFireFrame.mime,
        }
      : null,
  });
});

app.get("/api/latest", (req, res) => {
  res.json({ ok: true, ts: isoNow(), telemetry: latestTelemetry });
});

app.get("/api/history", (req, res) => {
  const limit = Math.max(1, Math.min(300, Number(req.query.limit || 120)));
  res.json({
    ok: true,
    ts: isoNow(),
    limit,
    items: telemetryHistory.slice(-limit),
  });
});

app.post("/fireframe", upload.single("frame"), (req, res) => {
  const ip = getClientIp(req);
  const confidence = Number(req.body?.confidence);

  if (!req.file || !req.file.buffer) {
    logLine("warn", "esp32.fireframe.invalid", {
      id: res.locals.reqId,
      ip,
      reason: "missing_frame",
    });
    pushRecent({ ts: isoNow(), type: "fireframe_invalid", ip, reason: "missing_frame" });
    return res.status(400).json({ ok: false, error: "missing_frame" });
  }

  latestFireFrame = {
    ts: isoNow(),
    ip,
    confidence: Number.isFinite(confidence) ? confidence : null,
    size: req.file.size,
    mime: req.file.mimetype || "image/jpeg",
    bytes: req.file.buffer,
  };

  logLine("warn", "esp32.fireframe", {
    id: res.locals.reqId,
    ip,
    confidence: latestFireFrame.confidence,
    size: latestFireFrame.size,
    mime: latestFireFrame.mime,
  });

  pushRecent({
    ts: isoNow(),
    type: "fireframe",
    ip,
    confidence: latestFireFrame.confidence,
    size: latestFireFrame.size,
    mime: latestFireFrame.mime,
  });

  res.json({ ok: true, ts: latestFireFrame.ts, size: latestFireFrame.size });
});

app.get("/api/fireframe-meta", (req, res) => {
  if (!latestFireFrame) return res.status(404).json({ ok: false, error: "no_fireframe" });
  res.json({
    ok: true,
    ts: isoNow(),
    frame: {
      ts: latestFireFrame.ts,
      confidence: latestFireFrame.confidence,
      size: latestFireFrame.size,
      ip: latestFireFrame.ip,
      mime: latestFireFrame.mime,
    },
  });
});

app.get("/api/fireframe", (req, res) => {
  if (!latestFireFrame) return res.status(404).json({ ok: false, error: "no_fireframe" });
  res.setHeader("Content-Type", latestFireFrame.mime);
  res.setHeader("Cache-Control", "no-store");
  res.send(latestFireFrame.bytes);
});

app.get("/ui", (req, res) => {
  res.type("text/html").send(`<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Aerobot Telemetry</title>
<style>
@import url("https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;600&display=swap");
:root {
  --bg: #f7f7f8;
  --panel: #ffffff;
  --border: #e5e7eb;
  --text: #0f172a;
  --muted: #6b7280;
  --accent: #10a37f;
  --danger: #d14343;
  --fire: #e11d48;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: "JetBrains Mono", "Consolas", monospace;
  color: var(--text);
  background: var(--bg);
  min-height: 100vh;
}
.wrap {
  position: relative;
  max-width: 1600px;
  margin: 0 auto;
  padding: 32px 24px 64px;
}
header {
  display: flex;
  align-items: flex-end;
  justify-content: space-between;
  gap: 16px;
  margin-bottom: 18px;
}
.eyebrow {
  font-size: 12px;
  letter-spacing: 1.6px;
  text-transform: uppercase;
  color: var(--muted);
}
h1 { margin: 6px 0 0; font-size: 28px; font-weight: 600; }
.status {
  padding: 8px 14px;
  border-radius: 999px;
  background: #ffffff;
  border: 1px solid var(--border);
  font-size: 13px;
  color: var(--accent);
}
.status.stale { color: var(--danger); border-color: rgba(209, 67, 67, 0.4); }
.banner{
  border: 1px solid rgba(209,67,67,0.35);
  background: rgba(209,67,67,0.08);
  border-radius: 14px;
  padding: 14px 16px;
  margin: 0 0 16px 0;
}
.banner.hidden{ display:none; }
.banner-title{
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 1.4px;
  color: var(--danger);
  font-weight: 700;
}
.banner-detail{ margin-top: 8px; font-size: 13px; color: var(--text); }

.banner.fire{
  border: 1px solid rgba(225,29,72,0.45);
  background: rgba(225,29,72,0.10);
}
.banner.fire .banner-title{
  color: var(--fire);
}

.meta {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  gap: 10px;
  margin-bottom: 18px;
  font-size: 13px;
  color: var(--muted);
}
.meta span { color: var(--text); font-weight: 500; margin-left: 6px; }
.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 12px;
}
.card {
  position: relative;
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.04);
}
.card h3 {
  margin: 0 0 10px;
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 1.2px;
  color: var(--muted);
}
.value { font-size: 24px; font-weight: 600; letter-spacing: 0.2px; }
.value.sm { font-size: 18px; }
.sub { margin-top: 6px; font-size: 13px; color: var(--muted); }
.row { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
.mini .label { font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; }
.predictions { display: grid; gap: 8px; font-size: 12px; color: var(--muted); }
.prediction { display: flex; justify-content: space-between; gap: 12px; }
.prediction .label { text-transform: uppercase; letter-spacing: 1px; color: var(--muted); font-size: 11px; }
.prediction .value { font-size: 12px; font-weight: 500; }
.prediction .value.warn { color: var(--danger); }
.card.wide { grid-column: span 2; }
.trend { margin-top: 10px; }
.sparkline {
  width: 100%;
  height: 64px;
  display: block;
  border-radius: 8px;
  border: 1px solid var(--border);
  background: #f9fafb;
}
.trend-label { margin-top: 6px; font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; }
.loco-box {
  position: relative;
  height: 96px;
  border-radius: 12px;
  border: 1px solid var(--border);
  background: #f9fafb;
  overflow: hidden;
}
.loco-box::before {
  content: "";
  position: absolute;
  inset: 0;
  opacity: 0;
  transition: opacity 0.25s ease;
  z-index: 1;
}
.loco-box::after {
  content: "";
  position: absolute;
  left: 50%;
  top: 8px;
  bottom: 8px;
  width: 2px;
  transform: translateX(-50%);
  background: linear-gradient(transparent, rgba(15, 23, 42, 0.12), transparent);
  z-index: 2;
}
.loco-box.fwd::before { opacity: 1; background: linear-gradient(180deg, rgba(16, 163, 127, 0.3), transparent 65%); }
.loco-box.back::before { opacity: 1; background: linear-gradient(0deg, rgba(16, 163, 127, 0.3), transparent 65%); }
.loco-box.left::before { opacity: 1; background: linear-gradient(90deg, rgba(16, 163, 127, 0.3), transparent 65%); }
.loco-box.right::before { opacity: 1; background: linear-gradient(270deg, rgba(16, 163, 127, 0.3), transparent 65%); }
.loco-car {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 120px;
  height: 54px;
  transform: translate(-50%, -50%);
  border-radius: 14px;
  background: #ffffff;
  border: 1px solid var(--border);
  box-shadow: 0 3px 8px rgba(0, 0, 0, 0.05);
  z-index: 3;
}
.loco-car::before {
  content: "";
  position: absolute;
  top: 6px;
  left: 50%;
  width: 42px;
  height: 4px;
  transform: translateX(-50%);
  border-radius: 999px;
  background: rgba(15, 23, 42, 0.12);
}
.light-box {
  --light: 0;
  margin-top: 10px;
  height: 78px;
  border-radius: 10px;
  border: 1px solid var(--border);
  background: rgba(255, 244, 214, calc(0.06 + var(--light) * 0.5));
  display: grid;
  place-items: center;
  transition: background 0.25s ease, box-shadow 0.25s ease;
  box-shadow: 0 0 calc(6px + var(--light) * 18px) rgba(255, 214, 120, calc(0.1 + var(--light) * 0.5));
}
.light-core {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background: rgba(255, 230, 150, calc(0.12 + var(--light) * 0.7));
  box-shadow: 0 0 calc(8px + var(--light) * 22px) rgba(255, 214, 120, calc(0.18 + var(--light) * 0.6));
}
.snap {
  width: 100%;
  height: 320px;
  border-radius: 12px;
  border: 1px solid var(--border);
  background: #f9fafb;
  overflow: hidden;
  display: grid;
  place-items: center;
}
.snap img { width: 100%; height: 100%; object-fit: cover; display: block; }
.snap.empty {
  background: repeating-linear-gradient(45deg,#f9fafb,#f9fafb 10px,#f3f4f6 10px,#f3f4f6 20px);
  color: var(--muted);
  font-size: 12px;
  letter-spacing: 0.8px;
  text-transform: uppercase;
}
.snap-meta {
  margin-top: 10px;
  font-size: 13px;
  color: var(--muted);
  display: flex;
  justify-content: space-between;
  gap: 12px;
  flex-wrap: wrap;
}
.snap-meta b { color: var(--text); font-weight: 600; }

@media (max-width: 700px) {
  header { flex-direction: column; align-items: flex-start; }
  .card.wide { grid-column: span 1; }
  .row { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .snap { height: 240px; }
}
@media (prefers-reduced-motion: reduce) {
  * { animation: none !important; transition: none !important; }
}
</style>
</head>
<body>
<div class="wrap">
<header>
  <div>
    <div class="eyebrow">Aerobot Prototype</div>
    <h1>Telemetry</h1>
  </div>
  <div id="statusPill" class="status" aria-live="polite">Waiting for data</div>
</header>

<div id="fireAlertBanner" class="banner fire hidden">
  <div class="banner-title">FIRE DETECTED</div>
  <div id="fireAlertBannerDetail" class="banner-detail">--</div>
</div>

<div id="alertBanner" class="banner hidden">
  <div class="banner-title">POSSIBLE GAS HOTSPOT FOUND</div>
  <div id="alertBannerDetail" class="banner-detail">--</div>
</div>

<div class="meta">
  <div>Last update<span id="lastUpdate">--</span></div>
  <div>Gas mode<span id="gasState">--</span></div>
</div>

<section class="grid">
  <div class="card wide">
    <h3>Locomotion</h3>
    <div id="locoBox" class="loco-box stop"><div class="loco-car"></div></div>
    <div id="locoModeText" class="sub">--</div>
  </div>

  <div class="card">
    <h3>Fire</h3>
    <div id="fireConfVal" class="value">--</div>
    <div id="fireStateVal" class="sub">--</div>
  </div>

  <div class="card">
    <h3>Distance</h3>
    <div id="distVal" class="value">--</div>
    <div class="sub">Ultrasonic</div>
  </div>

  <div class="card">
    <h3>Gas</h3>
    <div id="gasLevelVal" class="value">--</div>
    <div class="sub">Rise <span id="gasRiseVal">--</span></div>
    <div class="trend">
      <canvas id="gasTrend" class="sparkline" aria-label="Gas trend"></canvas>
      <div class="trend-label">Trend (recent)</div>
    </div>
  </div>

  <div class="card wide">
    <h3>Air</h3>
    <div class="row">
      <div class="mini"><div class="label">MQ2</div><div id="mq2Val" class="value sm">--</div></div>
      <div class="mini"><div class="label">MQ7</div><div id="mq7Val" class="value sm">--</div></div>
      <div class="mini"><div class="label">MQ135</div><div id="mq135Val" class="value sm">--</div></div>
    </div>
  </div>

  <div class="card wide">
    <h3>Client Side Predictions</h3>
    <div class="predictions">
      <div class="prediction"><div class="label">MQ2</div><div id="mq2Hint" class="value">LPG/smoke status unknown</div></div>
      <div class="prediction"><div class="label">MQ7</div><div id="mq7Hint" class="value">CO status unknown</div></div>
      <div class="prediction"><div class="label">MQ135</div><div id="mq135Hint" class="value">Air quality status unknown</div></div>
    </div>
  </div>

  <div class="card">
    <h3>Light</h3>
    <div id="ldrVal" class="value">--</div>
    <div class="sub">LDR</div>
    <div id="ldrBox" class="light-box"><div class="light-core"></div></div>
  </div>

  <div class="card wide">
    <h3>Fire Snapshot</h3>
    <div id="snapBox" class="snap empty">No snapshot yet</div>
    <div class="snap-meta">
      <div>Captured <b id="snapWhen">--</b></div>
      <div>Conf <b id="snapConf">--</b></div>
      <div>Size <b id="snapSize">--</b></div>
    </div>
  </div>
</section>
</div>

<script>
const locoLabels = ["STOP", "FWD", "BACK", "LEFT", "RIGHT"];
const gasLabels = ["PASSIVE", "HUNT", "LOCK"];
const locoClasses = ["stop", "fwd", "back", "left", "right"];
const gasHistory = [];
const GAS_HISTORY_MAX = 120;

const el = (id) => document.getElementById(id);
const clamp = (v, min, max) => Math.max(min, Math.min(max, v));

function setText(id, value) {
  const s = value === null || value === undefined || value === "" ? "--" : String(value);
  el(id).textContent = s;
}
function setNumber(id, value, suffix = "") {
  const n = Number(value);
  if (!Number.isFinite(n)) { el(id).textContent = "--"; return; }
  el(id).textContent = String(n) + suffix;
}
function formatAge(ts) {
  if (!ts) return "--";
  const ms = Date.now() - new Date(ts).getTime();
  if (!Number.isFinite(ms) || ms < 0) return "--";
  const sec = Math.floor(ms / 1000);
  if (sec < 60) return sec + "s ago";
  const min = Math.floor(sec / 60);
  if (min < 60) return min + "m ago";
  const hr = Math.floor(min / 60);
  return hr + "h ago";
}
function setStatus(text, stale = false) {
  const pill = el("statusPill");
  pill.textContent = text;
  pill.classList.toggle("stale", stale);
}
function setLocoMode(value) {
  const box = el("locoBox");
  for (const cls of locoClasses) box.classList.remove(cls);
  const idx = Number.isFinite(Number(value)) ? Number(value) : -1;
  const cls = locoClasses[idx] || "stop";
  box.classList.add(cls);
  setText("locoModeText", locoLabels[idx] || "--");
}
function pushGasSample(value) {
  if (!Number.isFinite(value)) return;
  gasHistory.push(value);
  if (gasHistory.length > GAS_HISTORY_MAX) {
    gasHistory.splice(0, gasHistory.length - GAS_HISTORY_MAX);
  }
}
function drawSparkline() {
  const canvas = el("gasTrend");
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (!w || !h) return;
  canvas.width = Math.floor(w * dpr);
  canvas.height = Math.floor(h * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const vals = gasHistory.filter(Number.isFinite);
  if (vals.length < 2) {
    ctx.strokeStyle = "rgba(16, 163, 127, 0.3)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();
    return;
  }

  const min = Math.min(...vals);
  const max = Math.max(...vals);
  const range = max - min || 1;

  ctx.beginPath();
  vals.forEach((v, i) => {
    const x = (i / (vals.length - 1)) * w;
    const y = h - ((v - min) / range) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = "rgba(16, 163, 127, 0.9)";
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

const baselines = { mq2: null, mq7: null, mq135: null };
const BASELINE_ALPHA = 0.03;
const MIN_DELTA = 18;

function ema(prev, next, a) {
  if (!Number.isFinite(next)) return prev;
  if (!Number.isFinite(prev)) return next;
  return prev + a * (next - prev);
}

function updateBaselines(mq2, mq7, mq135, gasStateNum) {
  if (gasStateNum === 2) return;
  baselines.mq2 = ema(baselines.mq2, mq2, BASELINE_ALPHA);
  baselines.mq7 = ema(baselines.mq7, mq7, BASELINE_ALPHA);
  baselines.mq135 = ema(baselines.mq135, mq135, BASELINE_ALPHA);
}

function classifyGasRelative(mq2, mq7, mq135) {
  const items = [
    { sensor: "MQ7",   gasName: "Carbon Monoxide (CO)",                also: "exhaust-like gases",                        v: mq7,   b: baselines.mq7 },
    { sensor: "MQ2",   gasName: "Methane / LPG / Smoke (combustibles)",also: "hydrogen + other combustibles",            v: mq2,   b: baselines.mq2 },
    { sensor: "MQ135", gasName: "Ammonia / VOCs (air quality)",        also: "benzene-ish + other VOCs",                 v: mq135, b: baselines.mq135 },
  ];

  for (const x of items) {
    const vOk = Number.isFinite(x.v);
    const bOk = Number.isFinite(x.b);
    x.delta = (vOk && bOk) ? (x.v - x.b) : -1e9;
  }

  items.sort((a, b) => b.delta - a.delta);
  const top = items[0];
  const strong = top.delta >= MIN_DELTA;

  let detail = "LOCK mode active, but baseline is still warming up.";
  if (Number.isFinite(top.delta) && top.delta > -1e8) {
    detail = strong
      ? ("Detected: " + top.gasName + " (Δ " + Math.round(top.delta) + ") — approx: " + top.also)
      : ("LOCK mode active, but no strong relative spike vs baseline yet (best match: " + top.gasName + ", Δ " + Math.round(top.delta) + ").");
  }

  return { top, strong, detail };
}

async function tick() {
  try {
    const r = await fetch("/api/latest", { cache: "no-store" });
    const j = await r.json();
    const t = j.telemetry;

    const banner = el("alertBanner");
    const bannerDetail = el("alertBannerDetail");
    const fireBanner = el("fireAlertBanner");
    const fireBannerDetail = el("fireAlertBannerDetail");

    if (!t) {
      setStatus("Aerobot Offline", true);
      setText("lastUpdate", "--");
      setText("gasState", "--");
      setLocoMode(null);
      banner.classList.add("hidden");
      fireBanner.classList.add("hidden");
      return;
    }

    const age = Date.now() - new Date(t.ts).getTime();
    const stale = age > 5000;
    setStatus(stale ? "Reconnecting" : "Connected", stale);

    setLocoMode(t.locoState);
    setText("gasState", gasLabels[t.gasState] || "--");
    setText("lastUpdate", formatAge(t.ts));

    setNumber("distVal", t.dist, " cm");
    setNumber("ldrVal", t.ldr);
    setNumber("gasLevelVal", t.gasLevel);
    setNumber("gasRiseVal", t.gasRise);
    setNumber("mq2Val", t.mq2);
    setNumber("mq7Val", t.mq7);
    setNumber("mq135Val", t.mq135);

    setText("fireStateVal", t.fire_state === null || t.fire_state === undefined ? "--" : (t.fire_state ? "ON" : "OFF"));
    const fireConf = t.fire_conf === null || t.fire_conf === undefined ? null : Number(t.fire_conf);
    if (fireConf === null || Number.isNaN(fireConf)) setText("fireConfVal", "--");
    else setText("fireConfVal", (fireConf * 100).toFixed(1) + "%");

    const fireOn = !!t.fire_state;
    const fireConfNum = (fireConf === null || fireConf === undefined) ? null : Number(fireConf);
    const FIRE_BANNER_CONF_TH = 0.70;
    const showFire = fireOn || (Number.isFinite(fireConfNum) && fireConfNum >= FIRE_BANNER_CONF_TH);

    if (showFire) {
      const parts = [];
      if (fireOn) parts.push("State: ON");
      if (Number.isFinite(fireConfNum)) parts.push("Confidence: " + (fireConfNum * 100).toFixed(1) + "%");
      fireBannerDetail.textContent = parts.length ? parts.join(" · ") : "Fire signal active";
      fireBanner.classList.remove("hidden");
    } else {
      fireBanner.classList.add("hidden");
    }

    const mq2Val = Number(t.mq2);
    const mq7Val = Number(t.mq7);
    const mq135Val = Number(t.mq135);
    const ldrVal = Number(t.ldr);
    const gasLevelVal = Number(t.gasLevel);
    const gasStateNum = Number(t.gasState);

    updateBaselines(mq2Val, mq7Val, mq135Val, gasStateNum);

    el("mq2Hint").textContent = Number.isFinite(mq2Val) ? "MQ2 reading updating" : "MQ2 status unknown";
    el("mq7Hint").textContent = Number.isFinite(mq7Val) ? "MQ7 reading updating" : "MQ7 status unknown";
    el("mq135Hint").textContent = Number.isFinite(mq135Val) ? "MQ135 reading updating" : "MQ135 status unknown";
    el("mq2Hint").classList.remove("warn");
    el("mq7Hint").classList.remove("warn");
    el("mq135Hint").classList.remove("warn");

    if (gasStateNum === 2) {
      const info = classifyGasRelative(mq2Val, mq7Val, mq135Val);
      bannerDetail.textContent = info.detail;
      banner.classList.remove("hidden");
    } else {
      banner.classList.add("hidden");
    }

    if (Number.isFinite(ldrVal)) {
      const lightPct = clamp(1 - (ldrVal / 1023), 0, 1);
      el("ldrBox").style.setProperty("--light", lightPct.toFixed(3));
    } else {
      el("ldrBox").style.setProperty("--light", "0");
    }

    pushGasSample(gasLevelVal);
    drawSparkline();
  } catch (e) {
    setStatus("Disconnected", true);
    el("alertBanner").classList.add("hidden");
    el("fireAlertBanner").classList.add("hidden");
  }
}

let lastFrameTs = null;
async function tickFrame() {
  try {
    const r = await fetch("/api/fireframe-meta", { cache: "no-store" });
    if (!r.ok) {
      const box = el("snapBox");
      box.classList.add("empty");
      box.textContent = "No snapshot yet";
      setText("snapWhen", "--");
      setText("snapConf", "--");
      setText("snapSize", "--");
      return;
    }
    const j = await r.json();
    const f = j.frame;
    if (!f) return;

    setText("snapWhen", formatAge(f.ts));
    setText("snapConf", Number.isFinite(Number(f.confidence)) ? (Number(f.confidence) * 100).toFixed(1) + "%" : "--");
    setText("snapSize", Number.isFinite(Number(f.size)) ? Math.round(Number(f.size) / 1024) + " KB" : "--");

    if (f.ts !== lastFrameTs) {
      lastFrameTs = f.ts;
      const box = el("snapBox");
      box.classList.remove("empty");
      box.innerHTML = '<img alt="Fire snapshot" src="/api/fireframe?ts=' + encodeURIComponent(f.ts) + '">';
    }
  } catch (e) {}
}

setInterval(tick, 900);
tick();
setInterval(tickFrame, 1200);
tickFrame();
</script>
</body>
</html>`);
});

app.get("/recent", (req, res) => {
  const limit = Math.max(1, Math.min(200, Number(req.query.limit || 50)));
  res.json({ ok: true, ts: isoNow(), limit, events: recent.slice(-limit) });
});

app.get("/selftest", (req, res) => {
  logLine("info", "esp32.selftest", {
    id: res.locals.reqId,
    ip: getClientIp(req),
    ua: safeHeader(req, "user-agent"),
  });
  pushRecent({ ts: isoNow(), type: "selftest", ip: getClientIp(req) });
  res.json({ ok: true, ts: isoNow() });
});

app.get("/fireevent", (req, res) => {
  const raw = req.query.confidence;
  const parsed = parseConfidence(raw);

  if (!parsed.ok) {
    logLine("warn", "esp32.fireevent.invalid", {
      id: res.locals.reqId,
      ip: getClientIp(req),
      confidence_raw: sanitize(raw),
      reason: parsed.reason,
    });
    pushRecent({
      ts: isoNow(),
      type: "fireevent_invalid",
      ip: getClientIp(req),
      confidence_raw: sanitize(raw),
      reason: parsed.reason,
    });
    return res.status(400).json({ ok: false, error: "invalid_confidence", reason: parsed.reason });
  }

  fireStats.count += 1;
  fireStats.lastAt = isoNow();
  fireStats.lastConfidence = parsed.value;
  fireStats.maxConfidence = Math.max(fireStats.maxConfidence, parsed.value);

  logLine("warn", "esp32.fireevent", {
    id: res.locals.reqId,
    ip: getClientIp(req),
    confidence: Number(parsed.value.toFixed(3)),
    note: parsed.reason === "clamped" ? "clamped_to_0_1" : undefined,
  });

  pushRecent({ ts: isoNow(), type: "fireevent", ip: getClientIp(req), confidence: parsed.value });
  res.json({ ok: true, ts: isoNow(), confidence: parsed.value });
});

app.get("/feed", (req, res) => {
  const raw = req.query.data;
  const data = sanitize(raw);
  const parsed = parseFeedData(data) || parseUnoLine(data);

  if (!data) {
    logLine("warn", "esp32.feed.invalid", { id: res.locals.reqId, ip: getClientIp(req), reason: "missing_data" });
    pushRecent({ ts: isoNow(), type: "feed_invalid", ip: getClientIp(req), reason: "missing_data" });
    return res.status(400).json({ ok: false, error: "missing_data" });
  }

  logLine("info", "esp32.feed", { id: res.locals.reqId, ip: getClientIp(req), data, parsed: parsed || undefined });

  if (parsed) {
    if (parsed.fire_conf != null) {
      fireStats.lastConfidence = parsed.fire_conf;
      fireStats.maxConfidence = Math.max(fireStats.maxConfidence, parsed.fire_conf);
      fireStats.lastAt = isoNow();
    }
    recordTelemetry(parsed, { ip: getClientIp(req), raw: data });
  }

  pushRecent({ ts: isoNow(), type: "feed", ip: getClientIp(req), data, parsed: parsed || undefined });
  res.json({ ok: true, ts: isoNow() });
});

app.use((req, res) => {
  res.status(404).json({ ok: false, error: "not_found" });
});

app.use((err, req, res, next) => {
  logLine("error", "server.error", {
    id: res.locals.reqId,
    path: req.originalUrl,
    err: sanitize(err?.message || String(err)),
  });
  res.status(500).json({ ok: false, error: "internal_error" });
});

app.listen(PORT, HOST, () => {
  logLine("info", "server.start", {
    host: HOST,
    port: PORT,
    ts: isoNow(),
    uptime: fmtUptime(),
    node: process.version,
  });
});
