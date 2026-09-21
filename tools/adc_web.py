#!/usr/bin/env python3
"""Serve the Doppler scope as a web page, so you can watch it on a phone.

The point is walking tests: you cannot wave at the radar and read a monitor
across the room at the same time. This puts the waterfall on whatever device
is in your hand.

    .venv/bin/python tools/adc_web.py
    # then open http://192.168.0.10:8080 on your phone

The board streams UDP to this machine; this process receives it, runs the same
STFT as tools/adc_scope.py, and pushes frames to browsers over Server-Sent
Events. SSE rather than WebSockets because it is one-way anyway and needs no
dependency outside the standard library; the browser reconnects on its own.

Bandwidth is one byte per spectrogram column per frame, base64'd: at 240
columns and 39 frames/s that is ~13 KB/s, which is nothing over WiFi and fine
on a phone hotspot.

**It binds all interfaces and has no authentication.** It is a read-only view
of an ADC on your own LAN, but do not run it on a network you do not trust;
`--bind 127.0.0.1` restricts it to this machine.

Only one process can hold the UDP port, so stop the other receivers first.
"""
from __future__ import annotations

import argparse
import base64
import json
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.adc_stream_rx import parse_all                  # noqa: E402
from tools.adc_scope import Pipeline, HZ_PER_MS            # noqa: E402

STATE_LOCK = threading.Lock()
SUBSCRIBERS: set[queue.Queue] = set()
SUB_LOCK = threading.Lock()


class Shared:
    """Everything the HTTP threads read and the UDP thread writes."""

    def __init__(self, args):
        self.args = args
        self.pipe = Pipeline(args.nfft, args.fmax, width=args.cols)
        self.fs = 20000.0
        self.channel = 0
        self.connected = False
        self.n_pkt = 0
        self.lost = 0
        self.drops = 0
        self.drops0 = None
        self.t0 = None
        self.last_rx = 0.0
        self.config_gen = 0

    def config(self):
        return {
            "gen": self.config_gen,
            "fs": self.fs,
            "fmax": self.pipe.fmax,
            "nfft": self.pipe.nfft,
            "cols": self.pipe.width,
            "row_ms": self.pipe.row_ms,
            "bin_ms": self.pipe.bin_ms,
            "vmax": self.pipe.fmax / HZ_PER_MS,
            "floor": self.args.floor,
            "ceil": self.args.ceil,
            "channel": self.channel,
            "hz_per_ms": HZ_PER_MS,
        }


def broadcast(kind, payload):
    dead = []
    with SUB_LOCK:
        subs = list(SUBSCRIBERS)
    msg = f"event: {kind}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n"
    for q in subs:
        try:
            q.put_nowait(msg)
        except queue.Full:
            # A phone that locked its screen stops draining. Drop its backlog
            # rather than stalling the UDP thread for everyone else.
            dead.append(q)
    for q in dead:
        with SUB_LOCK:
            SUBSCRIBERS.discard(q)


def udp_thread(sh, sock):
    import numpy as np
    prev_seq = None
    lo, hi = sh.args.floor, sh.args.ceil
    while True:
        try:
            data, _peer = sock.recvfrom(65535)
        except socket.timeout:
            if sh.connected and time.monotonic() - sh.last_rx > 2.0:
                sh.connected = False
                broadcast("status", {"connected": False})
            continue
        try:
            records = list(parse_all(data))
        except ValueError:
            continue

        for hdr, payload in records:
            sh.last_rx = time.monotonic()
            if not sh.connected:
                sh.connected = True
                with STATE_LOCK:
                    sh.fs = hdr.sample_rate
                    sh.channel = hdr.channel
                    sh.pipe.fs = hdr.sample_rate
                    sh.pipe.set_geometry()
                    sh.config_gen += 1
                    cfg = sh.config()
                sh.t0 = time.monotonic()
                broadcast("config", cfg)
            if sh.drops0 is None:
                sh.drops0 = hdr.dropped
            sh.drops = (hdr.dropped - sh.drops0) & 0xffffffff
            if prev_seq is not None:
                sh.lost += ((hdr.seq - prev_seq) & 0xffffffff) - 1
            prev_seq = hdr.seq
            sh.n_pkt += 1
            sh.channel = hdr.channel

            with STATE_LOCK:
                frames = list(sh.pipe.feed(payload))
                gen = sh.config_gen
            for fr in frames:
                cols = fr["cols"]
                with np.errstate(divide="ignore"):
                    db = 20.0 * np.log10(np.maximum(cols, 1e-6))
                lvl = np.clip((db - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
                lvl[db < lo] = 0
                broadcast("frame", {
                    "g": gen,
                    "dc": round(fr["dc"], 2),
                    "ac": round(fr["ac"], 3),
                    "hz": round(fr["pk_hz"], 1),
                    "ms": round(fr["pk_ms"], 3),
                    "amp": round(fr["pk_amp"], 2),
                    "r": 1 if fr["real"] else 0,
                    "pk": sh.n_pkt, "lo": sh.lost, "dr": sh.drops,
                    "c": base64.b64encode(lvl.tobytes()).decode(),
                })


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    shared: Shared = None

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif path == "/api/config":
            with STATE_LOCK:
                cfg = self.shared.config()
            self._send(200, "application/json", json.dumps(cfg).encode())
        elif path == "/stream":
            self.stream()
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if self.path.split("?")[0] != "/api/set":
            self._send(404, "text/plain", b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            self._send(400, "text/plain", b"bad json")
            return
        sh = self.shared
        with STATE_LOCK:
            kw = {}
            if "fmax" in req:
                kw["fmax"] = max(50.0, min(sh.fs / 2, float(req["fmax"])))
            if "nfft" in req:
                v = int(req["nfft"])
                kw["nfft"] = max(128, min(8192, 1 << (v.bit_length() - 1)))
            if kw:
                sh.pipe.set_geometry(**kw)
                sh.config_gen += 1
            cfg = sh.config()
        broadcast("config", cfg)
        self._send(200, "application/json", json.dumps(cfg).encode())

    def stream(self):
        q = queue.Queue(maxsize=120)
        with SUB_LOCK:
            SUBSCRIBERS.add(q)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            with STATE_LOCK:
                cfg = self.shared.config()
            self.wfile.write(f"event: config\ndata: {json.dumps(cfg)}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=10)
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")   # keeps proxies/phones honest
                    self.wfile.flush()
                    continue
                self.wfile.write(msg.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with SUB_LOCK:
                SUBSCRIBERS.discard(q)


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#06080f">
<title>Doppler Scope</title>
<style>
  :root{
    --bg:#06080f; --panel:#0d1220; --line:#1d2740;
    --ink:#e8eefc; --ink-2:#9fb0d0; --ink-3:#61708f;
    --hot:#7fc8ff; --warn:#ffb347; --bad:#ff6b6b; --ok:#54d98c;
    --gap:12px;
  }
  *{box-sizing:border-box}
  html,body{margin:0;padding:0;background:var(--bg);color:var(--ink);
    font:15px/1.4 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
    -webkit-text-size-adjust:100%}
  body{padding:max(10px,env(safe-area-inset-top)) 10px max(10px,env(safe-area-inset-bottom))}
  .wrap{max-width:900px;margin:0 auto;display:flex;flex-direction:column;gap:var(--gap)}
  .bar{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
  .pill{font-size:12px;padding:3px 9px;border-radius:999px;border:1px solid var(--line);
    color:var(--ink-2);white-space:nowrap}
  .pill.on{color:var(--ok);border-color:#1e5c3a}
  .pill.off{color:var(--bad);border-color:#5c1e1e}
  h1{font-size:15px;font-weight:600;margin:0;letter-spacing:.02em}
  .readout{background:var(--panel);border:1px solid var(--line);border-radius:14px;
    padding:14px 16px;display:flex;align-items:baseline;gap:14px;flex-wrap:wrap}
  .vel{font-size:clamp(38px,13vw,64px);font-weight:650;line-height:1;
    font-variant-numeric:tabular-nums;letter-spacing:-.02em}
  .vel.idle{color:var(--ink-3)}
  .vel.live{color:var(--hot)}
  .unit{font-size:16px;color:var(--ink-2);font-weight:500}
  .sub{display:flex;gap:16px;flex-wrap:wrap;margin-left:auto;text-align:right}
  .sub div{font-variant-numeric:tabular-nums}
  .sub b{display:block;font-size:17px;font-weight:600}
  .sub span{font-size:11px;color:var(--ink-3);text-transform:uppercase;letter-spacing:.06em}
  .plot{background:var(--panel);border:1px solid var(--line);border-radius:14px;
    padding:10px;display:flex;flex-direction:column;gap:6px}
  canvas{width:100%;display:block;border-radius:8px;background:#05070f;
    image-rendering:pixelated;touch-action:pan-y}
  #spec{height:70px}
  #wf{height:min(52vh,420px)}
  .axis{position:relative;height:16px;color:var(--ink-3);font-size:11px;
    font-variant-numeric:tabular-nums}
  .axis span{position:absolute;transform:translateX(-50%);white-space:nowrap}
  .axis span:first-child{transform:none}
  .axis span:last-child{transform:translateX(-100%)}
  .legend{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--ink-3)}
  .legend canvas{height:8px;width:120px;border-radius:4px;flex:0 0 auto}
  .ctl{display:flex;gap:8px;flex-wrap:wrap}
  button{flex:1 1 auto;min-width:64px;min-height:44px;padding:0 12px;
    background:#121a2c;color:var(--ink);border:1px solid var(--line);border-radius:10px;
    font:inherit;font-size:14px;cursor:pointer;-webkit-tap-highlight-color:transparent}
  button:active{background:#1b2540}
  button[aria-pressed="true"]{background:#173355;border-color:#2b5c96;color:#cfe6ff}
  .foot{font-size:12px;color:var(--ink-3);display:flex;gap:14px;flex-wrap:wrap;
    font-variant-numeric:tabular-nums}
  .foot b{color:var(--ink-2);font-weight:600}
  .note{font-size:12px;color:var(--ink-3);line-height:1.45}
</style>
</head>
<body>
<div class="wrap">
  <div class="bar">
    <h1>Doppler Scope</h1>
    <span class="pill off" id="conn">connecting</span>
    <span class="pill" id="rate">--</span>
    <span class="pill" id="res">--</span>
  </div>

  <div class="readout">
    <div><span class="vel idle" id="vel">--</span> <span class="unit">m/s</span></div>
    <div class="sub">
      <div><b id="hz">--</b><span>Hz</span></div>
      <div><b id="amp">--</b><span>amp</span></div>
      <div><b id="ac">--</b><span>AC rms</span></div>
      <div><b id="vmaxseen">--</b><span>max m/s</span></div>
    </div>
  </div>

  <div class="plot">
    <canvas id="spec"></canvas>
    <canvas id="wf"></canvas>
    <div class="axis" id="axis"></div>
    <div class="legend">
      <span>weak</span><canvas id="ramp"></canvas><span>strong</span>
      <span style="margin-left:auto" id="scale">--</span>
    </div>
  </div>

  <div class="ctl">
    <button data-fmax="250">1.5 m/s</button>
    <button data-fmax="500">3 m/s</button>
    <button data-fmax="1000">6 m/s</button>
    <button data-fmax="2000">12 m/s</button>
    <button data-fmax="3000">19 m/s</button>
  </div>
  <div class="ctl">
    <button id="pause" aria-pressed="false">Pause</button>
    <button id="clear">Clear</button>
    <button id="hold" aria-pressed="true">Peak hold</button>
  </div>

  <div class="foot">
    <div>packets <b id="pk">0</b></div>
    <div>lost <b id="lo">0</b></div>
    <div>board drops <b id="dr">0</b></div>
    <div>ch <b id="ch">-</b></div>
  </div>
  <p class="note">
    Newest row at the bottom, same as the terminal scope. Nothing moving means an
    empty plot &mdash; a CW Doppler radar has no output at zero velocity. The
    magnitude scale is absolute, not auto-ranged, so a target visibly fades as it
    moves away.
  </p>
</div>

<script>
(() => {
  "use strict";
  // Sequential ramp: one blue hue family, lightness rising monotonically to
  // white. Not a rainbow -- brightness alone carries magnitude, so it stays
  // readable in greyscale and for any colour vision.
  const STOPS = [[0,[5,7,15]],[0.15,[10,25,60]],[0.35,[18,60,120]],
                 [0.55,[33,110,190]],[0.75,[90,175,235]],[0.9,[175,220,250]],[1,[255,255,255]]];
  const LUT = new Uint8Array(256*3);
  for (let i=0;i<256;i++){
    const t=i/255; let a=STOPS[0], b=STOPS[STOPS.length-1];
    for (let s=0;s<STOPS.length-1;s++){ if(t>=STOPS[s][0]&&t<=STOPS[s+1][0]){a=STOPS[s];b=STOPS[s+1];break;} }
    const f=(t-a[0])/Math.max(1e-6,(b[0]-a[0]));
    for (let c=0;c<3;c++) LUT[i*3+c]=Math.round(a[1][c]+(b[1][c]-a[1][c])*f);
  }

  const $ = id => document.getElementById(id);
  const wf = $("wf"), wfx = wf.getContext("2d", {alpha:false});
  const sp = $("spec"), spx = sp.getContext("2d", {alpha:false});
  let cfg = null, paused = false, hold = true, peak = null, vmaxSeen = 0;
  let rowImg = null;

  (function drawRamp(){
    const c=$("ramp"), x=c.getContext("2d"); c.width=256; c.height=8;
    const img=x.createImageData(256,1);
    for(let i=0;i<256;i++){img.data[i*4]=LUT[i*3];img.data[i*4+1]=LUT[i*3+1];
      img.data[i*4+2]=LUT[i*3+2];img.data[i*4+3]=255;}
    x.putImageData(img,0,0); x.drawImage(c,0,0,256,1,0,0,256,8);
  })();

  function sizeCanvases(){
    if(!cfg) return;
    const n = cfg.cols;
    wf.width = n; wf.height = 260;
    sp.width = n; sp.height = 64;
    wfx.fillStyle="#05070f"; wfx.fillRect(0,0,wf.width,wf.height);
    spx.fillStyle="#05070f"; spx.fillRect(0,0,sp.width,sp.height);
    rowImg = wfx.createImageData(n,1);
    peak = new Float32Array(n);
    // Axis ticks in m/s -- an affine relabel of frequency, 161 Hz per m/s.
    const ax = $("axis"); ax.innerHTML = "";
    const N = 5;
    for(let i=0;i<N;i++){
      const s=document.createElement("span");
      s.style.left = (i/(N-1)*100)+"%";
      s.textContent = (cfg.vmax*i/(N-1)).toFixed(1);
      ax.appendChild(s);
    }
    $("scale").textContent = cfg.floor+" to "+cfg.ceil+" dB (counts)";
    $("res").textContent = cfg.bin_ms.toFixed(2)+" m/s bins · "+cfg.row_ms.toFixed(0)+" ms";
  }

  function drawFrame(d){
    const raw = atob(d.c), n = raw.length;
    if(!cfg || n !== cfg.cols) return;

    // waterfall: shift down one pixel, newest row at the bottom
    wfx.drawImage(wf, 0, -1);
    const px = rowImg.data;
    for(let i=0;i<n;i++){
      const v = raw.charCodeAt(i);
      px[i*4]=LUT[v*3]; px[i*4+1]=LUT[v*3+1]; px[i*4+2]=LUT[v*3+2]; px[i*4+3]=255;
    }
    wfx.putImageData(rowImg, 0, wf.height-1);

    // instantaneous spectrum + decaying peak hold
    spx.fillStyle="#05070f"; spx.fillRect(0,0,sp.width,sp.height);
    const H = sp.height;
    for(let i=0;i<n;i++){
      const v = raw.charCodeAt(i)/255;
      if(hold) peak[i] = Math.max(peak[i]*0.985, v); else peak[i] = 0;
      const h = Math.round(v*H);
      if(h>0){
        const c = raw.charCodeAt(i);
        spx.fillStyle = "rgb("+LUT[c*3]+","+LUT[c*3+1]+","+LUT[c*3+2]+")";
        spx.fillRect(i, H-h, 1, h);
      }
      if(hold && peak[i]>0.02){
        spx.fillStyle="rgba(127,200,255,.55)";
        spx.fillRect(i, H-Math.round(peak[i]*H)-1, 1, 1);
      }
    }
  }

  function setStats(d){
    const live = d.r === 1;
    const vel = $("vel");
    vel.textContent = live ? d.ms.toFixed(2) : "--";
    vel.className = "vel " + (live ? "live" : "idle");
    $("hz").textContent  = live ? d.hz.toFixed(0) : "--";
    $("amp").textContent = d.amp.toFixed(1);
    $("ac").textContent  = d.ac.toFixed(2);
    if(live && d.ms > vmaxSeen){ vmaxSeen = d.ms; $("vmaxseen").textContent = vmaxSeen.toFixed(2); }
    $("pk").textContent = d.pk; $("lo").textContent = d.lo; $("dr").textContent = d.dr;
    $("lo").style.color = d.lo ? "var(--warn)" : "";
    $("dr").style.color = d.dr ? "var(--warn)" : "";
  }

  function connect(){
    const es = new EventSource("/stream");
    es.addEventListener("config", e => {
      cfg = JSON.parse(e.data);
      sizeCanvases();
      $("rate").textContent = cfg.fs.toLocaleString()+" sps";
      $("ch").textContent = cfg.channel;
      document.querySelectorAll("[data-fmax]").forEach(b =>
        b.setAttribute("aria-pressed", String(Math.abs(+b.dataset.fmax - cfg.fmax) < 1)));
    });
    es.addEventListener("status", e => {
      const s = JSON.parse(e.data);
      if(!s.connected) setConn(false);
    });
    es.addEventListener("frame", e => {
      setConn(true);
      if(paused) return;
      const d = JSON.parse(e.data);
      if(cfg && d.g !== cfg.gen) return;   // stale frame from before a range change
      drawFrame(d); setStats(d);
    });
    es.onerror = () => setConn(false);
  }
  function setConn(ok){
    const p = $("conn");
    p.textContent = ok ? "live" : "no data";
    p.className = "pill " + (ok ? "on" : "off");
  }

  document.querySelectorAll("[data-fmax]").forEach(b => b.onclick = () => {
    fetch("/api/set", {method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify({fmax:+b.dataset.fmax})});
  });
  $("pause").onclick = e => { paused=!paused;
    e.target.setAttribute("aria-pressed", String(paused));
    e.target.textContent = paused ? "Resume" : "Pause"; };
  $("clear").onclick = () => { if(!cfg) return;
    wfx.fillStyle="#05070f"; wfx.fillRect(0,0,wf.width,wf.height);
    peak.fill(0); vmaxSeen=0; $("vmaxseen").textContent="--"; };
  $("hold").onclick = e => { hold=!hold;
    e.target.setAttribute("aria-pressed", String(hold)); if(peak) peak.fill(0); };

  connect();
})();
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--port", type=int, default=5005, help="UDP port from the board")
    ap.add_argument("--udp-bind", default="0.0.0.0")
    ap.add_argument("--fmax", type=float, default=1000.0)
    ap.add_argument("--nfft", type=int, default=1024)
    ap.add_argument("--cols", type=int, default=240, help="spectrogram columns")
    ap.add_argument("--floor", type=float, default=-12.0)
    ap.add_argument("--ceil", type=float, default=48.0)
    args = ap.parse_args()

    try:
        import numpy  # noqa: F401
    except ImportError:
        print("adc_web needs numpy", file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    try:
        sock.bind((args.udp_bind, args.port))
    except OSError as e:
        print(f"cannot bind UDP {args.udp_bind}:{args.port}: {e}", file=sys.stderr)
        print("  Another receiver already holds the port.", file=sys.stderr)
        return 1
    sock.settimeout(1.0)

    sh = Shared(args)
    Handler.shared = sh
    threading.Thread(target=udp_thread, args=(sh, sock), daemon=True).start()

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    srv.daemon_threads = True

    ips = []
    for probe in ("192.168.0.1", "1.1.1.1"):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((probe, 1))
            ips.append(s.getsockname()[0])
            s.close()
        except OSError:
            pass
    seen = []
    for ip in ips:
        if ip not in seen:
            seen.append(ip)
    print(f"UDP  {args.udp_bind}:{args.port}", flush=True)
    print("open on your phone:")
    for ip in seen or ["<this machine>"]:
        print(f"    http://{ip}:{args.http_port}")
    print("Ctrl-C to stop", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
