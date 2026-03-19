# -*- coding: utf-8 -*-
"""
phone_log_server.py

Лёгкий HTTP-сервер для дебага с телефона:
- /               -> HTML страница (лог + live frame)
- /api/log?n=200  -> JSON tail лога
- /api/clear      -> POST очистить лог
- /api/frame.jpg  -> последний JPEG кадр (обновляй через phone_frame_update)

Безопасно: если не используешь — просто не импортируй/не стартуй сервер.
"""

from __future__ import annotations

import time
import json
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# OpenCV+numpy нужны ТОЛЬКО если будешь пушить кадры.
try:
    import numpy as np
    import cv2
except Exception:
    np = None
    cv2 = None


# ---------------------------
# Ring log
# ---------------------------
class _RingLog:
    def __init__(self, maxlen: int = 4000):
        self._q = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, level: str, msg: str, **kv):
        item = {
            "ts": time.time(),
            "level": str(level),
            "msg": str(msg),
        }
        if kv:
            item.update(kv)
        with self._lock:
            self._q.append(item)

    def tail(self, n: int = 200):
        with self._lock:
            return list(self._q)[-max(1, int(n)):]

    def clear(self):
        with self._lock:
            self._q.clear()


_LOG = _RingLog()


def phone_log(level: str, msg: str, **kv):
    """Вызывай вместо print()."""
    _LOG.add(level, msg, **kv)


# ---------------------------
# Frame buffer (JPEG)
# ---------------------------
_FRAME_LOCK = threading.Lock()
_FRAME_JPEG: bytes | None = None
_FRAME_TS: float = 0.0


def phone_frame_update(frame_bgr, quality: int = 70):
    """
    Обновить последний кадр для /api/frame.jpg.
    frame_bgr: np.ndarray HxWx3 BGR uint8
    """
    global _FRAME_JPEG, _FRAME_TS

    if cv2 is None or np is None:
        return  # нет зависимостей — молча ничего не делаем

    if frame_bgr is None:
        return
    if not isinstance(frame_bgr, np.ndarray):
        return
    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3 or frame_bgr.size == 0:
        return

    try:
        img = np.ascontiguousarray(frame_bgr)
        if img.dtype != np.uint8:
            img = img.astype(np.uint8, copy=False)

        q = int(quality)
        q = 30 if q < 30 else 95 if q > 95 else q
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        if not ok:
            return

        data = buf.tobytes()
        with _FRAME_LOCK:
            _FRAME_JPEG = data
            _FRAME_TS = time.time()
    except Exception:
        return


def _get_frame_jpeg():
    with _FRAME_LOCK:
        return _FRAME_JPEG, _FRAME_TS


# ---------------------------
# HTTP server
# ---------------------------
class _ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Robot debug</title>
  <style>
    body {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace;
      margin: 12px;
    }
    .wrap { display: grid; grid-template-columns: 1fr; gap: 10px; }
    .card { border: 1px solid #eee; border-radius: 10px; padding: 10px; }
    .bar { position: sticky; top: 0; background: #fff; padding: 8px 0; z-index: 10; }
    button { padding: 8px 10px; margin-right: 8px; }
    input { padding: 8px 10px; width: 90px; }
    .row { padding: 6px 8px; border-bottom: 1px solid #eee; white-space: pre-wrap; }
    .ts { opacity: 0.6; margin-right: 8px; }
    .LINFO { color:#111; }
    .LWARN { color:#b36b00; }
    .LERR  { color:#b00020; font-weight: 700; }
    img { width: 100%; height: auto; border-radius: 10px; border: 1px solid #ddd; }
    .muted { opacity: .65; }
  </style>
</head>
<body>
  <div class="bar">
    <button onclick="toggle()"><span id="btn">Pause</span></button>
    <button onclick="clr()">Clear</button>
    <span class="muted">Logs:</span>
    <input id="n" value="250" inputmode="numeric"/>
    <span id="stat" class="muted"></span>
  </div>

  <div class="wrap">
    <div class="card">
      <div style="display:flex; align-items:center; justify-content:space-between; gap:10px;">
        <div><b>Live frame</b> <span id="ft" class="muted"></span></div>
        <div class="muted">/api/frame.jpg</div>
      </div>
      <div style="margin-top:10px;">
        <img id="img" src="/api/frame.jpg" alt="frame"/>
      </div>
    </div>

    <div class="card">
      <div style="display:flex; align-items:center; justify-content:space-between; gap:10px;">
        <div><b>Log tail</b></div>
        <div class="muted">/api/log</div>
      </div>
      <div id="log" style="margin-top:10px;"></div>
    </div>
  </div>

<script>
let paused=false;
function toggle(){
  paused=!paused;
  document.getElementById('btn').textContent=paused?'Resume':'Pause';
}

async function clr(){
  await fetch('/api/clear', {method:'POST'});
  document.getElementById('log').innerHTML='';
}

function bustUrl(u){ return u + (u.includes('?')?'&':'?') + 't=' + Date.now(); }

async function tick(){
  if(paused) return;

  // frame refresh (no-cache)
  const img = document.getElementById('img');
  img.src = bustUrl('/api/frame.jpg');

  // log refresh
  let n = parseInt(document.getElementById('n').value || '250', 10);
  if(!Number.isFinite(n) || n<20) n=250;
  if(n>2000) n=2000;

  const r = await fetch('/api/log?n='+n);
  const j = await r.json();
  const el = document.getElementById('log');
  el.innerHTML = '';

  for(const it of j){
    const d = new Date(it.ts*1000);
    const ts = d.toLocaleTimeString();
    const lvl = (it.level||'INFO').toUpperCase();
    const cls = lvl==='ERROR'?'LERR':(lvl==='WARN'?'LWARN':'LINFO');
    const extra = Object.keys(it).filter(k=>!['ts','level','msg'].includes(k))
                    .map(k=>` ${k}=${it[k]}`).join('');
    const row = document.createElement('div');
    row.className='row '+cls;
    row.textContent = `[${ts}] ${lvl}: ${it.msg}${extra}`;
    el.appendChild(row);
  }

  document.getElementById('stat').textContent = `showing last ${j.length} / ${n}`;
  window.scrollTo(0, document.body.scrollHeight);
}

setInterval(tick, 500);
tick();
</script>
</body>
</html>
"""


class _Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        # --- logs ---
        if self.path.startswith("/api/log"):
            try:
                n = 200
                if "?" in self.path:
                    qs = self.path.split("?", 1)[1]
                    for p in qs.split("&"):
                        if p.startswith("n="):
                            n = int(p.split("=", 1)[1])
            except Exception:
                n = 200
            data = _LOG.tail(n=n)
            body = json.dumps(data).encode("utf-8")
            return self._send(200, body, "application/json; charset=utf-8")

        # --- frame ---
        if self.path.startswith("/api/frame.jpg"):
            jpg, ts = _get_frame_jpeg()
            if jpg is None:
                # маленькая заглушка (пустая), чтобы браузер не ругался
                return self._send(200, b"", "image/jpeg")
            # можно добавить заголовок с временем кадра
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("X-Frame-TS", str(ts))
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
            return

        # --- index ---
        if self.path == "/" or self.path.startswith("/index"):
            return self._send(200, _HTML.encode("utf-8"), "text/html; charset=utf-8")

        return self._send(404, b"not found", "text/plain; charset=utf-8")

    def do_POST(self):
        if self.path == "/api/clear":
            _LOG.clear()
            return self._send(200, b"ok", "text/plain; charset=utf-8")
        return self._send(404, b"not found", "text/plain; charset=utf-8")

    def log_message(self, *args, **kwargs):
        # глушим стандартный лог сервера
        return


def start_phone_log_server(host: str = "0.0.0.0", port: int = 8088):
    """
    host=0.0.0.0 -> доступно по сети, с телефона в той же Wi-Fi сети.
    Возвращает объект сервера (можно не использовать).
    """
    srv = _ThreadedHTTPServer((host, int(port)), _Handler)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    phone_log("INFO", "phone debug server started", host=host, port=int(port))
    return srv