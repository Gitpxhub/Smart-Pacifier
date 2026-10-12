"""
============================================================
 Arin Acoustic Dead Zone — Live Sensor Dashboard
 Reads JSON from ESP32 Master over Serial (USB)

 Features:
   • COM port selector with live online/offline status per port
   • Switch ports without restarting the script
   • Live waveform charts: airflow (Raw ADC / m/s / L/min) and mic (Peak / Average)
   • Dead zone / flat-line alerts on airflow and microphone
   • Scrollable raw serial log with colour coding
============================================================
 SETUP     pip install -r requirements.txt
 RUN       python dashboard.py
 BROWSER   http://127.0.0.1:8050
============================================================
"""

import collections
import json
import re
import threading
import time
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports

import dash
from dash import dcc, html, Input, Output, State, ctx
import plotly.graph_objs as go

# ── Config ────────────────────────────────────────────────────
BAUD_RATE        = 115200
BUFFER_SECONDS   = 30
SAMPLE_RATE_HZ   = 20
BUFFER_LEN       = BUFFER_SECONDS * SAMPLE_RATE_HZ
UPDATE_MS        = 150          # dashboard refresh
PORT_SCAN_MS     = 2000         # how often to re-scan COM ports
OFFLINE_TIMEOUT  = 3.0          # seconds since last packet → offline

FLAT_WINDOW_SEC  = 3
FLAT_WINDOW_N    = FLAT_WINDOW_SEC * SAMPLE_RATE_HZ
FLAT_THRESHOLD   = {"mic_peak": 5.0, "vel": 0.02}

NOZZLE_DIAMETER_M = 0.006
NOZZLE_AREA_M2    = np.pi * (NOZZLE_DIAMETER_M / 2) ** 2

def vel_to_lpm(v): return v * NOZZLE_AREA_M2 * 1e3 * 60

# ── Plain-text line parser ─────────────────────────────────────
# Handles:  Raw Flow : 420  Air Velocity : 0.03 m/s  Mic Peak : 331  Mic Average : 2164
_RE_RAW_FLOW  = re.compile(r"Raw Flow\s*:\s*([\d.]+)",      re.IGNORECASE)
_RE_AIR_VEL   = re.compile(r"Air Velocity\s*:\s*([\d.]+)",  re.IGNORECASE)
_RE_MIC_PEAK  = re.compile(r"Mic Peak\s*:\s*([\d.]+)",      re.IGNORECASE)
_RE_MIC_AVG   = re.compile(r"Mic Average\s*:\s*([\d.]+)",   re.IGNORECASE)

def parse_plaintext(line: str):
    """
    Try to extract sensor values from a plain-text master line.
    Returns a dict with any keys found, or None if nothing matched.
    """
    pkt = {}
    m = _RE_RAW_FLOW.search(line)
    if m: pkt["raw_flow"] = float(m.group(1))
    m = _RE_AIR_VEL.search(line)
    if m: pkt["vel"]      = float(m.group(1))
    m = _RE_MIC_PEAK.search(line)
    if m: pkt["mic_peak"] = float(m.group(1))
    m = _RE_MIC_AVG.search(line)
    if m: pkt["mic_avg"]  = float(m.group(1))
    return pkt if pkt else None

# ── Colour palette ────────────────────────────────────────────
C = {
    "bg":        "#0D0F14",
    "surface":   "#151820",
    "surface2":  "#1C2030",
    "border":    "#252A35",
    "text":      "#C8CDD8",
    "muted":     "#5A6070",
    "raw_flow":  "#4B9EF5",
    "vel":       "#4FD98A",
    "lpm":       "#36C9C6",
    "mic_peak":  "#F5C842",
    "mic_avg":   "#EF9F27",
    "alert":     "#E24B4A",
    "alert_bg":  "#2A1212",
    "ok":        "#1D9E75",
    "online":    "#1D9E75",
    "offline":   "#E24B4A",
    "unknown":   "#EF9F27",
}

# ── Shared state ──────────────────────────────────────────────
lock = threading.Lock()

def _fresh_bufs():
    return {k: collections.deque(maxlen=BUFFER_LEN)
            for k in ["ts", "mic_peak", "mic_avg", "vel", "lpm", "raw_flow"]}

buf          = _fresh_bufs()
latest       = {}
alerts       = []
serial_log   = collections.deque(maxlen=200)

# Port management
active_port     = None          # currently connected port string
active_ser      = None          # open serial.Serial object
reader_thread   = None
serial_ok       = False
error_msg       = ""
last_packet_ts  = 0.0           # time.time() of last good packet

# Port scan results  { "COM3": "online"|"offline"|"unknown", ... }
port_status     = {}

# ── Helpers ───────────────────────────────────────────────────
def _esp_label(p):
    desc = (p.description or "") + (p.manufacturer or "")
    return any(k in desc for k in ["CP210","CH340","UART","USB Serial","Silicon","FTDI"])

def scan_ports():
    """Return list of (device, description, is_esp_like) for all ports."""
    return [(p.device, p.description or p.device, _esp_label(p))
            for p in serial.tools.list_ports.comports()]

def probe_port(device):
    """Try to open a port briefly — returns True if the port is accessible."""
    try:
        s = serial.Serial(device, BAUD_RATE, timeout=0.1)
        s.close()
        return True
    except serial.SerialException:
        return False

# ── Serial reader ─────────────────────────────────────────────
def _reset_buffers():
    global buf, latest, alerts, serial_log, last_packet_ts
    buf         = _fresh_bufs()
    latest      = {}
    alerts      = []
    serial_log  = collections.deque(maxlen=200)
    last_packet_ts = 0.0

def serial_reader(port: str, stop_event: threading.Event):
    global serial_ok, error_msg, latest, last_packet_ts

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        with lock:
            serial_ok = True
            error_msg = ""
    except serial.SerialException as e:
        with lock:
            serial_ok = False
            error_msg = str(e)
        return

    while not stop_event.is_set():
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            ts_label = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with lock:
                serial_log.appendleft(f"[{ts_label}]  {raw}")
                last_packet_ts = time.time()

            # ── Try JSON first, fall back to plain-text ──
            pkt = None
            if raw.startswith("{"):
                try:
                    pkt = json.loads(raw)
                    if "error" in pkt or "status" in pkt:
                        with lock:
                            error_msg = pkt.get("error", pkt.get("status", ""))
                        continue
                except json.JSONDecodeError:
                    pkt = None

            if pkt is None:
                pkt = parse_plaintext(raw)

            if not pkt:
                continue   # line carries no sensor data — logged but not graphed

            lpm  = vel_to_lpm(pkt.get("vel", 0.0))
            ts_s = pkt.get("ts", time.time())   # plain-text has no ts; use wall clock

            with lock:
                buf["ts"].append(ts_s)
                buf["mic_peak"].append(pkt.get("mic_peak", 0))
                buf["mic_avg"].append(pkt.get("mic_avg",   0))
                buf["vel"].append(pkt.get("vel",           0))
                buf["lpm"].append(lpm)
                buf["raw_flow"].append(pkt.get("raw_flow", 0))
                latest = dict(pkt)
                latest["lpm"] = round(lpm, 4)

                alerts.clear()
                for key, thr in FLAT_THRESHOLD.items():
                    series = list(buf[key])
                    if len(series) >= FLAT_WINDOW_N:
                        if np.std(series[-FLAT_WINDOW_N:]) < thr:
                            label = {
                                "mic_peak": "Microphone (GY-MAX4466)",
                                "vel":      "Air velocity sensor",
                            }[key]
                            alerts.append(
                                f"⚠  {label} — no change for "
                                f"{FLAT_WINDOW_SEC}s (dead zone or sensor fault)"
                            )

        except (json.JSONDecodeError, KeyError):
            continue
        except serial.SerialException as e:
            with lock:
                serial_ok = False
                error_msg = str(e)
            time.sleep(1)

    try:
        ser.close()
    except Exception:
        pass

# ── Port switcher ─────────────────────────────────────────────
_stop_event = None

def connect_to_port(port: str):
    global active_port, reader_thread, serial_ok, error_msg, _stop_event

    # Stop existing reader
    if _stop_event:
        _stop_event.set()
    if reader_thread and reader_thread.is_alive():
        reader_thread.join(timeout=2)

    with lock:
        _reset_buffers()
        serial_ok  = False
        error_msg  = ""
        active_port = port

    _stop_event   = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(port, _stop_event), daemon=True
    )
    reader_thread.start()

# ── Background port scanner ───────────────────────────────────
def port_scanner():
    global port_status
    while True:
        found = scan_ports()
        status = {}
        for device, desc, is_esp in found:
            if device == active_port and serial_ok:
                status[device] = ("online", desc, is_esp)
            else:
                accessible = probe_port(device)
                status[device] = ("offline" if not accessible else "unknown", desc, is_esp)
        with lock:
            port_status = status
        time.sleep(PORT_SCAN_MS / 1000)

threading.Thread(target=port_scanner, daemon=True).start()

# ── Chart layout template ─────────────────────────────────────
CHART_LAYOUT = dict(
    paper_bgcolor="rgba(0,0,0,0)",
    plot_bgcolor="rgba(0,0,0,0)",
    font=dict(color=C["text"], family="JetBrains Mono, monospace", size=11),
    margin=dict(l=52, r=16, t=28, b=36),
    xaxis=dict(
        showgrid=True, gridcolor=C["border"], gridwidth=0.5,
        zeroline=False, color=C["muted"],
        title=dict(text="time (s)", font=dict(size=10, color=C["muted"])),
    ),
    yaxis=dict(
        showgrid=True, gridcolor=C["border"], gridwidth=0.5,
        zeroline=True, zerolinecolor=C["border"], zerolinewidth=1,
        color=C["muted"],
    ),
    legend=dict(
        bgcolor="rgba(0,0,0,0)", borderwidth=0,
        orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0,
        font=dict(size=10),
    ),
    hovermode="x unified",
)

# ── UI helpers ────────────────────────────────────────────────
BTN = {
    "fontFamily": "JetBrains Mono, monospace",
    "fontSize":   "11px",
    "cursor":     "pointer",
    "borderRadius": "4px",
    "padding":    "4px 14px",
}

def stat_card(label, value_id, unit, color):
    return html.Div([
        html.Div(label, style={
            "fontSize": "10px", "letterSpacing": "0.1em",
            "color": C["muted"], "textTransform": "uppercase",
            "marginBottom": "4px",
        }),
        html.Div([
            html.Span("—", id=value_id, style={
                "fontSize": "22px", "fontWeight": "600",
                "color": color, "fontFamily": "JetBrains Mono, monospace",
            }),
            html.Span(f" {unit}", style={
                "fontSize": "11px", "color": C["muted"], "marginLeft": "4px",
            }),
        ]),
    ], style={
        "background": C["surface"], "border": f"1px solid {C['border']}",
        "borderRadius": "8px", "padding": "14px 18px",
        "minWidth": "120px", "flex": "1",
    })

def chart_panel(title, graph_id):
    return html.Div([
        html.Div(title, style={
            "fontSize": "11px", "color": C["muted"],
            "letterSpacing": "0.08em", "textTransform": "uppercase",
            "marginBottom": "6px",
        }),
        dcc.Graph(id=graph_id, config={"displayModeBar": False},
                  style={"height": "180px"}),
    ], style={
        "background": C["surface"], "border": f"1px solid {C['border']}",
        "borderRadius": "8px", "padding": "14px 16px",
        "marginBottom": "12px",
    })

# ── App ───────────────────────────────────────────────────────
app = dash.Dash(__name__, title="Arin — Sensor Dashboard", update_title=None)

app.layout = html.Div([

    html.Link(rel="preconnect", href="https://fonts.googleapis.com"),
    html.Link(rel="stylesheet",
              href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600&display=swap"),

    # ── Header ──────────────────────────────────────────────
    html.Div([
        html.Div([
            html.Div("ARIN", style={
                "fontSize": "11px", "letterSpacing": "0.25em",
                "color": C["muted"], "fontWeight": "600",
            }),
            html.Div("Acoustic Dead Zone — Live Sensor Monitor", style={
                "fontSize": "17px", "fontWeight": "600",
                "color": C["text"], "marginTop": "2px",
            }),
        ]),
        html.Div([
            html.Div(id="conn-badge", style={"fontSize": "12px"}),
            html.Div(id="clock", style={
                "fontSize": "11px", "color": C["muted"],
                "fontFamily": "JetBrains Mono, monospace", "marginTop": "3px",
            }),
        ], style={"textAlign": "right"}),
    ], style={
        "display": "flex", "justifyContent": "space-between",
        "alignItems": "flex-start",
        "padding": "18px 24px 14px",
        "borderBottom": f"1px solid {C['border']}",
        "marginBottom": "16px",
    }),

    # ── COM Port Selector panel ──────────────────────────────
    html.Div([
        # Panel title
        html.Div([
            html.Span("COM PORT", style={
                "fontSize": "11px", "letterSpacing": "0.14em",
                "color": C["muted"], "fontWeight": "600",
            }),
            html.Span("  —  select the port your ESP32 master is connected to",
                      style={"fontSize": "10px", "color": C["muted"]}),
        ], style={"marginBottom": "12px"}),

        # Port list (rendered dynamically)
        html.Div(id="port-list", style={
            "display": "flex", "flexWrap": "wrap", "gap": "8px",
        }),

        # Refresh button
        html.Div([
            html.Button("⟳  Refresh ports", id="btn-refresh-ports", n_clicks=0,
                style={**BTN,
                    "background": "transparent",
                    "border": f"1px solid {C['border']}",
                    "color": C["muted"], "marginTop": "12px",
                }),
        ]),

    ], style={
        "background": C["surface"], "border": f"1px solid {C['border']}",
        "borderRadius": "8px", "padding": "16px 20px",
        "margin": "0 24px 16px",
    }),

    # Hidden store: which port the user last clicked
    dcc.Store(id="selected-port", data=None),

    # ── Alert banner ─────────────────────────────────────────
    html.Div(id="alert-banner", style={"padding": "0 24px 10px"}),

    # ── Stat cards ───────────────────────────────────────────
    html.Div([
        stat_card("Raw Flow",    "stat-raw",      "ADC",   C["raw_flow"]),
        stat_card("Air Velocity","stat-vel",       "m/s",   C["vel"]),
        stat_card("Flow (L/min)","stat-lpm",       "L/min", C["lpm"]),
        stat_card("Mic Peak",    "stat-mic-peak",  "ADC",   C["mic_peak"]),
        stat_card("Mic Average", "stat-mic-avg",   "ADC",   C["mic_avg"]),
    ], style={
        "display": "flex", "gap": "10px", "flexWrap": "wrap",
        "padding": "0 24px", "marginBottom": "18px",
    }),

    # ── Charts ───────────────────────────────────────────────
    html.Div([
        chart_panel("Airflow — raw ADC · velocity (m/s) · flow (L/min)", "chart-vel"),
        chart_panel("Microphone — peak & average amplitude (GY-MAX4466)",  "chart-mic"),
    ], style={
        "padding": "0 24px", "marginBottom": "16px",
    }),

    # ── Serial log ───────────────────────────────────────────
    html.Div([
        html.Div([
            html.Div([
                html.Span("▶  ", style={"color": C["ok"]}),
                html.Span("MASTER SERIAL LOG", style={
                    "fontSize": "11px", "letterSpacing": "0.12em",
                    "color": C["muted"], "fontWeight": "600",
                }),
                html.Span("  —  every line the ESP32 master prints",
                          style={"fontSize": "10px", "color": C["muted"]}),
            ]),
            html.Button("Clear", id="btn-clear-log", n_clicks=0, style={
                **BTN,
                "background": "transparent",
                "border": f"1px solid {C['border']}",
                "color": C["muted"],
            }),
        ], style={
            "display": "flex", "justifyContent": "space-between",
            "alignItems": "center", "marginBottom": "8px",
        }),
        html.Div(id="serial-log", style={
            "height": "200px", "overflowY": "auto",
            "background": "#09090D",
            "border": f"1px solid {C['border']}",
            "borderRadius": "6px", "padding": "10px 14px",
            "fontFamily": "JetBrains Mono, monospace",
            "fontSize": "11px", "lineHeight": "1.7",
            "color": C["text"], "whiteSpace": "pre",
        }),
    ], style={
        "background": C["surface"], "border": f"1px solid {C['border']}",
        "borderRadius": "8px", "padding": "16px 20px",
        "margin": "0 24px 28px",
    }),

    # Tickers
    dcc.Interval(id="tick",      interval=UPDATE_MS,   n_intervals=0),
    dcc.Interval(id="port-tick", interval=PORT_SCAN_MS, n_intervals=0),

], style={
    "background": C["bg"], "minHeight": "100vh",
    "fontFamily": "JetBrains Mono, monospace", "color": C["text"],
})

# ── Callback: port list rendering ─────────────────────────────
@app.callback(
    Output("port-list",      "children"),
    Output("selected-port",  "data"),
    Input("port-tick",       "n_intervals"),
    Input("btn-refresh-ports","n_clicks"),
    Input({"type": "port-btn", "index": dash.ALL}, "n_clicks"),
    State("selected-port",   "data"),
    prevent_initial_call=False,
)
def update_port_list(_, _refresh, port_clicks, current_port):
    triggered = ctx.triggered_id

    # If a port button was clicked, extract which port
    new_port = current_port
    if isinstance(triggered, dict) and triggered.get("type") == "port-btn":
        clicked_port = triggered["index"]
        if clicked_port != current_port:
            new_port = clicked_port
            connect_to_port(clicked_port)

    with lock:
        ps = dict(port_status)

    if not ps:
        # Nothing found yet — show placeholder
        return [html.Div("No COM ports found. Connect your ESP32 and click Refresh.",
                         style={"color": C["muted"], "fontSize": "12px"})], new_port

    cards = []
    for device, (status, desc, is_esp) in sorted(ps.items()):
        is_selected = (device == new_port)

        # Dot colour
        dot_color = {
            "online":  C["online"],
            "offline": C["offline"],
            "unknown": C["unknown"],
        }.get(status, C["muted"])

        # Status label
        status_label = {
            "online":  "ONLINE",
            "offline": "OFFLINE",
            "unknown": "AVAILABLE",
        }.get(status, status.upper())

        # Card border highlights active port
        border_color = C["ok"] if is_selected else C["border"]
        bg_color     = C["surface2"] if is_selected else C["surface"]

        # Short description — trim if too long
        short_desc = desc if len(desc) < 35 else desc[:32] + "…"

        cards.append(
            html.Div([
                # Top row: dot + status
                html.Div([
                    html.Span("●  ", style={
                        "color": dot_color, "fontSize": "13px",
                    }),
                    html.Span(status_label, style={
                        "fontSize": "10px", "letterSpacing": "0.1em",
                        "color": dot_color, "fontWeight": "600",
                    }),
                    html.Span("  ESP32" if is_esp else "", style={
                        "fontSize": "9px", "color": C["muted"],
                        "marginLeft": "6px",
                        "background": "#1C2030",
                        "padding": "1px 5px", "borderRadius": "3px",
                    }),
                ], style={"marginBottom": "6px"}),

                # Port name
                html.Div(device, style={
                    "fontSize": "15px", "fontWeight": "600",
                    "color": C["ok"] if is_selected else C["text"],
                    "marginBottom": "3px",
                }),

                # Description
                html.Div(short_desc, style={
                    "fontSize": "10px", "color": C["muted"],
                    "marginBottom": "10px",
                }),

                # Connect button
                html.Button(
                    "Connected ✓" if is_selected else "Connect",
                    id={"type": "port-btn", "index": device},
                    n_clicks=0,
                    disabled=is_selected,
                    style={
                        **BTN,
                        "background": C["ok"] if is_selected else "transparent",
                        "border": f"1px solid {C['ok'] if is_selected else C['border']}",
                        "color": "#fff" if is_selected else C["muted"],
                        "width": "100%",
                        "opacity": "1" if not is_selected else "0.7",
                    },
                ),
            ], style={
                "background": bg_color,
                "border": f"1px solid {border_color}",
                "borderRadius": "8px",
                "padding": "12px 14px",
                "minWidth": "180px",
                "maxWidth": "220px",
                "transition": "border-color 0.2s",
            })
        )

    return cards, new_port


# ── Callback: main dashboard refresh ─────────────────────────
@app.callback(
    Output("clock",        "children"),
    Output("conn-badge",   "children"),
    Output("alert-banner", "children"),
    Output("stat-raw",      "children"),
    Output("stat-vel",      "children"),
    Output("stat-lpm",      "children"),
    Output("stat-mic-peak", "children"),
    Output("stat-mic-avg",  "children"),
    Output("chart-vel",   "figure"),
    Output("chart-mic",   "figure"),
    Output("serial-log",  "children"),
    Input("tick",           "n_intervals"),
    Input("btn-clear-log",  "n_clicks"),
    State("selected-port",  "data"),
)
def refresh(_, _clear, sel_port):
    if ctx.triggered_id == "btn-clear-log":
        with lock:
            serial_log.clear()

    with lock:
        ts       = list(buf["ts"])
        mic_peak = list(buf["mic_peak"])
        mic_avg  = list(buf["mic_avg"])
        vel      = list(buf["vel"])
        lpm      = list(buf["lpm"])
        raw_flow = list(buf["raw_flow"])
        cur      = dict(latest)
        alrt     = list(alerts)
        log      = list(serial_log)
        ok       = serial_ok
        emsg     = error_msg
        lpt      = last_packet_ts

    clock_str = datetime.now().strftime("%H:%M:%S")

    # ── Connection badge ──
    if sel_port is None:
        badge = html.Span("○  No port selected",
                          style={"color": C["muted"], "fontSize": "12px"})
    elif ok and (time.time() - lpt) < OFFLINE_TIMEOUT:
        badge = html.Span(f"●  ONLINE  —  {sel_port}",
                          style={"color": C["online"], "fontSize": "12px"})
    elif emsg:
        badge = html.Span(f"✕  OFFLINE  —  {sel_port}  ({emsg})",
                          style={"color": C["offline"], "fontSize": "11px"})
    else:
        badge = html.Span(f"○  Connecting to {sel_port}…",
                          style={"color": C["unknown"], "fontSize": "12px"})

    # ── Alert banner ──
    banner = [
        html.Div(msg, style={
            "background": C["alert_bg"], "border": f"1px solid {C['alert']}",
            "borderRadius": "6px", "padding": "10px 16px",
            "color": C["alert"], "fontSize": "12px", "marginBottom": "6px",
        }) for msg in alrt
    ]

    # ── Stats ──
    def fmt(v, d=2): return f"{v:.{d}f}" if v is not None else "—"
    s_raw      = fmt(cur.get("raw_flow"),  0)
    s_vel      = fmt(cur.get("vel"),       3)
    s_lpm      = fmt(cur.get("lpm"),       3)
    s_mic_peak = fmt(cur.get("mic_peak"),  0)
    s_mic_avg  = fmt(cur.get("mic_avg"),   0)

    # ── Charts ──
    def line(x, y, name, color, dash="solid"):
        return go.Scatter(x=x, y=y, name=name, mode="lines",
                          line=dict(color=color, width=1.4, dash=dash))

    def fig(*traces, ytitle=""):
        f = go.Figure(list(traces))
        f.update_layout(**CHART_LAYOUT)
        f.update_layout(yaxis_title=ytitle)
        return f

    # Airflow chart: raw ADC + velocity (m/s) + L/min
    fig_vel = fig(
        line(ts, raw_flow, "Raw ADC", C["raw_flow"]),
        line(ts, vel,      "m/s",     C["vel"]),
        line(ts, lpm,      "L/min",   C["lpm"], dash="dot"),
        ytitle="ADC  |  m/s  |  L/min",
    )

    # Mic chart: peak and average on same axes
    fig_mic = fig(
        line(ts, mic_peak, "Mic Peak",    C["mic_peak"]),
        line(ts, mic_avg,  "Mic Average", C["mic_avg"], dash="dash"),
        ytitle="ADC amplitude",
    )

    # ── Log ──
    def log_color(line):
        if '"error"' in line:  return C["alert"]
        if '"status"' in line: return C["ok"]
        if "{" in line:        return C["muted"]
        return C["text"]

    log_children = (
        [html.Div(e, style={"color": log_color(e)}) for e in log]
        if log else
        [html.Div("Waiting for data…",
                  style={"color": C["muted"], "fontStyle": "italic"})]
    )

    return (
        clock_str, badge, banner,
        s_raw, s_vel, s_lpm, s_mic_peak, s_mic_avg,
        fig_vel, fig_mic,
        log_children,
    )


# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n  Dashboard →  http://127.0.0.1:8050\n")
    app.run(debug=False, host="127.0.0.1", port=8050)