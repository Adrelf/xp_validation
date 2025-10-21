#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
import traceback
from typing import List, Dict, Tuple, Optional

import numpy as np
import minimalmodbus
import serial
from flask import Flask, request, jsonify, render_template_string, Response

# ========= Logging =========
import logging
from logging.handlers import RotatingFileHandler

logger = logging.getLogger("pressgen")
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
fh = RotatingFileHandler("pressgen.log", maxBytes=512*1024, backupCount=3, encoding="utf-8")
fh.setLevel(logging.DEBUG)
fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
ch.setFormatter(fmt); fh.setFormatter(fmt)
if not logger.handlers:
    logger.addHandler(ch); logger.addHandler(fh)

# ========= Config =========
NPZ_PATH = os.environ.get("PRESSURE_DB", "database_pressure_generator.npz")
DB_VALUES_UNIT = os.environ.get("DB_VALUES_UNIT", "mmHg").lower()  # "mmhg" ou "pa"
STEP_SECONDS = 5.0
DRY_RUN = bool(int(os.environ.get("DRY_RUN", "0")))  # 1 = pas d'écriture Modbus, logs uniquement

REG_SETPOINT_FLOAT = 154
REG_OUTPUT_PERCENT = 138
CMD_START_OUTPUT = 16384
CMD_STOP_OUTPUT = 0

# ========= Modbus =========
def find_instrument(baud: int = 9600, addr: int = 255):
    ports = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2']
    for p in ports:
        try:
            inst = minimalmodbus.Instrument(p, addr)
            inst.serial.baudrate = baud
            inst.serial.bytesize = 8
            inst.serial.parity   = serial.PARITY_NONE
            inst.serial.stopbits = 1
            inst.mode            = minimalmodbus.MODE_RTU
            inst.debug           = False
            inst.byteorder = minimalmodbus.BYTEORDER_LITTLE_SWAP
            inst.clear_buffers_before_each_transaction = True
            inst.serial.timeout = 0.5
            logger.info(f"[Modbus] Instrument détecté sur {p}")
            return inst
        except Exception as e:
            logger.debug(f"[Modbus] Port {p} non valide: {e}")
            continue
    logger.warning("[Modbus] Aucun instrument détecté sur les ports connus.")
    return None

def to_pa(val: float) -> float:
    return float(val) * 133.322 if DB_VALUES_UNIT == "mmhg" else float(val)

# ========= DB loader (t_full / data_avg_full) =========
def _append_measure(out: List[Dict], idx: int, t, y):
    t = np.asarray(t, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    n = min(t.size, y.size)
    if n == 0:
        out.append({"id": idx, "t": np.array([], float), "data": np.array([], float),
                    "duration": 0.0, "n": 0})
        return
    t = t[:n]; y = y[:n]
    # remet t à 0
    t = t - t[0]
    out.append({"id": idx, "t": t, "data": y, "duration": float(t[-1]), "n": int(n)})

def load_database(npz_path: str) -> List[Dict]:
    if not os.path.exists(npz_path):
        raise FileNotFoundError(f"Base introuvable : {npz_path}")
    npz = np.load(npz_path, allow_pickle=True)
    if not ("t_full" in npz.files and "data_avg_full" in npz.files):
        raise ValueError("Structure NPZ attendue: 't_full' et 'data_avg_full' (dtype=object)")

    t_arr = npz["t_full"]
    y_arr = npz["data_avg_full"]
    measures: List[Dict] = []
    for i in range(len(t_arr)):
        _append_measure(measures, i+1, t_arr[i], y_arr[i])  # ids 1..100
    logger.info(f"[DB] Base chargée: {len(measures)} mesures depuis {npz_path}")
    return measures

# ========= Sequencer =========
class SequenceManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.running = False

        self.measure_id: Optional[int] = None
        self.t: Optional[np.ndarray] = None
        self.v: Optional[np.ndarray] = None  # valeurs telles que dans la base (mmHg ou Pa)
        self.current_idx = 0

        self.instrument: Optional[minimalmodbus.Instrument] = None
        self.last_error: Optional[str] = None

        # debug / status
        self.last_sent_unit_value: Optional[float] = None
        self.last_sent_pa: Optional[float] = None
        self.sent_count = 0

    def set_instrument(self, inst):
        with self.lock:
            self.instrument = inst

    def prepare(self, measure: Dict):
        with self.lock:
            self.t = measure["t"]
            self.v = measure["data"]
            self.current_idx = 0
            self.measure_id = measure["id"]
        logger.debug(f"[SEQ] Mesure {measure['id']} préparée: {len(self.t)} points, durée ~{measure['duration']:.1f}s")

    def start(self, measure: Dict, mode: str = "resume"):
        with self.lock:
            if self.running:
                raise RuntimeError("Une séquence est déjà en cours.")
            if self.instrument is None and not DRY_RUN:
                raise RuntimeError("Aucun instrument Modbus détecté.")
            if (self.measure_id != measure["id"]) or (self.t is None) or (self.v is None):
                self.prepare(measure)
            if mode not in ("resume","restart"):
                mode = "resume"
            if mode == "restart":
                self.current_idx = 0
            if self.v is None or self.v.size == 0:
                raise RuntimeError("Mesure vide.")
            self.stop_event.clear()
            self.running = True
            self.last_error = None
            self.sent_count = 0
            logger.info(f"[SEQ] START mesure={self.measure_id} mode={mode} points={self.v.size} port={getattr(self.instrument.serial,'port', 'DRY') if self.instrument else 'DRY'}")
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()

    def stop(self):
        with self.lock:
            if not self.running:
                logger.info("[SEQ] STOP demandé (aucune séquence en cours).")
                return
            logger.info("[SEQ] STOP demandé.")
            self.stop_event.set()

    def status(self) -> Dict:
        with self.lock:
            total = int(self.v.size) if self.v is not None else 0
            idx = int(self.current_idx)
            cur_val = float(self.v[idx]) if (self.v is not None and idx < total and total > 0) else None
            return {
                "running": self.running,
                "measure_id": self.measure_id,
                "index": idx,
                "total": total,
                "progress": (idx / total) if total > 0 else 0.0,
                "current_value_unit": DB_VALUES_UNIT,
                "current_value": cur_val,
                "last_error": self.last_error,
                "last_sent_unit_value": self.last_sent_unit_value,
                "last_sent_pa": self.last_sent_pa,
                "sent_count": self.sent_count
            }

    def _write_setpoint(self, pa_value: float):
        if DRY_RUN:
            logger.debug(f"[Modbus][DRY] write_float({REG_SETPOINT_FLOAT}, {pa_value:.2f})")
            return
        self.instrument.write_float(REG_SETPOINT_FLOAT, pa_value)

    def _write_output(self, value: int):
        if DRY_RUN:
            logger.debug(f"[Modbus][DRY] write_register({REG_OUTPUT_PERCENT}, {value})")
            return
        self.instrument.write_register(REG_OUTPUT_PERCENT, value, functioncode=6)

    def _safe_stop_output(self):
        try:
            self._write_output(CMD_STOP_OUTPUT)
        except Exception as e:
            logger.error(f"[SEQ] Erreur lors du STOP sortie: {e}")

    def _run_loop(self):
        try:
            while True:
                with self.lock:
                    if self.stop_event.is_set():
                        logger.debug("[SEQ] Stop event détecté.")
                        break
                    idx = self.current_idx
                    vals = self.v
                    N = vals.size if vals is not None else 0

                if (vals is None) or (idx >= N):
                    logger.info("[SEQ] Fin: plus de points à envoyer.")
                    break

                unit_val = float(vals[idx])
                pa_val = to_pa(unit_val)
                logger.debug(f"[SEQ] idx={idx}/{N} setpoint={unit_val:.3f} {DB_VALUES_UNIT} -> {pa_val:.2f} Pa")

                try:
                    self._write_setpoint(pa_val)
                    self._write_output(CMD_START_OUTPUT)
                except Exception as e:
                    self.last_error = f"{type(e).__name__}: {e}"
                    logger.error(f"[SEQ] Erreur écriture Modbus: {self.last_error}")
                    break

                with self.lock:
                    self.last_sent_unit_value = unit_val
                    self.last_sent_pa = pa_val
                    self.sent_count += 1

                # Maintien ~5 s avec stop réactif
                t0 = time.time()
                while (time.time() - t0) < STEP_SECONDS:
                    if self.stop_event.is_set():
                        logger.debug("[SEQ] Stop event pendant palier.")
                        break
                    time.sleep(0.1)

                with self.lock:
                    if not self.stop_event.is_set():
                        self.current_idx += 1

                if self.stop_event.is_set():
                    break

            self._safe_stop_output()

        except Exception as e:
            self.last_error = f"{type(e).__name__}: {e}"
            logger.error(f"[SEQ] Exception boucle: {self.last_error}\n{traceback.format_exc()}")

        finally:
            with self.lock:
                self.running = False
            logger.info("[SEQ] Thread terminé.")

# ========= Flask =========
app = Flask(__name__)
instrument = None
manager = SequenceManager()
MEASURES: List[Dict] = []
LOAD_ERROR: Optional[str] = None

HTML_PAGE = """
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8" />
  <title>Générateur de pression - Séquences NPZ</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: system-ui, Arial, sans-serif; margin: 20px; }
    .row { display: flex; gap: 16px; align-items: center; flex-wrap: wrap; }
    .box { border: 1px solid #ddd; border-radius: 8px; padding: 12px; }
    #chartBox { max-width: 980px; }
    button { padding: 8px 14px; }
    .danger { background: #b30000; color: white; }
    .ok { color: #0a0; } .warn { color: #c60; } .note { color:#555; }
    pre { max-height: 220px; overflow:auto; background:#fafafa; border:1px solid #eee; padding:8px; }
  </style>
</head>
<body>
  <h2>Génération de pression depuis la base NPZ</h2>

  <div class="row">
    <div class="box">
      <div><b>Fichier base</b> : <code id="dbPath"></code></div>
      <div id="dbStatus" class="note"></div>
      <div><b>Instrument</b> : <span id="instStatus">scan...</span></div>
    </div>
  </div>

  <div class="row" style="margin-top: 10px;">
    <div class="box">
      <label for="measureSelect"><b>Mesure :</b></label>
      <select id="measureSelect"></select>
      <span id="meta"></span>
    </div>

    <div class="box">
      <label><input type="radio" name="mode" value="resume" checked /> Reprendre</label>
      <label><input type="radio" name="mode" value="restart" /> Recommencer (t=0)</label>
      <button id="btnStart">Start</button>
      <button class="danger" id="btnStop">Stop mesure</button>
    </div>
  </div>

  <div id="chartBox" class="box" style="margin-top: 12px;">
    <canvas id="chart" height="90"></canvas>
  </div>

  <div class="box" style="margin-top: 12px;">
    <b>Statut</b> : <span id="statusLine" class="note"></span>
  </div>

  <div class="box" style="margin-top: 12px;">
    <div style="display:flex;align-items:center;gap:8px;">
      <b>Logs récents (server)</b>
      <button id="btnRefreshLogs">Rafraîchir</button>
    </div>
    <pre id="logBox"></pre>
  </div>

<script>
let chartRef = null;
let currentId = null;

function fmtMin(seconds){
  const m = (seconds/60.0);
  return "≈ " + m.toFixed(1) + " min";
}

async function loadInfo(){
  const r = await fetch('/db_info');
  const j = await r.json();
  document.getElementById('dbPath').textContent = j.db_path || "";
  if(!j.ok){
    document.getElementById('dbStatus').innerHTML = "<span class='warn'>"+ (j.error||"Erreur base") +"</span>";
  } else {
    document.getElementById('dbStatus').innerHTML = "<span class='ok'>Base chargée ("+j.n_measures+" mesures)</span> — Unité base : <b>"+j.db_unit+"</b> — Pas d'envoi : <b>"+j.step_s+" s</b>";
    const sel = document.getElementById('measureSelect');
    sel.innerHTML = "";
    j.measures.forEach(m => {
      const opt = document.createElement('option');
      opt.value = m.id;
      opt.textContent = "Mesure " + m.id + " ("+ m.n +" pts, " + fmtMin(m.duration) + ")";
      sel.appendChild(opt);
    });
    if(j.measures.length>0){
      currentId = j.measures[0].id;
      sel.value = currentId;
      updateMeta(j.measures[0]);
      loadAndPlot(currentId);
    }
  }
  const s = await (await fetch('/status')).json();
  document.getElementById('instStatus').textContent = s.instrument_found ? ("OK ("+s.instrument_port+")") : "Non détecté";
  refreshLogs();
}

function updateMeta(m){
  document.getElementById('meta').textContent = " — " + m.n + " points, durée " + fmtMin(m.duration);
}

async function loadAndPlot(measureId){
  const r = await fetch('/measurement/'+measureId);
  const j = await r.json();
  if(!j.ok){ alert("Erreur: "+j.error); return; }
  currentId = measureId;
  const t = j.t, y = j.data;

  const ctx = document.getElementById('chart').getContext('2d');
  const points = t.map((ti, i) => ({ x: ti, y: y[i] }));
    const data = {
    datasets: [{
        label: "data ("+j.unit+")",
        data: points,
        parsing: false,
        borderColor: "#0074D9",
        pointRadius: 0,
    }]
  };
  const options = {
    responsive: true,
    plugins: { legend: { display: true } },
    scales: {
        x: { title: { display: true, text: 't (s)' }, type: 'linear' },
        y: { title: { display: true, text: 'data ('+j.unit+')' } }
    }
  };

  if(chartRef){ chartRef.destroy(); }
  chartRef = new Chart(ctx, { type: 'line', data, options });
  console.log("[UI] Mesure", measureId, "chargée et tracée (", y.length, "points )");
}

document.getElementById('measureSelect').addEventListener('change', async (e)=>{
  const id = parseInt(e.target.value);
  const r = await fetch('/measurement_meta/'+id);
  const m = await r.json();
  if(m.ok){ updateMeta(m); }
  loadAndPlot(id);
});

document.getElementById('btnStart').addEventListener('click', async ()=>{
  if(currentId===null){ return; }
  const mode = document.querySelector('input[name="mode"]:checked').value;
  console.log("[UI] Start demandé id=", currentId, "mode=", mode);
  const r = await fetch('/start', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ id: currentId, mode })
  });
  const j = await r.json();
  if(!j.ok){ alert("Erreur Start: "+(j.error||"")); }
  refreshLogs();
});

document.getElementById('btnStop').addEventListener('click', async ()=>{
  console.log("[UI] Stop demandé");
  await fetch('/stop', {method:'POST'});
  refreshLogs();
});

async function tick(){
  const r = await fetch('/status');
  const s = await r.json();
  const p = Math.round((s.progress||0)*100);
  let txt = s.running ? ("Exécution : " + p + "% — index " + s.index + "/" + s.total) : "À l'arrêt";
  if(s.current_value!=null){
    txt += " — valeur courante: " + s.current_value.toFixed(2) + " " + s.current_value_unit;
  }
  if(s.last_sent_pa!=null){
    txt += " — dernier envoi: " + s.last_sent_pa.toFixed(1) + " Pa";
  }
  if(s.last_error){
    txt += " — erreur: " + s.last_error;
  }
  document.getElementById('statusLine').textContent = txt;
}

async function refreshLogs(){
  const r = await fetch('/logs');
  if(r.ok){
    const txt = await r.text();
    document.getElementById('logBox').textContent = txt;
  }
}

document.getElementById('btnRefreshLogs').addEventListener('click', refreshLogs);

setInterval(tick, 700);
setInterval(refreshLogs, 3000);
loadInfo();
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/db_info")
def db_info():
    if LOAD_ERROR:
        return jsonify({"ok": False, "error": LOAD_ERROR, "db_path": NPZ_PATH})
    return jsonify({
        "ok": True,
        "db_path": NPZ_PATH,
        "db_unit": "mmHg" if DB_VALUES_UNIT == "mmhg" else "Pa",
        "step_s": STEP_SECONDS,
        "n_measures": len(MEASURES),
        "measures": [{"id": m["id"], "n": m["n"], "duration": m["duration"]} for m in MEASURES]
    })

@app.route("/measurement/<int:mid>")
def measurement(mid: int):
    m = next((x for x in MEASURES if x["id"] == mid), None)
    if m is None:
        return jsonify({"ok": False, "error": f"Mesure {mid} introuvable"})
    # envoie la mesure brute (affichage instantané du graphe complet)
    return jsonify({
        "ok": True,
        "id": m["id"],
        "t": m["t"].tolist(),
        "data": m["data"].tolist(),
        "unit": "mmHg" if DB_VALUES_UNIT == "mmhg" else "Pa",
        "duration": m["duration"],
        "n": m["n"]
    })

@app.route("/measurement_meta/<int:mid>")
def measurement_meta(mid: int):
    m = next((x for x in MEASURES if x["id"] == mid), None)
    if m is None:
        return jsonify({"ok": False})
    return jsonify({"ok": True, "id": m["id"], "n": m["n"], "duration": m["duration"]})

@app.route("/start", methods=["POST"])
def start_route():
    payload = request.get_json(force=True, silent=True) or {}
    try:
        mid = int(payload.get("id"))
        mode = str(payload.get("mode", "resume")).lower()
    except Exception:
        logger.error("[HTTP] /start payload invalide: %s", payload)
        return jsonify({"ok": False, "error": "Requête invalide"}), 400

    measure = next((x for x in MEASURES if x["id"] == mid), None)
    if measure is None:
        logger.error(f"[HTTP] /start mesure {mid} introuvable")
        return jsonify({"ok": False, "error": f"Mesure {mid} introuvable"}), 404

    try:
        logger.info(f"[HTTP] /start id={mid} mode={mode}")
        manager.start(measure, mode=mode)
        return jsonify({"ok": True, "message": f"Démarrage mesure {mid} ({mode})"})
    except Exception as e:
        logger.error(f"[HTTP] /start erreur: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

@app.route("/stop", methods=["POST"])
def stop_route():
    logger.info("[HTTP] /stop")
    manager.stop()
    return jsonify({"ok": True, "message": "Stop demandé"})

@app.route("/status")
def status_route():
    st = manager.status()
    inst = manager.instrument
    st.update({
        "instrument_found": (inst is not None) or DRY_RUN,
        "instrument_port": getattr(inst.serial, "port", "DRY") if inst else ("DRY" if DRY_RUN else None)
    })
    return jsonify(st)

@app.route("/logs")
def logs_route():
    # renvoie la fin du fichier de logs + quelques lignes de la session console (si présentes)
    try:
        path = "pressgen.log"
        if not os.path.exists(path):
            return Response("Aucun log pour l'instant.\n", mimetype="text/plain")
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()[-400:]  # dernières lignes
        return Response("".join(lines), mimetype="text/plain")
    except Exception as e:
        return Response(f"Erreur lecture logs: {e}\n", mimetype="text/plain", status=500)

# ========= Bootstrap =========
def bootstrap():
    global instrument, MEASURES, LOAD_ERROR
    try:
        MEASURES = load_database(NPZ_PATH)
    except Exception as e:
        LOAD_ERROR = f"{type(e).__name__}: {e}"
        MEASURES = []
        logger.error(f"[DB] Erreur chargement: {LOAD_ERROR}")

    try:
        if not DRY_RUN:
            instrument = find_instrument()
            if instrument:
                manager.set_instrument(instrument)
                logger.info(f"[BOOT] Instrument prêt sur {instrument.serial.port}")
            else:
                logger.warning("[BOOT] Instrument non détecté (Start échouera).")
        else:
            logger.warning("[BOOT] DRY_RUN=1 : aucune écriture Modbus, logs uniquement.")
    except Exception as e:
        logger.error(f"[BOOT] Erreur instrument: {e}")

if __name__ == "__main__":
    bootstrap()
    app.run(host="0.0.0.0", port=5100, debug=False)
