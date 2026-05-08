/* ═══════════════════════════════════════════════════════════════════════
   МКС-3.0  //  MOTOR FLEET COMMAND SYSTEM  —  APPLICATION LOGIC
   Covers every endpoint in the FastAPI backend v3.0
═══════════════════════════════════════════════════════════════════════ */

"use strict";

// ── GLOBAL STATE ─────────────────────────────────────────────────────
const S = {
  motors:           {},   // { id: status }
  selectedMotor:    null,
  telMotor:         null,
  ctrlMotor:        null,
  faultMotor:       null,
  faultMode:        "single",   // "single" | "fleet"
  cfgMotor:         null,
  histMotor:        null,
  espMotor:         null,
  pollTimer:        null,
  configBounds:     {},
  catalogueLoaded:  false,
};

// ── API ───────────────────────────────────────────────────────────────
function base() {
  return document.getElementById("base-url").value.replace(/\/$/, "");
}

async function api(method, path, body = null) {
  const opts = {
    method,
    headers: { "Content-Type": "application/json" },
  };
  if (body) opts.body = JSON.stringify(body);
  const res = await fetch(base() + path, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw { status: res.status, detail: data.detail || "Unknown error", data };
  return data;
}

// ── CLOCK ─────────────────────────────────────────────────────────────
function startClock() {
  function tick() {
    const now = new Date();
    document.getElementById("clock-time").textContent =
      now.toTimeString().slice(0, 8);
    document.getElementById("clock-date").textContent =
      now.toLocaleDateString("en-GB", { day: "2-digit", month: "short", year: "numeric" }).toUpperCase();
  }
  tick();
  setInterval(tick, 1000);
}

// ── TOAST ─────────────────────────────────────────────────────────────
function toast(msg, type = "info") {
  const area = document.getElementById("toast-area");
  const el = document.createElement("div");
  el.className = `toast toast-${type}`;
  el.textContent = msg;
  area.appendChild(el);
  setTimeout(() => el.remove(), 3200);
}

// ── CONFIRM MODAL ────────────────────────────────────────────────────
function confirm(title, body) {
  return new Promise(resolve => {
    document.getElementById("confirm-title").textContent = title;
    document.getElementById("confirm-body").textContent = body;
    const modal = document.getElementById("confirm-modal");
    modal.classList.remove("hidden");
    const yes = document.getElementById("confirm-yes");
    const no  = document.getElementById("confirm-no");
    const cleanup = (val) => {
      modal.classList.add("hidden");
      yes.onclick = null; no.onclick = null;
      resolve(val);
    };
    yes.onclick = () => cleanup(true);
    no.onclick  = () => cleanup(false);
  });
}

// ── FLEET MODAL ───────────────────────────────────────────────────────
function showFleetModal(title, data) {
  document.getElementById("modal-title").textContent = title;
  const body = document.getElementById("modal-body");

  const m = data.motors || {};
  const ids = Object.keys(m);

  let html = `<div style="color:var(--text-dim);font-size:11px;margin-bottom:10px">
    Total: <b style="color:var(--text)">${data.total ?? ids.length}</b>
    &nbsp;|&nbsp; Succeeded: <b style="color:var(--green)">${data.succeeded ?? "—"}</b>
    &nbsp;|&nbsp; Skipped: <b style="color:var(--amber)">${data.skipped ?? "—"}</b>
    &nbsp;|&nbsp; Failed: <b style="color:var(--red)">${data.failed ?? "—"}</b>
    &nbsp;|&nbsp; ${data.timestamp ?? ""}
  </div>`;

  if (ids.length) {
    html += `<table class="fleet-result-table">
      <thead><tr><th>MOTOR ID</th><th>STATUS</th><th>DETAIL</th></tr></thead><tbody>`;
    for (const [id, r] of Object.entries(m)) {
      const cls = r.status === "success" ? "ok" : r.status === "skipped" ? "skip" : "err";
      const detail = r.message || r.reason || r.error || (r.new_state ? `→ ${r.new_state}` : "");
      html += `<tr><td>${id}</td><td class="${cls}">${r.status.toUpperCase()}</td><td>${detail}</td></tr>`;
    }
    html += "</tbody></table>";
  } else {
    html += `<pre style="color:var(--green);font-size:11px">${JSON.stringify(data, null, 2)}</pre>`;
  }

  body.innerHTML = html;
  document.getElementById("fleet-modal").classList.remove("hidden");
}

document.getElementById("modal-close").onclick = () =>
  document.getElementById("fleet-modal").classList.add("hidden");

// ── TAB SWITCHING ────────────────────────────────────────────────────
document.querySelectorAll(".tab").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach(t => t.classList.remove("active"));
    document.querySelectorAll(".tabpane").forEach(p => p.classList.add("hidden"));
    btn.classList.add("active");
    const pane = document.getElementById(`tab-${btn.dataset.tab}`);
    if (pane) {
      pane.classList.remove("hidden");
      if (btn.dataset.tab === "catalogue" && !S.catalogueLoaded) loadCatalogue();
    }
  });
});

// ── HEALTH ORBS ──────────────────────────────────────────────────────
function setOrb(id, state) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.remove("ok", "warn", "err");
  el.classList.add(state);
}

async function checkHealth() {
  try {
    const h = await api("GET", "/health");
    setOrb("horb-api", "ok");
    setOrb("horb-cache", h.cache && h.cache !== "not initialised" ? "ok" : "warn");
    setOrb("horb-db", h.database === "connected" ? "ok" : "warn");
    return h;
  } catch {
    setOrb("horb-api", "err");
    setOrb("horb-cache", "err");
    setOrb("horb-db", "err");
  }
}

document.getElementById("btn-health").onclick = async () => {
  const h = await checkHealth();
  if (h) toast(`API healthy | cache: ${h.cache} | db: ${h.database}`, "success");
  else toast("API unreachable", "error");
};

// ── FLEET STATUS POLL ─────────────────────────────────────────────────
async function pollFleetStatus() {
  try {
    const data = await api("GET", "/fleet/status");
    const motors = data.motors || data;
    S.motors = motors;
    updateRoster(motors);
    updateHeaderStats(motors);
    syncSelects(Object.keys(motors));
    if (S.telMotor && motors[S.telMotor]) updateTelemetry(motors[S.telMotor]);
    if (S.ctrlMotor && motors[S.ctrlMotor]) updateCtrlStrip(motors[S.ctrlMotor]);
  } catch (e) {
    setOrb("horb-api", "err");
  }
}

async function refreshAll() {
  await checkHealth();
  await pollFleetStatus();
  // also refresh config bounds silently
  try {
    const b = await api("GET", "/config/bounds");
    S.configBounds = b.bounds || b;
  } catch {}
}

document.getElementById("btn-refresh").onclick = () => { refreshAll(); toast("Refreshed", "info"); };

// poll interval
function applyPollInterval() {
  if (S.pollTimer) clearInterval(S.pollTimer);
  const v = parseInt(document.getElementById("poll-interval").value);
  if (v > 0) S.pollTimer = setInterval(pollFleetStatus, v);
}
document.getElementById("poll-interval").onchange = applyPollInterval;

// ── HEADER STATS ──────────────────────────────────────────────────────
function updateHeaderStats(motors) {
  const values = Object.values(motors);
  const total   = values.length;
  const running = values.filter(m => m.state === "RUNNING" || m.running).length;
  const fault   = values.filter(m => m.state === "FAULT").length;
  const idle    = values.filter(m => m.state === "IDLE").length;
  const faults  = values.reduce((s, m) => s + (m.faults ? m.faults.length : 0), 0);
  const connected = values.filter(m => m.connection_status === "CONNECTED").length;

  document.getElementById("fsh-total").textContent   = total;
  document.getElementById("fsh-running").textContent = running;
  document.getElementById("fsh-fault").textContent   = fault;
  document.getElementById("fsh-idle").textContent    = idle;
  document.getElementById("cbar-faults").textContent    = faults;
  document.getElementById("cbar-connected").textContent = connected;
  document.getElementById("fleet-count") &&
    (document.getElementById("roster-count").textContent = `${total}`);
}

// ── ROSTER ────────────────────────────────────────────────────────────
function updateRoster(motors) {
  const roster = document.getElementById("motor-roster");
  if (!Object.keys(motors).length) {
    roster.innerHTML = `<div class="empty-roster">NO MOTORS REGISTERED<br><small>ADD A MOTOR ABOVE</small></div>`;
    return;
  }
  roster.innerHTML = "";
  for (const [id, m] of Object.entries(motors)) {
    const faultCount = m.faults ? m.faults.length : 0;
    const el = document.createElement("div");
    el.className = "roster-item" + (S.selectedMotor === id ? " active" : "");
    el.innerHTML = `
      <div class="ri-dot ${m.state}"></div>
      <div class="ri-id">${id}</div>
      <div class="ri-state">${m.state}</div>
      ${faultCount ? `<div class="ri-fault-count">⚡${faultCount}</div>` : ""}`;
    el.onclick = () => selectMotor(id);
    roster.appendChild(el);
  }
  document.getElementById("roster-count").textContent = Object.keys(motors).length;
}

// ── SELECT MOTOR (sync all selects) ───────────────────────────────────
function syncSelects(ids) {
  // Map each select to the S.* variable that tracks its current motor selection.
  // We restore from S.* not sel.value so that a poll rebuild never drops the
  // user's selection while they are mid-interaction (Bugs 6 & 9 fix).
  const selectMap = {
    "tel-motor-select":   S.telMotor,
    "ctrl-motor-select":  S.ctrlMotor,
    "fault-motor-select": S.faultMotor,
    "cfg-motor-select":   S.cfgMotor,
    "hist-motor-select":  S.histMotor,
    "esp-motor-select":   S.espMotor,
  };
  Object.entries(selectMap).forEach(([selId, currentVal]) => {
    const sel = document.getElementById(selId);
    if (!sel) return;
    sel.innerHTML = `<option value="">— SELECT —</option>`;
    ids.forEach(id => {
      const opt = document.createElement("option");
      opt.value = id; opt.textContent = id;
      if (id === currentVal) opt.selected = true;
      sel.appendChild(opt);
    });
  });
  document.getElementById("fw-apidocs-link").textContent = base() + "/docs";
}

function selectMotor(id) {
  S.selectedMotor = id;
  document.querySelectorAll(".roster-item").forEach(el => el.classList.remove("active"));
  document.querySelectorAll(".roster-item").forEach(el => {
    if (el.querySelector(".ri-id")?.textContent === id) el.classList.add("active");
  });
  ["tel-motor-select","ctrl-motor-select","fault-motor-select",
   "cfg-motor-select","hist-motor-select","esp-motor-select"].forEach(selId => {
    const sel = document.getElementById(selId);
    if (sel && [...sel.options].some(o => o.value === id)) sel.value = id;
  });
  setTelMotor(id);
  setCtrlMotor(id);
}

// ── TELEMETRY ─────────────────────────────────────────────────────────
document.getElementById("tel-motor-select").onchange = function() {
  setTelMotor(this.value);
};

function setTelMotor(id) {
  S.telMotor = id;
  const noSel = document.getElementById("tel-no-motor");
  const content = document.getElementById("tel-content");
  if (!id) { noSel.classList.remove("hidden"); content.classList.add("hidden"); return; }
  noSel.classList.add("hidden"); content.classList.remove("hidden");
  if (S.motors[id]) updateTelemetry(S.motors[id]);
}

function updateTelemetry(m) {
  // State banner
  const stateName = document.getElementById("tel-state-name");
  stateName.textContent = m.state;
  stateName.className = `state-${m.state}`;
  document.getElementById("tel-motor-name").textContent = m.name || S.telMotor;
  document.getElementById("tel-powered").textContent = m.powered_on ? "POWERED ON" : "UNPOWERED";
  document.getElementById("tel-grace").textContent =
    m.startup_grace_active ? "GRACE WINDOW ACTIVE" : "GRACE EXPIRED";

  const hs = m.health_score ?? 100;
  document.getElementById("tel-health-fill").style.width = hs + "%";
  document.getElementById("tel-health-fill").style.background =
    hs > 70 ? "var(--green)" : hs > 40 ? "var(--amber)" : "var(--red)";
  document.getElementById("tel-health-val").textContent = hs.toFixed(1);

  // Fault strip
  const strip = document.getElementById("tel-fault-strip");
  strip.innerHTML = "";
  if (m.faults && m.faults.length) {
    m.faults.forEach(f => {
      const pill = document.createElement("div");
      const sev = faultSeverity(f);
      pill.className = `fault-pill fault-${sev}`;
      pill.textContent = f;
      strip.appendChild(pill);
    });
  }

  // Gauges
  setGauge("voltage", m.voltage_v, 20, m.voltage_v < 10.5 ? "crit" : m.voltage_v < 11.5 ? "warn" : "ok");
  setGauge("current", m.current_a, 6.5, m.current_a > 5.5 ? "crit" : m.current_a > 4.5 ? "warn" : "ok");
  setGauge("speed", m.speed_rpm, 3500, "ok");
  setGauge("temp", m.temperature_c, 80,
    m.temperature_c > 70 ? "crit" : m.temperature_c > 55 ? "warn" : "ok");
  setGauge("eff", m.efficiency_pct, 100,
    m.efficiency_pct > 0 && m.efficiency_pct < 25 ? "warn" : "ok");
  setGauge("health", hs, 100,
    hs > 70 ? "ok" : hs > 40 ? "warn" : "crit");

  // Detail tables — electrical
  set("td-bemf",       fmt(m.back_emf_v, 2));
  set("td-bemf-pred",  fmt(m.back_emf_predicted_v, 2));
  set("td-ke",         fmt(m.ke_live, 5));
  set("td-ke-dev",     fmt(m.ke_deviation_pct, 2));
  set("td-km",         fmt(m.km_nameplate, 4));
  set("td-reff",       fmt(m.R_eff_ohm, 4));
  set("td-iavg",       fmt(m.current_avg_a, 2));
  set("td-ripple",     fmt(m.current_ripple_pct, 1));
  // thermal
  set("td-shell",      fmt(m.temperature_c, 1));
  set("td-winding",    fmt(m.winding_temp_c, 1));
  set("td-rise",       fmt(m.temp_rise_rate_c_s, 3));
  // mechanical
  set("td-omega",      fmt(m.omega_rad_s, 3));
  set("td-torque",     fmt(m.torque_nm, 4));
  set("td-torque-load",fmt(m.torque_load_nm, 4));
  set("td-speed-pct",  fmt(m.speed_pct_of_max, 1));
  set("td-load-pct",   fmt(m.load_pct, 1));
  // power
  set("td-pin",        fmt(m.power_input_w, 2));
  set("td-pmech",      fmt(m.power_mechanical_w, 2));
  set("td-pcopper",    fmt(m.power_copper_w, 2));
  set("td-pfriction",  fmt(m.power_friction_w, 3));
  set("td-ploss",      fmt(m.power_loss_w, 2));
  set("td-eff2",       fmt(m.efficiency_pct, 1));
  // state
  set("td-state",      m.state);
  set("td-powered",    m.powered_on ? "YES" : "NO");
  set("td-running",    m.running ? "YES" : "NO");
  set("td-grace",      m.startup_grace_active ? "ACTIVE" : "EXPIRED");
  set("td-connstatus", m.connection_status || "—");
  set("td-lastupdate", m.timestamp ? new Date(m.timestamp * 1000).toLocaleTimeString() : "—");

  // Fault counts
  const fc = m.fault_counts || {};
  const fcGrid = document.getElementById("tel-fault-counts");
  fcGrid.innerHTML = Object.entries(fc).map(([name, count]) =>
    `<div class="fc-item">
       <span class="fc-name">${name.replace("_"," ")}</span>
       <span class="fc-count ${count > 0 ? "active" : ""}">${count}</span>
     </div>`).join("");
}

function setGauge(name, val, max, level) {
  const pct = Math.min(100, (val / max) * 100);
  const card = document.getElementById(`gc-${name}`);
  if (card) {
    card.classList.remove("gauge-ok","gauge-warn","gauge-crit");
    card.classList.add(`gauge-${level}`);
  }
  const vEl = document.getElementById(`gv-${name}`);
  if (vEl) vEl.textContent = fmt(val, name === "speed" ? 0 : 1);
  const bar = document.getElementById(`gb-${name}`);
  if (bar) bar.style.width = pct + "%";
}

// ── CONTROL TAB ───────────────────────────────────────────────────────
document.getElementById("ctrl-motor-select").onchange = function() {
  setCtrlMotor(this.value);
};

function setCtrlMotor(id) {
  S.ctrlMotor = id;
  const noSel = document.getElementById("ctrl-no-motor");
  const content = document.getElementById("ctrl-content");
  if (!id) { noSel.classList.remove("hidden"); content.classList.add("hidden"); return; }
  noSel.classList.add("hidden"); content.classList.remove("hidden");
  if (S.motors[id]) updateCtrlStrip(S.motors[id]);
}

function updateCtrlStrip(m) {
  const badge = document.getElementById("ctrl-state-badge");
  badge.textContent = m.state;
  badge.className = `state-${m.state}`;
  document.getElementById("ctrl-motor-label").textContent =
    `${S.ctrlMotor} | HEALTH: ${m.health_score ?? "—"}`;
  const fl = document.getElementById("ctrl-fault-list");
  fl.innerHTML = (m.faults || []).map(f => {
    const sev = faultSeverity(f);
    return `<span class="fault-pill fault-${sev}" style="font-size:9px">${f}</span>`;
  }).join("");
}

// Motor control buttons
async function motorCmd(endpoint, method = "POST") {
  const id = S.ctrlMotor;
  if (!id) return toast("No motor selected", "warn");
  try {
    const res = await api(method, endpoint.replace("{id}", id));
    toast(`${id}: ${res.message || res.status || "OK"}`, "success");
    await pollFleetStatus();
    if (S.motors[id]) {
      updateCtrlStrip(S.motors[id]);
      if (S.telMotor === id) updateTelemetry(S.motors[id]);
    }
  } catch (e) {
    toast(`${id}: ${e.detail || "Error"}`, "error");
  }
}

document.getElementById("cbtn-power-on").onclick  = () => motorCmd("/motor/{id}/power_on");
document.getElementById("cbtn-start").onclick     = () => motorCmd("/motor/{id}/start");
document.getElementById("cbtn-stop").onclick      = () => motorCmd("/motor/{id}/stop");
document.getElementById("cbtn-power-off").onclick = () => motorCmd("/motor/{id}/power_off");
document.getElementById("cbtn-emergency").onclick = async () => {
  const ok = await confirm("EMERGENCY STOP", `Emergency stop motor ${S.ctrlMotor}? This sets state=FAULT and requires power cycle.`);
  if (ok) motorCmd("/motor/{id}/emergency");
};
document.getElementById("cbtn-delete").onclick = async () => {
  const ok = await confirm("UNREGISTER MOTOR", `Unregister ${S.ctrlMotor}? This removes it from the fleet.`);
  if (ok) {
    try {
      await api("DELETE", `/motor/${S.ctrlMotor}`);
      toast(`${S.ctrlMotor} unregistered`, "success");
      S.ctrlMotor = null;
      await refreshAll();
    } catch (e) { toast(e.detail, "error"); }
  }
};

// Sensor inject
document.querySelectorAll(".preset-btn").forEach(btn => {
  btn.onclick = () => {
    document.getElementById("si-voltage").value = btn.dataset.v;
    document.getElementById("si-current").value = btn.dataset.i;
    document.getElementById("si-speed").value   = btn.dataset.n;
    document.getElementById("si-temp").value    = btn.dataset.t;
  };
});

document.getElementById("btn-inject").onclick = async () => {
  const id = S.ctrlMotor;
  if (!id) return toast("No motor selected", "warn");
  const body = {
    motor_id:    id,
    voltage:     parseFloat(document.getElementById("si-voltage").value),
    current:     parseFloat(document.getElementById("si-current").value),
    speed:       parseInt(document.getElementById("si-speed").value),
    temperature: parseFloat(document.getElementById("si-temp").value),
  };
  try {
    const res = await api("POST", "/update", body);
    document.getElementById("inject-result").textContent = `✓ ${res.message}`;
    toast("Sensor data injected", "success");
    await pollFleetStatus();
  } catch (e) {
    document.getElementById("inject-result").textContent = `✕ ${e.detail}`;
    toast(`Inject failed: ${e.detail}`, "error");
  }
};

// ── FLEET OPERATIONS ─────────────────────────────────────────────────
document.querySelectorAll("[data-fleet]").forEach(btn => {
  btn.onclick = async () => {
    const action = btn.dataset.fleet;
    const isEmg = action === "emergency";
    if (isEmg) {
      const ok = await confirm("FLEET EMERGENCY STOP",
        "Emergency stop ALL motors in the fleet? All motors will enter FAULT state. Power cycle required to restart.");
      if (!ok) return;
    }
    try {
      const res = await api("POST", `/fleet/${action}`);
      const label = action.toUpperCase().replace("_", " ");
      showFleetModal(`FLEET ${label} — RESULT`, res);
      toast(`Fleet ${label}: ${res.succeeded}/${res.total} OK`, isEmg ? "warn" : "success");
      await pollFleetStatus();
    } catch (e) {
      toast(`Fleet ${action} failed: ${e.detail}`, "error");
    }
  };
});

// ── REGISTER MOTOR ────────────────────────────────────────────────────
document.getElementById("btn-register").onclick = async () => {
  const input = document.getElementById("new-motor-id");
  const id = input.value.trim().toUpperCase();
  if (!id) return toast("Enter a motor ID", "warn");
  try {
    const res = await api("POST", "/motor/create", { motor_id: id });
    toast(res.message, "success");
    input.value = "";
    await refreshAll();
  } catch (e) {
    toast(`Register failed: ${e.detail}`, "error");
  }
};
document.getElementById("new-motor-id").addEventListener("keydown", e => {
  if (e.key === "Enter") document.getElementById("btn-register").click();
});

// ── FAULT DIAGNOSIS TAB ───────────────────────────────────────────────
document.getElementById("btn-fault-single").onclick = () => {
  S.faultMode = "single";
  document.getElementById("btn-fault-single").classList.add("active");
  document.getElementById("btn-fault-fleet").classList.remove("active");
  document.getElementById("fault-motor-select").style.display = "";
  loadFaultDiagnosis();
};
document.getElementById("btn-fault-fleet").onclick = () => {
  S.faultMode = "fleet";
  document.getElementById("btn-fault-fleet").classList.add("active");
  document.getElementById("btn-fault-single").classList.remove("active");
  document.getElementById("fault-motor-select").style.display = "none";
  loadFaultDiagnosis();
};
document.getElementById("fault-motor-select").onchange = function() {
  S.faultMotor = this.value;
  loadFaultDiagnosis();
};
document.getElementById("btn-fault-refresh").onclick = loadFaultDiagnosis;

async function loadFaultDiagnosis() {
  const noSel  = document.getElementById("fault-no-sel");
  const content = document.getElementById("fault-content");

  if (S.faultMode === "fleet") {
    noSel.classList.add("hidden"); content.classList.remove("hidden");
    try {
      const data = await api("GET", "/fleet/faults");
      renderFleetFaults(data);
    } catch (e) { toast("Failed to load fleet faults: " + e.detail, "error"); }
  } else {
    const id = S.faultMotor;
    if (!id) { noSel.classList.remove("hidden"); content.classList.add("hidden"); return; }
    noSel.classList.add("hidden"); content.classList.remove("hidden");
    try {
      const data = await api("GET", `/motor/${id}/faults`);
      renderMotorFaults(data);
    } catch (e) { toast("Failed to load faults: " + e.detail, "error"); }
  }
}

function renderMotorFaults(data) {
  const d = data.diagnosis || data;
  document.getElementById("fsb-total").textContent    = d.active_fault_count ?? (d.faults?.length ?? 0);
  document.getElementById("fsb-severity").textContent = d.highest_severity || "NONE";
  document.getElementById("fsb-safe").textContent     = d.safe_to_run === false ? "⚠ NO" : "✓ YES";
  document.getElementById("fsb-action").textContent   = d.immediate_action || "";

  const lr = d.live_readings || {};
  set("flr-v",   fmt(lr.voltage_v ?? lr.voltage, 2));
  set("flr-i",   fmt(lr.current_a ?? lr.current, 2));
  set("flr-n",   lr.speed_rpm ?? lr.speed ?? "—");
  set("flr-t",   fmt(lr.shell_temp_c ?? lr.temperature, 1));
  set("flr-tw",  fmt(lr.winding_temp_c ?? lr.winding_temp, 1));
  set("flr-eff", fmt(lr.efficiency_pct ?? lr.efficiency, 1));

  const cards = document.getElementById("fault-cards-area");
  const faults = d.faults || [];
  if (!faults.length) {
    cards.innerHTML = `<div class="no-sel-msg" style="padding:30px">✓ NO ACTIVE FAULTS — MOTOR HEALTHY</div>`;
    return;
  }
  cards.innerHTML = faults.map(f => renderFaultCard(f)).join("");
}

function renderFleetFaults(data) {
  const motors = data.motors || {};
  document.getElementById("fsb-total").textContent    = data.total_active_faults ?? 0;
  document.getElementById("fsb-severity").textContent = data.fleet_worst_severity || "NONE";
  document.getElementById("fsb-safe").textContent     = data.fleet_safe_to_run === false ? "⚠ NO" : "✓ YES";
  document.getElementById("fsb-action").textContent   = "FLEET-WIDE DIAGNOSIS";

  set("flr-v","—"); set("flr-i","—"); set("flr-n","—");
  set("flr-t","—"); set("flr-tw","—"); set("flr-eff","—");

  const cards = document.getElementById("fault-cards-area");
  if (!Object.keys(motors).length) {
    cards.innerHTML = `<div class="no-sel-msg">NO MOTORS REGISTERED</div>`;
    return;
  }
  cards.innerHTML = Object.entries(motors).map(([id, diag]) => {
    const d = diag.diagnosis || diag;
    const faults = d.faults || [];
    if (!faults.length) {
      return `<div class="fault-card" style="border-left-color:var(--green)">
        <div class="fc-hdr"><span class="fc-name" style="color:var(--green)">✓ ${id}</span><span style="font-size:10px;color:var(--text-mute)">NO FAULTS</span></div></div>`;
    }
    return `<div style="margin-bottom:8px">
      <div style="font-family:var(--font-hdr);font-size:11px;color:var(--amber);padding:6px 10px;background:var(--bg3);border:1px solid var(--border);border-bottom:none;letter-spacing:2px">▸ MOTOR: ${id}</div>
      ${faults.map(f => renderFaultCard(f)).join("")}
    </div>`;
  }).join("");
}

function renderFaultCard(f) {
  const sev = f.severity || "WARNING";
  return `<div class="fault-card sev-${sev}">
    <div class="fc-hdr">
      <span class="fc-name" style="color:${sevColor(sev)}">${f.fault_type || f.name || "FAULT"}</span>
      <span class="fc-sev-badge">${sev}</span>
      <span class="fc-cat">${f.category || ""}</span>
      ${f.auto_stop ? `<span class="fc-autostop">AUTO-STOP</span>` : ""}
    </div>
    <div class="fc-body">
      <div class="fc-field full"><label>DESCRIPTION</label><p>${f.description || "—"}</p></div>
      <div class="fc-field"><label>CAUSE</label><p>${f.cause || "—"}</p></div>
      <div class="fc-field"><label>EFFECT</label><p>${f.effect || "—"}</p></div>
      <div class="fc-field full"><label>COULD ALSO BE</label><p>${f.could_also_be || f.could_also_be || "—"}</p></div>
    </div>
    <div class="fc-action">${f.action || f.recommended_action || "—"}</div>
  </div>`;
}

// ── CONFIGURATION TAB ─────────────────────────────────────────────────
document.getElementById("cfg-motor-select").onchange = function() {
  S.cfgMotor = this.value;
  // Auto-load config when motor is selected — fixes the "select then click Load"
  // confusion where the panel stays empty after selecting (Bug 7 fix).
  if (this.value) loadMotorConfig();
  else {
    document.getElementById("cfg-no-motor").classList.remove("hidden");
    document.getElementById("cfg-content").classList.add("hidden");
  }
};
document.getElementById("btn-cfg-load").onclick = loadMotorConfig;
document.getElementById("btn-cfg-reset").onclick = async () => {
  if (!S.cfgMotor) return toast("Select a motor first", "warn");
  const ok = await confirm("RESET CONFIGURATION",
    `Reset all thresholds for ${S.cfgMotor} to engineering defaults?`);
  if (!ok) return;
  try {
    const res = await api("DELETE", `/motor/${S.cfgMotor}/config`);
    toast(res.message || "Reset to defaults", "success");
    loadMotorConfig();
  } catch (e) { toast("Reset failed: " + e.detail, "error"); }
};

async function loadMotorConfig() {
  const id = S.cfgMotor;
  if (!id) return toast("Select a motor first", "warn");
  try {
    const data = await api("GET", `/motor/${id}/config`);
    const cfg = data.config || data;
    document.getElementById("cfg-no-motor").classList.add("hidden");
    document.getElementById("cfg-content").classList.remove("hidden");
    renderConstants(cfg.constants || {});
    renderThresholds(cfg.thresholds || {});
  } catch (e) { toast("Load config failed: " + e.detail, "error"); }
}

function renderConstants(consts) {
  const grid = document.getElementById("cfg-constants-grid");
  const labels = {
    R: "Winding Resistance [Ω]", L: "Armature Inductance [H]",
    Km: "Torque/Back-EMF Constant [N·m/A]", alpha_cu: "Copper Temp Coeff [/°C]",
    b: "Viscous Damping [N·m·s/rad]", J: "Rotor Inertia [kg·m²]",
    N_max: "No-Load Max Speed [RPM]", I_nom: "Nominal Current [A]",
    I_max: "Max Current [A]", Rth_ws: "Winding→Shell Thermal Res [°C/W]",
    Rth_sa: "Shell→Ambient Thermal Res [°C/W]", Cw: "Winding Thermal Cap [J/°C]",
    Cs: "Shell Thermal Cap [J/°C]",
  };
  grid.innerHTML = Object.entries(consts).map(([k, v]) =>
    `<div class="const-item">
       <div class="const-key">${k}</div>
       <div class="const-val">${typeof v === "number" ? v.toPrecision(4) : v}</div>
       <div class="const-note">${labels[k] || ""}</div>
     </div>`).join("");
}

function renderThresholds(thresholds) {
  const grid = document.getElementById("cfg-thresholds-grid");
  let hasModified = false;
  grid.innerHTML = Object.entries(thresholds).map(([key, info]) => {
    const val       = info.value ?? info;
    const isDefault = info.modified === false;
    const modified  = info.modified === true;
    const readOnly  = info.read_only === true;
    const bounds    = info.bounds || S.configBounds[key];
    if (modified) hasModified = true;
    if (readOnly) return `
      <div class="threshold-item">
        <div class="th-hdr"><span class="th-key">${key}</span>
          <span style="font-size:9px;color:var(--text-mute);background:var(--bg4);padding:1px 5px">READ-ONLY</span>
        </div>
        <div class="th-input-row">
          <input class="th-input" disabled value="${val}">
        </div>
      </div>`;
    return `
      <div class="threshold-item ${modified ? "modified" : ""}">
        <div class="th-hdr">
          <span class="th-key">${key}</span>
          ${modified ? `<span class="th-modified-badge">MODIFIED</span>` : ""}
        </div>
        <div class="th-input-row">
          <input class="th-input" id="th-${key}" type="number"
            value="${val}"
            ${bounds ? `min="${bounds.min}" max="${bounds.max}"` : ""}
            step="${key.includes("ms") || key.includes("persist") ? 1 : 0.01}">
        </div>
        ${bounds ? `<div class="th-range">BOUNDS: <span>${bounds.min} → ${bounds.max}</span></div>` : ""}
        ${info.default !== undefined ? `<div class="th-range">DEFAULT: <span>${info.default}</span></div>` : ""}
      </div>`;
  }).join("");

  const badge = document.getElementById("cfg-modified-badge");
  if (hasModified) badge.classList.remove("hidden");
  else badge.classList.add("hidden");
}

document.getElementById("btn-cfg-apply").onclick = async () => {
  const id = S.cfgMotor;
  if (!id) return toast("No motor selected", "warn");
  const payload = {};
  document.querySelectorAll(".th-input:not(:disabled)").forEach(inp => {
    const key = inp.id.replace("th-", "");
    const val = parseFloat(inp.value);
    if (!isNaN(val)) payload[key] = val;
  });
  if (!Object.keys(payload).length) return toast("No changes to apply", "warn");
  try {
    const res = await api("PATCH", `/motor/${id}/config`, payload);
    const applied  = Object.keys(res.applied || {});
    const rejected = Object.keys(res.rejected || {});
    let msg = "";
    if (applied.length)  msg += `✓ Applied: ${applied.join(", ")}. `;
    if (rejected.length) msg += `✕ Rejected: ${rejected.join(", ")}.`;
    document.getElementById("cfg-apply-result").innerHTML =
      `<span style="color:var(--green)">${msg}</span>`;
    toast(`Config applied (${applied.length} fields)`, "success");
    if (rejected.length) toast(`Rejected: ${rejected.length} fields`, "warn");
    loadMotorConfig();
  } catch (e) { toast("Apply failed: " + e.detail, "error"); }
};

// ── HISTORY TAB ───────────────────────────────────────────────────────
document.getElementById("hist-motor-select").onchange = function() {
  S.histMotor = this.value;
};
document.getElementById("btn-hist-load").onclick = loadHistory;

document.querySelectorAll(".htab").forEach(btn => {
  btn.onclick = () => {
    document.querySelectorAll(".htab").forEach(t => t.classList.remove("active"));
    document.querySelectorAll(".htabpane").forEach(p => p.classList.add("hidden"));
    btn.classList.add("active");
    const pane = document.getElementById(`htab-${btn.dataset.htab}`);
    if (pane) pane.classList.remove("hidden");
  };
});

async function loadHistory() {
  const id    = document.getElementById("hist-motor-select").value;
  const hours = document.getElementById("hist-hours").value;
  if (!id) return toast("Select a motor first", "warn");
  S.histMotor = id;
  document.getElementById("hist-no-motor").classList.add("hidden");
  document.getElementById("hist-content").classList.remove("hidden");
  try {
    const [sensors, faults, commands] = await Promise.all([
      api("GET", `/motor/${id}/history?hours=${hours}`),
      api("GET", `/motor/${id}/faults/history`),
      api("GET", `/motor/${id}/commands/history`),
    ]);
    renderSensorHistory(sensors);
    renderFaultHistory(faults);
    renderCommandHistory(commands);
  } catch (e) {
    toast("History load failed: " + e.detail + " (DATABASE_URL required)", "warn");
  }
}

function renderSensorHistory(data) {
  const rows = data.readings || [];
  document.getElementById("hist-sensor-meta").textContent =
    `${data.count ?? rows.length} readings — last ${data.hours ?? "?"} hour(s) — motor: ${data.motor_id ?? ""}`;
  const tbody = document.getElementById("hist-sensor-body");
  if (!rows.length) {
    tbody.innerHTML = `<tr><td colspan="9" style="color:var(--text-mute);text-align:center;padding:20px">No data — DATABASE_URL required on backend</td></tr>`;
    return;
  }
  tbody.innerHTML = rows.map(r => `<tr>
    <td>${r.timestamp?.replace("T", " ").slice(0, 19)}</td>
    <td class="mono">${fmt(r.voltage, 2)}</td>
    <td class="mono">${fmt(r.current, 2)}</td>
    <td class="mono">${r.speed ?? "—"}</td>
    <td class="mono">${fmt(r.temperature, 1)}</td>
    <td class="mono">${fmt(r.efficiency, 1)}</td>
    <td class="mono">${fmt(r.power_input, 2)}</td>
    <td class="mono">${fmt(r.power_mechanical, 2)}</td>
    <td class="mono">${fmt(r.health_score, 1)}</td>
  </tr>`).join("");
}

function renderFaultHistory(data) {
  const rows = data.faults || [];
  const tbody = document.getElementById("hist-fault-body");
  if (!rows.length) {
    tbody.innerHTML = `<tr><td colspan="4" style="color:var(--text-mute);text-align:center;padding:20px">No fault history</td></tr>`;
    return;
  }
  tbody.innerHTML = rows.map(r => `<tr>
    <td>${r.occurred_at?.replace("T", " ").slice(0, 19)}</td>
    <td style="color:${sevColor(r.severity)}">${r.fault_type}</td>
    <td class="sev-cell-${r.severity}">${r.severity}</td>
    <td style="font-size:10px;color:var(--text-dim)">${JSON.stringify(r.details || {})}</td>
  </tr>`).join("");
}

function renderCommandHistory(data) {
  const rows = data.commands || [];
  const tbody = document.getElementById("hist-cmd-body");
  if (!rows.length) {
    tbody.innerHTML = `<tr><td colspan="3" style="color:var(--text-mute);text-align:center;padding:20px">No command history</td></tr>`;
    return;
  }
  tbody.innerHTML = rows.map(r => `<tr>
    <td>${r.queued_at?.replace("T", " ").slice(0, 19)}</td>
    <td style="color:var(--cyan)">${r.command}</td>
    <td style="color:${r.status === "EXECUTED" ? "var(--green)" : r.status === "FAILED" ? "var(--red)" : "var(--text-dim)"}">${r.status}</td>
  </tr>`).join("");
}

// ── FAULT CATALOGUE ───────────────────────────────────────────────────
async function loadCatalogue() {
  S.catalogueLoaded = true;
  document.getElementById("catalogue-loading").classList.remove("hidden");
  try {
    const data = await api("GET", "/fault/catalogue");
    const grid = document.getElementById("catalogue-grid");
    const cat  = data.catalogue || data;
    grid.innerHTML = Object.entries(cat).map(([name, f]) => {
      const sev = f.severity || "WARNING";
      return `<div class="cat-card cat-${sev}">
        <div class="cat-hdr">
          <span class="cat-name">${name}</span>
          <span class="fc-sev-badge" style="background:${sevBg(sev)};border-color:${sevColor(sev)};color:${sevColor(sev)}">${sev}</span>
          <span class="fc-cat">${f.category || ""}</span>
          ${f.auto_stop
            ? `<span class="cat-autostop">AUTO-STOP</span>`
            : `<span class="cat-nostp">ALERT ONLY</span>`}
        </div>
        <div class="cat-body">
          <div class="cat-field full"><label>DESCRIPTION</label><p>${f.description}</p></div>
          <div class="cat-field"><label>CAUSE</label><p>${f.cause}</p></div>
          <div class="cat-field"><label>EFFECT</label><p>${f.effect}</p></div>
          <div class="cat-field full"><label>COULD ALSO BE</label><p>${f.could_also_be}</p></div>
        </div>
        <div class="cat-action">${f.action}</div>
      </div>`;
    }).join("");
    document.getElementById("catalogue-loading").classList.add("hidden");
    grid.classList.remove("hidden");
    toast(`Loaded ${data.count} fault types from catalogue`, "success");
  } catch (e) {
    document.getElementById("catalogue-loading").textContent = "Failed to load catalogue";
    toast("Catalogue load failed: " + e.detail, "error");
  }
}

// ── ESP32 TAB ─────────────────────────────────────────────────────────
document.getElementById("esp-motor-select").onchange = function() {
  S.espMotor = this.value;
};

document.getElementById("btn-poll-cmd").onclick = async () => {
  const id = document.getElementById("esp-motor-select").value;
  if (!id) return toast("Select a motor first", "warn");
  try {
    const data = await api("GET", `/motor/${id}/command`);
    document.getElementById("esp-cmd-result").textContent = JSON.stringify(data, null, 2);
    toast("Command polled", "info");
  } catch (e) {
    document.getElementById("esp-cmd-result").textContent = `Error: ${e.detail}`;
    toast("Poll failed: " + e.detail, "error");
  }
};

document.getElementById("btn-fleet-conns").onclick = async () => {
  try {
    const data = await api("GET", "/fleet/connections");
    const grid = document.getElementById("fleet-conn-grid");
    const conns = data.connections || {};
    if (!Object.keys(conns).length) {
      grid.innerHTML = `<div style="color:var(--text-mute);font-size:11px">No motors registered</div>`;
      return;
    }
    grid.innerHTML = Object.entries(conns).map(([id, status]) =>
      `<div class="conn-chip conn-${status}">
         <div class="dot"></div>
         <span>${id}</span>
         <span class="conn-label">${status.replace("_", " ")}</span>
       </div>`).join("");
    toast("Connections refreshed", "info");
  } catch (e) { toast("Failed: " + e.detail, "error"); }
};

document.getElementById("btn-stats").onclick = async () => {
  try {
    const data = await api("GET", "/stats");
    document.getElementById("stats-result").textContent = JSON.stringify(data, null, 2);
    toast("Stats loaded", "info");
  } catch (e) {
    document.getElementById("stats-result").textContent = `Error: ${e.detail}`;
    toast("Stats failed: " + e.detail, "error");
  }
};

document.getElementById("btn-esp-health").onclick = async () => {
  try {
    const data = await api("GET", "/health");
    document.getElementById("health-result").textContent = JSON.stringify(data, null, 2);
    toast("Health OK", "success");
  } catch (e) {
    document.getElementById("health-result").textContent = `Error: ${e.detail || e}`;
    toast("Health check failed", "error");
  }
};

// ── UTILITIES ─────────────────────────────────────────────────────────
function fmt(val, dec = 2) {
  if (val === undefined || val === null) return "—";
  const n = parseFloat(val);
  if (isNaN(n)) return "—";
  return n.toFixed(dec);
}

function set(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val ?? "—";
}

function faultSeverity(faultName) {
  const critical = ["STALL", "THERMAL_RUNAWAY", "WINDING_SHORT"];
  const severe   = ["OVERHEAT", "HIGH_CURRENT", "VOLTAGE_DROP"];
  if (critical.includes(faultName)) return "CRITICAL";
  if (severe.includes(faultName))   return "SEVERE";
  return "WARNING";
}

function sevColor(sev) {
  if (sev === "CRITICAL") return "var(--red)";
  if (sev === "SEVERE")   return "var(--amber)";
  return "var(--cyan)";
}

function sevBg(sev) {
  if (sev === "CRITICAL") return "var(--red-dim)";
  if (sev === "SEVERE")   return "var(--amber-dim)";
  return "var(--cyan-dim)";
}

// ── INIT ──────────────────────────────────────────────────────────────
async function init() {
  startClock();
  await refreshAll();
  applyPollInterval();
}

init();
