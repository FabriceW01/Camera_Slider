const state = {
  config: null,
  status: null,
  pollHandle: null,
  jogPadActive: false,
};

const els = {
  statusConnection: document.getElementById("statusConnection"),
  statusState: document.getElementById("statusState"),
  statusMode: document.getElementById("statusMode"),
  statusHomed: document.getElementById("statusHomed"),
  statusPosition: document.getElementById("statusPosition"),
  statusLimits: document.getElementById("statusLimits"),
  statusLastError: document.getElementById("statusLastError"),
  stopButton: document.getElementById("stopButton"),
  homeButton: document.getElementById("homeButton"),
  jogSpeed: document.getElementById("jogSpeed"),
  jogSpeedValue: document.getElementById("jogSpeedValue"),
  jogLeftButton: document.getElementById("jogLeftButton"),
  jogRightButton: document.getElementById("jogRightButton"),
  jogPad: document.getElementById("jogPad"),
  jogPadIndicator: document.getElementById("jogPadIndicator"),
  setAButton: document.getElementById("setAButton"),
  setBButton: document.getElementById("setBButton"),
  goAButton: document.getElementById("goAButton"),
  goBButton: document.getElementById("goBButton"),
  abRepeatMode: document.getElementById("abRepeatMode"),
  runAbButton: document.getElementById("runAbButton"),
  keyframes: document.getElementById("keyframes"),
  keyframeRowTemplate: document.getElementById("keyframeRowTemplate"),
  addKeyframeButton: document.getElementById("addKeyframeButton"),
  loadDemoKeyframesButton: document.getElementById("loadDemoKeyframesButton"),
  saveProgramButton: document.getElementById("saveProgramButton"),
  runProgramButton: document.getElementById("runProgramButton"),
  pauseProgramButton: document.getElementById("pauseProgramButton"),
  resumeProgramButton: document.getElementById("resumeProgramButton"),
  stopProgramButton: document.getElementById("stopProgramButton"),
  programRepeatMode: document.getElementById("programRepeatMode"),
  settingsForm: document.getElementById("settingsForm"),
  apSettings: document.getElementById("apSettings"),
  debugOutput: document.getElementById("debugOutput"),
};

function currentJogSpeed() {
  return Number(els.jogSpeed.value || 0);
}

function stepsToMm(steps) {
  if (!state.config?.mechanics?.steps_per_mm) {
    return 0;
  }
  return steps / state.config.mechanics.steps_per_mm;
}

async function apiRequest(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok || payload.ok === false) {
    throw new Error(payload?.error?.message || `Request failed for ${path}`);
  }
  return payload.data;
}

async function postJson(path, body = {}) {
  return apiRequest(path, {
    method: "POST",
    body: JSON.stringify(body),
  });
}

function updateJogSpeedLabel() {
  els.jogSpeedValue.textContent = `${currentJogSpeed()} steps/s`;
}

function setStatusConnection(text, ok) {
  els.statusConnection.textContent = text;
  els.statusConnection.style.color = ok ? "#145a64" : "#a63636";
}

function renderStatus() {
  if (!state.status) {
    return;
  }

  const status = state.status;
  const ab = status.ab || {};
  els.statusState.textContent = status.state;
  els.statusMode.textContent = status.active_mode;
  els.statusHomed.textContent = status.homed ? "Yes" : "No";
  els.statusPosition.textContent = `${status.position_mm?.toFixed?.(2) ?? "0.00"} mm (${status.position_steps} steps)`;
  els.statusLimits.textContent = `L:${status.limits.left_pressed ? "PRESSED" : "OK"} / R:${status.limits.right_pressed ? "PRESSED" : "OK"}`;
  els.statusLastError.textContent = status.last_error || "No recent errors.";

  els.goAButton.disabled = !ab.has_a;
  els.goBButton.disabled = !ab.has_b;

  els.debugOutput.textContent = JSON.stringify(
    {
      status,
      config: state.config,
    },
    null,
    2
  );
}

function fillSettingsForm() {
  if (!state.config) {
    return;
  }

  const cfg = state.config;
  document.getElementById("cfgMaxSpeed").value = cfg.motion.max_speed_steps_per_sec;
  document.getElementById("cfgAcceleration").value = cfg.motion.acceleration_steps_per_sec2;
  document.getElementById("cfgJogSpeed").value = cfg.motion.jog_speed_steps_per_sec;
  document.getElementById("cfgHomingSpeed").value = cfg.homing.homing_speed_steps_per_sec;
  document.getElementById("cfgHomingSlowSpeed").value = cfg.homing.homing_slow_speed_steps_per_sec;
  document.getElementById("cfgBackoff").value = cfg.homing.homing_backoff_steps;
  document.getElementById("cfgDebounce").value = cfg.homing.limit_debounce_ms;
  document.getElementById("cfgTravel").value = cfg.mechanics.slider_travel_mm;
  document.getElementById("cfgStepsPerMm").value = cfg.mechanics.steps_per_mm;
  document.getElementById("cfgHomeSide").value = cfg.homing.home_reference_side;
  document.getElementById("cfgInvertDirection").checked = cfg.axis_behavior.invert_direction_output;

  els.jogSpeed.max = String(Math.max(100, Number(cfg.motion.jog_speed_steps_per_sec)));
  els.jogSpeed.value = String(Math.round(cfg.motion.jog_speed_steps_per_sec));
  updateJogSpeedLabel();

  els.apSettings.textContent = `${cfg.wifi_ap.ssid} | ${cfg.wifi_ap.ip_address} | channel ${cfg.wifi_ap.channel} | password ${cfg.wifi_ap.password}`;
}

function addKeyframeRow(positionMm = 0, timeMs = 0) {
  const fragment = els.keyframeRowTemplate.content.cloneNode(true);
  const row = fragment.querySelector(".keyframe-row");
  row.querySelector(".keyframe-position").value = positionMm;
  row.querySelector(".keyframe-time").value = timeMs;
  row.querySelector(".remove-keyframe").addEventListener("click", () => row.remove());
  els.keyframes.appendChild(fragment);
}

function collectKeyframes() {
  return [...els.keyframes.querySelectorAll(".keyframe-row")].map((row) => ({
    position_mm: Number(row.querySelector(".keyframe-position").value || 0),
    time_ms: Number(row.querySelector(".keyframe-time").value || 0),
  }));
}

async function refreshConfig() {
  state.config = await apiRequest("/api/config");
  fillSettingsForm();
}

async function refreshStatus() {
  try {
    state.status = await apiRequest("/api/status");
    setStatusConnection("Connected", true);
    renderStatus();
  } catch (error) {
    setStatusConnection("Disconnected", false);
    els.statusLastError.textContent = error.message;
  }
}

function startPolling() {
  if (state.pollHandle) {
    clearInterval(state.pollHandle);
  }
  state.pollHandle = setInterval(refreshStatus, 500);
}

function bindHoldJog(button, signedSpeed) {
  const start = async () => {
    try {
      await postJson("/api/jog", { speed_steps_per_sec: signedSpeed() });
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  };
  const stop = async () => {
    try {
      await postJson("/api/jog", { action: "stop" });
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  };

  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointerleave", stop);
  button.addEventListener("pointercancel", stop);
}

function bindJogPad() {
  const stop = async () => {
    state.jogPadActive = false;
    els.jogPadIndicator.style.left = "calc(50% - 1.125rem)";
    try {
      await postJson("/api/jog", { action: "stop" });
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  };

  const move = async (event) => {
    if (!state.jogPadActive) {
      return;
    }

    const rect = els.jogPad.getBoundingClientRect();
    const ratio = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    const clamped = Math.max(-1, Math.min(1, ratio));
    const speed = Math.round(clamped * currentJogSpeed());
    const offset = ((clamped + 1) / 2) * (rect.width - els.jogPadIndicator.offsetWidth);
    els.jogPadIndicator.style.left = `${offset}px`;
    try {
      await postJson("/api/jog", { speed_steps_per_sec: speed });
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  };

  els.jogPad.addEventListener("pointerdown", (event) => {
    state.jogPadActive = true;
    move(event);
  });
  els.jogPad.addEventListener("pointermove", move);
  els.jogPad.addEventListener("pointerup", stop);
  els.jogPad.addEventListener("pointerleave", stop);
  els.jogPad.addEventListener("pointercancel", stop);
}

function bindActions() {
  els.stopButton.addEventListener("click", () => postJson("/api/stop").catch((error) => (els.statusLastError.textContent = error.message)));
  els.homeButton.addEventListener("click", () => postJson("/api/home").catch((error) => (els.statusLastError.textContent = error.message)));
  els.setAButton.addEventListener("click", () => postJson("/api/ab/set_a").catch((error) => (els.statusLastError.textContent = error.message)));
  els.setBButton.addEventListener("click", () => postJson("/api/ab/set_b").catch((error) => (els.statusLastError.textContent = error.message)));
  els.runAbButton.addEventListener("click", () => postJson("/api/ab/run", { repeat_mode: els.abRepeatMode.value }).catch((error) => (els.statusLastError.textContent = error.message)));

  els.goAButton.addEventListener("click", () => {
    if (!state.status?.ab?.has_a) {
      return;
    }
    postJson("/api/move", { mode: "absolute", position_steps: state.status.ab.a_position_steps }).catch((error) => (els.statusLastError.textContent = error.message));
  });
  els.goBButton.addEventListener("click", () => {
    if (!state.status?.ab?.has_b) {
      return;
    }
    postJson("/api/move", { mode: "absolute", position_steps: state.status.ab.b_position_steps }).catch((error) => (els.statusLastError.textContent = error.message));
  });

  els.addKeyframeButton.addEventListener("click", () => addKeyframeRow(0, 0));
  els.loadDemoKeyframesButton.addEventListener("click", () => {
    els.keyframes.innerHTML = "";
    addKeyframeRow(0, 0);
    addKeyframeRow(250, 10000);
    addKeyframeRow(750, 22000);
    addKeyframeRow(1000, 32000);
  });
  els.saveProgramButton.addEventListener("click", async () => {
    try {
      await postJson("/api/program", { keyframes: collectKeyframes() });
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  });
  els.runProgramButton.addEventListener("click", () => postJson("/api/program/run", { repeat_mode: els.programRepeatMode.value }).catch((error) => (els.statusLastError.textContent = error.message)));
  els.pauseProgramButton.addEventListener("click", () => postJson("/api/program/pause").catch((error) => (els.statusLastError.textContent = error.message)));
  els.resumeProgramButton.addEventListener("click", () => postJson("/api/program/resume").catch((error) => (els.statusLastError.textContent = error.message)));
  els.stopProgramButton.addEventListener("click", () => postJson("/api/program/stop").catch((error) => (els.statusLastError.textContent = error.message)));

  els.settingsForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    try {
      await postJson("/api/config", {
        motion: {
          max_speed_steps_per_sec: Number(document.getElementById("cfgMaxSpeed").value),
          acceleration_steps_per_sec2: Number(document.getElementById("cfgAcceleration").value),
          jog_speed_steps_per_sec: Number(document.getElementById("cfgJogSpeed").value),
        },
        homing: {
          homing_speed_steps_per_sec: Number(document.getElementById("cfgHomingSpeed").value),
          homing_slow_speed_steps_per_sec: Number(document.getElementById("cfgHomingSlowSpeed").value),
          homing_backoff_steps: Number(document.getElementById("cfgBackoff").value),
          limit_debounce_ms: Number(document.getElementById("cfgDebounce").value),
          home_reference_side: document.getElementById("cfgHomeSide").value,
        },
        mechanics: {
          slider_travel_mm: Number(document.getElementById("cfgTravel").value),
          steps_per_mm: Number(document.getElementById("cfgStepsPerMm").value),
        },
        axis_behavior: {
          invert_direction_output: document.getElementById("cfgInvertDirection").checked,
        },
      });
      await refreshConfig();
      await refreshStatus();
    } catch (error) {
      els.statusLastError.textContent = error.message;
    }
  });

  els.jogSpeed.addEventListener("input", updateJogSpeedLabel);
  bindHoldJog(els.jogLeftButton, () => -currentJogSpeed());
  bindHoldJog(els.jogRightButton, () => currentJogSpeed());
  bindJogPad();
}

async function bootstrap() {
  addKeyframeRow(0, 0);
  addKeyframeRow(1000, 15000);
  updateJogSpeedLabel();
  bindActions();
  await refreshConfig();
  await refreshStatus();
  startPolling();
}

bootstrap().catch((error) => {
  setStatusConnection("Disconnected", false);
  els.statusLastError.textContent = error.message;
});
