document.addEventListener("DOMContentLoaded", () => {
  // -------------------- Globals & Elements --------------------
  let hrChart;

  const bpmEl         = document.getElementById("bpm");
  const hrStatusEl    = document.getElementById("hr-status");
  const speedEl       = document.getElementById("speed");
  const speedometerEl = document.getElementById("speedometer");

  // Toast elements
  const warningEl       = document.getElementById("warning");
  const warningTextEl   = document.getElementById("warning-text");
  const warningCloseBtn = document.getElementById("warning-close");

  // Buttons
  const simulateBtn = document.getElementById("simulate-heart-attack");
  const resetBtn    = document.getElementById("reset-heart-attack");

  // -------------------- Toast Controller --------------------
  // If durationMs is null/0, the toast stays until hideWarning() is called.
  let toastTimer = null;

  function showWarning(message, durationMs = 6000) {
    if (!warningEl) { console.warn("Toast container missing"); return; }
    warningTextEl.textContent = message;
    warningEl.classList.remove("hidden");
    warningEl.classList.remove("show");
    void warningEl.offsetWidth; // restart transition
    warningEl.classList.add("show");
    if (toastTimer) { clearTimeout(toastTimer); toastTimer = null; }
    if (durationMs && durationMs > 0) {
      toastTimer = setTimeout(hideWarning, durationMs);
    }
  }

  function showWarningHTML(html, durationMs = 10000) {
    if (!warningEl) { console.warn("Toast container missing"); return; }
    warningTextEl.innerHTML = html; // allows link in message
    warningEl.classList.remove("hidden");
    warningEl.classList.remove("show");
    void warningEl.offsetWidth;
    warningEl.classList.add("show");
    if (toastTimer) { clearTimeout(toastTimer); toastTimer = null; }
    if (durationMs && durationMs > 0) {
      toastTimer = setTimeout(hideWarning, durationMs);
    }
  }

  function hideWarning() {
    if (!warningEl) return;
    warningEl.classList.remove("show");
    toastTimer = null;
  }

  if (warningCloseBtn) {
    warningCloseBtn.addEventListener("click", hideWarning);
  }

  // Rising-edge detection + cooldown to prevent spam
  let prevDrowsy = false, prevYawn = false, prevDistract = false;
  let lastWarnTs = 0;
  const WARN_COOLDOWN_MS = 8000;

  function maybeShowWarningFromFlags(drowsy, yawn, distract) {
    const now = Date.now();
    const drowsyRise   = drowsy && !prevDrowsy;
    const yawnRise     = yawn && !prevYawn;
    const distractRise = distract && !prevDistract;

    let message = null;
    if (drowsyRise || yawnRise) {
      message = "Take a short break for a few minutes.";
    } else if (distractRise) {
      message = "Please focus on the road.";
    }

    if (message && (now - lastWarnTs >= WARN_COOLDOWN_MS)) {
      showWarning(message, 6000);
      lastWarnTs = now;
    }

    prevDrowsy = drowsy;
    prevYawn = yawn;
    prevDistract = distract;
  }

  // -------------------- Chart --------------------
  function ensureChart() {
    if (hrChart) return;
    const canvas = document.getElementById("hrChart");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    hrChart = new Chart(ctx, {
      type: "line",
      data: {
        labels: [],
        datasets: [{
          label: "Heart Rate (bpm)",
          data: [],
          borderWidth: 2,
          fill: false
        }]
      },
      options: {
        animation: false,
        responsive: true,
        scales: { y: { beginAtZero: false } }
      }
    });
  }

  // -------------------- Eased Speedometer Animation --------------------
  let displayedSpeed = 100;   // what's currently shown
  let targetSpeed    = 100;   // latest backend value
  let animStartTime  = 0;
  let animDuration   = 800;   // ms; will vary based on hazard/normal
  let animFrom       = 100;
  let animRunning    = false;

  // Cubic ease-out (fast start, slow finish)
  function easeOutCubic(t) {
    return 1 - Math.pow(1 - t, 3);
  }

  function startSpeedTween(newTarget, durationMs) {
    animFrom      = displayedSpeed;
    targetSpeed   = newTarget;
    animDuration  = Math.max(150, durationMs|0);
    animStartTime = performance.now();
    if (!animRunning) {
      animRunning = true;
      requestAnimationFrame(stepSpeedAnim);
    }
  }

  function stepSpeedAnim(nowTs) {
    if (!animRunning) return;
    const t = Math.min(1, (nowTs - animStartTime) / animDuration);
    const k = easeOutCubic(t);
    const value = animFrom + (targetSpeed - animFrom) * k;
    displayedSpeed = value;

    // Update DOM
    if (speedEl)       speedEl.textContent = Math.round(displayedSpeed);
    if (speedometerEl) speedometerEl.value = Math.round(displayedSpeed);

    if (t < 1) {
      requestAnimationFrame(stepSpeedAnim);
    } else {
      animRunning = false;
    }
  }

  // -------------------- API Pollers --------------------
  let hazardPrev = false;     // for persistent banner edge detection
  let seenRealHR = false;     // chart stays blank until real sensor data

  async function pullHeartRate() {
    try {
      const res = await fetch("/heart_rate");
      const data = await res.json();

      if (bpmEl) bpmEl.textContent = data.bpm;

      if (hrStatusEl) {
        hrStatusEl.textContent = data.status;

        let cls = "";
        if (data.status === "Abnormal") {
          cls = "danger";
        } else if (data.status === "Normal") {
          cls = "ok";
        } else if (data.status === "NoRead") {
          cls = "warn";       // highlight NoRead in yellow
        } else {
          cls = "";           // neutral
        }
        hrStatusEl.className = `pill ${cls}`;
      }

      ensureChart();
      if (hrChart) {
        const hasSeries = Array.isArray(data.series) && data.series.length > 0;
        if (hasSeries) {
          seenRealHR = true;
          hrChart.data.labels = data.series.map((_, i) => i + 1);
          hrChart.data.datasets[0].data = data.series;
          hrChart.update();
        } else if (!seenRealHR) {
          // keep blank (no labels/data) until first real sample arrives
          hrChart.data.labels = [];
          hrChart.data.datasets[0].data = [];
          hrChart.update();
        }
      }
    } catch {}
  }

  async function pullCar() {
    try {
      const res = await fetch("/car_status");
      const data = await res.json();

      // Animate speedometer & numeric value with ease
      const backendSpeed = Number(data.speed) || 0;
      const isHazard = !!data.hazard;

      // Longer, gentler animation when hazard is active so UI matches motor easing
      const duration = isHazard ? 2200 : 700;
      startSpeedTween(backendSpeed, duration);

      // Toggle the blinking EMERGENCY badge
      const hz = document.getElementById("hazard-badge");
      if (hz) {
        if (isHazard) {
          hz.classList.remove("hidden");
          hz.classList.add("blink");
        } else {
          hz.classList.add("hidden");
          hz.classList.remove("blink");
        }
      }

      // ---------------- PERSISTENT GPS banner while hazard is active ----------------
      if (isHazard && !hazardPrev) {
        try {
          const locRes = await fetch("/emergency_location");
          const info = await locRes.json();
          if (info?.sent) {
            const name     = info?.hospital?.name || "nearby hospital";
            const address  = info?.hospital?.address || "";
            const dist     = typeof info?.distance_km === "number" ? `${info.distance_km.toFixed(1)} km` : "—";
            const eta      = typeof info?.eta_min === "number" ? `~${info.eta_min} min` : "—";
            const mapsUrl  = info?.maps_url || "#";

            const html = `
              <div>
                <div><strong>Emergency activated.</strong></div>
                <div>Location sent to <strong>${name}</strong> (${dist}, ${eta}).</div>
                <div>${address ? address + " • " : ""}<a href="${mapsUrl}" target="_blank" rel="noopener">Open in Maps</a></div>
              </div>
            `;
            // null duration => persistent until reset/close
            showWarningHTML(html, null);
          } else {
            showWarning("Emergency activated. Location sent to a nearby hospital.", null);
          }
        } catch {
          showWarning("Emergency activated. Location sent to a nearby hospital.", null);
        }
      } else if (!isHazard && hazardPrev) {
        // Hazard cleared (e.g., via Reset) => remove banner immediately
        hideWarning();
      }
      hazardPrev = isHazard;

    } catch {}
  }

  async function pullDriver() {
    try {
      const res = await fetch("/driver_status");
      const data = await res.json();

      const drowsy   = !!data.drowsiness;
      const yawn     = !!data.yawning;
      const distract = !!data.distraction;

      const dEl = document.getElementById("drowsiness");
      const yEl = document.getElementById("yawning");
      const sEl = document.getElementById("distraction");
      if (dEl) dEl.textContent = drowsy ? "Yes" : "No";
      if (yEl) yEl.textContent = yawn ? "Yes" : "No";
      if (sEl) sEl.textContent = distract ? "Yes" : "No";

      // Trigger toast logic on rising edges (short, non-persistent tips)
      maybeShowWarningFromFlags(drowsy, yawn, distract);
    } catch {}
  }

  // -------------------- Controls --------------------
  async function simulateEmergency() {
    try {
      const res = await fetch("/simulate_heart_attack");
      const data = await res.json();
      alert(data.message);
      // Persistent banner is handled in the next pullCar tick
      pullCar();
    } catch {}
  }

  async function resetEmergency() {
    try {
      const res = await fetch("/reset_heart_attack");
      const data = await res.json();
      alert(data.message);
      pullCar(); // will hide the persistent banner if hazard is off
    } catch {}
  }

  if (simulateBtn) simulateBtn.addEventListener("click", simulateEmergency);
  if (resetBtn)    resetBtn.addEventListener("click", resetEmergency);

  // -------------------- Kickoff + Polling --------------------
  // Initialize displayed speed from current progress value
  if (speedometerEl) {
    displayedSpeed = Number(speedometerEl.value) || 100;
    targetSpeed    = displayedSpeed;
    if (speedEl) speedEl.textContent = Math.round(displayedSpeed);
  }

  pullHeartRate();
  pullCar();
  pullDriver();

  // Faster polling (1s) so the graph/speed feel live
  setInterval(() => {
    pullHeartRate();
    pullCar();
    pullDriver();
  }, 1000);
});
