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
  let toastTimer = null;

  function showWarning(message, durationMs = 6000) {
    if (!warningEl) { console.warn("Toast container missing"); return; }
    warningTextEl.textContent = message;
    warningEl.classList.remove("hidden");
    warningEl.classList.remove("show");
    void warningEl.offsetWidth; // restart transition
    warningEl.classList.add("show");
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(hideWarning, durationMs);
  }

  function showWarningHTML(html, durationMs = 10000) {
    if (!warningEl) { console.warn("Toast container missing"); return; }
    warningTextEl.innerHTML = html; // allows link in message
    warningEl.classList.remove("hidden");
    warningEl.classList.remove("show");
    void warningEl.offsetWidth;
    warningEl.classList.add("show");
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(hideWarning, durationMs);
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

    // Priority: Drowsy/Yawn > Distraction
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

  // -------------------- API Pollers --------------------
  async function pullHeartRate() {
    try {
      const res = await fetch("/heart_rate");
      const data = await res.json();

      if (bpmEl)      bpmEl.textContent = data.bpm;
      if (hrStatusEl) {
        hrStatusEl.textContent = data.status;
        hrStatusEl.className = `pill ${data.status === "Abnormal" ? "danger" : "ok"}`;
      }

      ensureChart();
      if (hrChart) {
        hrChart.data.labels = data.series.map((_, i) => i + 1);
        hrChart.data.datasets[0].data = data.series;
        hrChart.update();
      }
    } catch {}
  }

  async function pullCar() {
    try {
      const res = await fetch("/car_status");
      const data = await res.json();
      if (speedEl)       speedEl.textContent = `${data.speed}`;
      if (speedometerEl) speedometerEl.value = data.speed;
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

      // Trigger toast logic
      maybeShowWarningFromFlags(drowsy, yawn, distract);
    } catch {}
  }

  // -------------------- Emergency Location Notice --------------------
  function showEmergencyNotice(info) {
    // Build a friendly message with a link to Maps
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
    showWarningHTML(html, 12000); // longer display for emergency
  }

  async function notifyEmergencyLocation() {
    try {
      const res = await fetch("/emergency_location");
      const info = await res.json();
      if (info?.sent) {
        showEmergencyNotice(info);
      } else {
        showWarning("Emergency activated. Location sent to a nearby hospital.", 8000);
      }
    } catch {
      showWarning("Emergency activated. Location sent to a nearby hospital.", 8000);
    }
  }

  // -------------------- Controls --------------------
  async function simulateEmergency() {
    try {
      const res = await fetch("/simulate_heart_attack");
      const data = await res.json();
      alert(data.message);
      // Fetch & show dummy hospital/location info
      await notifyEmergencyLocation();
      // Update car UI
      pullCar();
    } catch {}
  }

  async function resetEmergency() {
    try {
      const res = await fetch("/reset_heart_attack");
      const data = await res.json();
      alert(data.message);
      pullCar();
    } catch {}
  }

  if (simulateBtn) simulateBtn.addEventListener("click", simulateEmergency);
  if (resetBtn)    resetBtn.addEventListener("click", resetEmergency);

  // -------------------- Kickoff + Polling --------------------
  pullHeartRate();
  pullCar();
  pullDriver();

  setInterval(() => {
    pullHeartRate();
    pullCar();
    pullDriver();
  }, 2000);
});
