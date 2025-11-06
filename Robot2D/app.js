// --- Parámetros del robot (en cm) ---
const l1 = 12, l2 = 12, tool = 2;
const scale = 10;

// --- Tiempos de trayectoria ---
const ti = 0;
const tf = 20;
const steps = 200; // cantidad de puntos para la interpolación
const dt = (tf - ti) / steps; // intervalo de tiempo

// --- Canvas principales ---
const canvas = document.getElementById("robotCanvas");
const ctx = canvas.getContext("2d");
const msg = document.getElementById("msg");

// --- Canvases para gráficas ---
const q1Plot = document.getElementById("q1Plot");
const q2Plot = document.getElementById("q2Plot");
const ctxQ1 = q1Plot.getContext("2d");
const ctxQ2 = q2Plot.getContext("2d");

// --- Historial de posiciones ---
let trayectoria = [];
let q1 = 0, q2 = 0;

// === FUNCIONES DE CINEMÁTICA ===

// Cinemática directa (considerando herramienta)
function fk(q1, q2) {
  const x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2) + tool * Math.cos(q1 + q2);
  const y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2) + tool * Math.sin(q1 + q2);
  return { x, y };
}

// Cinemática inversa
function ik(x, y) {
  const D = Math.sqrt(x * x + y * y);
  if (D === 0) return null;
  const ux = x / D, uy = y / D;
  const xw = x - tool * ux;
  const yw = y - tool * uy;

  const d = (xw ** 2 + yw ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2);
  if (Math.abs(d) > 1) return null;

  const q2 = Math.atan2(Math.sqrt(1 - d ** 2), d);
  const q1 = Math.atan2(yw, xw) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
  return { q1, q2 };
}

// === FUNCIONES DE DIBUJO ===

// Cuadrícula y ejes
function drawGrid() {
  const w = canvas.width, h = canvas.height;
  const step = 10;
  ctx.clearRect(0, 0, w, h);
  ctx.save();
  ctx.translate(w / 2, h / 2);
  ctx.scale(1, -1);
  ctx.fillStyle = "rgba(10,10,20,0.9)";
  ctx.fillRect(-w / 2, -h / 2, w, h);

  ctx.strokeStyle = "rgba(255,255,255,0.15)";
  for (let x = -w / 2; x <= w / 2; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, -h / 2);
    ctx.lineTo(x, h / 2);
    ctx.stroke();
  }
  for (let y = -h / 2; y <= h / 2; y += step) {
    ctx.beginPath();
    ctx.moveTo(-w / 2, y);
    ctx.lineTo(w / 2, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(255, 0, 149, 0.8)";
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-w / 2, 0); ctx.lineTo(w / 2, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, -h / 2); ctx.lineTo(0, h / 2); ctx.stroke();

  ctx.restore();
}

// Dibujo del robot y trayectoria
function drawRobot(q1, q2) {
  drawGrid();
  ctx.save();
  ctx.translate(canvas.width / 2, canvas.height / 2);
  ctx.scale(1, -1);

  const j1 = { x: l1 * Math.cos(q1), y: l1 * Math.sin(q1) };
  const j2 = { x: j1.x + l2 * Math.cos(q1 + q2), y: j1.y + l2 * Math.sin(q1 + q2) };
  const eff = { x: j2.x + tool * Math.cos(q1 + q2), y: j2.y + tool * Math.sin(q1 + q2) };

  trayectoria.push(eff);

  // Trayectoria punteada
  ctx.setLineDash([5, 5]);
  ctx.strokeStyle = "rgba(255,255,255,0.5)";
  ctx.beginPath();
  for (let i = 0; i < trayectoria.length - 1; i++) {
    ctx.moveTo(trayectoria[i].x * scale, trayectoria[i].y * scale);
    ctx.lineTo(trayectoria[i + 1].x * scale, trayectoria[i + 1].y * scale);
  }
  ctx.stroke();
  ctx.setLineDash([]);

  // Eslabones
  ctx.lineWidth = 6;
  ctx.strokeStyle = "#ffdd00ff";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(j1.x * scale, j1.y * scale); ctx.stroke();
  ctx.strokeStyle = "#00ff88";
  ctx.beginPath(); ctx.moveTo(j1.x * scale, j1.y * scale); ctx.lineTo(j2.x * scale, j2.y * scale); ctx.stroke();

  // Pinza
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#ffffff";
  ctx.beginPath(); ctx.moveTo(j2.x * scale, j2.y * scale); ctx.lineTo(eff.x * scale, eff.y * scale); ctx.stroke();

  // Base y efector
  ctx.fillStyle = "#7b00ffff";
  ctx.beginPath(); ctx.arc(0, 0, 5, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = "#ff1744";
  ctx.beginPath(); ctx.arc(eff.x * scale, eff.y * scale, 5, 0, 2 * Math.PI); ctx.fill();

  ctx.restore();
}

// === GRAFICAR EVOLUCIÓN DE q1(t) y q2(t) (en grados) ===
function plotTrajectory(q1Array, q2Array) {
  ctxQ1.clearRect(0, 0, q1Plot.width, q1Plot.height);
  ctxQ2.clearRect(0, 0, q2Plot.width, q2Plot.height);

  const scaleX = q1Plot.width / q1Array.length;
  const degQ1 = q1Array.map(q => q * 180 / Math.PI);
  const degQ2 = q2Array.map(q => q * 180 / Math.PI);

  // Escalado vertical automático para rango visible
  const minQ1 = Math.min(...degQ1);
  const maxQ1 = Math.max(...degQ1);
  const minQ2 = Math.min(...degQ2);
  const maxQ2 = Math.max(...degQ2);

  const rangeQ1 = maxQ1 - minQ1 || 1;
  const rangeQ2 = maxQ2 - minQ2 || 1;

  // --- q1d(t) ---
  ctxQ1.strokeStyle = "#ff0051ff";
  ctxQ1.beginPath();
  for (let i = 0; i < degQ1.length; i++) {
    const x = i * scaleX;
    const y = q1Plot.height - ((degQ1[i] - minQ1) / rangeQ1) * q1Plot.height;
    i === 0 ? ctxQ1.moveTo(x, y) : ctxQ1.lineTo(x, y);
  }
  ctxQ1.stroke();

  // Etiquetas q1
  ctxQ1.fillStyle = "#ff007bff";
  ctxQ1.font = "bold 14px Poppins";
  const lastQ1 = degQ1[degQ1.length - 1].toFixed(1);
  ctxQ1.fillText(`q₁d(t): ${lastQ1}°`, 10, 20);

  // --- q2d(t) ---
  ctxQ2.strokeStyle = "#bea6f7ff";
  ctxQ2.beginPath();
  for (let i = 0; i < degQ2.length; i++) {
    const x = i * scaleX;
    const y = q2Plot.height - ((degQ2[i] - minQ2) / rangeQ2) * q2Plot.height;
    i === 0 ? ctxQ2.moveTo(x, y) : ctxQ2.lineTo(x, y);
  }
  ctxQ2.stroke();

  // Etiquetas q2
  ctxQ2.fillStyle = "#bea6f7ff";
  ctxQ2.font = "bold 14px Poppins";
  const lastQ2 = degQ2[degQ2.length - 1].toFixed(1);
  ctxQ2.fillText(`q₂d(t): ${lastQ2}°`, 10, 20);
}


// === MOVIMIENTO INTERPOLADO ===
function moveTo(x, y) {
  const target = ik(x, y);
  if (!target) {
    msg.textContent = "⚠️ Punto fuera de alcance";
    msg.style.color = "tomato";
    alert("El punto está fuera del espacio de trabajo");
    return;
  }
  msg.textContent = "";

  const q1_i = q1, q2_i = q2;
  const q1_f = target.q1, q2_f = target.q2;

  const q1Array = [], q2Array = [];
  let t = ti;
  let i = 0;

  function stepAnim() {
    if (i > steps) return;
    const alpha = i / steps; // interpolación lineal
    const q1d = q1_i + (q1_f - q1_i) * alpha;
    const q2d = q2_i + (q2_f - q2_i) * alpha;

    drawRobot(q1d, q2d);
    q1Array.push(q1d);
    q2Array.push(q2d);
    plotTrajectory(q1Array, q2Array);

    i++;
    t += dt;
    requestAnimationFrame(stepAnim);
  }

  stepAnim();
  q1 = q1_f;
  q2 = q2_f;
}

// === BOTONES ===
document.getElementById("mover").addEventListener("click", () => {
  const x = parseFloat(document.getElementById("x").value);
  const y = parseFloat(document.getElementById("y").value);
  moveTo(x, y);
});

document.getElementById("home").addEventListener("click", () => {
  trayectoria = [];
  moveTo(14, 14);
});

document.getElementById("limpiar").addEventListener("click", () => {
  trayectoria = [];
  drawRobot(q1, q2);
});

// === INICIO ===
const start = ik(14, 14);
q1 = start.q1; q2 = start.q2;
drawRobot(q1, q2);
