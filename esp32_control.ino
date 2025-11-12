#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

// =================== CONFIGURACIÓN GENERAL ===================
static const uint8_t NUM_MOTORES = 5;
static const uint8_t ARDUINO_I2C_ADDRESS = 0x10;
static const uint8_t COMANDO_I2C_OBJETIVO = 0x01;
static const size_t ESTADO_FLOTES_POR_MOTOR = 4;
static const size_t ESTADO_BUFFER_LENGTH = 1 + NUM_MOTORES * (1 + ESTADO_FLOTES_POR_MOTOR * sizeof(float));

// Pines I2C por defecto en ESP32
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;

// Credenciales de la red Wi-Fi creada por el ESP32
static const char *AP_SSID = "Brazo-SCADA";
static const char *AP_PASSWORD = "Soldador360";

WebServer server(80);

static float objetivosLocales[NUM_MOTORES] = {0.0f};
static bool objetivoLocalValido[NUM_MOTORES] = {false};

struct MotorSnapshot
{
  bool encoderDetectado = false;
  bool objetivoVigente = false;
  bool enCorreccion = false;
  bool calibrado = false;
  bool calibrando = false;
  bool calibracionError = false;
  float angulo = 0.0f;
  float rangoMin = 0.0f;
  float rangoMax = 0.0f;
  float progresoCalibracion = 0.0f;
  bool objetivoWebValido = false;
  float objetivoWeb = 0.0f;
};

// =================== PLANTILLA HTML ===================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>SCADA Brazo Robótico</title>
<style>
  :root {
    color-scheme: dark;
    --bg-primary: #0a101a;
    --bg-panel: #121c29;
    --bg-highlight: #1f2f46;
    --accent: #1ee2f4;
    --accent-soft: rgba(30, 226, 244, 0.35);
    --warning: #f7a046;
    --danger: #ff4f6d;
    --success: #68f7a3;
    font-family: "Rajdhani", "Segoe UI", sans-serif;
  }
  * {
    box-sizing: border-box;
  }
  body {
    margin: 0;
    background: radial-gradient(circle at top right, rgba(30, 226, 244, 0.2), transparent 50%),
                radial-gradient(circle at bottom left, rgba(104, 247, 163, 0.15), transparent 55%),
                var(--bg-primary);
    color: #d8e6f5;
    min-height: 100vh;
    display: flex;
    align-items: stretch;
    justify-content: center;
    padding: 24px;
  }
  body::before {
    content: "";
    position: fixed;
    inset: 0;
    background: repeating-linear-gradient(115deg, rgba(30, 226, 244, 0.05) 0, rgba(30, 226, 244, 0.05) 2px, transparent 2px, transparent 16px);
    pointer-events: none;
    mix-blend-mode: screen;
    opacity: 0.6;
  }
  .dashboard {
    width: min(1100px, 100%);
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 24px;
  }
  header {
    grid-column: 1 / -1;
    background: linear-gradient(145deg, rgba(18, 28, 41, 0.95), rgba(12, 20, 32, 0.9));
    border: 1px solid rgba(30, 226, 244, 0.2);
    border-radius: 16px;
    padding: 24px;
    box-shadow: 0 16px 32px rgba(0, 0, 0, 0.35);
    position: relative;
    overflow: hidden;
  }
  header::after {
    content: "";
    position: absolute;
    inset: 0;
    background: linear-gradient(120deg, rgba(30, 226, 244, 0.15), transparent 55%);
    pointer-events: none;
  }
  h1 {
    margin: 0;
    font-size: clamp(1.8rem, 3vw, 2.6rem);
    letter-spacing: 0.08em;
    text-transform: uppercase;
  }
  .status-bar {
    margin-top: 12px;
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    align-items: center;
  }
  .status-indicator {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 6px 16px;
    border-radius: 999px;
    background: rgba(30, 226, 244, 0.15);
    border: 1px solid rgba(30, 226, 244, 0.3);
    letter-spacing: 0.06em;
    text-transform: uppercase;
    font-size: 0.85rem;
  }
  .status-indicator.offline {
    background: rgba(255, 79, 109, 0.12);
    border-color: rgba(255, 79, 109, 0.3);
  }
  .status-indicator.online::before {
    content: "";
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--success);
    box-shadow: 0 0 8px rgba(104, 247, 163, 0.9);
  }
  .status-indicator.offline::before {
    content: "";
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--danger);
    box-shadow: 0 0 8px rgba(255, 79, 109, 0.9);
  }
  .panel {
    background: rgba(12, 20, 32, 0.88);
    border: 1px solid rgba(30, 226, 244, 0.2);
    border-radius: 18px;
    padding: 20px;
    backdrop-filter: blur(6px);
    position: relative;
    overflow: hidden;
  }
  .panel::before {
    content: "";
    position: absolute;
    inset: 0;
    border-radius: inherit;
    border: 1px solid rgba(104, 247, 163, 0.1);
    pointer-events: none;
  }
  .panel-title {
    margin: 0 0 16px 0;
    font-size: 1.2rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: #9dd5ff;
  }
  form {
    display: grid;
    gap: 14px;
  }
  label {
    font-size: 0.85rem;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: rgba(216, 230, 245, 0.75);
  }
  select, input[type="number"] {
    width: 100%;
    padding: 12px 14px;
    border-radius: 12px;
    border: 1px solid rgba(30, 226, 244, 0.35);
    background: rgba(12, 22, 34, 0.85);
    color: #f0f6ff;
    font-size: 1rem;
    transition: border 0.2s ease, box-shadow 0.2s ease;
  }
  select:focus, input[type="number"]:focus {
    outline: none;
    border-color: var(--accent);
    box-shadow: 0 0 0 3px rgba(30, 226, 244, 0.25);
  }
  button {
    padding: 14px 16px;
    border-radius: 12px;
    border: none;
    font-size: 1rem;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    background: linear-gradient(135deg, rgba(30, 226, 244, 0.85), rgba(13, 160, 220, 0.85));
    color: #05121e;
    font-weight: 600;
    cursor: pointer;
    position: relative;
    overflow: hidden;
    transition: transform 0.2s ease;
  }
  button::after {
    content: "";
    position: absolute;
    inset: 0;
    background: linear-gradient(135deg, rgba(255, 255, 255, 0.25), transparent);
    opacity: 0;
    transition: opacity 0.2s ease;
  }
  button:hover {
    transform: translateY(-2px);
  }
  button:hover::after {
    opacity: 1;
  }
  .motors-grid {
    display: grid;
    gap: 18px;
  }
  .motor-card {
    position: relative;
    padding: 18px;
    border-radius: 16px;
    background: rgba(10, 18, 28, 0.85);
    border: 1px solid rgba(30, 226, 244, 0.2);
    overflow: hidden;
    transition: transform 0.2s ease, border 0.2s ease;
  }
  .motor-card.online {
    border-color: rgba(104, 247, 163, 0.4);
  }
  .motor-card.offline {
    border-color: rgba(255, 79, 109, 0.35);
    opacity: 0.65;
  }
  .motor-card.error {
    border-color: rgba(255, 79, 109, 0.55);
  }
  .motor-card::before {
    content: "";
    position: absolute;
    inset: 0;
    background: linear-gradient(120deg, rgba(30, 226, 244, 0.18), transparent 60%);
    opacity: 0;
    transition: opacity 0.3s ease;
    pointer-events: none;
  }
  .motor-card.highlight::before {
    opacity: 1;
  }
  .motor-id {
    font-size: 1.1rem;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    margin-bottom: 8px;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .motor-range {
    margin-top: 6px;
    font-size: 0.9rem;
    color: rgba(216, 230, 245, 0.75);
  }
  .calibration-status {
    margin-top: 10px;
    font-size: 0.8rem;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: rgba(216, 230, 245, 0.65);
  }
  .progress-track {
    width: 100%;
    height: 6px;
    border-radius: 999px;
    background: rgba(30, 226, 244, 0.15);
    overflow: hidden;
    margin-top: 6px;
  }
  .progress-fill {
    height: 100%;
    border-radius: inherit;
    background: linear-gradient(135deg, rgba(30, 226, 244, 0.9), rgba(104, 247, 163, 0.85));
    width: 0%;
    transition: width 0.3s ease;
  }
  .motor-angle {
    font-size: 2.1rem;
    font-weight: 600;
    color: var(--accent);
  }
  .motor-target {
    font-size: 1rem;
    color: rgba(216, 230, 245, 0.7);
  }
  .tags {
    margin-top: 12px;
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
  }
  .tag {
    padding: 4px 10px;
    border-radius: 999px;
    font-size: 0.75rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    border: 1px solid rgba(30, 226, 244, 0.3);
    background: rgba(30, 226, 244, 0.15);
  }
  .tag.warning {
    border-color: rgba(247, 160, 70, 0.4);
    background: rgba(247, 160, 70, 0.15);
  }
  .tag.danger {
    border-color: rgba(255, 79, 109, 0.4);
    background: rgba(255, 79, 109, 0.15);
  }
  .panel-subtitle {
    margin: 0 0 14px 0;
    font-size: 0.85rem;
    color: rgba(216, 230, 245, 0.7);
  }
  .workspace-grid {
    display: grid;
    gap: 16px;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    margin-bottom: 16px;
  }
  .workspace-card {
    border: 1px solid rgba(30, 226, 244, 0.25);
    border-radius: 14px;
    padding: 16px;
    background: rgba(10, 18, 28, 0.8);
    display: grid;
    gap: 10px;
  }
  .workspace-card.disabled {
    opacity: 0.6;
    border-color: rgba(255, 79, 109, 0.35);
  }
  .workspace-title {
    font-size: 0.95rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: #9dd5ff;
  }
  .workspace-range {
    font-size: 0.8rem;
    color: rgba(216, 230, 245, 0.65);
  }
  .workspace-status {
    font-size: 0.85rem;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: rgba(216, 230, 245, 0.7);
    padding: 10px 12px;
    border-radius: 12px;
    background: rgba(30, 226, 244, 0.1);
    border: 1px solid rgba(30, 226, 244, 0.2);
  }
  .workspace-status.error {
    border-color: rgba(255, 79, 109, 0.4);
    background: rgba(255, 79, 109, 0.1);
    color: rgba(255, 170, 185, 0.85);
  }
  .workspace-status.success {
    border-color: rgba(104, 247, 163, 0.4);
    background: rgba(104, 247, 163, 0.12);
    color: rgba(180, 255, 215, 0.85);
  }
  .footer-note {
    grid-column: 1 / -1;
    text-align: center;
    font-size: 0.8rem;
    color: rgba(216, 230, 245, 0.55);
  }
  #toast {
    position: fixed;
    bottom: 32px;
    right: 32px;
    padding: 14px 18px;
    border-radius: 14px;
    background: rgba(18, 28, 41, 0.95);
    border: 1px solid rgba(30, 226, 244, 0.25);
    color: #f0f6ff;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    opacity: 0;
    transform: translateY(12px);
    transition: opacity 0.3s ease, transform 0.3s ease;
    pointer-events: none;
  }
  #toast.show {
    opacity: 1;
    transform: translateY(0);
  }
  #toast.success {
    border-color: rgba(104, 247, 163, 0.5);
    box-shadow: 0 0 14px rgba(104, 247, 163, 0.35);
  }
  #toast.error {
    border-color: rgba(255, 79, 109, 0.45);
    box-shadow: 0 0 14px rgba(255, 79, 109, 0.35);
  }
</style>
</head>
<body>
<div class="dashboard">
  <header>
    <h1>Panel Industrial Brazo Robótico</h1>
    <div class="status-bar">
      <span class="status-indicator offline" id="link-status">Sin conexión</span>
      <span>Actualización: <strong id="last-update">—</strong></span>
      <span>Red: <strong>"Brazo-SCADA"</strong></span>
      <span>IP: <strong id="ap-ip">-</strong></span>
    </div>
  </header>

  <section class="panel">
    <h2 class="panel-title">Orden de Movimiento</h2>
    <form id="command-form">
      <div>
        <label for="motor">Motor</label>
        <select id="motor" required>
          <option value="1">Motor 1</option>
          <option value="2">Motor 2</option>
          <option value="3">Motor 3</option>
          <option value="4">Motor 4</option>
          <option value="5">Motor 5</option>
        </select>
      </div>
      <div>
        <label for="angle">Ángulo objetivo (°)</label>
        <input type="number" id="angle" required step="0.1" value="0" />
      </div>
      <button type="submit">Enviar Comando</button>
    </form>
  </section>

  <section class="panel" style="grid-column: span 2;">
    <h2 class="panel-title">Campo Operativo</h2>
    <p class="panel-subtitle">Ingrese una coordenada para todos los motores calibrados. El sistema verificará los rangos detectados antes de enviar las órdenes.</p>
    <form id="workspace-form">
      <div class="workspace-grid" id="workspace-grid"></div>
      <div class="workspace-status" id="workspace-status">Esperando calibración de motores...</div>
      <button type="submit" style="margin-top: 16px;">Enviar Coordenada</button>
    </form>
  </section>

  <section class="panel" style="grid-column: span 2;">
    <h2 class="panel-title">Estado de Motores</h2>
    <div class="motors-grid" id="motors"></div>
  </section>

  <p class="footer-note">Interfaz SCADA ligera ejecutándose en el ESP32. Actualización automática cada 1.5 segundos.</p>
</div>
<div id="toast"></div>
<script>
  const motorsContainer = document.getElementById('motors');
  const statusBadge = document.getElementById('link-status');
  const lastUpdate = document.getElementById('last-update');
  const toast = document.getElementById('toast');
  const workspaceGrid = document.getElementById('workspace-grid');
  const workspaceStatus = document.getElementById('workspace-status');
  const workspaceForm = document.getElementById('workspace-form');
  let motorsCache = [];

  const showToast = (message, type = 'success') => {
    toast.textContent = message;
    toast.className = '';
    toast.classList.add(type === 'error' ? 'error' : 'success');
    toast.classList.add('show');
    setTimeout(() => toast.classList.remove('show'), 2200);
  };

  const renderMotors = (motors) => {
    motorsContainer.innerHTML = '';
    motors.forEach((motor) => {
      const card = document.createElement('div');
      card.classList.add('motor-card');
      card.classList.add(motor.encoder ? 'online' : 'offline');
      if (motor.adjusting) {
        card.classList.add('highlight');
        setTimeout(() => card.classList.remove('highlight'), 600);
      }
      if (motor.calibrationError) {
        card.classList.add('error');
      }
      const objetivoTexto = motor.target === null ? '—' : `${motor.target.toFixed(2)}°`;
      let origenClase = 'warning';
      let origenEtiqueta = 'Sin orden';
      if (motor.targetValid)
      {
        origenEtiqueta = motor.webTarget ? 'Orden web' : 'Orden externa';
        origenClase = motor.webTarget ? '' : 'warning';
      }
      const rangoTexto = motor.calibrated ? `${motor.rangeMin.toFixed(1)}° – ${motor.rangeMax.toFixed(1)}°` : 'No disponible';
      let estadoCalibracion = 'Esperando calibración';
      let progresoValor = Number.isFinite(motor.calibrationProgress) ? motor.calibrationProgress : 0;
      let progresoPercent = Math.max(0, Math.min(100, Math.round(progresoValor * 100)));
      if (progresoValor < 0) {
        progresoPercent = 100;
      }
      if (motor.calibrationError) {
        estadoCalibracion = 'Error de calibración';
      } else if (motor.calibrated) {
        estadoCalibracion = 'Calibrado';
        progresoPercent = 100;
      } else if (motor.calibrating) {
        estadoCalibracion = `Calibrando ${progresoPercent}%`;
      }
      const calibracionClase = motor.calibrationError ? 'danger' : (motor.calibrated ? 'success' : (motor.calibrating ? 'warning' : 'warning'));
      card.innerHTML = `
        <div class="motor-id">Motor ${motor.index}</div>
        <div class="motor-angle">${motor.angle.toFixed(2)}°</div>
        <div class="motor-target">Objetivo web: ${objetivoTexto}</div>
        <div class="motor-range">Rango: ${rangoTexto}</div>
        <div class="calibration-status">${estadoCalibracion}</div>
        <div class="progress-track"><div class="progress-fill" style="width:${progresoPercent}%;"></div></div>
        <div class="tags">
          <span class="tag ${motor.encoder ? '' : 'danger'}">${motor.encoder ? 'Encoder' : 'Sin encoder'}</span>
          <span class="tag ${motor.targetValid ? '' : 'warning'}">${motor.targetValid ? 'Setpoint activo' : 'Setpoint no definido'}</span>
          <span class="tag ${origenClase}">${origenEtiqueta}</span>
          <span class="tag ${motor.adjusting ? 'warning' : ''}">${motor.adjusting ? 'Corrigiendo' : 'Estable'}</span>
          <span class="tag ${calibracionClase}">${motor.calibrationError ? 'Error' : (motor.calibrated ? 'Calibrado' : (motor.calibrating ? 'Calibrando' : 'Pendiente'))}</span>
        </div>
      `;
      motorsContainer.appendChild(card);
    });
  };

  const evaluateCoordinates = (updateStatus = true) => {
    const result = {
      valid: false,
      allCalibrated: false,
      coords: [],
      message: 'Sin datos',
    };

    if (!workspaceGrid || !workspaceStatus || motorsCache.length === 0)
    {
      if (updateStatus && workspaceStatus)
      {
        workspaceStatus.textContent = 'Esperando calibración de motores...';
        workspaceStatus.classList.remove('error', 'success');
      }
      return result;
    }

    const inputs = workspaceGrid.querySelectorAll('input[data-index]');
    if (inputs.length === 0)
    {
      if (updateStatus)
      {
        workspaceStatus.textContent = 'No hay motores disponibles.';
        workspaceStatus.classList.add('error');
        workspaceStatus.classList.remove('success');
      }
      return result;
    }

    const allCalibrated = motorsCache.every((motor) => motor.calibrated);
    result.allCalibrated = allCalibrated;
    if (!allCalibrated)
    {
      if (updateStatus)
      {
        workspaceStatus.textContent = 'Calibración pendiente en uno o más motores.';
        workspaceStatus.classList.add('error');
        workspaceStatus.classList.remove('success');
      }
      result.message = 'Calibración pendiente';
      return result;
    }

    let fueraDeRango = false;
    let mensaje = 'Coordenada válida en el campo operativo.';

    inputs.forEach((input) => {
      const motorIndex = Number(input.dataset.index);
      const motor = motorsCache[motorIndex];
      if (!motor)
      {
        return;
      }
      const valor = Number(input.value);
      if (Number.isNaN(valor))
      {
        fueraDeRango = true;
        mensaje = `Valor inválido para el motor ${motor.index}.`;
        return;
      }
      if (valor < motor.rangeMin - 0.0001 || valor > motor.rangeMax + 0.0001)
      {
        fueraDeRango = true;
        mensaje = `Motor ${motor.index} fuera de rango (${motor.rangeMin.toFixed(1)}° – ${motor.rangeMax.toFixed(1)}°).`;
      }
      result.coords.push({ motor: motor.index, value: valor });
    });

    result.valid = !fueraDeRango;
    result.message = mensaje;

    if (updateStatus)
    {
      workspaceStatus.textContent = mensaje;
      if (fueraDeRango)
      {
        workspaceStatus.classList.add('error');
        workspaceStatus.classList.remove('success');
      }
      else
      {
        workspaceStatus.classList.remove('error');
        workspaceStatus.classList.add('success');
      }
    }

    return result;
  };

  const updateWorkspacePanel = (motors) => {
    if (!workspaceGrid)
    {
      return;
    }

    const valoresPrevios = {};
    workspaceGrid.querySelectorAll('input[data-index]').forEach((input) => {
      valoresPrevios[input.id] = input.value;
    });

    workspaceGrid.innerHTML = '';

    motors.forEach((motor, idx) => {
      const card = document.createElement('div');
      card.classList.add('workspace-card');
      if (!motor.calibrated)
      {
        card.classList.add('disabled');
      }

      const inputId = `workspace-motor-${motor.index}`;

      const titulo = document.createElement('label');
      titulo.className = 'workspace-title';
      titulo.setAttribute('for', inputId);
      titulo.textContent = `Motor ${motor.index}`;

      const input = document.createElement('input');
      input.type = 'number';
      input.step = '0.1';
      input.id = inputId;
      input.dataset.index = String(idx);
      input.value = valoresPrevios[inputId] ?? motor.angle.toFixed(1);
      if (motor.calibrated)
      {
        input.min = motor.rangeMin.toFixed(2);
        input.max = motor.rangeMax.toFixed(2);
        input.disabled = false;
      }
      else
      {
        input.disabled = true;
      }

      const rango = document.createElement('div');
      rango.className = 'workspace-range';
      rango.textContent = motor.calibrated
        ? `Rango permitido: ${motor.rangeMin.toFixed(1)}° – ${motor.rangeMax.toFixed(1)}°`
        : 'Pendiente de calibración.';

      card.appendChild(titulo);
      card.appendChild(input);
      card.appendChild(rango);
      workspaceGrid.appendChild(card);
    });

    evaluateCoordinates(true);
  };

  const refreshStatus = async () => {
    try {
      const response = await fetch('/status');
      if (!response.ok) {
        throw new Error('Estado HTTP ' + response.status);
      }
      const payload = await response.json();
      const motors = (payload.motors || []).map((motor) => ({
        ...motor,
        angle: Number(motor.angle),
        target: motor.target === null ? null : Number(motor.target),
        webTarget: Boolean(motor.webTarget),
        targetValid: Boolean(motor.targetValid),
        encoder: Boolean(motor.encoder),
        adjusting: Boolean(motor.adjusting),
        calibrated: Boolean(motor.calibrated),
        calibrating: Boolean(motor.calibrating),
        calibrationError: Boolean(motor.calibrationError),
        rangeMin: Number(motor.rangeMin),
        rangeMax: Number(motor.rangeMax),
        calibrationProgress: Number(motor.calibrationProgress),
      }));
      renderMotors(motors);
      motorsCache = motors;
      updateWorkspacePanel(motors);
      statusBadge.textContent = 'Conectado';
      statusBadge.classList.remove('offline');
      statusBadge.classList.add('online');
      lastUpdate.textContent = new Date().toLocaleTimeString();
    } catch (error) {
      statusBadge.textContent = 'Sin datos';
      statusBadge.classList.add('offline');
      statusBadge.classList.remove('online');
      motorsCache = [];
      if (workspaceGrid && workspaceStatus)
      {
        workspaceGrid.innerHTML = '';
        workspaceStatus.textContent = 'Sin conexión con el Arduino.';
        workspaceStatus.classList.add('error');
        workspaceStatus.classList.remove('success');
      }
    }
  };

  if (workspaceForm)
  {
    workspaceForm.addEventListener('input', () => {
      evaluateCoordinates(true);
    });

    workspaceForm.addEventListener('submit', async (event) => {
      event.preventDefault();
      const evaluacion = evaluateCoordinates(true);
      if (!evaluacion.allCalibrated)
      {
        showToast('Motores sin calibrar', 'error');
        return;
      }
      if (!evaluacion.valid)
      {
        showToast('Coordenada fuera de rango', 'error');
        return;
      }

      try
      {
        for (const coord of evaluacion.coords)
        {
          const response = await fetch('/command', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/x-www-form-urlencoded'
            },
            body: new URLSearchParams({ motor: String(coord.motor), angle: String(coord.value) }),
          });
          if (!response.ok)
          {
            const message = await response.text();
            throw new Error(message || 'Error al enviar coordenada');
          }
        }
        showToast('Coordenada enviada');
        workspaceStatus.textContent = 'Coordenada aplicada exitosamente.';
        workspaceStatus.classList.remove('error');
        workspaceStatus.classList.add('success');
        refreshStatus();
      }
      catch (err)
      {
        showToast('Fallo de envío', 'error');
        workspaceStatus.textContent = 'Error de comunicación I2C.';
        workspaceStatus.classList.add('error');
        workspaceStatus.classList.remove('success');
      }
    });
  }

  document.getElementById('command-form').addEventListener('submit', async (event) => {
    event.preventDefault();
    const motor = document.getElementById('motor').value;
    const angle = document.getElementById('angle').value;
    try {
      const response = await fetch('/command', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: new URLSearchParams({ motor, angle }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || 'Error al enviar comando');
      }
      showToast('Comando enviado');
      refreshStatus();
    } catch (error) {
      showToast('Fallo de envío', 'error');
    }
  });

  setInterval(refreshStatus, 1500);
  window.addEventListener('load', () => {
    refreshStatus();
    fetch('/ap-ip').then(res => res.text()).then(ip => {
      document.getElementById('ap-ip').textContent = ip || '-';
    });
  });
</script>
</body>
</html>
)rawliteral";

// =================== PROTOTIPOS ===================

bool enviarObjetivo(uint8_t motor, float angulo, bool reiniciar = true);
bool obtenerEstadoMotores(MotorSnapshot (&estado)[NUM_MOTORES], uint8_t &reportados);
void handleRoot();
void handleStatus();
void handleCommand();
void handleNotFound();
void handleApIp();

// =================== SETUP ===================

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("Iniciando interfaz SCADA en ESP32..."));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000UL);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.print(F("Red creada: "));
  Serial.println(AP_SSID);
  Serial.print(F("IP de acceso: "));
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/command", HTTP_POST, handleCommand);
  server.on("/ap-ip", HTTP_GET, handleApIp);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println(F("Servidor web iniciado."));
}

// =================== LOOP ===================

void loop()
{
  server.handleClient();
}

// =================== HANDLERS ===================

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApIp()
{
  server.send(200, "text/plain", WiFi.softAPIP().toString());
}

void handleStatus()
{
  MotorSnapshot estado[NUM_MOTORES];
  uint8_t reportados = 0;
  if (!obtenerEstadoMotores(estado, reportados))
  {
    server.send(503, "application/json", "{\"error\":\"Arduino no responde\"}");
    return;
  }

  String respuesta = F("{\"motors\":[");
  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    if (i > 0)
    {
      respuesta += ',';
    }
    respuesta += F("{\"index\":");
    respuesta += String(i + 1);
    respuesta += F(",\"encoder\":");
    respuesta += estado[i].encoderDetectado ? F("true") : F("false");
    respuesta += F(",\"targetValid\":");
    respuesta += estado[i].objetivoVigente ? F("true") : F("false");
    respuesta += F(",\"adjusting\":");
    respuesta += estado[i].enCorreccion ? F("true") : F("false");
    respuesta += F(",\"angle\":");
    respuesta += String(estado[i].angulo, 2);
    respuesta += F(",\"target\":");
    if (estado[i].objetivoWebValido)
    {
      respuesta += String(estado[i].objetivoWeb, 2);
    }
    else
    {
      respuesta += F("null");
    }
    respuesta += F(",\"webTarget\":");
    respuesta += estado[i].objetivoWebValido ? F("true") : F("false");
    respuesta += F(",\"calibrated\":");
    respuesta += estado[i].calibrado ? F("true") : F("false");
    respuesta += F(",\"calibrating\":");
    respuesta += estado[i].calibrando ? F("true") : F("false");
    respuesta += F(",\"calibrationError\":");
    respuesta += estado[i].calibracionError ? F("true") : F("false");
    respuesta += F(",\"rangeMin\":");
    respuesta += String(estado[i].rangoMin, 2);
    respuesta += F(",\"rangeMax\":");
    respuesta += String(estado[i].rangoMax, 2);
    respuesta += F(",\"calibrationProgress\":");
    respuesta += String(estado[i].progresoCalibracion, 2);
    respuesta += '}';
  }
  respuesta += F("]}");
  server.send(200, "application/json", respuesta);
}

void handleCommand()
{
  if (!server.hasArg("motor") || !server.hasArg("angle"))
  {
    server.send(400, "application/json", "{\"error\":\"Parámetros motor y angle requeridos\"}");
    return;
  }

  int motor = server.arg("motor").toInt();
  float angulo = server.arg("angle").toFloat();

  if (motor < 1 || motor > NUM_MOTORES)
  {
    server.send(400, "application/json", "{\"error\":\"Motor fuera de rango\"}");
    return;
  }

  uint8_t motorIndex = static_cast<uint8_t>(motor - 1);

  if (!enviarObjetivo(motorIndex, angulo, true))
  {
    server.send(502, "application/json", "{\"error\":\"Falló la comunicación I2C\"}");
    return;
  }

  objetivosLocales[motorIndex] = angulo;
  objetivoLocalValido[motorIndex] = true;

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleNotFound()
{
  server.send(404, "application/json", "{\"error\":\"No encontrado\"}");
}

// =================== COMUNICACIÓN I2C ===================

bool enviarObjetivo(uint8_t motor, float angulo, bool reiniciar)
{
  union
  {
    float valor;
    uint8_t bytes[sizeof(float)];
  } conversion;

  conversion.valor = angulo;

  Wire.beginTransmission(ARDUINO_I2C_ADDRESS);
  Wire.write(COMANDO_I2C_OBJETIVO);
  Wire.write(motor);
  Wire.write(conversion.bytes, sizeof(float));
  Wire.write(reiniciar ? 1 : 0);
  uint8_t resultado = Wire.endTransmission();
  if (resultado != 0)
  {
    Serial.print(F("Error I2C al enviar objetivo: "));
    Serial.println(resultado);
    return false;
  }
  return true;
}

bool obtenerEstadoMotores(MotorSnapshot (&estado)[NUM_MOTORES], uint8_t &reportados)
{
  reportados = 0;
  size_t bytesRecibidos = Wire.requestFrom(static_cast<int>(ARDUINO_I2C_ADDRESS), static_cast<int>(ESTADO_BUFFER_LENGTH));
  if (bytesRecibidos != ESTADO_BUFFER_LENGTH)
  {
    while (Wire.available())
    {
      Wire.read();
    }
    Serial.println(F("Lectura de estado incompleta."));
    return false;
  }

  if (!Wire.available())
  {
    return false;
  }

  reportados = Wire.read();
  if (reportados > NUM_MOTORES)
  {
    reportados = NUM_MOTORES;
  }

  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    MotorSnapshot snapshot;
    uint8_t flags = Wire.available() ? Wire.read() : 0;
    snapshot.encoderDetectado = (flags & 0x01) != 0;
    snapshot.objetivoVigente = (flags & 0x02) != 0;
    snapshot.enCorreccion = (flags & 0x04) != 0;
    snapshot.calibrado = (flags & 0x08) != 0;
    snapshot.calibrando = (flags & 0x10) != 0;
    snapshot.calibracionError = (flags & 0x20) != 0;

    union
    {
      float valor;
      uint8_t bytes[sizeof(float)];
    } conversion;

    Wire.readBytes(conversion.bytes, sizeof(float));
    snapshot.angulo = conversion.valor;

    Wire.readBytes(conversion.bytes, sizeof(float));
    snapshot.rangoMin = conversion.valor;

    Wire.readBytes(conversion.bytes, sizeof(float));
    snapshot.rangoMax = conversion.valor;

    Wire.readBytes(conversion.bytes, sizeof(float));
    snapshot.progresoCalibracion = conversion.valor;

    if (!snapshot.objetivoVigente)
    {
      objetivoLocalValido[i] = false;
    }

    snapshot.objetivoWebValido = objetivoLocalValido[i];
    if (snapshot.objetivoWebValido)
    {
      snapshot.objetivoWeb = objetivosLocales[i];
    }
    else
    {
      snapshot.objetivoWeb = 0.0f;
    }

    estado[i] = snapshot;
  }

  return true;
}
