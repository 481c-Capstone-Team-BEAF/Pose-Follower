/* ═══════════════════════════════════════════
   Fishing Fun! — Hello Stretch Controller
   app.js
   ═══════════════════════════════════════════ */

// ── Configuration ─────────────────────────────────────────────
const CONFIG = {
  rosbridgeUrl:   'ws://localhost:9090',
  cameraStreamUrl: 'http://localhost:8080/stream?topic=/camera/color/image_raw',
  cameraTopic:     '/camera/color/image_raw',
  reconnectDelay:  3000,   // ms before retrying rosbridge connection
};

// ── State ──────────────────────────────────────────────────────
const state = {
  ros:          null,
  connected:    false,
  reconnecting: false,
  catches:      [],
  fps:          0,
  frameCount:   0,
  lastFpsTime:  performance.now(),
  robotStatus:  'IDLE',
  position:     '—',
  gripper:      '—',
  lastCmd:      '—',
};

// ── DOM refs ───────────────────────────────────────────────────
const $ = id => document.getElementById(id);

const DOM = {
  statusDot:    () => $('status-dot'),
  statusText:   () => $('status-text'),
  robotDot:     () => $('robot-dot'),
  robotText:    () => $('robot-text'),
  toastBar:     () => $('toast-bar'),
  toastIcon:    () => $('toast-icon'),
  toastText:    () => $('toast-text'),
  catchTray:    () => $('catch-tray'),
  logBox:       () => $('log-box'),
  cameraStream: () => $('camera-stream'),
  camPlaceholder:()=> $('cam-placeholder'),
  fpsDisplay:   () => $('fps-display'),
  scoreNum:     () => $('score-num'),
  srStatus:     () => $('sr-status'),
  srPosition:   () => $('sr-position'),
  srGripper:    () => $('sr-gripper'),
  srLastCmd:    () => $('sr-last-cmd'),
};

// ── Logger ─────────────────────────────────────────────────────
function log(msg, type = 'info') {
  const box  = DOM.logBox();
  const now  = new Date();
  const time = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;

  const entry = document.createElement('div');
  entry.className = `log-entry ${type}`;
  entry.innerHTML = `<span class="le-time">${time}</span><span class="le-msg">${msg}</span>`;
  box.appendChild(entry);

  // Keep last 60 entries
  while (box.children.length > 60) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

// ── Toast ──────────────────────────────────────────────────────
let toastTimer = null;
function toast(icon, msg, type = '', duration = 3000) {
  const bar = DOM.toastBar();
  DOM.toastIcon().textContent = icon;
  DOM.toastText().textContent = msg;
  bar.className = `toast-bar ${type}`;
  clearTimeout(toastTimer);
  // Reset to neutral after duration
  toastTimer = setTimeout(() => {
    bar.className = 'toast-bar';
    DOM.toastIcon().textContent = '🎣';
    DOM.toastText().textContent = 'Ready to fish!';
  }, duration);
}

// ── Robot State UI ─────────────────────────────────────────────
function setRobotStatus(status) {
  state.robotStatus = status;
  const el = DOM.srStatus();
  el.textContent = status;
  el.className = 'sr-value ' + status.toLowerCase();
}

function updateStateRow(id, value) {
  const el = $(id);
  if (el) el.textContent = value;
}

// ── ROS Bridge connection ──────────────────────────────────────
function connectRosbridge() {
  if (typeof ROSLIB === 'undefined') {
    log('roslibjs not loaded — running in demo mode', 'warn');
    setConnectionUI(false);
    return;
  }

  log(`Initializing — connecting to rosbridge…`, 'info');
  setConnectionUI('connecting');

  state.ros = new ROSLIB.Ros({ url: CONFIG.rosbridgeUrl });

  state.ros.on('connection', () => {
    state.connected = true;
    state.reconnecting = false;
    setConnectionUI(true);
    log(`Connected to rosbridge at ${CONFIG.rosbridgeUrl}`, 'info');
    setRobotStatus('IDLE');
    startCameraStream();
  });

  state.ros.on('error', (err) => {
    log(`WebSocket error`, 'error');
    setConnectionUI(false);
  });

  state.ros.on('close', () => {
    state.connected = false;
    setConnectionUI(false);
    setRobotStatus('IDLE');
    log(`Disconnected from rosbridge — retrying in ${CONFIG.reconnectDelay/1000}s…`, 'warn');
    if (!state.reconnecting) {
      state.reconnecting = true;
      setTimeout(connectRosbridge, CONFIG.reconnectDelay);
    }
  });
}

function setConnectionUI(status) {
  // status: true = online, false = offline, 'connecting' = connecting
  const dot  = DOM.statusDot();
  const text = DOM.statusText();
  if (status === true) {
    dot.className  = 'status-dot online';
    text.textContent = 'Connected';
  } else if (status === 'connecting') {
    dot.className  = 'status-dot connecting';
    text.textContent = 'Connecting…';
  } else {
    dot.className  = 'status-dot offline';
    text.textContent = 'Disconnected';
  }
}

// ── Camera stream ──────────────────────────────────────────────
function startCameraStream() {
  const img  = DOM.cameraStream();
  const ph   = DOM.camPlaceholder();
  img.src    = CONFIG.cameraStreamUrl;
  img.style.display = 'block';
  ph.style.display  = 'none';
  img.onerror = () => {
    img.style.display = 'none';
    ph.style.display  = 'flex';
    log('Camera stream unavailable', 'warn');
  };
  // FPS counter
  img.onload = () => {
    state.frameCount++;
    const now = performance.now();
    if (now - state.lastFpsTime >= 1000) {
      state.fps = state.frameCount;
      state.frameCount = 0;
      state.lastFpsTime = now;
      DOM.fpsDisplay().textContent = `${state.fps} fps`;
    }
  };
}

// ── Send ROS command ───────────────────────────────────────────
// Replace the service name/type with your actual ROS 2 service.
// Current stub calls /fishing_game/command (pose_follower/FishCommand).
function sendRosCommand(cmd) {
  if (!state.connected || !state.ros) return false;

  const client = new ROSLIB.Service({
    ros:         state.ros,
    name:        '/fishing_game/command',
    serviceType: 'pose_follower/FishCommand',
  });

  const req = new ROSLIB.ServiceRequest({ command: cmd });
  client.callService(req,
    (res) => {
      log(`Command '${cmd}' succeeded: ${res.message || 'ok'}`, 'info');
      setRobotStatus('IDLE');
    },
    (err) => {
      log(`Command '${cmd}' failed: ${err}`, 'error');
      setRobotStatus('ERROR');
      toast('❌', `Command failed: ${err}`, 'error', 4000);
    }
  );
  return true;
}

// ── Main command dispatcher ────────────────────────────────────
const FISH_META = {
  grab_red:    { label: 'Red Fish',    icon: '🐠', color: 'red'    },
  grab_blue:   { label: 'Blue Fish',   icon: '🐟', color: 'blue'   },
  grab_yellow: { label: 'Yellow Fish', icon: '🐡', color: 'yellow' },
  grab_green:  { label: 'Green Fish',  icon: '🐢', color: 'green'  },
};

window.sendCommand = function(cmd, toastIcon, toastMsg) {
  // Update last-cmd state row
  state.lastCmd = cmd;
  updateStateRow('sr-last-cmd', cmd.replace(/_/g, ' '));

  // Show toast
  toast(toastIcon, toastMsg, 'success');

  // Log
  log(`Sending command: ${cmd}`, 'info');

  // If it's a fish grab, log the catch
  if (cmd.startsWith('grab_')) {
    setRobotStatus('BUSY');
    const meta = FISH_META[cmd];
    if (meta) {
      setTimeout(() => {
        addCatch(meta);
        setRobotStatus('IDLE');
        log(`Caught ${meta.label}! 🎉`, 'info');
      }, 1200);
    }
  } else if (cmd === 'estop') {
    setRobotStatus('IDLE');
    toast('🛑', 'Emergency stop!', 'error', 4000);
    log('E-STOP triggered!', 'error');
  } else if (cmd === 'go_to_gameboard') {
    setRobotStatus('BUSY');
    updateStateRow('sr-position', 'Gameboard');
    setTimeout(() => setRobotStatus('IDLE'), 2500);
  } else if (cmd === 'return_home') {
    setRobotStatus('BUSY');
    updateStateRow('sr-position', 'Home');
    setTimeout(() => setRobotStatus('IDLE'), 2000);
  } else if (cmd === 'stow_arm') {
    setRobotStatus('BUSY');
    updateStateRow('sr-position', 'Stowed');
    setTimeout(() => setRobotStatus('IDLE'), 2000);
  }

  // Send to ROS (no-op if not connected)
  const sent = sendRosCommand(cmd);
  if (!sent) {
    log(`Demo mode — command '${cmd}' not sent`, 'warn');
  }
};

// ── Catch tray ─────────────────────────────────────────────────
function addCatch(meta) {
  state.catches.push(meta);
  renderCatch();
  DOM.scoreNum().textContent = state.catches.length;
}

function renderCatch() {
  const tray = DOM.catchTray();
  if (state.catches.length === 0) {
    tray.innerHTML = '<span class="catch-empty">No fish yet — get catching!</span>';
    return;
  }
  tray.innerHTML = state.catches
    .map(f => `<span class="catch-pill ${f.color}">${f.icon} ${f.label}</span>`)
    .join('');
}

// ── E-Stop (always works) ──────────────────────────────────────
window.eStop = function() {
  toast('🛑', 'Emergency stop sent!', 'error', 5000);
  log('E-STOP triggered!', 'error');
  setRobotStatus('IDLE');
  // Send even without connection check for safety
  if (state.ros) {
    const pub = new ROSLIB.Topic({
      ros:             state.ros,
      name:            '/e_stop',
      messageType:     'std_msgs/Bool',
    });
    pub.publish(new ROSLIB.Message({ data: true }));
  }
};

// ── Init ───────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  log('Fishing Fun! controller loaded 🎣', 'info');
  renderCatch();
  connectRosbridge();

  // Inject bubbles
  for (let i = 0; i < 6; i++) {
    const b = document.createElement('div');
    b.className = 'bubble';
    const size = 10 + Math.random() * 20;
    b.style.cssText = `
      width:${size}px; height:${size}px;
      left:${5 + Math.random() * 90}%;
      bottom:-30px;
      animation-duration:${10 + Math.random() * 14}s;
      animation-delay:${Math.random() * 10}s;
    `;
    document.body.appendChild(b);
  }
});