/*
 * CleanBot Backend Server
 * Runs on Render — handles all robot API calls from both
 * the dashboard (browser) and the ESP32 firmware (HTTP client)
 */

const express    = require('express');
const cors       = require('cors');
const path       = require('path');

const app  = express();
const PORT = process.env.PORT || 3000;

app.use(cors());
app.use(express.json());

// Serve the dashboard HTML as static files
app.use(express.static(path.join(__dirname, '../public')));

// ── In-memory state (resets on server restart) ──────────────
let robotState = {
  robot_status : 'idle',      // 'idle' | 'running' | 'collecting'
  x            : 0,
  y            : 0,
  heading      : 0,
  relay        : false,
  bldc         : 0,           // 0-255 throttle
  servo        : 0,           // degrees
  last_seen    : null,
  last_message : '',
};

let commandQueue = [];        // Array of {type, ...params}
let commandHistory = [];      // Last 50 executed commands


// ════════════════════════════════════════════════════════════
//  DASHBOARD APIs  (called by the browser frontend)
// ════════════════════════════════════════════════════════════

// POST /api/robot/command  — manual drive command
app.post('/api/robot/command', (req, res) => {
  const { command, speed = 200, distance, degrees } = req.body;
  const validCmds = ['forward','backward','turn_left','turn_right','stop','servo_sweep'];
  if (!validCmds.includes(command)) {
    return res.status(400).json({ error: 'Invalid command' });
  }
  commandQueue.push({ type: command, speed, distance, degrees, ts: Date.now() });
  if (command !== 'stop') robotState.robot_status = 'running';
  else robotState.robot_status = 'idle';
  logHistory(command, { speed, distance, degrees });
  res.json({ ok: true, queued: commandQueue.length });
});

// POST /api/robot/relay  — toggle electromagnet
app.post('/api/robot/relay', (req, res) => {
  const { state: s } = req.body;
  robotState.relay = s === 'on';
  commandQueue.push({ type: 'relay', state: robotState.relay ? 'on' : 'off', ts: Date.now() });
  res.json({ ok: true, relay: robotState.relay });
});

// POST /api/robot/collect — trigger full collection cycle
app.post('/api/robot/collect', (req, res) => {
  const { duration = 30 } = req.body;
  robotState.robot_status = 'collecting';
  commandQueue.push({ type: 'relay', state: 'on', ts: Date.now() });
  commandQueue.push({ type: 'bldc_ramp', from: 0, to: 255, duration_ms: 5000, ts: Date.now() });
  commandQueue.push({ type: 'servo_sweep', from: 0, to: 90, ts: Date.now() });
  commandQueue.push({ type: 'hold', duration_ms: (duration - 15) * 1000, ts: Date.now() });
  commandQueue.push({ type: 'bldc_ramp', from: 255, to: 0, duration_ms: 3000, ts: Date.now() });
  commandQueue.push({ type: 'servo_sweep', from: 90, to: 0, ts: Date.now() });
  commandQueue.push({ type: 'relay', state: 'off', ts: Date.now() });
  logHistory('collection_cycle', { duration });
  res.json({ ok: true, queued: commandQueue.length });
});

// POST /api/robot/navigate — send parsed NLP command queue from dashboard
app.post('/api/robot/navigate', (req, res) => {
  const { commands } = req.body;
  if (!Array.isArray(commands) || commands.length === 0) {
    return res.status(400).json({ error: 'commands array required' });
  }
  commands.forEach(cmd => commandQueue.push({ ...cmd, ts: Date.now() }));
  robotState.robot_status = 'running';
  res.json({ ok: true, queued: commandQueue.length });
});

// GET /api/robot/history — last 50 executed commands
app.get('/api/robot/history', (req, res) => {
  res.json({ history: commandHistory });
});


// ════════════════════════════════════════════════════════════
//  ESP32 APIs  (called by the ESP32 firmware)
// ════════════════════════════════════════════════════════════

// GET /api/robot/status — ESP32 polls this to know if it should run
app.get('/api/robot/status', (req, res) => {
  res.json({
    robot_status : robotState.robot_status,
    x            : robotState.x,
    y            : robotState.y,
    heading      : robotState.heading,
    relay        : robotState.relay,
    bldc         : robotState.bldc,
    servo        : robotState.servo,
    last_message : robotState.last_message,
    uptime       : process.uptime(),
  });
});

// GET /api/robot/next_command — ESP32 fetches the next thing to do
app.get('/api/robot/next_command', (req, res) => {
  robotState.last_seen = new Date().toISOString();
  if (commandQueue.length === 0) {
    robotState.robot_status = 'idle';
    return res.json({ status: 'completed', command: null, remaining: 0 });
  }
  const cmd = commandQueue.shift();
  res.json({
    command   : cmd,
    remaining : commandQueue.length,
    status    : 'running',
  });
});

// POST /api/robot/position — ESP32 reports its real position
app.post('/api/robot/position', (req, res) => {
  const { x, y, heading } = req.body;
  if (x !== undefined) robotState.x = parseFloat(x);
  if (y !== undefined) robotState.y = parseFloat(y);
  if (heading !== undefined) robotState.heading = parseFloat(heading);
  robotState.last_seen = new Date().toISOString();
  res.json({ ok: true });
});

// POST /api/robot/message — ESP32 sends status messages to dashboard
app.post('/api/robot/message', (req, res) => {
  const { message } = req.body;
  robotState.last_message = message || '';
  logHistory('esp32_message', { message });
  res.json({ ok: true });
});

// POST /api/robot/component_state — ESP32 reports component states
app.post('/api/robot/component_state', (req, res) => {
  const { relay, bldc, servo } = req.body;
  if (relay  !== undefined) robotState.relay = relay;
  if (bldc   !== undefined) robotState.bldc  = bldc;
  if (servo  !== undefined) robotState.servo = servo;
  res.json({ ok: true });
});

// POST /api/robot/stop — emergency stop from dashboard
app.post('/api/robot/stop', (req, res) => {
  commandQueue = [];
  commandQueue.push({ type: 'stop', ts: Date.now() });
  commandQueue.push({ type: 'relay', state: 'off', ts: Date.now() });
  commandQueue.push({ type: 'bldc_ramp', from: 255, to: 0, duration_ms: 1000, ts: Date.now() });
  robotState.robot_status = 'idle';
  res.json({ ok: true, message: 'Emergency stop issued' });
});


// ════════════════════════════════════════════════════════════
//  HEALTH CHECK  (Render uses this to verify the service)
// ════════════════════════════════════════════════════════════
app.get('/health', (req, res) => {
  res.json({
    status     : 'ok',
    robot      : robotState.robot_status,
    queue_size : commandQueue.length,
    last_seen  : robotState.last_seen,
    uptime_sec : Math.round(process.uptime()),
  });
});

// Fallback — serve dashboard for any unmatched route
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, '../public/index.html'));
});


// ── Helpers ─────────────────────────────────────────────────
function logHistory(type, params) {
  commandHistory.unshift({ type, params, ts: new Date().toISOString() });
  if (commandHistory.length > 50) commandHistory.pop();
}


app.listen(PORT, () => {
  console.log(`CleanBot server running on port ${PORT}`);
  console.log(`Dashboard: http://localhost:${PORT}`);
  console.log(`Health:    http://localhost:${PORT}/health`);
});
