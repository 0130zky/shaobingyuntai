/* Native VS Code webview. No CDN, no external network, no serial access here. */
const api = acquireVsCodeApi();
const $ = id => document.getElementById(id);
const FOLD = 'fold', PITCH = 'pitch';
const ERRORS = {0: '失能', 1: '使能', 8: '过压', 9: '欠压', 10: '过流', 11: 'MOS过温',
  12: '线圈过温', 13: '通讯丢失', 14: '过载'};
const Z_MODES = ['失能', '准备折叠Pitch', '正在折叠', '展开完成', '折叠完成', '正在展开', '展开后Pitch调平', '保持'];
const FAULTS = {0: '无', 1: '遥控失联', 2: '周期异常', 3: '反馈过期', 4: '驱动状态/温度',
  6: '发送失败', 7: '堵转/移动超时', 9: '右杆未回中', 10: '阶段超时', 11: '配置/输入无效'};
const SWITCHES = {1: '上', 2: '下', 3: '中'};
let nextId = 0, state = null, busy = false, autoTried = false;
let hoverT = null, plot = null, lastLogKey = '';
const waiting = new Map();

function call(method, params = {}) {
  return new Promise((resolve, reject) => {
    const id = ++nextId;
    const timer = setTimeout(() => { waiting.delete(id); reject(new Error('读取超时')); }, 30000);
    waiting.set(id, {resolve, reject, timer});
    api.postMessage({id, method, params});
  });
}
window.addEventListener('message', event => {
  const response = event.data, request = waiting.get(response.id);
  if (!request) return;
  waiting.delete(response.id); clearTimeout(request.timer);
  response.error ? request.reject(new Error(response.error)) : request.resolve(response.result);
});

function error(message = '') { $('error').textContent = message; $('error').hidden = !message; }
function num(value) { return Number.isFinite(value) ? String(Math.round(value)) : '—'; }
function deg(value) { return Number.isFinite(value) ? `${value.toFixed(2)}°` : '—'; }
function signed(value) { return Number.isFinite(value) ? `${value >= 0 ? '+' : ''}${Math.round(value)}` : '—'; }
function temp(value) { return Number.isFinite(value) ? `${value} °C` : '—'; }
function ageText(ms, seen) {
  if (!seen || !Number.isFinite(ms) || ms >= 0xFFFFFFFF) return '未收到';
  return `${Math.round(ms)} ms`;
}
function rate(history) {
  if (!history || history.length < 2) return null;
  const first = history[0], last = history.at(-1);
  const span = last.t - first.t;
  return span > 0 ? (history.length - 1) / span : null;
}

function renderConnection() {
  const connected = !!state.connected, hostAge = state.data_age_s;
  const hz = rate(state.history);
  let text;
  if (!connected) text = state.error ? `未连接 · ${state.error}` : '未连接 · 选择串口后点“连接”';
  else if (hostAge == null) text = `已连接 ${state.port} · 等待状态行…`;
  else if (hostAge > 1) text = `已连接 ${state.port} · ${hostAge.toFixed(1)} s 没有新状态行`;
  else text = `已连接 ${state.port} · ${hz == null ? '' : `约 ${hz.toFixed(0)} Hz · `}已收 ${state.received} 行`;
  $('connection').textContent = text;
  $('toggle').textContent = connected ? '断开' : '连接';
}

function renderPorts() {
  const ports = state.ports || [], select = $('port');
  const signature = ports.join('|');
  if (select.dataset.signature !== signature) {
    select.replaceChildren();
    for (const port of ports) {
      const option = document.createElement('option');
      option.value = port; option.textContent = port; select.append(option);
    }
    select.dataset.signature = signature;
  }
  const selected = state.port || ports[0] || '';
  if (selected && ![...select.options].some(option => option.value === selected)) {
    const option = document.createElement('option');
    option.value = selected; option.textContent = selected; select.append(option);
  }
  select.value = selected;
  select.disabled = ports.length === 0 && !state.port;
}

function axisHealth(axis, hostAge) {
  if (!axis) return ['等待反馈', ''];
  if (!axis.seen) return ['无电机反馈', 'bad'];
  if (hostAge != null && hostAge > 1) return ['状态行中断', 'warn'];
  if (axis.age_ms > 250) return ['反馈过期', 'warn'];
  if (axis.error_code > 1) return ['驱动器故障', 'bad'];
  return ['反馈正常', 'good'];
}

function renderAxis(prefix, axis, hostAge) {
  const badge = $(`${prefix}Health`), health = axisHealth(axis, hostAge);
  badge.textContent = health[0];
  badge.className = `badge ${health[1]}`.trim();
  $(`${prefix}Raw`).textContent = axis ? num(axis.raw) : '—';
  $(`${prefix}Meter`).style.width = axis ? `${Math.min(100, Math.max(0, axis.raw / 65535 * 100))}%` : '0%';
  $(`${prefix}Deg`).textContent = axis ? deg(axis.deg) : '—';
  $(`${prefix}Goal`).textContent = axis && Number.isFinite(axis.goal_deg)
    ? `${deg(axis.goal_deg)} / ${num(axis.goal_raw)}` : '—';
  $(`${prefix}Error`).textContent = axis ? signed(axis.error_raw) : '—';
  $(`${prefix}Error`).style.color = axis && Math.abs(axis.error_raw) > 96 ? 'var(--critical)' : '';
  $(`${prefix}Age`).textContent = axis ? ageText(axis.age_ms, axis.seen) : '—';
  $(`${prefix}Drive`).textContent = axis && Number.isFinite(axis.error_code)
    ? (ERRORS[axis.error_code] || `故障码 ${axis.error_code}`) : '—';
  $(`${prefix}Temp`).textContent = axis ? temp(axis.temp_c) : '—';
}

function renderStatus() {
  const latest = state.latest;
  if (!latest) {
    for (const id of ['modeChip', 'switchChip', 'runChip', 'faultChip', 'seqChip']) {
      $(id).textContent = id === 'modeChip' ? '模式 —' : id === 'switchChip' ? 'S1 — · S2 —'
        : id === 'runChip' ? '板端状态 —' : id === 'faultChip' ? '故障 —' : 'seq —';
    }
    return;
  }
  $('modeChip').textContent = `模式 ${Z_MODES[latest.z_mode] ?? latest.z_mode}`;
  $('switchChip').textContent = `S1 ${SWITCHES[latest.s1] ?? '—'} · S2 ${SWITCHES[latest.s2] ?? '—'}`;
  const run = [];
  run.push(latest.valid ? '标定有效' : '标定无效');
  run.push(latest.armed ? '控制就绪' : '控制未就绪');
  run.push(latest.rc ? '遥控在线' : '遥控失联');
  run.push(latest.master ? '主控允许' : '主控关闭');
  run.push(latest.cal ? '标定模式' : '运行模式');
  $('runChip').textContent = run.join(' · ');
  const fault = latest.fault ?? 0;
  $('faultChip').textContent = `故障 ${FAULTS[fault] || fault}`;
  $('faultChip').style.color = fault ? 'var(--critical)' : '';
  $('seqChip').textContent = `seq ${latest.seq ?? '—'} · 无效行 ${state.invalid}`;
}

const PHASES = {build: '编译', flash: '烧录', done: '结束'};

function renderJob(job) {
  const running = !!job?.running;
  for (const id of ['buildBtn', 'flashBtn', 'programBtn']) $(id).disabled = running;
  $('cancelBtn').hidden = !running;

  let text;
  if (!job) text = '未运行';
  else if (running) text = `正在${PHASES[job.phase] || job.phase}… ${job.duration_s.toFixed(0)} s`;
  else if (job.error) text = job.error;
  else if (job.exit_code === 0) text = `完成 · ${job.duration_s.toFixed(1)} s`;
  else text = `失败（退出码 ${job.exit_code}）· ${job.duration_s.toFixed(1)} s`;
  $('buildState').textContent = text;
  $('buildState').style.color = !job || running ? '' : job.exit_code === 0 ? 'var(--good)' : 'var(--critical)';

  const log = $('buildLog');
  const key = job ? job.output.map(row => row.t + row.text).join('\n') : '';
  if (key !== lastLogKey) {
    lastLogKey = key;
    const nearBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 24;
    log.textContent = job && job.output.length
      ? job.output.map(row => `${row.t}  ${row.text}`).join('\n')
      : '尚未运行编译或烧录。';
    const tone = !job || running ? '' : job.exit_code === 0 ? 'done' : job.error === '已取消' ? '' : 'failed';
    log.className = `build-log ${tone}`.trim();
    if (nearBottom) log.scrollTop = log.scrollHeight;
  }
}

function render() {
  if (!state) return;
  renderConnection();
  renderPorts();
  const latest = state.latest, hostAge = state.data_age_s;
  renderAxis(FOLD, latest?.fold, hostAge);
  renderAxis(PITCH, latest?.pitch, hostAge);
  renderStatus();
  renderJob(state.job);
  renderDT7();
  draw();
}

function css(name) { return getComputedStyle(document.body).getPropertyValue(name).trim(); }

function niceStep(value) {
  const power = 10 ** Math.floor(Math.log10(Math.max(1, value)));
  const n = value / power;
  return (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * power;
}
function tickText(value) {
  return Math.abs(value) >= 10000 ? `${(value / 1000).toFixed(value % 1000 ? 1 : 0)}K` : String(Math.round(value));
}

function draw() {
  const canvas = $('chart'), rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = window.devicePixelRatio || 1;
  const width = rect.width, height = rect.height;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const history = state?.history || [];
  const surface = css('--surface'), primary = css('--text'), muted = css('--secondary');
  const grid = css('--grid'), axisColor = css('--axis');
  const foldColor = css('--series-fold'), pitchColor = css('--series-pitch');
  const left = 58, right = width - (width >= 480 ? 96 : 16), top = 12, bottom = height - 26;

  ctx.font = '10px ' + (getComputedStyle(document.body).fontFamily || 'sans-serif');
  if (history.length < 2) {
    ctx.fillStyle = muted; ctx.textAlign = 'center';
    ctx.fillText(state?.connected ? '等待板端状态行…' : '未连接', width / 2, height / 2);
    plot = null;
    return;
  }

  const end = history.at(-1).t, xmin = -30, xmax = 0;
  const visible = history.filter(row => row.t >= end + xmin);
  const values = [];
  for (const row of visible) {
    for (const key of ['fold_raw', 'pitch_raw', 'fold_goal_raw', 'pitch_goal_raw']) {
      if (Number.isFinite(row[key])) values.push(row[key]);
    }
  }
  let low = Math.min(...values), high = Math.max(...values);
  if (!values.length) { low = 0; high = 65535; }
  let span = high - low;
  if (span < 200) { const mid = (high + low) / 2; low = mid - 100; high = mid + 100; span = 200; }
  low = Math.max(0, low); high = Math.min(65535, high);
  const step = niceStep(span / 4);
  low = Math.max(0, Math.floor(low / step) * step);
  high = Math.min(65535, Math.ceil(high / step) * step);
  if (high <= low) high = low + step;

  const x = t => left + (t - xmin) / (xmax - xmin) * (right - left);
  const y = v => bottom - (v - low) / (high - low) * (bottom - top);

  ctx.strokeStyle = grid; ctx.lineWidth = 1;
  ctx.fillStyle = muted; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  const ticks = Math.max(1, Math.floor((high - low) / step + 1e-9));
  for (let i = 0; i <= ticks; i++) {
    const value = low + step * i, yy = y(value);
    ctx.beginPath(); ctx.moveTo(left, yy); ctx.lineTo(right, yy); ctx.stroke();
    ctx.fillText(tickText(value), left - 7, yy);
  }
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (let i = 0; i <= 3; i++) {
    const value = xmin + (xmax - xmin) * i / 3, xx = x(value);
    ctx.fillText(value === 0 ? '0 s' : `${value}`, xx, height - 15);
  }
  ctx.strokeStyle = axisColor;
  ctx.beginPath(); ctx.moveTo(left, bottom); ctx.lineTo(right, bottom); ctx.stroke();

  ctx.save();
  ctx.beginPath(); ctx.rect(left, top, right - left, bottom - top); ctx.clip();
  const lines = [
    {key: 'fold_goal_raw', color: foldColor, dash: true},
    {key: 'pitch_goal_raw', color: pitchColor, dash: true},
    {key: 'fold_raw', color: foldColor, dash: false},
    {key: 'pitch_raw', color: pitchColor, dash: false},
  ];
  for (const line of lines) {
    ctx.beginPath();
    ctx.strokeStyle = line.color;
    ctx.globalAlpha = line.dash ? .55 : 1;
    ctx.lineWidth = line.dash ? 1 : 2;
    ctx.lineJoin = ctx.lineCap = 'round';
    ctx.setLineDash(line.dash ? [5, 4] : []);
    let started = false;
    for (const row of visible) {
      const value = row[line.key];
      if (!Number.isFinite(value)) { started = false; continue; }
      const px = x(row.t - end), py = y(value);
      started ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      started = true;
    }
    ctx.stroke();
  }
  ctx.setLineDash([]); ctx.globalAlpha = 1;

  const last = visible.at(-1);
  if (hoverT != null) {
    const px = x(hoverT - end);
    ctx.strokeStyle = axisColor; ctx.lineWidth = .8;
    ctx.beginPath(); ctx.moveTo(px, top); ctx.lineTo(px, bottom); ctx.stroke();
  }
  ctx.restore();

  // End dots are markers, not data spans: draw them unclipped so the ring stays round.
  for (const [key, color] of [['fold_raw', foldColor], ['pitch_raw', pitchColor]]) {
    if (!Number.isFinite(last[key])) continue;
    const px = x(last.t - end), py = y(last[key]);
    ctx.beginPath(); ctx.fillStyle = surface; ctx.arc(px, py, 6, 0, Math.PI * 2); ctx.fill();
    ctx.beginPath(); ctx.fillStyle = color; ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
  }

  // Direct end labels supplement the legend; text stays in ink, not series color.
  if (width >= 480) {
    const labels = [];
    for (const [key, name] of [['fold_raw', '折叠'], ['pitch_raw', 'Pitch']]) {
      if (Number.isFinite(last[key])) labels.push({text: `${name} ${Math.round(last[key])}`, y: y(last[key])});
    }
    if (labels.length === 2 && Math.abs(labels[0].y - labels[1].y) < 14) {
      const highLabel = labels[0].y <= labels[1].y ? labels[0] : labels[1];
      const lowLabel = highLabel === labels[0] ? labels[1] : labels[0];
      highLabel.y -= 8; lowLabel.y += 8;
    }
    ctx.fillStyle = primary; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    for (const label of labels) {
      const yy = Math.min(bottom - 4, Math.max(top + 4, label.y));
      ctx.fillText(label.text, right + 10, yy);
    }
  }

  plot = {left, right, top, bottom, end, xmin, xmax, visible};
}

function showTooltip(event) {
  if (!plot) return;
  const px = event.offsetX;
  if (px < plot.left || px > plot.right) { hideTooltip(); return; }
  const t = plot.xmin + (px - plot.left) / (plot.right - plot.left) * (plot.xmax - plot.xmin);
  hoverT = plot.end + t;
  let nearest = null;
  for (const row of plot.visible) {
    if (!nearest || Math.abs(row.t - hoverT) < Math.abs(nearest.t - hoverT)) nearest = row;
  }
  if (!nearest) { hideTooltip(); return; }
  hoverT = nearest.t;
  const relative = nearest.t - plot.end;
  // History carries raw counts only; degrees shown in the cards are current values.
  $('tooltip').innerHTML =
    `<strong>${relative.toFixed(1)} s</strong><br>` +
    `折叠 ${num(nearest.fold_raw)}<br>` +
    `Pitch ${num(nearest.pitch_raw)}`;
  $('tooltip').hidden = false;
  const wrapWidth = $('chart').clientWidth;
  const tipWidth = $('tooltip').offsetWidth || 150;
  const left = Math.min(wrapWidth - tipWidth - 4, Math.max(4, event.offsetX + 12));
  $('tooltip').style.left = `${left}px`;
  $('tooltip').style.top = `${Math.max(4, event.offsetY - 44)}px`;
  draw();
}
function hideTooltip() {
  hoverT = null; $('tooltip').hidden = true; draw();
}

async function action(method, params) {
  if (busy) return;
  busy = true;
  try {
    state = await call(method, params);
    autoTried = true;
    error('');
    render();
  } catch (e) { error(e.message); }
  finally { busy = false; }
}

async function poll() {
  if (!busy) {
    busy = true;
    try {
      state = await call('state');
      render();
      if (!state.connected && !autoTried) {
        autoTried = true;
        if ((state.ports || []).length) {
          try { state = await call('connect', {port: ''}); error(''); render(); }
          catch (e) { error(e.message); }
        }
      }
    } catch (e) { error(e.message); }
    finally { busy = false; }
  }
  setTimeout(poll, 250);
}

$('toggle').onclick = () => action(state?.connected ? 'disconnect' : 'connect', {port: $('port').value});
$('refresh').onclick = async () => {
  try {
    const result = await call('ports');
    if (state) {
      state.ports = result.ports;
      renderPorts();
      if (!state.connected && result.ports.length) {
        try { state = await call('connect', {port: ''}); error(''); render(); }
        catch (e) { error(e.message); }
      }
    }
  } catch (e) { error(e.message); }
};
$('port').onchange = () => { if (state?.connected) action('connect', {port: $('port').value}); };
$('buildBtn').onclick = () => { releaseKeys(); action('build'); };
$('flashBtn').onclick = () => { releaseKeys(); action('flash'); };
$('programBtn').onclick = () => { releaseKeys(); action('program'); };
$('cancelBtn').onclick = () => action('cancel');
$('chart').addEventListener('mousemove', showTooltip);
$('chart').addEventListener('mouseleave', hideTooltip);
window.addEventListener('resize', draw);
new MutationObserver(() => { if (state) draw(); })
  .observe(document.body, {attributes: true, attributeFilter: ['class']});

let heldKeys = 0, inputPending = false;
function renderDT7() {
  const relay = state.dt7 || {}, select = $('dt7Port');
  if (select.dataset.signature !== (state.ports || []).join('|')) {
    const previous = select.value;
    select.replaceChildren();
    for (const port of state.ports || []) {
      const option = document.createElement('option'); option.value = option.textContent = port; select.append(option);
    }
    select.value = relay.port || previous || (state.ports || []).find(p => p.includes('0001A0000000')) || '';
    select.dataset.signature = (state.ports || []).join('|');
  }
  select.disabled = !!relay.connected;
  $('dt7Toggle').textContent = relay.connected ? '断开 DT7' : '连接 DT7';
  const sample = state.latest;
  $('dt7State').textContent = relay.error || (relay.connected
    ? `已连接 ${relay.port} · 921600 · 已发 ${relay.frames} 帧 · 按键 0x${(relay.keys || 0).toString(16)}` : 'DT7 未连接')
    + ` · 板端键盘 ${sample?.keys == null ? '等待遥测' : '0x' + sample.keys.toString(16)} · 鼠标 ${sample?.mouse?.join(', ') || '—'}`;
  if (!relay.connected || state.job?.running) heldKeys = 0;
  for (const button of document.querySelectorAll('[data-key]')) {
    button.disabled = !relay.connected || !!state.job?.running;
    button.classList.toggle('primary', !!(heldKeys & Number(button.dataset.key)));
  }
}
async function sendKeys() {
  if (inputPending || !state?.dt7?.connected || state.job?.running) return;
  inputPending = true;
  try { await call('dt7_input', {keys: heldKeys}); }
  catch (e) { heldKeys = 0; error(e.message); }
  finally { inputPending = false; }
}
function releaseKeys() {
  heldKeys = 0;
  if (state?.dt7?.connected) call('dt7_input', {keys: 0}).catch(e => error(e.message));
}
$('dt7Toggle').onclick = () => {
  releaseKeys(); action(state?.dt7?.connected ? 'dt7_disconnect' : 'dt7_connect', {port: $('dt7Port').value});
};
$('dt7Stop').onclick = releaseKeys;
const keyBits = {KeyW: 1, KeyS: 2, KeyA: 4, KeyD: 8};
$('keyPad').addEventListener('keydown', e => {
  const bit = keyBits[e.code];
  if (!bit || !state?.dt7?.connected || state.job?.running) return;
  e.preventDefault(); heldKeys |= bit; sendKeys();
});
window.addEventListener('keyup', e => {
  const bit = keyBits[e.code]; if (bit) { heldKeys &= ~bit; sendKeys(); }
});
for (const button of document.querySelectorAll('[data-key]')) {
  button.addEventListener('pointerdown', e => {
    e.preventDefault(); button.focus(); button.setPointerCapture(e.pointerId);
    heldKeys |= Number(button.dataset.key); sendKeys();
  });
  for (const event of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(event, () => {
    heldKeys &= ~Number(button.dataset.key); sendKeys();
  });
}
$('keyPad').addEventListener('focusout', e => { if (!$('keyPad').contains(e.relatedTarget)) releaseKeys(); });
window.addEventListener('blur', releaseKeys);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseKeys(); });
setInterval(() => { if (!document.hidden) sendKeys(); }, 75);
poll();
