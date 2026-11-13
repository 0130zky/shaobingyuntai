let pitchSubmitted=0;
/* Native VS Code webview; no CDN, external network or serial access. */
const api = acquireVsCodeApi();
let nextId = 0, state, detail, selected = null, mode = 'shot', busy = false, hover = null, phaseSubmitted = 0;
const waiting = new Map(), colors = ['#ff7a5c', '#35c4a0', '#6b9dff'], wheels = ['12点轮', '4点轮', '8点轮'];
const $ = id => document.getElementById(id);
function call(method, params = {}) {
  return new Promise((resolve, reject) => {
    const id = ++nextId;
    const timer = setTimeout(() => { waiting.delete(id); reject(new Error('读取超时，请重新打开面板')); }, 65000);
    waiting.set(id, {resolve, reject, timer}); api.postMessage({id, method, params});
  });
}
window.addEventListener('message', event => {
  const response = event.data, request = waiting.get(response.id);
  if (!request) return;
  waiting.delete(response.id); clearTimeout(request.timer);
  response.error ? request.reject(new Error(response.error)) : request.resolve(response.result);
});
function error(message = '') { $('error').textContent = message; $('error').hidden = !message; }
function fmt(value, digits = 0) { return value == null || !Number.isFinite(value) ? '—' : value.toFixed(digits); }
// 拨盘 8 格、每格 45°。界面按顺时针为正（固件逆时针为正），多圈累计对应用户无意义，
// 只显示“当前物理位置落在一个 45° 格里的偏移”，它直接就是让当前位置成为格点所需的初始偏移。
function dialSlot() {
  const value = state?.latest?.dial_angle_deg;
  if (!Number.isFinite(value)) return null;
  const full = -value;
  let slot = full - 45 * Math.round(full / 45);
  if (Object.is(slot, -0)) slot = 0;
  return {full, slot};
}
function cell(row, text, className) { const item = document.createElement('td'); item.textContent = text; if (className) item.className = className; row.append(item); }
function setMode(next) {
  mode = next; $('modeLive').classList.toggle('active', mode === 'live'); $('modeShot').classList.toggle('active', mode === 'shot');
  $('metrics').hidden = mode === 'live'; $('zoom').disabled = mode === 'live'; $('deltaSection').hidden = mode === 'live';
  $('definitions').hidden = mode === 'live'; renderDetail(); draw();
}
async function choose(identity) { selected = identity; await fetchDetail(); renderList(); }
async function fetchDetail() {
  const id = selected ?? state?.shots.at(-1)?.id;
  if (id == null) { detail = null; renderDetail(); draw(); return; }
  if (detail?.id !== id) detail = await call('shot', {id});
  renderDetail(); draw();
}
function renderList() {
  const signature = (state.shots || []).map(s => s.id).join(',') + ':' + selected;
  if ($('shotList').dataset.signature === signature) return;
  $('shotList').dataset.signature = signature; $('shotList').replaceChildren();
  const target = state.status?.target_mps ?? 16.45, tolerance = state.status?.tolerance_mps ?? .05;
  for (const shot of [...state.shots].reverse()) {
    const row = document.createElement('tr');
    row.classList.toggle('selected', shot.id === (selected ?? state.shots.at(-1)?.id));
    cell(row, shot.count); cell(row, fmt(shot.speed, 3));
    cell(row, !shot.valid ? '窗口不足' : Math.abs(shot.speed - target) <= tolerance ? '达标' : shot.speed > target ? '偏高' : '偏低',
      Math.abs(shot.speed - target) <= tolerance ? 'good' : 'bad');
    row.onclick = () => choose(shot.id).catch(e => error(e.message)); $('shotList').append(row);
  }
}
function renderState() {
  const s = state.status || {}, age = state.data_age_s;
  $('connection').textContent = state.mode === 'replay' ? '历史回放 · 不下发串口控制' :
    state.loading ? '正在载入后台记录…' : age == null ? '等待后台采集数据' : age > 5 ? `暂无新采样（距上次 ${Math.round(age)} 秒）` : s.connected ? '后台采集在线 · VS Code共享数据' : '后台采集未连接';
  $('source').textContent = state.source || '还没有采集文件；先启动 tools/shot_monitor.sh';
  $('readout').replaceChildren();
  const latest = state.latest || {}, values = [latest.rpm_12h, latest.rpm_4h, latest.rpm_8h, latest.target_rpm];
  values.forEach((value, index) => {
    const card = document.createElement('div'); card.className = 'card';
    const title = document.createElement('span'); title.textContent = index < 3 ? wheels[index] : '共同目标';
    const number = document.createElement('strong'); number.textContent = fmt(value) + ' rpm';
    if (index < 3) number.className = 'wheel' + index;
    card.append(title, number); $('readout').append(card);
  });
  // 只显示槽位内偏移（±22.5°），多圈累计放 title，避免 -5263.8 这种没法用的数
  const dial = dialSlot();
  const slotText = dial == null ? '—' : `${dial.slot >= 0 ? '+' : ''}${dial.slot.toFixed(1)}°`;
  const dialCard = document.createElement('div'); dialCard.className = 'card';
  const dialTitle = document.createElement('span'); dialTitle.textContent = '拨盘槽位角度（顺时针+）';
  const dialNumber = document.createElement('strong'); dialNumber.textContent = slotText;
  dialCard.title = dial == null ? '' : `多圈累计 ${dial.full.toFixed(1)}°；一格 45°`;
  dialCard.append(dialTitle, dialNumber); $('readout').append(dialCard);
  $('dialAngle').textContent = dial == null ? '槽位角度（顺时针+） —'
    : `槽位角度（顺时针+） ${slotText}${Math.abs(dial.slot) > 10 ? '（超出±10°）' : ''}`;
  const pe=s.pitch_encoder;
  const pitchAge=pe ? (Date.now()/1000-pe.received_unix_s)*1000+pe.feedback_age_ms : Infinity;
  const pitchReplay = state.mode === 'replay';
  const hasPitch = !pitchReplay && pe;
  const pitchValue = hasPitch
    ? `原始码 ${pe.raw} / 65535 · ${fmt(pe.position_rad,4)} rad · ${fmt(pe.position_deg,2)}° · 速度 ${fmt(pe.velocity_dps,2)}°/s · ${pe.error_code===0?'失能':pe.error_code===1?'使能':'故障码 '+pe.error_code}`
    : '—';
  let pitchFeedback = '等待板端编码反馈…';
  if (pitchReplay) pitchFeedback = '回放模式';
  else if (hasPitch) {
    pitchFeedback = s.connected && pitchAge <= 500
      ? `实时反馈 · ${Math.max(0, Math.round(pitchAge))} ms`
      : '反馈过期 · 上次读数';
  }
  if ($('pitchEncoder').textContent !== pitchValue) $('pitchEncoder').textContent = pitchValue;
  if ($('pitchFeedback').textContent !== pitchFeedback) $('pitchFeedback').textContent = pitchFeedback;
  const pr=s.pitch_limit_reply;
  if(pitchSubmitted && pr?.received_unix_s>=pitchSubmitted) {
    const msg={177:'下限已记录，请手动移到上限并记录',178:'上限已记录，请记录下限',190:'行程不符合57.75°±3°，已清除两点，请检查量程/机械位置后重录',191:'拒绝：需CH6关闭、遥控在线、Pitch已失能且反馈新鲜静止'};
    $('pitchStatus').textContent=msg[pr.code] || `两点核对通过：零点 ${(pr.value_mrad/1000).toFixed(3)} rad，方向 ${pr.code===179?'+1':'-1'}。仅预览，未写入配置。`;
    pitchSubmitted=0;
  } else if(pitchSubmitted && Date.now()/1000-pitchSubmitted>4) {
    $('pitchStatus').textContent='未收到板端确认；未确认标定成功。';pitchSubmitted=0;
  }
  const reply = s.dial_phase_reply;
  if (phaseSubmitted && reply?.received_unix_s >= phaseSubmitted) {
    $('phaseStatus').textContent = reply.accepted ? `板端已应用：顺时针 ${fmt(-reply.phase_deg, 2)}°，重新使能后生效；掉电固定写 DIAL_PHASE_DEFAULT_DEG = ${fmt(reply.phase_deg, 2)}f` : '板端拒绝：请先关闭CH6，再应用。'; phaseSubmitted = 0;
  } else if (phaseSubmitted && Date.now() / 1000 - phaseSubmitted > 4) {
    $('phaseStatus').textContent = '未收到板端确认，请检查下行TX→RX接线。'; phaseSubmitted = 0;
  }
  renderDiagnostics(latest);
  renderList();
}
function renderDiagnostics(latest) {
  $('diagnosticRows').replaceChildren();
  const available = latest.diag_version === 1 && Number.isFinite(latest.diag_age_ms) && latest.diag_age_ms <= 60;
  const reasons = [[1,'停轮/低目标'],[2,'目标不一致'],[4,'反馈缺失/超时'],[8,'反馈时差超过1ms'],[16,'速度超出同步范围'],[32,'差速处于30rpm死区'],[64,'参考爬升中/保护锁存']];
  const mask = latest.sync_block_mask || 0, count = latest.pid_window_cycles;
  $('diagnosticStatus').textContent = !available ? '此记录暂无有效诊断；需烧录新增固件并使用新版后台采集。' :
    `诊断距主采样 ${fmt(latest.diag_age_ms)}ms；PID周期 ${fmt(latest.pid_period_us)}µs（窗口最大 ${fmt(latest.pid_period_max_us)}µs），运行 ${fmt(latest.pid_runtime_us)}µs（最大 ${fmt(latest.pid_runtime_max_us)}µs）；反馈时差 ${fmt(latest.fb_skew_ms)}ms；同步：${mask === 0 ? '本拍已生效' : reasons.filter(([bit]) => mask & bit).map(([, text]) => text).join('、')}；窗口生效 ${fmt(latest.sync_active_cycles)}/${fmt(count)}拍。`;
  for (let w=0; w<3; w++) {
    const suffix=['12h','4h','8h'][w], row=document.createElement('tr');
    cell(row,wheels[w],'wheel'+w);
    for (const key of [`fb_age_${suffix}_ms`,`sync_trim_${suffix}_rpm`,`pid_ref_${suffix}_rpm`]) cell(row,available ? fmt(latest[key],key.startsWith('sync') ? 1 : 0) : '—');
    cell(row,available ? `${fmt(latest['limit_cycles_'+suffix])}/${fmt(count)}` : '—');
    $('diagnosticRows').append(row);
  }
}
function renderDetail() {
  $('metricRows').replaceChildren();
  if (mode === 'live') { $('detailTitle').textContent = '实时三轮叠加 · 最近8秒'; return; }
  if (!detail) { $('detailTitle').textContent = '等待发弹，或打开历史CSV'; return; }
  const contact = Number.isFinite(detail.contact_ms) && detail.contact_wheel >= 0
    ? ` · 最早接触 ${detail.contact_ms > 0 ? '+' : ''}${fmt(detail.contact_ms, 0)} ms（${wheels[detail.contact_wheel]}）` : '';
  $('detailTitle').textContent = `记录 ${detail.count} · 弹速 ${fmt(detail.speed, 3)} m/s${contact} · ${detail.note || '测量完成'}`;
  for (let w = 0; w < 3; w++) {
    const row = document.createElement('tr'); cell(row, wheels[w], 'wheel' + w);
    const values = [detail.bases[w], detail.mins[w], detail.drop_rpm[w], detail.drop[w] == null ? null : detail.drop[w] * 100,
      detail.after_rpm[w], detail.after_delta[w], detail.recovery_ms[w], detail.overshoot_rpm[w]];
    values.forEach((v, i) => cell(row, fmt(v, i === 3 ? 2 : i === 5 ? 1 : 0))); $('metricRows').append(row);
  }
}
function chart(canvas, series, limits, units) {
  const rect = canvas.getBoundingClientRect(); if (!rect.width) return;
  const ratio = window.devicePixelRatio || 1, width = rect.width, height = rect.height;
  canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio);
  const left = 62, right = width - 14, top = 12, bottom = height - 27;
  const visible = series.flatMap(line => line.points.filter(p => p[0] >= limits[0] && p[0] <= limits[1]).map(p => p[1])).filter(Number.isFinite);
  let low = visible.length ? Math.min(...visible) : 0, high = visible.length ? Math.max(...visible) : 1;
  const pad = Math.max(1, (high - low) * .10); low -= pad; high += pad;
  const x = v => left + (v - limits[0]) / (limits[1] - limits[0]) * (right - left);
  const y = v => bottom - (v - low) / (high - low) * (bottom - top);
  const foreground = getComputedStyle(document.body).getPropertyValue('--fg').trim() || '#aab';
  ctx.font = '10px sans-serif'; ctx.fillStyle = foreground; ctx.strokeStyle = '#687b8b40'; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const yy = top + (bottom - top) * i / 4; ctx.beginPath(); ctx.moveTo(left, yy); ctx.lineTo(right, yy); ctx.stroke();
    ctx.textAlign = 'right'; ctx.fillText(Math.round(high - (high - low) * i / 4).toString(), left - 7, yy + 3);
    const xx = left + (right - left) * i / 4; ctx.textAlign = 'center'; ctx.fillText((limits[0] + (limits[1] - limits[0]) * i / 4).toFixed(mode === 'live' ? 1 : 0), xx, height - 8);
  }
  ctx.save(); ctx.beginPath(); ctx.rect(left, top, right - left, bottom - top); ctx.clip();
  if (limits[0] <= 0 && limits[1] >= 0) { ctx.strokeStyle = '#aab6c880'; ctx.beginPath(); ctx.moveTo(x(0), top); ctx.lineTo(x(0), bottom); ctx.stroke(); }
  for (const line of series) {
    ctx.strokeStyle = colors[line.wheel]; ctx.lineWidth = line.dash ? 1.2 : 1.8; ctx.setLineDash(line.dash ? [5, 4] : []); ctx.beginPath();
    let drawing=false;
    for (const p of line.points) {
      if (!Number.isFinite(p[1])) { drawing=false; continue; }
      drawing ? ctx.lineTo(x(p[0]), y(p[1])) : ctx.moveTo(x(p[0]), y(p[1])); drawing=true;
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);
  if (hover != null) { ctx.strokeStyle = '#b7bfcb'; ctx.lineWidth = .7; ctx.beginPath(); ctx.moveTo(x(hover), top); ctx.lineTo(x(hover), bottom); ctx.stroke(); }
  ctx.restore();
  canvas._plot = {limits, left, right, series, units};
}
function draw() {
  const rpm = [], delta = [], current = [], command = $('command').checked;
  let limits = {full: [-300, 600], contact: [-20, 80], recover: [0, 200]}[$('zoom').value];
  if (mode === 'live') {
    const rows = state?.live || [], end = rows.at(-1)?.[0] || 0; limits = [-8, 0];
    for (let w = 0; w < 3; w++) {
      rpm.push({wheel: w, points: rows.map(r => [r[0] - end, r[1 + w]])});
      current.push({wheel: w, points: rows.map(r => [r[0] - end, r[8 + w]])});
      if (command) current.push({wheel: w, dash: true, points: rows.map(r => [r[0] - end, r[5 + w]])});
    }
  } else if (detail?.seg) {
    const seg = detail.seg;
    for (let w = 0; w < 3; w++) {
      rpm.push({wheel: w, points: seg.t.map((t, i) => [t * 1000, seg.rpm[i][w]])});
      delta.push({wheel: w, points: seg.t.map((t, i) => [t * 1000, seg.rpm[i][w] - detail.bases[w]])});
      current.push({wheel: w, points: seg.t.map((t, i) => [t * 1000, seg.current_raw[i][w]])});
      if (command) current.push({wheel: w, dash: true, points: seg.t.map((t, i) => [t * 1000, seg.command_raw[i][w]])});
    }
  }
  const diagnostic = mode === 'live' ? state?.diagnostic_live || [] :
    detail?.diagnostic_seg ? detail.diagnostic_seg.t.map((t,i) => [t*1000,...detail.diagnostic_seg.values[i]]) : [];
  const diagEnd = mode === 'live' ? state?.live?.at(-1)?.[0] || 0 : 0;
  const diagPoints = index => diagnostic.map(r => [r[0]-diagEnd, r[1] === 1 && Number.isFinite(r[25]) && r[25] <= 60 ? r[1+index] : null]);
  chart($('feedbackAge'), [0,1,2].map(w => ({wheel:w,points:diagPoints(6+w)})), limits, 'ms');
  chart($('syncTrim'), [0,1,2].map(w => ({wheel:w,points:diagPoints(11+w)})), limits, 'rpm');
  chart($('pidTiming'), [2,4,3].map((index,w) => ({wheel:w,label:['PID周期','PID运行耗时','窗口最大周期'][w],points:diagPoints(index)})), limits, 'µs');
  chart($('rpm'), rpm, limits, 'rpm'); chart($('delta'), delta, limits, 'Δrpm'); chart($('current'), current, limits, 'raw');
  $('axisLabel').textContent = mode === 'live' ? '横轴：距最新采样 / s' : '横轴：相对遥测掉速起点 / ms';
}
for (const id of ['rpm', 'delta', 'current', 'feedbackAge', 'syncTrim', 'pidTiming']) {
  $(id).addEventListener('mousemove', event => {
    const plot = $(id)._plot; if (!plot) return;
    hover = plot.limits[0] + (event.offsetX - plot.left) / (plot.right - plot.left) * (plot.limits[1] - plot.limits[0]);
    const values = plot.series.map(line => {
      const closest = line.points.reduce((best, point) => !best || Math.abs(point[0] - hover) < Math.abs(best[0] - hover) ? point : best, null);
      return closest ? `${line.label || wheels[line.wheel]}${line.dash ? '指令' : ''}: ${fmt(closest[1])} ${plot.units}` : '';
    });
    $(id).title = values.join('\n'); draw();
  });
  $(id).addEventListener('mouseleave', () => { hover = null; draw(); });
}
$('modeLive').onclick = () => setMode('live'); $('modeShot').onclick = () => setMode('shot');
$('zoom').onchange = draw; $('command').onchange = draw; window.addEventListener('resize', draw);
$('latest').onclick = () => choose(null).catch(e => error(e.message));
async function action(method) {
  if (busy) { setTimeout(() => action(method), 100); return; }
  busy = true; error();
  try {
    const result = await call(method);
    if (method === 'export') $('connection').textContent = `已导出 ${result.shots} 发逐轮统计`;
    else { state = result; selected = null; detail = null; renderState(); await fetchDetail(); }
  } catch (e) { error(e.message); } finally { busy = false; }
}
$('replay').onclick = () => action('replay'); $('live').onclick = () => action('live'); $('export').onclick = () => action('export');
$('apply').onclick = async () => {
  try { error(); const result = await call('phase', {clockwise: Number($('phase').value)}); phaseSubmitted = result.created_unix_s; $('phaseStatus').textContent = '已提交，等待板端确认…'; }
  catch (e) { error(e.message); }
};
$('alignDial').onclick = async () => {
  try {
    error();
    const dial = dialSlot();
    if (!dial) throw new Error('还没有拨盘角度数据');
    const clockwise = Number(dial.slot.toFixed(1));
    if (Math.abs(clockwise) > 10) throw new Error(`当前槽位角度 ${clockwise.toFixed(1)}° 超出初始偏移 ±10°；请先调整零点，让这个值进入 ±10° 再点`);
    $('phase').value = clockwise.toFixed(1);
    const result = await call('phase', {clockwise});
    phaseSubmitted = result.created_unix_s;
    $('phaseStatus').textContent = '已提交，等待板端确认…';
  } catch (e) { error(e.message); }
};
async function poll() {
  if (!busy) {
    busy = true;
    try { state = await call('state'); renderState(); await fetchDetail(); draw(); error(); }
    catch (e) { error(e.message); } finally { busy = false; }
  }
  setTimeout(poll, 500);
}
poll();

for(const [id,endpoint] of [['pitchLower',1],['pitchUpper',2]]) {
  $(id).onclick=async()=>{
    if(pitchSubmitted)return;
    try { const r=await call('pitch_limit',{endpoint});pitchSubmitted=r.created_unix_s;$('pitchStatus').textContent='等待板端确认…'; }
    catch(e){$('pitchStatus').textContent=e.message;}
  };
}
