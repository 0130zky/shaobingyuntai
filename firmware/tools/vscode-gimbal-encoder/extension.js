const vscode = require('vscode');
const fs = require('fs');
const path = require('path');
const cp = require('child_process');
const crypto = require('crypto');
const readline = require('readline');
let panel, backend, output, sequence = 0;
const pending = new Map();

function locateRoot() {
  const roots = (vscode.workspace.workspaceFolders || []).map(f => f.uri.fsPath);
  const file = vscode.window.activeTextEditor?.document.uri.fsPath;
  if (file) {
    let parent = path.dirname(file);
    for (let i = 0; i < 5; i++) { roots.push(parent); parent = path.dirname(parent); }
  }
  for (const root of roots) {
    for (const suffix of ['', 'firmware']) {
      const candidate = path.join(root, suffix);
      if (fs.existsSync(path.join(candidate, 'tools/gimbal_encoder_vscode.py'))) return candidate;
    }
  }
  throw new Error('请先打开包含哨兵折叠云台固件的工程目录（firmware/tools/gimbal_encoder_vscode.py）。');
}

function startBackend(root) {
  if (backend) return;
  const candidates = process.platform === 'win32'
    ? [path.join(root, '.venv/Scripts/python.exe'), path.join(root, 'build/host-tests/Scripts/python.exe')]
    : [path.join(root, '.venv/bin/python'), path.join(root, 'build/host-tests/bin/python')];
  const python = candidates.find(candidate => fs.existsSync(candidate));
  if (!python) throw new Error('请先创建工程.venv并安装tools/requirements.txt。');
  backend = cp.spawn(python, ['-u', path.join(root, 'tools/gimbal_encoder_vscode.py')],
    {cwd: root, windowsHide: true, env: {...process.env, PYTHONUTF8: '1'}, stdio: ['pipe', 'pipe', 'pipe']});
  const child = backend;
  readline.createInterface({input: child.stdout}).on('line', line => {
    try {
      const response = JSON.parse(line), job = pending.get(response.id);
      if (job) {
        pending.delete(response.id); clearTimeout(job.timer);
        response.error ? job.reject(new Error(response.error)) : job.resolve(response.result);
      }
    } catch (error) { output.appendLine(String(error)); }
  });
  child.stderr.on('data', data => output.append(data.toString()));
  function fail(error) {
    if (backend !== child) return;
    backend = undefined;
    for (const job of pending.values()) { clearTimeout(job.timer); job.reject(error); }
    pending.clear();
  }
  child.on('error', fail);
  child.on('exit', () => fail(new Error('编码器后台已停止，关闭并重新打开面板即可恢复。')));
}

function rpc(method, params = {}) {
  return new Promise((resolve, reject) => {
    if (!backend) return reject(new Error('编码器后台未运行'));
    const id = ++sequence;
    const timer = setTimeout(() => { pending.delete(id); reject(new Error('读取超时，请重新打开面板')); }, 30000);
    pending.set(id, {resolve, reject, timer});
    backend.stdin.write(JSON.stringify({id, method, params}) + '\n');
  });
}

function stop() {
  const child = backend; backend = undefined;
  for (const job of pending.values()) { clearTimeout(job.timer); job.reject(new Error('面板已关闭')); }
  pending.clear(); if (child) child.kill();
}

async function open(context) {
  try {
    if (panel) { panel.reveal(); return; }
    const root = locateRoot(); startBackend(root);
    panel = vscode.window.createWebviewPanel('robot_templateGimbal', '云台编码器 · 折叠 / Pitch', vscode.ViewColumn.Beside,
      {enableScripts: true, retainContextWhenHidden: true,
       localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'media')]});
    const webview = panel.webview;
    const nonce = crypto.randomBytes(24).toString('base64');
    let html = fs.readFileSync(path.join(context.extensionPath, 'media/index.html'), 'utf8');
    html = html.replaceAll('{{cspSource}}', webview.cspSource).replaceAll('{{nonce}}', nonce)
      .replaceAll('{{style}}', webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media/style.css')).toString())
      .replaceAll('{{script}}', webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media/dashboard.js')).toString());
    webview.html = html;
    panel.onDidChangeViewState(e => { if (!e.webviewPanel.visible) rpc('dt7_input', {keys: 0}).catch(() => {}); });
    panel.onDidDispose(() => { panel = undefined; stop(); });
    webview.onDidReceiveMessage(async message => {
      const allowed = ['state', 'connect', 'disconnect', 'ports', 'build', 'flash', 'program', 'cancel', 'dt7_connect', 'dt7_disconnect', 'dt7_input'];
      if (!allowed.includes(message.method)) {
        webview.postMessage({id: message.id, error: 'Unsupported method: ' + message.method}); return;
      }
      if (message.method === 'flash' || message.method === 'program') {
        const label = message.method === 'flash' ? '编译并烧录' : '仅烧录现有固件';
        const detail = message.method === 'flash' ? '重新编译后会写入并校验' : '会写入当前 build/SentryH7 中的固件并校验';
        const choice = await vscode.window.showWarningMessage(
          `${label}：${detail}，随后复位运行。板子会重启，电机可能动作。`,
          {modal: true}, '确认烧录');
        if (choice !== '确认烧录') {
          try { webview.postMessage({id: message.id, result: await rpc('state')}); }
          catch (error) { webview.postMessage({id: message.id, error: String(error.message || error)}); }
          return;
        }
      }
      try {
        const result = await rpc(message.method, message.params || {});
        webview.postMessage({id: message.id, result});
      } catch (error) { webview.postMessage({id: message.id, error: String(error.message || error)}); }
    });
  } catch (error) { vscode.window.showErrorMessage(String(error.message || error)); }
}

function activate(context) {
  output = vscode.window.createOutputChannel('robot_template 云台编码器');
  context.subscriptions.push(output, vscode.commands.registerCommand('robot_templateGimbal.open', () => open(context)),
    vscode.window.registerUriHandler({handleUri: () => open(context)}));
  const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 99);
  item.text = '$(circuit-board) 云台编码器'; item.command = 'robot_templateGimbal.open';
  item.tooltip = '在VS Code中查看折叠与Pitch的DM4310原始编码';
  try { locateRoot(); item.show(); } catch (_) { /* Only show for a matching workspace. */ }
  context.subscriptions.push(item);
}

module.exports = {activate, deactivate: stop};
