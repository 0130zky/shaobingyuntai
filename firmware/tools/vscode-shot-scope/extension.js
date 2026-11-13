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
    for (const suffix of ['', 'robot_template', 'robot_template/robot_template']) {
      const candidate = path.join(root, suffix);
      if (fs.existsSync(path.join(candidate, 'tools/shot_scope_vscode.py'))) return candidate;
    }
  }
  throw new Error('请先在VS Code中打开包含robot_template工程的工作区。');
}
function startBackend(root) {
  if (backend) return;
  const candidates = process.platform === 'win32'
    ? [path.join(root, '.venv/Scripts/python.exe'), path.join(root, 'build/host-tests/Scripts/python.exe')]
    : [path.join(root, '.venv/bin/python'), path.join(root, 'build/host-tests/bin/python')];
  const python = candidates.find(candidate => fs.existsSync(candidate));
  if (!python) throw new Error('请先创建工程.venv并安装tools/requirements.txt。');
  backend = cp.spawn(python, ['-u', path.join(root, 'tools/shot_scope_vscode.py')], {cwd: root, windowsHide: true, env: {...process.env, PYTHONUTF8: '1'}, stdio: ['pipe', 'pipe', 'pipe']});
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
  child.on('exit', () => fail(new Error('分析进程已停止，关闭并重新打开发弹监测即可恢复。')));
}
function rpc(method, params = {}) {
  return new Promise((resolve, reject) => {
    if (!backend) return reject(new Error('分析进程未运行'));
    const id = ++sequence;
    const timer = setTimeout(() => { pending.delete(id); reject(new Error('分析超时，请稍后重试')); }, 60000);
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
    panel = vscode.window.createWebviewPanel('robot_templateScope', '发弹监测 · robot_template', vscode.ViewColumn.Beside,
      {enableScripts: true, retainContextWhenHidden: true, localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'media')]});
    const webview = panel.webview;
    const nonce = crypto.randomBytes(24).toString('base64');
    let html = fs.readFileSync(path.join(context.extensionPath, 'media/index.html'), 'utf8');
    html = html.replaceAll('{{cspSource}}', webview.cspSource).replaceAll('{{nonce}}', nonce)
      .replaceAll('{{style}}', webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media/style.css')).toString())
      .replaceAll('{{script}}', webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media/dashboard.js')).toString());
    webview.html = html;
    panel.onDidDispose(() => { panel = undefined; stop(); });
    webview.onDidReceiveMessage(async message => {
      const allowed = ['state', 'shot', 'phase', 'pitch_limit', 'live', 'replay', 'export'];
      if (!allowed.includes(message.method)) { webview.postMessage({id: message.id, error: 'Unsupported method: ' + message.method}); return; }
      try {
        let params = message.params || {};
        if (message.method === 'replay') {
          const files = await vscode.window.showOpenDialog({defaultUri: vscode.Uri.file(path.join(root, 'tools/records')), filters: {'CSV': ['csv']}, canSelectMany: false});
          if (!files) throw new Error('已取消打开记录');
          params = {path: files[0].fsPath};
        }
        if (message.method === 'export') {
          const file = await vscode.window.showSaveDialog({defaultUri: vscode.Uri.file(path.join(root, 'tools/records/shots_per_wheel.csv')), filters: {'CSV': ['csv']}});
          if (!file) throw new Error('已取消导出');
          params = {path: file.fsPath};
        }
        const result = await rpc(message.method, params);
        if (message.method === 'export') vscode.window.showInformationMessage(`已导出 ${result.shots} 发逐轮统计：${result.path}`);
        webview.postMessage({id: message.id, result});
      } catch (error) { webview.postMessage({id: message.id, error: String(error.message || error)}); }
    });
  } catch (error) { vscode.window.showErrorMessage(String(error.message || error)); }
}
function activate(context) {
  output = vscode.window.createOutputChannel('robot_template 发弹监测');
  context.subscriptions.push(output, vscode.commands.registerCommand('robot_templateScope.open', () => open(context)),
    vscode.window.registerUriHandler({handleUri: () => open(context)}));
  const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 100);
  item.text = '$(pulse) 发弹监测'; item.command = 'robot_templateScope.open'; item.tooltip = '在VS Code中打开三轮上位机';
  try { locateRoot(); item.show(); } catch (_) { /* Only show for a matching workspace. */ }
  context.subscriptions.push(item);
}
module.exports = {activate, deactivate: stop};
