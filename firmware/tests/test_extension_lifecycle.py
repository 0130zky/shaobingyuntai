"""Exercise the real extension's Python launch/close path using a fake VS Code API."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = r'''
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const EventEmitter = require('events');
const root = process.argv[2];
let command, launch, errors = [];
const child = new EventEmitter();
child.stdout = new EventEmitter(); child.stderr = new EventEmitter();
child.stdin = {write() {}}; child.kill = () => child.killed = true;
const uri = value => ({fsPath:value, toString:()=>value});
const vscode = {
  workspace: {workspaceFolders:[{uri:uri(root)}]},
  ViewColumn: {Beside:2}, StatusBarAlignment:{Right:1},
  Uri: {joinPath:(base,...tail)=>uri(path.join(base.fsPath,...tail))},
  commands: {registerCommand:(name,callback)=>{command=callback;return {}; }},
  window: {
    createOutputChannel:()=>({append(){},appendLine(){}}),
    createStatusBarItem:()=>({show(){}}), registerUriHandler:()=>({}),
    showErrorMessage:error=>errors.push(error),
    createWebviewPanel:()=>({onDidDispose(){}, webview:{
      asWebviewUri:value=>value, onDidReceiveMessage(){}, postMessage(){}}})
  }
};
const moduleResult = {exports:{}};
const mockfs = Object.create(fs);
mockfs.existsSync = file => /[\\/]\.venv[\\/](Scripts|bin)[\\/]python/.test(file) || fs.existsSync(file);
const dependencies = {
  vscode, fs:mockfs, path, crypto:require('crypto'),
  readline:{createInterface:()=>({on(){}})},
  child_process:{spawn:(exe,args,options)=>{launch={exe,args,options};return child;}}
};
const extension = path.join(root,'tools/vscode-shot-scope');
vm.runInNewContext(fs.readFileSync(path.join(extension,'extension.js'),'utf8'), {
  require:name=>dependencies[name], module:moduleResult,
  process:{platform:process.platform, env:{TEST_ENV:'1'}}, setTimeout, clearTimeout
});
(async()=>{
  moduleResult.exports.activate({extensionPath:extension,extensionUri:uri(extension),subscriptions:[]});
  await command(); assert.deepEqual(errors,[]);
  assert(launch.exe.endsWith(process.platform==='win32' ? 'Scripts\\python.exe' : 'bin/python'));
  assert(launch.options.windowsHide); assert.equal(launch.options.env.PYTHONUTF8,'1');
  assert(launch.args[1].endsWith('shot_scope_vscode.py'));
  moduleResult.exports.deactivate(); assert(child.killed);
  console.log('PASS: real VS Code extension launches the selected Python hidden and stops its own child');
})().catch(error=>{console.error(error);process.exitCode=1;});
'''


class ExtensionLifecycleTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node CLI unavailable')
    def test_real_extension_launch_and_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / 'test.js'
            script.write_text(SCRIPT, encoding='utf-8')
            subprocess.run(['node', str(script), str(ROOT)], check=True)


if __name__ == '__main__':
    unittest.main()
