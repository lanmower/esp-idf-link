#!/usr/bin/env node
const { spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');

function listPorts() {
  const r = spawnSync('powershell', ['-NoProfile', '-Command',
    "Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'COM\\d+' } | ForEach-Object { $_.Name }"],
    { encoding: 'utf8' });
  const ports = [];
  for (const line of (r.stdout || '').split('\n')) {
    const name = line.trim();
    const m = name.match(/\((COM\d+)\)/);
    if (m && !/Bluetooth/i.test(name)) ports.push(m[1]);
  }
  return ports;
}

const arg = process.argv[2];
let port = arg;
if (!port || arg === '--list') {
  const ports = listPorts();
  if (arg === '--list') {
    console.log(ports.length ? ports.join('\n') : 'no COM ports found');
    process.exit(ports.length ? 0 : 1);
  }
  if (!ports.length) {
    console.error('no COM port found -- plug the ticker in over USB, or pass a port: node flash-ticker.js COM9');
    process.exit(2);
  }
  if (ports.length > 1) {
    console.error(`several serial ports (${ports.join(', ')}) -- pass the one that is the ESP32`);
    process.exit(2);
  }
  port = ports[0];
}

const root = __dirname;
const tableFile = path.join(root, 'build', 'partition_table', 'partition-table.bin');

function firstAppOffset(file) {
  const table = fs.readFileSync(file);
  for (let i = 0; i + 32 <= table.length; i += 32) {
    if (table[i] === 0xaa && table[i + 1] === 0x50 && table[i + 2] === 0) {
      return table.readUInt32LE(i + 4);
    }
  }
  throw new Error(`no app partition in ${file}`);
}

const images = [
  ['0x1000', path.join(root, 'build', 'bootloader', 'bootloader.bin')],
  ['0x8000', tableFile],
  ['0x' + firstAppOffset(tableFile).toString(16), path.join(root, 'build', 'link-idf-example.bin')],
];
const args = ['-m', 'esptool', '--chip', 'esp32', '--port', port, '--baud', '921600', 'write-flash', '-z'];
for (const [addr, file] of images) args.push(addr, file);

console.log(`flashing ${port}: ${images.map(([a, f]) => `${a} ${path.basename(f)}`).join('   ')}`);
const r = spawnSync('python', args, { stdio: 'inherit' });
if (r.status !== 0) {
  console.error('esptool failed -- if it cannot connect, hold IO0 (BOOT) while it starts');
  process.exit(r.status === null ? 1 : r.status);
}
console.log('flash done -- the ticker boots into the mesh on its own');
