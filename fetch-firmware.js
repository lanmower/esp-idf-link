#!/usr/bin/env node
const { spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const root = __dirname;
const repo = 'lanmower/esp-idf-link';

function gh(args, opts = {}) {
  return spawnSync('gh', args, { encoding: 'utf8', ...opts });
}

function die(msg) {
  console.error(msg);
  process.exit(1);
}

if (spawnSync('gh', ['--version'], { encoding: 'utf8' }).status !== 0) {
  die('gh not found -- install the GitHub CLI and run: gh auth login');
}

const IMAGES = [
  ['build/bootloader/bootloader.bin', '0x1000'],
  ['build/partition_table/partition-table.bin', '0x8000'],
  ['build/link-idf-example.bin', '0x10000'],
];

let runId = null;
let wantSha = null;
let doFlash = true;

for (let i = 2; i < process.argv.length; i++) {
  const a = process.argv[i];
  if (a === '--run') runId = process.argv[++i];
  else if (a === '--sha') wantSha = process.argv[++i];
  else if (a === '--no-flash') doFlash = false;
  else die(`unknown arg: ${a}`);
}

let runs;
if (runId) {
  runs = [{ databaseId: Number(runId) }];
} else {
  const r = gh(['run', 'list', '--workflow=Build', '-b', 'main',
    '--json', 'databaseId,headSha,headBranch,conclusion,status', '--limit', '20']);
  if (r.status !== 0) die(`gh run list failed: ${r.stderr || r.stdout}`);
  runs = JSON.parse(r.stdout);
  if (wantSha) {
    runs = runs.filter((x) => x.headSha === wantSha || x.headSha.startsWith(wantSha));
    if (!runs.length) die(`no Build run found for sha ${wantSha}`);
  } else {
    runs = runs.filter((x) => x.conclusion === 'success');
  }
  if (!runs.length) die('no successful Build run on main -- nothing to download');
}

let picked = null;
for (const cand of runs) {
  const r = gh(['api', `repos/${repo}/actions/runs/${cand.databaseId}/artifacts`,
    '--jq', '[.artifacts[] | select(.name | startswith("ticker-firmware")) | .name] | .[0] // empty']);
  const name = (r.stdout || '').trim();
  if (name) {
    picked = { id: cand.databaseId, artifact: name, sha: cand.headSha };
    break;
  }
}
if (!picked) {
  die('no ticker-firmware-* artifact on any candidate run -- the build predates the upload step, '
    + 'or the run never finished. Re-run it: gh workflow run Build.yml');
}

console.log(`run ${picked.id}  sha ${picked.sha || '?'}  artifact ${picked.artifact}`);

const tmp = fs.mkdtempSync(path.join(require('os').tmpdir(), 'ticker-fw-'));
const d = gh(['run', 'download', String(picked.id), '-n', picked.artifact, '-D', tmp]);
if (d.status !== 0) die(`gh run download failed: ${d.stderr || d.stdout}`);

const roots = [tmp, path.join(tmp, picked.artifact)];
const found = [];
for (const c of roots) {
  const hits = IMAGES.map(([rel]) => {
    const withPrefix = path.join(c, rel);
    if (fs.existsSync(withPrefix)) return withPrefix;
    const bare = path.join(c, rel.replace(/^build[\\/]/, ''));
    if (fs.existsSync(bare)) return bare;
    return null;
  });
  if (hits.every(Boolean)) { found.push(...hits); break; }
}
if (!found.length) die(`artifact did not contain ${IMAGES.map(([r]) => r).join(', ')} -- refusing to clobber build/`);

IMAGES.forEach(([rel], i) => {
  const src = found[i];
  const dst = path.join(root, rel);
  fs.mkdirSync(path.dirname(dst), { recursive: true });
  fs.copyFileSync(src, dst);
  const st = fs.statSync(dst);
  console.log(`${rel}  ${st.size} bytes`);
});

fs.rmSync(tmp, { recursive: true, force: true });

console.log(`\nfirmware staged into ${path.join(root, 'build')}`);
if (!doFlash) process.exit(0);

console.log('flashing -- hold IO0 (BOOT) if esptool cannot connect\n');
const f = spawnSync(process.execPath, [path.join(root, 'flash-ticker.js')], { stdio: 'inherit' });
process.exit(f.status === null ? 1 : f.status);
