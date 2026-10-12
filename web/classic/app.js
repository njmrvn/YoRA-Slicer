import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { initRobot } from './robot.js';

const $ = (id) => document.getElementById(id);
const STORE_KEY = 'yora.settings.v1';
const NO_RESLICE_GROUPS = new Set(['Robot & bed', 'Extruder I/O']);
const KIND_COLORS = { 1: '#f07b12', 2: '#f5c518', 3: '#d64545', 4: '#9b59d0', 5: '#2fae66' };
const KIND_LABELS = { 1: 'Outer wall', 2: 'Inner wall', 3: 'Sparse infill', 4: 'Solid infill', 5: 'Brim' };
const FLOATS = 13; // x0 y0 z0 x1 y1 z1 kind level layer speed e_per_mm layer_height move

const app = {
  spec: [], defaults: {}, values: {}, model: null, result: null,
  ext: null, trv: null, dirty: false, mode: 'prepare', busy: false,
  hooks: [], onToolpathChanged: null,
  // origin markers in the 3D view, each shown/hidden on its own (view toolbar > Show)
  markers: (() => { try { return { robot: true, tcp: true, bed: true, ...JSON.parse(localStorage.getItem('yora.markers') || '{}') }; } catch (_) { return { robot: true, tcp: true, bed: true }; } })(),
  markerHooks: [],
};
let robotUI = null;

// ------------------------------------------------------------------ helpers
function toast(msg, kind = 'info', ms = 6000) {
  const el = document.createElement('div');
  el.className = 'toast ' + kind;
  el.textContent = msg;
  $('toasts').appendChild(el);
  setTimeout(() => el.remove(), ms);
}
async function api(path, opts = {}) {
  const r = await fetch(path, opts);
  if (!r.ok) {
    let msg = r.statusText;
    try { msg = (await r.json()).error || msg; } catch (_) { /* not JSON */ }
    throw new Error(msg);
  }
  return r;
}
const postJSON = (path, body) => api(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
const fmtTime = (s) => { const h = Math.floor(s / 3600), m = Math.round((s % 3600) / 60); return h ? `${h} h ${m} min` : `${m} min`; };
const fmt = (v, d = 1) => Number(v).toLocaleString(undefined, { maximumFractionDigits: d, minimumFractionDigits: d });
function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

// ------------------------------------------------------------------ 3D view
const view = {};
function initView() {
  const host = $('viewport');
  view.renderer = new THREE.WebGLRenderer({ antialias: true });
  view.renderer.setPixelRatio(window.devicePixelRatio);
  host.prepend(view.renderer.domElement);
  view.scene = new THREE.Scene();
  view.camera = new THREE.PerspectiveCamera(40, 1, 1, 100000);
  view.camera.up.set(0, 0, 1);
  view.controls = new OrbitControls(view.camera, view.renderer.domElement);
  view.controls.enableDamping = true;
  view.controls.screenSpacePanning = true;
  view.scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(1, -1.5, 2);
  view.scene.add(sun);
  view.bed = new THREE.Group();
  view.scene.add(view.bed);
  view.lineMat = new LineMaterial({ vertexColors: true, linewidth: 2.2, worldUnits: false });
  view.travelMat = new THREE.LineBasicMaterial({ color: 0x8a94a6, transparent: true, opacity: 0.55 });
  new ResizeObserver(resize).observe(host);
  resize();
  applyTheme();
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
  (function loop() { requestAnimationFrame(loop); view.controls.update(); view.renderer.render(view.scene, view.camera); })();
}
function applyTheme() { view.scene.background = new THREE.Color(cssVar('--viewport') || '#e9ecf1'); buildBed(); }
function resize() {
  const host = $('viewport');
  const w = host.clientWidth, h = host.clientHeight;
  view.renderer.setSize(w, h, false);
  view.camera.aspect = w / Math.max(h, 1);
  view.camera.updateProjectionMatrix();
  view.lineMat.resolution.set(w, h);
}
function buildBed() {
  if (!view.bed) return;
  view.bed.clear();
  const bx = app.values.bed_x || 1000, by = app.values.bed_y || 1000, bz = app.values.bed_z || 800;
  const dark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  const plate = new THREE.Mesh(new THREE.PlaneGeometry(bx, by),
    new THREE.MeshStandardMaterial({ color: dark ? 0x2a3038 : 0xd5dae1, roughness: 0.95 }));
  plate.position.set(bx / 2, by / 2, -0.5);
  view.bed.add(plate);
  const step = bx > 600 || by > 600 ? 100 : 50;
  const pts = [];
  for (let x = 0; x <= bx + 1e-6; x += step) pts.push(x, 0, 0, x, by, 0);
  for (let y = 0; y <= by + 1e-6; y += step) pts.push(0, y, 0, bx, y, 0);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  view.bed.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: dark ? 0x3b434e : 0xb9c0ca })));
  const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(bx, by, bz)),
    new THREE.LineBasicMaterial({ color: dark ? 0x46505d : 0xa6aebb, transparent: true, opacity: 0.6 }));
  box.position.set(bx / 2, by / 2, bz / 2);
  view.bed.add(box);
  const axes = new THREE.AxesHelper(Math.min(bx, by) * 0.15);
  axes.position.z = 0.2;
  axes.visible = app.markers.bed;
  view.bed.add(axes);
  view.bedAxes = axes;
}
function applyMarkers() {
  if (view.bedAxes) view.bedAxes.visible = app.markers.bed;
  app.markerHooks.forEach((f) => f(app.markers));
  try { localStorage.setItem('yora.markers', JSON.stringify(app.markers)); } catch (_) { /* storage blocked */ }
}
function setView(kind) {
  const b = sceneBounds();
  const c = b.getCenter(new THREE.Vector3());
  const size = b.getSize(new THREE.Vector3()).length() || 1000;
  const d = size * 1.15;
  const dirs = { iso: [-0.6, -1, 0.75], top: [0, -0.001, 1], front: [0, -1, 0.15], fit: null };
  const dir = dirs[kind] ? new THREE.Vector3(...dirs[kind]).normalize()
    : view.camera.position.clone().sub(view.controls.target).normalize();
  view.controls.target.copy(c);
  view.camera.position.copy(c).addScaledVector(dir, d);
  view.camera.near = d / 1000; view.camera.far = d * 20;
  view.camera.updateProjectionMatrix();
}
function sceneBounds() {
  const b = new THREE.Box3();
  if (app.mode === 'preview' && app.result?.stats?.bbox) {
    const [x0, y0, z0, x1, y1, z1] = app.result.stats.bbox;
    b.set(new THREE.Vector3(x0, y0, z0), new THREE.Vector3(x1, y1, z1));
  } else if (view.mesh) b.setFromObject(view.mesh);
  else b.set(new THREE.Vector3(0, 0, 0), new THREE.Vector3(app.values.bed_x || 1000, app.values.bed_y || 1000, 100));
  if (app.mode === 'robot') {
    b.union(new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(app.values.bed_x || 1000, app.values.bed_y || 1000, 0)));
    if (app.robotGroup?.visible) b.union(new THREE.Box3().setFromObject(app.robotGroup));
  }
  return b;
}

// ------------------------------------------------------------------ settings form
function buildForm() {
  const form = $('settingsForm');
  form.innerHTML = '';
  const groups = [...new Set(app.spec.map((f) => f.group))].filter((g) => g !== 'Robot cell'); // shown in the Robot tab
  groups.forEach((group, gi) => {
    const det = document.createElement('details');
    det.open = gi < 2 || group === 'Extruder I/O';
    det.innerHTML = `<summary>${group}</summary><div class="fields"></div>`;
    const box = det.querySelector('.fields');
    app.spec.filter((f) => f.group === group).forEach((f) => box.appendChild(fieldRow(f)));
    form.appendChild(det);
  });
}
function fieldRow(f) {
  const row = document.createElement('div');
  row.className = 'field';
  row.dataset.key = f.key;
  const name = document.createElement('label');
  name.className = 'name';
  name.textContent = f.label;
  name.htmlFor = 'f_' + f.key;
  if (f.help) { const sm = document.createElement('small'); sm.textContent = f.help; name.appendChild(sm); }
  const ctl = document.createElement('div');
  ctl.className = 'ctl';
  let input;
  if (f.type === 'bool') {
    input = document.createElement('input'); input.type = 'checkbox'; input.checked = !!app.values[f.key];
  } else if (f.type === 'choice') {
    input = document.createElement('select');
    f.options.forEach((o) => input.add(new Option(o.replace(/_/g, ' '), o)));
    input.value = app.values[f.key];
  } else {
    input = document.createElement('input');
    input.type = f.type === 'text' ? 'text' : 'number';
    if (f.type !== 'text') {
      input.step = f.type === 'int' ? '1' : 'any';
      if (f.min !== undefined) input.min = f.min;
      if (f.max !== undefined) input.max = f.max;
    }
    input.value = app.values[f.key];
  }
  input.id = 'f_' + f.key;
  input.addEventListener('change', () => onSetting(f, input));
  ctl.appendChild(input);
  if (f.unit) { const u = document.createElement('span'); u.className = 'unit'; u.textContent = f.unit; ctl.appendChild(u); }
  row.append(name, ctl);
  markChanged(row, f);
  return row;
}
function readInput(f, input) {
  if (f.type === 'bool') return input.checked;
  if (f.type === 'int') return Math.round(Number(input.value));
  if (f.type === 'float') return Number(input.value);
  return input.value;
}
function markChanged(row, f) { row.classList.toggle('changed', app.values[f.key] !== app.defaults[f.key]); }
function onSetting(f, input) {
  let v = readInput(f, input);
  if ((f.type === 'int' || f.type === 'float')) {
    if (!Number.isFinite(v)) v = app.defaults[f.key];
    if (f.min !== undefined) v = Math.max(f.min, v);
    if (f.max !== undefined) v = Math.min(f.max, v);
    input.value = v;
  }
  app.values[f.key] = v;
  markChanged(input.closest('.field'), f);
  persist();
  if (f.key.startsWith('bed_')) { buildBed(); if (app.model) applyTransform(); }
  if (app.result && app.result.source === 'slice' && !NO_RESLICE_GROUPS.has(f.group)) setDirty(true);
  if (f.group === 'Extruder I/O' && app.result) toast('Extruder I/O changed: exports use the new values. The flow-level preview updates on the next slice.', 'info', 4000);
  app.hooks.forEach((h) => h(f));
}
function persist() { try { localStorage.setItem(STORE_KEY, JSON.stringify(app.values)); } catch (_) { /* storage unavailable */ } }
function restore() { try { return JSON.parse(localStorage.getItem(STORE_KEY) || '{}'); } catch (_) { return {}; } }
function setValues(vals) {
  app.values = { ...app.defaults };
  for (const k of Object.keys(app.defaults)) if (vals && k in vals) app.values[k] = vals[k];
  buildForm(); buildBed(); persist();
  if (robotUI) robotUI.rebuild();
}
function setDirty(d) {
  app.dirty = d;
  if (d) setStatusNote('Settings changed — slice again to update the toolpath.');
}

// ------------------------------------------------------------------ model
async function openFile(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  if (ext === 'robot') {   // robot model file: load it as the robot model and show the Robot tab
    setMode('robot');
    return app.loadRobotFile?.(file);
  }
  const isGcode = ['gcode', 'gco', 'g', 'nc'].includes(ext);
  const q = `name=${encodeURIComponent(file.name)}&settings=${encodeURIComponent(JSON.stringify(app.values))}`;
  busy(true, isGcode ? 'Reading G-code…' : 'Loading model…');
  try {
    const body = await file.arrayBuffer();
    if (isGcode) {
      const res = await (await api('/api/gcode?' + q, { method: 'POST', body })).json();
      clearMesh();
      app.model = null;
      updateObjectPanel();
      $('dropHint').classList.add('hidden');
      await showResult(res);
      const ig = Object.entries(res.import?.ignored || {}).map(([k, v]) => `${k}×${v}`).join(', ');
      toast(`Imported ${file.name}: ${res.stats.layers} layers (layers from ${res.import.layers_from}).${ig ? '\nIgnored: ' + ig : ''}`);
    } else {
      const res = await (await api('/api/model?' + q, { method: 'POST', body })).json();
      app.model = res.model;
      app.result = null; clearToolpath();
      await loadMesh();
      updateObjectPanel();
      $('dropHint').classList.add('hidden');
      setMode('prepare');
      setView('iso');
      setStatusNote(`${file.name}: ${res.model.triangles.toLocaleString()} triangles. Set the print settings, then Slice.`);
    }
  } catch (e) { toast(e.message, 'error'); } finally { busy(false); refreshButtons(); }
}
async function loadMesh() {
  const buf = await (await api('/api/model.bin')).arrayBuffer();
  clearMesh();
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf), 3));
  g.computeVertexNormals();
  view.mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0x3fb8a8, roughness: 0.55, metalness: 0.05, transparent: true, opacity: 1 }));
  view.scene.add(view.mesh);
  updateVisibility();
}
function clearMesh() { if (view.mesh) { view.scene.remove(view.mesh); view.mesh.geometry.dispose(); view.mesh = null; } }
function updateObjectPanel() {
  const m = app.model;
  $('objectEmpty').hidden = !!m; $('objectBody').hidden = !m;
  if (!m) return;
  $('objName').textContent = m.name;
  const s = m.size.map((v) => fmt(v, 1)).join(' × ');
  $('objSize').textContent = `${s} mm` + (m.fits ? '' : '  — outside the print area');
  $('objSize').classList.toggle('bad', !m.fits);
  const t = m.transform;
  ['sx', 'sy', 'sz'].forEach((id, i) => { $(id).value = t.scale[i]; });
  ['rx', 'ry', 'rz'].forEach((id, i) => { $(id).value = t.rotation[i]; });
  $('px').value = fmt((m.min[0] + m.max[0]) / 2, 1).replace(/,/g, '');
  $('py').value = fmt((m.min[1] + m.max[1]) / 2, 1).replace(/,/g, '');
}
let transformTimer = null;
function onTransformInput(e) {
  if ($('lockScale').checked && ['sx', 'sy', 'sz'].includes(e.target.id)) ['sx', 'sy', 'sz'].forEach((id) => { $(id).value = e.target.value; });
  clearTimeout(transformTimer);
  transformTimer = setTimeout(() => applyTransform(true), 350);
}
async function applyTransform(useInputs = false, recenter = false) {
  if (!app.model) return;
  const t = app.model.transform;
  const body = {
    settings: app.values,
    transform: useInputs ? {
      scale: ['sx', 'sy', 'sz'].map((id) => Number($(id).value) || 100),
      rotation: ['rx', 'ry', 'rz'].map((id) => Number($(id).value) || 0),
      center: recenter ? null : [Number($('px').value), Number($('py').value)],
    } : { ...t, center: recenter ? null : t.center },
  };
  try {
    app.model = (await (await postJSON('/api/object', body)).json()).model;
    await loadMesh();
    updateObjectPanel();
    if (app.result?.source === 'slice') setDirty(true);
  } catch (e) { toast(e.message, 'error'); }
}

// ------------------------------------------------------------------ slicing
async function slice() {
  if (!app.model || app.busy) return;
  busy(true, 'Slicing…');
  try {
    await postJSON('/api/slice', { settings: app.values });
    for (;;) {
      await new Promise((r) => setTimeout(r, 250));
      const st = await (await api('/api/status')).json();
      if (st.state === 'running') { progress(st.progress, `${st.stage === 'planning' ? 'Planning moves' : 'Slicing'}… ${Math.round(st.progress * 100)}%`); continue; }
      if (st.state === 'error') throw new Error(st.error);
      if (st.state === 'done') { await showResult(st.result); break; }
    }
    setDirty(false);
  } catch (e) { toast('Slicing failed: ' + e.message, 'error', 10000); } finally { busy(false); refreshButtons(); }
}
async function showResult(res) {
  app.result = res;
  const [e, t] = await Promise.all([
    api('/api/toolpath.bin?kind=extrude').then((r) => r.arrayBuffer()),
    api('/api/toolpath.bin?kind=travel').then((r) => r.arrayBuffer()),
  ]);
  app.ext = new Float32Array(e); app.trv = new Float32Array(t);
  const nl = res.layer_z.length;
  $('layerRange').max = Math.max(nl - 1, 0);
  $('layerRange').value = Math.max(nl - 1, 0);
  buildColors();
  if (app.onToolpathChanged) app.onToolpathChanged();
  setMode(app.mode === 'robot' ? 'robot' : 'preview');
  updateLayers();
  renderLegend();
  renderStats();
  (res.warnings || []).forEach((w) => toast(w, 'warn', 10000));
  setView('iso');
}
function clearToolpath() {
  for (const k of ['extObj', 'trvObj']) if (view[k]) { view.scene.remove(view[k]); view[k].geometry.dispose(); view[k] = null; }
  clearBeads();
  app.ext = app.trv = null; app.colors = null;
}

// ------------------------------------------------------------------ extrusion view (beads with real width x layer height)
// One rounded bead section (unit size) instanced per segment. X along the move, Y across (width), Z down from the
// nozzle height (top of the bead = nozzle Z). Width comes from the extruded volume, so over/under-extrusion shows.
function beadGeometry() {
  const seg = 12, ring = [];
  for (let k = 0; k < seg; k++) { const a = (k / seg) * Math.PI * 2; ring.push([0.5 * Math.cos(a), -0.5 + 0.5 * Math.sin(a)]); }
  const pos = [], nrm = [], idx = [];
  for (const x of [-0.5, 0.5]) ring.forEach(([y, z]) => { pos.push(x, y, z); nrm.push(0, y, z + 0.5); });
  for (let k = 0; k < seg; k++) { const k1 = (k + 1) % seg; idx.push(k, k1, seg + k, k1, seg + k1, seg + k); }
  for (const [x, s] of [[-0.5, -1], [0.5, 1]]) {    // end caps
    const c = pos.length / 3;
    pos.push(x, 0, -0.5); nrm.push(s, 0, 0);
    ring.forEach(([y, z]) => { pos.push(x, y, z); nrm.push(s, 0, 0); });
    for (let k = 0; k < seg; k++) { const a = c + 1 + k, b = c + 1 + (k + 1) % seg; if (s > 0) idx.push(c, a, b); else idx.push(c, b, a); }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(nrm, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}
function beadWidth(ePerMm, h) {
  const v = app.values;
  let area = ePerMm * (v.e_mode === 'filament' ? Math.PI * (Number(v.filament_diameter || 1.75) / 2) ** 2 : 1) / (Number(v.flow || 100) / 100);
  if (!(area > 0)) return Number(v.line_width || 1);
  // inverse of planner.e_per_mm: stadium bead when wider than high, else an ellipse
  return area >= Math.PI * h * h / 4 ? (area - Math.PI * h * h / 4) / h + h : 4 * area / (Math.PI * h);
}
function clearBeads() {
  (view.beads || []).forEach((m) => { if (m) { view.scene.remove(m); m.dispose(); } });
  view.beads = null;
}
function buildBeads() {
  clearBeads();
  if (!app.ext || !app.result) return;
  view.beadGeo = view.beadGeo || beadGeometry();
  view.beadMat = view.beadMat || new THREE.MeshStandardMaterial({ roughness: 0.6, metalness: 0.0 });
  const eo = app.result.extrude_offsets, nl = app.result.layer_z.length;
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), sc = new THREE.Vector3(), p = new THREE.Vector3();
  const X = new THREE.Vector3(), Y = new THREE.Vector3(), Z = new THREE.Vector3(), up = new THREE.Vector3(0, 0, 1), basis = new THREE.Matrix4();
  const col = new THREE.Color();
  view.beads = [];
  for (let L = 0; L < nl; L++) {
    const a = eo[L], b = eo[L + 1];
    if (b <= a) { view.beads.push(null); continue; }
    const mesh = new THREE.InstancedMesh(view.beadGeo, view.beadMat, b - a);
    for (let i = a; i < b; i++) {
      const o = i * FLOATS, e = app.ext;
      X.set(e[o + 3] - e[o], e[o + 4] - e[o + 1], e[o + 5] - e[o + 2]);
      const len = X.length();
      const h = e[o + 11] || Number(app.values.layer_height || 1);
      const w = beadWidth(e[o + 10], h);
      if (len < 1e-6) X.set(1, 0, 0); else X.divideScalar(len);
      Y.crossVectors(up, X); if (Y.lengthSq() < 1e-12) Y.set(0, 1, 0); Y.normalize();
      Z.crossVectors(X, Y);
      basis.makeBasis(X, Y, Z);
      q.setFromRotationMatrix(basis);
      p.set((e[o] + e[o + 3]) / 2, (e[o + 1] + e[o + 4]) / 2, (e[o + 2] + e[o + 5]) / 2);
      sc.set(len + w * 0.5, w, h);              // a little longer than the move so corners close
      m4.compose(p, q, sc);
      mesh.setMatrixAt(i - a, m4);
      col.setRGB(app.colors[i * 6], app.colors[i * 6 + 1], app.colors[i * 6 + 2]);
      mesh.setColorAt(i - a, col);
    }
    mesh.instanceMatrix.needsUpdate = true;
    mesh.instanceColor.needsUpdate = true;
    mesh.computeBoundingSphere();
    view.scene.add(mesh);
    view.beads.push(mesh);
  }
}
function showAsBeads() { return $('showAs').value === 'beads'; }

function levelColor(level, max) {
  if (level <= 0) return new THREE.Color('#9aa3b2');
  const t = max <= 1 ? 1 : (level - 1) / (max - 1);
  return new THREE.Color().setHSL(0.62 - 0.62 * t, 0.85, 0.5);
}
function buildColors() {
  if (!app.ext) return;
  const mode = $('colorMode').value;
  const n = app.ext.length / FLOATS;
  const col = new Float32Array(n * 6);
  const maxL = app.result.stats.flow_max_level || 7;
  let smin = Infinity, smax = -Infinity;
  if (mode === 'speed') for (let i = 0; i < n; i++) { const v = app.ext[i * FLOATS + 9]; smin = Math.min(smin, v); smax = Math.max(smax, v); }
  const kindCols = Object.fromEntries(Object.entries(KIND_COLORS).map(([k, v]) => [k, new THREE.Color(v)]));
  const c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    const o = i * FLOATS;
    if (mode === 'kind') c.copy(kindCols[app.ext[o + 6]] || kindCols[3]);
    else if (mode === 'level') c.copy(levelColor(app.ext[o + 7], maxL));
    else { const t = smax > smin ? (app.ext[o + 9] - smin) / (smax - smin) : 1; c.setHSL(0.62 - 0.62 * t, 0.8, 0.5); }
    col.set([c.r, c.g, c.b, c.r, c.g, c.b], i * 6);
  }
  app.colors = col; app.speedRange = [smin, smax];
  clearBeads();   // rebuilt with the new colours when shown
}
let layerFrame = 0;
function updateLayers() {
  cancelAnimationFrame(layerFrame);
  layerFrame = requestAnimationFrame(() => {
    if (!app.result || !app.ext) return;
    const top = Number($('layerRange').value);
    const lo = $('singleLayer').checked ? top : 0;
    const eo = app.result.extrude_offsets, to = app.result.travel_offsets;
    if (view.extObj) { view.scene.remove(view.extObj); view.extObj.geometry.dispose(); view.extObj = null; }
    if (showAsBeads()) {
      if (!view.beads) buildBeads();
      view.beads.forEach((m, L) => { if (m) m.userData.inRange = L >= lo && L <= top; });
    } else {
      if (view.beads) view.beads.forEach((m) => { if (m) m.userData.inRange = false; });
      const a = eo[lo], b = eo[top + 1];
      const pos = new Float32Array((b - a) * 6);
      for (let i = a, j = 0; i < b; i++, j += 6) pos.set(app.ext.subarray(i * FLOATS, i * FLOATS + 6), j);
      const g = new LineSegmentsGeometry();
      if (b > a) { g.setPositions(pos); g.setColors(app.colors.subarray(a * 6, b * 6)); }
      view.extObj = new LineSegments2(g, view.lineMat);
      view.scene.add(view.extObj);
    }

    const ta = to[lo], tb = to[top + 1];
    const tpos = new Float32Array((tb - ta) * 6);
    for (let i = ta, j = 0; i < tb; i++, j += 6) tpos.set(app.trv.subarray(i * FLOATS, i * FLOATS + 6), j);
    if (view.trvObj) { view.scene.remove(view.trvObj); view.trvObj.geometry.dispose(); }
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(tpos, 3));
    view.trvObj = new THREE.LineSegments(tg, view.travelMat);
    view.scene.add(view.trvObj);

    $('layerLabel').textContent = `${top + 1}`;
    $('layerZ').textContent = `${fmt(app.result.layer_z[top] ?? 0, 2)} mm`;
    applyCut();
    updateVisibility();
  });
}

// ------------------------------------------------------------------ progressive toolpath (robot simulation)
// app.cut = toolpath move the nozzle has reached (null = show whole layers). The top layer is drawn only up to
// that move, so the path appears behind the nozzle as the simulation plays. Only draw counts change per frame.
app.cut = null;
function firstSegAfter(move, a, b) {      // first extrusion segment in [a, b) whose move index is > move
  let lo = a, hi = b;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (app.ext[mid * FLOATS + 12] > move) hi = mid; else lo = mid + 1; }
  return lo;
}
function applyCut() {
  if (!app.result || !app.ext) return;
  const top = Number($('layerRange').value);
  const lo = $('singleLayer').checked ? top : 0;
  const eo = app.result.extrude_offsets;
  const end = app.cut == null ? eo[top + 1] : firstSegAfter(app.cut, eo[top], eo[top + 1]);
  if (view.extObj && view.extObj.geometry.attributes.instanceStart) view.extObj.geometry.instanceCount = Math.max(0, end - eo[lo]);
  if (view.beads) view.beads.forEach((m, L) => { if (m) m.count = L === top ? Math.max(0, end - eo[top]) : eo[L + 1] - eo[L]; });
}
app.showProgress = (move) => {
  if (!app.result || !app.ext) return;
  if (move == null) { if (app.cut != null) { app.cut = null; applyCut(); } return; }
  app.cut = move;
  const n = app.ext.length / FLOATS;
  if (!n) return;
  const i = Math.min(firstSegAfter(move, 0, n), n - 1);     // next segment to print decides the layer shown
  const layer = app.ext[i * FLOATS + 8];
  if (Number($('layerRange').value) !== layer) { $('layerRange').value = layer; updateLayers(); } else applyCut();
};
function updateVisibility() {
  const preview = (app.mode === 'preview' || app.mode === 'robot') && !!app.result;
  if (view.mesh) {
    view.mesh.visible = !preview || $('showModel').checked;
    view.mesh.material.opacity = preview ? 0.18 : 1;
    view.mesh.material.depthWrite = !preview;
  }
  if (view.extObj) view.extObj.visible = preview;
  if (view.beads) view.beads.forEach((m) => { if (m) m.visible = preview && !!m.userData.inRange; });
  if (view.trvObj) view.trvObj.visible = preview && $('showTravel').checked;
  $('legend').hidden = !preview;
  $('layerBox').hidden = !preview;
}
function renderLegend() {
  const box = $('legendItems');
  box.innerHTML = '';
  const st = app.result.stats;
  const mode = $('colorMode').value;
  const item = (color, label, val) => {
    const d = document.createElement('div'); d.className = 'legend-item';
    d.innerHTML = `<span class="sw" style="background:${color}"></span><span></span><span class="val"></span>`;
    d.children[1].textContent = label; d.children[2].textContent = val || '';
    box.appendChild(d);
  };
  if (mode === 'kind') {
    const by = st.by_kind_mm || {};
    const names = { 1: 'outer wall', 2: 'inner wall', 3: 'sparse infill', 4: 'solid infill', 5: 'brim' };
    Object.keys(KIND_LABELS).forEach((k) => { const mm = by[names[k]] || 0; if (mm > 0) item(KIND_COLORS[k], KIND_LABELS[k], `${fmt(mm / 1000, 1)} m`); });
  } else if (mode === 'level') {
    const maxL = st.flow_max_level, hist = st.flow_level_hist || [];
    for (let l = maxL; l >= 1; l--) if (hist[l]) item('#' + levelColor(l, maxL).getHexString(), `Level ${l}${l === maxL ? ' (full)' : ''}`, `${hist[l]} moves`);
    const unit = app.values.flow_mode === 'rate' ? 'E/s' : 'E/mm';
    const d = document.createElement('div'); d.className = 'muted';
    d.textContent = `Full scale ${fmt(st.flow_full_scale, 2)} ${unit}`;
    box.appendChild(d);
  } else {
    const [a, b] = app.speedRange || [0, 0];
    item('#' + new THREE.Color().setHSL(0, 0.8, 0.5).getHexString(), 'Fast', `${fmt(b, 0)} mm/s`);
    item('#' + new THREE.Color().setHSL(0.62, 0.8, 0.5).getHexString(), 'Slow', `${fmt(a, 0)} mm/s`);
  }
}
function renderStats() {
  const st = app.result?.stats;
  if (!st) return;
  const parts = [
    ['Layers', st.layers],
    ['Print time', fmtTime(st.time_s)],
    ['Path', `${fmt(st.extrude_mm / 1000, 1)} m`],
    ['Material', `${fmt(st.mass_g / 1000, 2)} kg · ${fmt(st.volume_cm3, 0)} cm³`],
    ['Peak flow', `${fmt(st.max_volumetric_mm3s, 0)} mm³/s (${fmt(st.max_volumetric_mm3s * 3.6 * app.values.density / 1000, 2)} kg/h)`],
    ['Flow level changes', `~${st.io_switches_est}`],
    ['Moves', st.moves.toLocaleString()],
    ['Layer time', `${fmt(st.layer_time_min / 60, 1)}–${fmt(st.layer_time_max / 60, 1)} min`],
  ];
  const bar = $('statusbar');
  bar.innerHTML = '';
  parts.forEach(([k, v]) => { const s = document.createElement('span'); s.innerHTML = `${k} <b></b>`; s.querySelector('b').textContent = v; bar.appendChild(s); });
  if (app.result.source === 'gcode') { const s = document.createElement('span'); s.textContent = '· imported G-code'; bar.appendChild(s); }
}
function setStatusNote(text) {
  if (app.result && !app.dirty) return renderStats();
  $('statusbar').innerHTML = '';
  const s = document.createElement('span'); s.textContent = text; $('statusbar').appendChild(s);
}

// ------------------------------------------------------------------ export
async function downloadFrom(path, body) {
  const r = await postJSON(path, body);
  const blob = await r.blob();
  const cd = r.headers.get('Content-Disposition') || '';
  const name = (cd.match(/filename="([^"]+)"/) || [])[1] || 'output';
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  return r.headers;
}
async function exportAct() {
  if (app.dirty && !confirm('Settings changed since the last slice. Export the old toolpath anyway?')) return;
  busy(true, 'Writing Borunte program…');
  try {
    const h = await downloadFrom('/api/export/act', { settings: app.values });
    const info = JSON.parse(h.get('X-Export-Info') || '{}');
    const a = info.actions || {};
    toast(`Borunte package saved: ${h.get('X-Saved-Path')}\n${info.instructions} instructions · ${a[10] || 0} linear moves · ${a[200] || 0} extruder outputs\n${info.log || ''}`, 'info', 12000);
    if (info.robot_checked === false) toast('Not robot-checked: open the Robot tab and run "Check robot path" so the export uses solved tool rotations and a joint move to known start joints.', 'warn', 12000);
    else if (info.robot_ok === false) toast('The robot check found problems with this job. See the Robot tab before running it.', 'error', 12000);
    if (info.instructions > 10000) {
      toast(`This program has ${info.instructions.toLocaleString()} instructions. Check your controller's program size limit before loading it; ` +
        'raise "Path simplify tolerance" or the line width to reduce points.', 'warn', 15000);
    }
  } catch (e) { toast('Export failed: ' + e.message, 'error', 10000); } finally { busy(false); }
}
async function exportGcode() {
  try {
    const h = await downloadFrom('/api/export/gcode', { settings: app.values });
    toast('G-code saved: ' + h.get('X-Saved-Path'));
  } catch (e) { toast('Export failed: ' + e.message, 'error'); }
}

// ------------------------------------------------------------------ profiles
async function loadProfiles(selected) {
  const spec = await (await api('/api/spec')).json();
  const sel = $('profileSelect');
  sel.innerHTML = '<option value="">(defaults)</option>';
  spec.profiles.forEach((p) => sel.add(new Option(p, p)));
  if (selected) sel.value = selected;
}
async function onProfile() {
  const name = $('profileSelect').value;
  try {
    setValues(name ? await (await api('/api/profile?name=' + encodeURIComponent(name))).json() : app.defaults);
    if (app.model) applyTransform();
    if (app.result?.source === 'slice') setDirty(true);
    toast(name ? `Profile loaded: ${name}` : 'Defaults restored', 'info', 2500);
  } catch (e) { toast(e.message, 'error'); }
}
async function saveProfile() {
  const name = prompt('Profile name', $('profileSelect').value || 'My FGF profile');
  if (!name) return;
  try {
    await postJSON('/api/profile', { name, settings: app.values });
    await loadProfiles(name.replace(/[^A-Za-z0-9 _.-]/g, '_').trim());
    toast('Profile saved: ' + name, 'info', 2500);
  } catch (e) { toast(e.message, 'error'); }
}

// ------------------------------------------------------------------ ui state
function busy(on, text = '') {
  app.busy = on;
  $('progress').hidden = !on;
  if (on) progress(0.02, text);
  refreshButtons();
}
function progress(frac, text) { $('progressBar').style.width = `${Math.round(frac * 100)}%`; $('progressText').textContent = text; }
function refreshButtons() {
  $('btnSlice').disabled = app.busy || !app.model;
  const has = !!app.result && !app.busy;
  ['btnExportAct', 'btnExportGcode'].forEach((id) => { $(id).disabled = !has; });
  $('tabPreview').disabled = !app.result;
}
function setMode(m) {
  const was = app.mode;
  app.mode = m;
  $('tabPrepare').classList.toggle('active', m === 'prepare');
  $('tabPreview').classList.toggle('active', m === 'preview');
  $('tabRobot').classList.toggle('active', m === 'robot');
  $('sideRobot').hidden = m !== 'robot';
  $('sidePrepare').hidden = m === 'robot';
  if (robotUI) { if (m === 'robot') robotUI.enter(); else if (was === 'robot') robotUI.leave(); }
  updateVisibility();
}

// ------------------------------------------------------------------ boot
async function boot() {
  initView();
  const spec = await (await api('/api/spec')).json();
  app.spec = spec.spec; app.defaults = spec.defaults;
  setValues(restore());
  await loadProfiles();
  setView('iso');

  $('fileInput').addEventListener('change', (e) => { if (e.target.files[0]) openFile(e.target.files[0]); e.target.value = ''; });
  (await (await api('/api/samples')).json()).forEach((f) => $('sampleSelect').add(new Option(f, f)));
  $('sampleSelect').addEventListener('change', async (e) => {
    const name = e.target.value;
    e.target.value = '';
    if (!name) return;
    const blob = await (await api('/samples/' + encodeURIComponent(name))).blob();
    openFile(new File([blob], name));
  });
  $('btnSlice').addEventListener('click', slice);
  $('btnExportAct').addEventListener('click', exportAct);
  $('btnExportGcode').addEventListener('click', exportGcode);
  $('tabPrepare').addEventListener('click', () => setMode('prepare'));
  $('tabPreview').addEventListener('click', () => app.result && setMode('preview'));
  $('tabRobot').addEventListener('click', () => setMode('robot'));
  robotUI = initRobot({ app, view, $, api, postJSON, toast, fieldRow, persist, fmt, fmtTime, updateLayers, fitView: () => setView('fit'),
    mergeValues: (vals) => setValues({ ...app.values, ...vals }) });
  $('layerRange').addEventListener('input', () => { app.cut = null; updateLayers(); });
  $('singleLayer').addEventListener('change', updateLayers);
  try { $('showAs').value = localStorage.getItem('yora.showAs') || 'lines'; } catch (_) { /* storage blocked */ }
  $('showAs').addEventListener('change', () => {
    try { localStorage.setItem('yora.showAs', $('showAs').value); } catch (_) { /* storage blocked */ }
    updateLayers();
  });
  $('showTravel').addEventListener('change', updateVisibility);
  $('showModel').addEventListener('change', updateVisibility);
  $('colorMode').addEventListener('change', () => { buildColors(); updateLayers(); renderLegend(); });
  $('profileSelect').addEventListener('change', onProfile);
  $('btnSaveProfile').addEventListener('click', saveProfile);
  $('btnCenter').addEventListener('click', () => applyTransform(true, true));
  ['sx', 'sy', 'sz', 'rx', 'ry', 'rz', 'px', 'py'].forEach((id) => $(id).addEventListener('input', onTransformInput));
  document.querySelectorAll('.view-tools button').forEach((b) => b.addEventListener('click', () => setView(b.dataset.view)));
  document.querySelectorAll('.show-menu input[data-marker]').forEach((c) => {
    c.checked = !!app.markers[c.dataset.marker];
    c.addEventListener('change', () => { app.markers[c.dataset.marker] = c.checked; applyMarkers(); });
  });
  window.addEventListener('keydown', (e) => {
    if (e.target.matches('input, select, textarea')) return;
    if (!app.result) return;
    const r = $('layerRange');
    if (e.key === 'ArrowUp') { r.value = Math.min(Number(r.max), Number(r.value) + 1); app.cut = null; updateLayers(); }
    if (e.key === 'ArrowDown') { r.value = Math.max(0, Number(r.value) - 1); app.cut = null; updateLayers(); }
  });
  const vp = $('viewport');
  vp.addEventListener('dragover', (e) => { e.preventDefault(); $('dropHint').classList.remove('hidden'); $('dropHint').classList.add('drag'); });
  vp.addEventListener('dragleave', () => { $('dropHint').classList.remove('drag'); if (app.model || app.result) $('dropHint').classList.add('hidden'); });
  vp.addEventListener('drop', (e) => {
    e.preventDefault(); $('dropHint').classList.remove('drag');
    const f = e.dataTransfer.files[0];
    if (f) openFile(f);
  });
  refreshButtons();
}
boot().catch((e) => toast('Startup failed: ' + e.message + '\n(The 3D view needs internet access once to load three.js.)', 'error', 20000));
