import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { initRobot } from './robot.js';
import { initPostEditor } from './post.js';
import { initMaterials } from './material.js';
import { initGizmo } from './gizmo.js';
import { initPattern } from './pattern.js';
import { initHistory } from './history.js';

const $ = (id) => document.getElementById(id);
const STORE_KEY = 'yora.settings.v1';
const NO_RESLICE_GROUPS = new Set(['Robot & bed', 'Extruder I/O']);
// settings groups of the machine: shown in the Setup step (Bed & controller), not in the Prepare form
const MACHINE_GROUPS = ['Robot & bed', 'Extruder I/O'];
// Colours of the 3D view, legend and charts come from the CSS tokens (tokens.css), read at run time. three.js does
// not understand OKLCH, so cssColor() converts it (OKLCH -> OKLab -> linear sRGB). The palette is read again on
// every theme change (applyTheme), so nothing is duplicated here.
const KIND_TOKENS = { 1: '--kind-outer', 2: '--kind-inner', 3: '--kind-sparse', 4: '--kind-solid', 5: '--kind-brim', 7: '--kind-raft' };
function cssColor(name, c = new THREE.Color()) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const m = v.match(/^oklch\(\s*([\d.]+)(%?)\s+([\d.]+)\s+([\d.]+)/i);
  if (!m) return v ? c.set(v) : c.set(0x888888);
  const L = parseFloat(m[1]) / (m[2] ? 100 : 1), C = parseFloat(m[3]), h = parseFloat(m[4]) * Math.PI / 180;
  const a = C * Math.cos(h), b = C * Math.sin(h);
  const l = (L + 0.3963377774 * a + 0.2158037573 * b) ** 3, mm = (L - 0.1055613458 * a - 0.0638541728 * b) ** 3,
    s = (L - 0.0894841775 * a - 1.2914855480 * b) ** 3;
  const cl = (x) => Math.min(1, Math.max(0, x));
  return c.setRGB(cl(4.0767416621 * l - 3.3077115913 * mm + 0.2309699292 * s),
    cl(-1.2684380046 * l + 2.6097574011 * mm - 0.3413193965 * s),
    cl(-0.0041960863 * l - 0.7034186147 * mm + 1.7076147010 * s), THREE.LinearSRGBColorSpace);
}
let PAL = null;
function palette() { return PAL || (PAL = readPalette()); }
function readPalette() {
  const hex = (n) => cssColor(n).getHex();
  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  return {
    bg: cssColor('--viewport'), plate: hex('--surface-1'), grid: hex('--grid'), box: hex('--grid-major'),
    model: hex('--model'), travel: hex('--travel'), points: hex('--points'),
    plane: hex('--guide'), base: hex('--guide-2'), seam: hex('--seam'),
    kinds: Object.fromEntries(Object.entries(KIND_TOKENS).map(([k, t]) => [k, cssColor(t)])),
    kindNone: cssColor('--kind-none'),
    ramp: [0, 1, 2, 3, 4].map((i) => cssColor(`--ramp-${i}`)),
    tool: hex('--tool'), joint: hex('--robot-joint'), arm: hex('--text-dim'), tip: hex('--text'),
    danger: hex('--danger'), ok: hex('--ok'),
    err: { 1: hex('--err-reach'), 2: hex('--err-limit'), 4: hex('--err-sing'), 8: hex('--err-config'), 32: hex('--err-collision'), 64: hex('--err-collision') },
    chart: { bg: css('--chart-bg'), line: css('--chart-line'), limit: css('--chart-limit'), text: css('--text-dim'), err: css('--danger') },
  };
}
// colour for t in 0..1 on the sequential data scale (flow level, speed)
function rampColor(t, c = new THREE.Color()) {
  const r = palette().ramp;
  const x = Math.min(Math.max(t, 0), 1) * (r.length - 1), i = Math.min(Math.floor(x), r.length - 2);
  return c.copy(r[i]).lerp(r[i + 1], x - i);
}
const KIND_LABELS = { 1: 'Outer wall', 2: 'Inner wall', 3: 'Sparse infill', 4: 'Solid infill', 5: 'Brim', 7: 'Raft' };
const FLOATS = 13; // x0 y0 z0 x1 y1 z1 kind level layer speed e_per_mm layer_height move
// Settings shown with "Essentials" (the rest is one click away under "All")
const ESSENTIAL = new Set(['layer_height', 'first_layer_height', 'line_width', 'walls', 'top_layers', 'bottom_layers', 'spiral_vase',
  'brim_loops', 'raft_type', 'raft_offset', 'raft_layers', 'raft_angle', 'seam', 'layer_change', 'ramp_length', 'point_mode', 'point_spacing', 'pattern', 'pattern_amplitude', 'pattern_wavelength', 'pattern_shift', 'print_strategy', 'slice_angle', 'slice_direction', 'base_height', 'infill_density', 'infill_pattern', 'outer_wall_speed', 'inner_wall_speed',
  'infill_speed', 'travel_speed', 'flow', 'nozzle_diameter', 'bed_x', 'bed_y', 'blending', 'extruder_enable_do', 'flow_level_dos',
  'start_delay']);

const app = {
  spec: [], defaults: {}, values: {}, model: null, result: null,
  ext: null, trv: null, dirty: false, mode: 'prepare', busy: false,
  hooks: [], onToolpathChanged: null,
  // origin markers in the 3D view, each shown/hidden on its own (view toolbar > Show)
  markers: (() => { try { return { robot: true, tcp: true, bed: true, ...JSON.parse(localStorage.getItem('yora.markers') || '{}') }; } catch (_) { return { robot: true, tcp: true, bed: true }; } })(),
  markerHooks: [],
};
let robotUI = null;
let postUI = null;
let matUI = null;
let gizmoUI = null;
let patternUI = null;

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
    // the page is newer than the app server still running (it loads its code only when started)
    if (r.status === 404 && path.startsWith('/api/') && (msg === 'Unknown endpoint' || msg === 'Not found')) {
      msg = 'The app was updated: close this window and start YoRA Slicer.bat again to use this.';
    }
    throw new Error(msg);
  }
  return r;
}
const postJSON = (path, body) => api(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
const fmtTime = (s) => { const h = Math.floor(s / 3600), m = Math.round((s % 3600) / 60); return h ? `${h} h ${m} min` : `${m} min`; };
const fmt = (v, d = 1) => Number(v).toLocaleString(undefined, { maximumFractionDigits: d, minimumFractionDigits: d });

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
  view.travelMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.55 });   // colour: applyTheme
  view.pointsMat = new THREE.PointsMaterial({ size: 4, sizeAttenuation: false });
  new ResizeObserver(resize).observe(host);
  resize();
  applyTheme();
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
  initPlaneDrag();
  initSeamTools();
  initHelpTips();
  (function loop() { requestAnimationFrame(loop); view.controls.update(); view.renderer.render(view.scene, view.camera); })();
}
function applyTheme() {
  PAL = null;                                        // read the tokens of the new theme
  const p = palette();
  view.scene.background = p.bg.clone();              // flat, no gradient
  view.travelMat.color.setHex(p.travel);
  view.pointsMat.color.setHex(p.points);
  if (view.modelMat) view.modelMat.color.setHex(p.model);
  if (view.selBox) view.selBox.material.color.copy(cssColor('--accent'));
  buildBed();
  updateStrategyViz();
  if (app.ext) { buildColors(); updateLayers(); renderLegend(); }
  document.dispatchEvent(new Event('yora-theme'));   // robot.js repaints the robot, tool and charts
}
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
  const p = palette();
  const plate = new THREE.Mesh(new THREE.PlaneGeometry(bx, by),
    new THREE.MeshStandardMaterial({ color: p.plate, roughness: 0.95 }));
  plate.position.set(bx / 2, by / 2, -0.5);
  view.bed.add(plate);
  const step = bx > 600 || by > 600 ? 100 : 50;
  const pts = [];
  for (let x = 0; x <= bx + 1e-6; x += step) pts.push(x, 0, 0, x, by, 0);
  for (let y = 0; y <= by + 1e-6; y += step) pts.push(0, y, 0, bx, y, 0);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  view.bed.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: p.grid })));
  const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(bx, by, bz)),
    new THREE.LineBasicMaterial({ color: p.box }));
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
  if (app.mode === 'robot' || app.mode === 'setup') {
    b.union(new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(app.values.bed_x || 1000, app.values.bed_y || 1000, 0)));
    if (app.robotGroup?.visible) b.union(new THREE.Box3().setFromObject(app.robotGroup));
  }
  return b;
}

// ------------------------------------------------------------------ settings form
app.detail = (() => { try { return localStorage.getItem('yora.detail') || 'essential'; } catch (_) { return 'essential'; } })();
// settings groups the user has open (the first group on a first start)
function openGroups() {
  if (!app.openGroups) {
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem('yora.openGroups')); } catch (_) { /* none */ }
    app.openGroups = new Set(Array.isArray(saved) ? saved : [app.spec[0] && app.spec[0].group]);
  }
  return app.openGroups;
}
function buildForm() {
  const form = $('settingsForm');
  form.innerHTML = '';
  const q = ($('settingsSearch')?.value || '').trim().toLowerCase();
  const all = app.detail === 'all' || !!q;
  const groups = [...new Set(app.spec.map((f) => f.group))].filter((g) => g !== 'Robot cell' && !MACHINE_GROUPS.includes(g)); // shown in Setup
  let shown = 0;
  groups.forEach((group) => {
    const fields = app.spec.filter((f) => f.group === group && !f.hidden && (all || ESSENTIAL.has(f.key)) &&
      (!q || f.label.toLowerCase().includes(q) || (f.help || '').toLowerCase().includes(q) || f.key.includes(q)));
    if (!fields.length) return;
    shown += fields.length;
    const det = document.createElement('details');
    det.open = q ? true : openGroups().has(group);
    det.innerHTML = `<summary>${group}<span class="count"></span></summary><div class="fields"></div>`;
    const box = det.querySelector('.fields');
    fields.forEach((f) => { const row = fieldRow(f); if (q) row.classList.add('search-hit'); box.appendChild(row); });
    const changed = app.spec.filter((f) => f.group === group && app.values[f.key] !== app.defaults[f.key]).length;
    det.querySelector('.count').textContent = changed ? `${changed} changed` : '';
    // each group stays open until it is closed (remembered; not while searching, where all matches open)
    det.addEventListener('toggle', () => {
      if (q) return;
      const open = openGroups();
      if (det.open) open.add(group); else open.delete(group);
      try { localStorage.setItem('yora.openGroups', JSON.stringify([...open])); } catch (_) { /* storage blocked */ }
    });
    form.appendChild(det);
  });
  if (!shown) form.innerHTML = '<div class="no-results">No setting matches. Try another word, or switch to All.</div>';
}
// Setup step, Bed & controller: the program language (the same choice as in Export) and the machine settings groups
function buildMachineForm() {
  const box = $('machineForm');
  if (!box || !app.spec.length) return;
  box.innerHTML = '';
  const post = document.createElement('details');
  post.open = true;
  post.innerHTML = '<summary>Controller program</summary><div class="fields"><div class="field"><label class="name" for="setupPost">Post-processor' +
    '<small>The robot program language and events used by Export (Borunte, KUKA, G-code or your own). It can still be changed in Export.</small></label>' +
    '<div class="ctl"><select id="setupPost"></select></div></div></div>';
  box.appendChild(post);
  post.querySelector('.name').classList.add('has-help');
  $('setupPost').addEventListener('change', (e) => {
    const sel = $('postSelect');
    sel.value = e.target.value;
    sel.dispatchEvent(new Event('change', { bubbles: true }));
    updateMachineLine();
  });
  fillSetupPost();
  for (const group of MACHINE_GROUPS) {
    const fields = app.spec.filter((f) => f.group === group && !f.hidden);
    if (!fields.length) continue;
    const det = document.createElement('details');
    det.open = true;
    det.innerHTML = `<summary>${group === 'Robot & bed' ? 'Bed & controller numbers' : group}</summary><div class="fields"></div>`;
    fields.forEach((f) => det.querySelector('.fields').appendChild(fieldRow(f)));
    box.appendChild(det);
  }
}
function fillSetupPost() {
  const src = $('postSelect'), dst = $('setupPost');
  if (!src || !dst) return;
  dst.innerHTML = src.innerHTML;
  dst.value = app.values.post_name || src.value;
}
// the machine the job is sliced for, at the top of Prepare
function updateMachineLine() {
  const v = app.values || {};
  let cell = '';
  try { cell = localStorage.getItem('yora.cell') || ''; } catch (_) { /* storage blocked */ }
  const bed = [v.bed_x, v.bed_y, v.bed_z].map((n) => fmt(Number(n) || 0, 0)).join(' × ');
  const parts = [cell || v.robot_model || 'No robot', `bed ${bed} mm`, v.post_name].filter(Boolean);
  if ($('machineText')) { $('machineText').textContent = parts.join(' · '); $('machineText').title = parts.join('\n'); }
}
function fieldRow(f) {
  const row = document.createElement('div');
  row.className = 'field';
  row.dataset.key = f.key;
  const name = document.createElement('label');
  name.className = 'name';
  name.textContent = f.label;
  name.htmlFor = 'f_' + f.key;
  if (f.help) { const sm = document.createElement('small'); sm.textContent = f.help; name.appendChild(sm); name.classList.add('has-help'); }
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
  app.history?.mark();
  let v = readInput(f, input);
  if ((f.type === 'int' || f.type === 'float')) {
    if (!Number.isFinite(v)) v = app.defaults[f.key];
    if (f.min !== undefined) v = Math.max(f.min, v);
    if (f.max !== undefined) v = Math.min(f.max, v);
    input.value = v;
  }
  app.values[f.key] = v;
  markChanged(input.closest('.field'), f);
  const det = input.closest('details');
  if (det && det.querySelector('.count')) {
    const n = app.spec.filter((g) => g.group === f.group && app.values[g.key] !== app.defaults[g.key]).length;
    det.querySelector('.count').textContent = n ? `${n} changed` : '';
  }
  persist();
  if (f.key.startsWith('bed_')) { buildBed(); if (app.model) applyTransform(); }
  if (STRATEGY_KEYS.has(f.key)) updateStrategyViz();
  if (f.key.startsWith('seam')) updateSeamViz();
  if (app.result && app.result.source === 'slice' && !NO_RESLICE_GROUPS.has(f.group)) setDirty(true);
  if (f.group === 'Extruder I/O' && app.result) toast('Extruder I/O changed: exports use the new values. The flow-level preview updates on the next slice.', 'info', 4000);
  app.hooks.forEach((h) => h(f));
}
function persist() { try { localStorage.setItem(STORE_KEY, JSON.stringify(app.values)); } catch (_) { /* storage unavailable */ } }
function restore() { try { return JSON.parse(localStorage.getItem(STORE_KEY) || '{}'); } catch (_) { return {}; } }
function setValues(vals) {
  app.values = { ...app.defaults };
  for (const k of Object.keys(app.defaults)) if (vals && k in vals) app.values[k] = vals[k];
  buildForm(); buildMachineForm(); buildBed(); persist(); updateStrategyViz(); updateSeamViz(); updateMachineLine();
  if (matUI) matUI.show();
  if (robotUI) robotUI.rebuild();
}
function setDirty(d) {
  app.dirty = d;
  if (d) setStatusNote('Settings changed — slice again to update the toolpath.');
}

// ------------------------------------------------------------------ model
async function openFile(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  if (ext === 'yora') return openProject(file);
  if (ext === 'robot') {   // robot model file: load it as the robot model and show the Setup step
    setMode('setup');
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
      app.model = null; app.selected = null;
      app.history?.reset();
      updateObjectPanel();
      $('dropHint').classList.add('hidden');
      await showResult(res);
      const ig = Object.entries(res.import?.ignored || {}).map(([k, v]) => `${k}×${v}`).join(', ');
      toast(`Imported ${file.name}: ${res.stats.layers} layers (layers from ${res.import.layers_from}).${ig ? '\nIgnored: ' + ig : ''}`);
    } else {
      const first = !app.model;
      app.history?.mark();
      app.plateBusy = (app.plateBusy || 0) + 1;
      let res;
      try { res = await (await api('/api/model?' + q, { method: 'POST', body })).json(); } finally { app.plateBusy--; }
      app.model = res.model;
      app.selected = res.added;
      app.result = null; clearToolpath();
      await loadMesh();
      updateObjectPanel();
      $('dropHint').classList.add('hidden');
      setMode('prepare');
      if (first) setView('iso');
      const added = res.model.parts.find((p) => p.id === res.added);
      setStatusNote(`${file.name}: ${added.triangles.toLocaleString()} triangles` +
        (res.model.parts.length > 1 ? `, added as part ${res.model.parts.length}.` : '. Set the print settings, then Slice.'));
    }
  } catch (e) { toast(e.message, 'error'); } finally { busy(false); refreshButtons(); }
}
// The parts on the bed: view.mesh is a group with one mesh per part (userData.id), all sharing view.modelMat.
// view.meshPos holds every vertex (x y z ...) of all parts.
async function loadMesh() {
  const buf = await (await api('/api/model.bin')).arrayBuffer();
  clearMesh();
  const all = new Float32Array(buf);
  view.meshPos = all;
  if (!view.modelMat) view.modelMat = new THREE.MeshStandardMaterial({ color: palette().model, roughness: 0.55, metalness: 0.05, transparent: true, opacity: 1 });
  view.mesh = new THREE.Group();
  view.parts = {};
  const parts = app.model?.parts || [{ id: 0, start: 0, triangles: all.length / 9 }];
  for (const p of parts) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(all.slice(p.start * 9, (p.start + p.triangles) * 9), 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, view.modelMat);
    m.userData.id = p.id;
    view.parts[p.id] = m;
    view.mesh.add(m);
  }
  view.scene.add(view.mesh);
  if (!parts.some((p) => p.id === app.selected)) app.selected = parts.length ? parts[parts.length - 1].id : null;
  updateSelection();
  $('seamPanel').hidden = false;
  updateStrategyViz();
  updateSeamViz();
  if (gizmoUI) gizmoUI.refresh();
  updateVisibility();
}
// ---- help pop-up: rest the pointer on a setting's name for HELP_DELAY_MS (or click the name) to read its help ----
const HELP_DELAY_MS = 3000;
function initHelpTips() {
  const tip = document.createElement('div');
  tip.className = 'help-tip';
  tip.setAttribute('role', 'tooltip');
  document.body.appendChild(tip);
  let timer = 0, cur = null;
  const hide = () => { clearTimeout(timer); cur = null; tip.classList.remove('show'); };
  const show = (name) => {
    const help = name.querySelector('small');
    if (!help) return;
    tip.innerHTML = '';
    const b = document.createElement('b');
    b.textContent = name.firstChild ? name.firstChild.textContent : '';
    tip.append(b, document.createTextNode(help.textContent));
    const r = name.getBoundingClientRect();
    tip.style.left = '0px'; tip.style.top = '0px';
    tip.classList.add('show');
    const w = tip.offsetWidth, h = tip.offsetHeight;
    // left of the settings panel when there is room, else under the name; always inside the window
    let x = r.left - w - 12, y = r.top - 4;
    if (x < 8) { x = Math.min(r.left, window.innerWidth - w - 8); y = r.bottom + 6; }
    if (y + h > window.innerHeight - 8) y = Math.max(8, r.top - h - 6);
    tip.style.left = `${x}px`; tip.style.top = `${y}px`;
  };
  document.addEventListener('mouseover', (ev) => {
    const name = ev.target.closest && ev.target.closest('.field .name.has-help');
    if (name === cur) return;
    hide();
    if (!name) return;
    cur = name;
    timer = setTimeout(() => show(name), HELP_DELAY_MS);
  });
  document.addEventListener('click', (ev) => {
    const name = ev.target.closest && ev.target.closest('.field .name.has-help');
    if (!name) return hide();
    clearTimeout(timer); cur = name; show(name);
  });
  document.addEventListener('scroll', hide, true);
  document.addEventListener('keydown', (ev) => { if (ev.key === 'Escape') hide(); });
}

// ---- seam tools (Prepare): pick a point / draw a line on the part; the current seam shown on the model ----
const seamTool = { mode: null, pts: [], strokes: [] };
function setSetting(key, v) {   // through the settings form, so it is saved and marks the slice out of date
  const inp = $('f_' + key);
  if (inp) {
    if (inp.type === 'checkbox') inp.checked = !!v; else inp.value = v;
    inp.dispatchEvent(new Event('change', { bubbles: true }));
    return;
  }
  app.history?.mark();
  app.values[key] = v; persist();
  if (app.result && app.result.source === 'slice') setDirty(true);
  app.hooks.forEach((h) => h(app.spec.find((f) => f.key === key) || { key, group: '' }));
  if (key.startsWith('seam')) updateSeamViz();
}
function parseSeamPath(text) {
  return String(text || '').split(';').map((p) => p.split(',').map(Number)).filter((v) => v.length >= 3 && v.every(Number.isFinite));
}
function updateSeamViz() {
  if (view.seamViz) {
    view.scene.remove(view.seamViz);
    view.seamViz.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    view.seamViz = null;
  }
  const v = app.values || {};
  const label = { aligned: 'aligned', rear: 'rear', direction: `direction ${v.seam_angle}°`, custom: 'point',
    path: 'drawn line', nearest: 'nearest', random: 'random' }[v.seam] || v.seam;
  if ($('seamState')) $('seamState').textContent = seamTool.mode ? '' : label;
  if (!view.mesh) return;
  const col = palette().seam, g = new THREE.Group();
  const box = new THREE.Box3().setFromObject(view.mesh);
  const r = Math.max(2, box.getSize(new THREE.Vector3()).length() * 0.006);
  const dot = (p) => { const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 10), new THREE.MeshBasicMaterial({ color: col, depthTest: false, transparent: true })); m.position.copy(p); m.renderOrder = 9; g.add(m); };
  const line = (pts) => {
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const l = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: col, depthTest: false, transparent: true }));
    l.renderOrder = 9; g.add(l);
  };
  let pts = null;
  if (seamTool.mode === 'draw') pts = seamTool.pts.map((p) => new THREE.Vector3(...p));
  else if (v.seam === 'path') pts = parseSeamPath(v.seam_path).map((p) => new THREE.Vector3(...p));
  if (pts) {
    if (pts.length > 1) line(pts);
    pts.forEach((p, i) => { if (i === 0 || i === pts.length - 1 || pts.length < 30) dot(p); });
  } else if (v.seam === 'custom') {   // a point: the seam is the part's surface closest to this vertical line
    const x = Number(v.seam_x), y = Number(v.seam_y);
    line([new THREE.Vector3(x, y, box.min.z), new THREE.Vector3(x, y, box.max.z)]);
    dot(new THREE.Vector3(x, y, (box.min.z + box.max.z) / 2));
  } else if (v.seam === 'direction' || v.seam === 'rear') {
    const a = (v.seam === 'rear' ? 90 : Number(v.seam_angle)) * Math.PI / 180;
    const c = box.getCenter(new THREE.Vector3()), d = new THREE.Vector3(Math.cos(a), Math.sin(a), 0);
    const L = box.getSize(new THREE.Vector3()).length() * 0.45;
    g.add(new THREE.ArrowHelper(d, c.clone().addScaledVector(d, L * 0.6), L * 0.5, col, L * 0.08, L * 0.05));
  }
  if (!g.children.length) return;
  view.seamViz = g;
  view.scene.add(g);
  updateVisibility();
}
function initSeamTools() {
  const el = view.renderer.domElement, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  const hit = (ev) => {
    if (!view.mesh) return null;
    const r = el.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, view.camera);
    view.modelMat.side = THREE.DoubleSide;
    const h = ray.intersectObject(view.mesh, true)[0];
    return h ? h.point : null;
  };
  const defaults = { ...view.controls.mouseButtons };
  const hints = {
    pick: 'Click on the part where the seam should be. Esc cancels.',
    draw: 'Left-drag (or click) on the part to paint the seam line, bottom to top. Right-drag turns the view. Enter or Done finishes, Backspace undoes, Esc cancels.',
  };
  const setTool = (m) => {
    seamTool.mode = m;
    $('seamPick').classList.toggle('on', m === 'pick');
    $('seamDraw').classList.toggle('on', m === 'draw');
    $('seamDraw').textContent = m === 'draw' ? 'Done' : 'Draw line';
    $('seamUndo').hidden = m !== 'draw';
    $('seamHint').textContent = m ? hints[m] : 'Pick a point or draw a line on the part to place the seam. More options under Seam & layer change.';
    // drawing: the left button paints, the right button turns the view
    view.controls.mouseButtons = m === 'draw' ? { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE } : { ...defaults };
    el.style.cursor = m ? 'crosshair' : '';
    if (m && app.mode !== 'prepare') setMode('prepare');
    updateSeamViz();
  };
  const finishDraw = () => {
    const pts = seamTool.pts;
    setTool(null);
    if (pts.length) {
      setSetting('seam_path', pts.map((p) => p.map((c) => c.toFixed(2)).join(',')).join('; '));
      setSetting('seam', 'path');
      toast(`Seam line set (${pts.length} points). Slice to apply.`, 'info', 4000);
    }
  };
  $('seamPick').addEventListener('click', () => setTool(seamTool.mode === 'pick' ? null : 'pick'));
  $('seamDraw').addEventListener('click', () => {
    if (seamTool.mode === 'draw') return finishDraw();
    seamTool.pts = []; seamTool.strokes = [];
    setTool('draw');
  });
  $('seamUndo').addEventListener('click', () => { seamTool.pts.splice(seamTool.strokes.pop() ?? 0); updateSeamViz(); });
  $('seamReset').addEventListener('click', () => { setTool(null); setSetting('seam', 'aligned'); });
  let down = null, painting = false;
  el.addEventListener('pointerdown', (ev) => {
    if (!seamTool.mode || ev.button !== 0) return;
    down = [ev.clientX, ev.clientY];
    if (seamTool.mode === 'draw') {
      const p = hit(ev);
      if (!p) return;
      painting = true;
      seamTool.strokes.push(seamTool.pts.length);
      seamTool.pts.push([p.x, p.y, p.z]);
      el.setPointerCapture(ev.pointerId);
      updateSeamViz();
      ev.stopImmediatePropagation();
    }
  }, true);
  el.addEventListener('pointermove', (ev) => {
    if (!painting) return;
    const p = hit(ev);
    if (!p) return;
    const last = seamTool.pts[seamTool.pts.length - 1];
    if (Math.hypot(p.x - last[0], p.y - last[1], p.z - last[2]) < 3) return;   // a point every 3 mm
    seamTool.pts.push([p.x, p.y, p.z]);
    cancelAnimationFrame(seamTool.frame);
    seamTool.frame = requestAnimationFrame(updateSeamViz);
  }, true);
  el.addEventListener('pointerup', (ev) => {
    if (painting) { painting = false; try { el.releasePointerCapture(ev.pointerId); } catch (_) { /* released */ } return; }
    if (seamTool.mode !== 'pick' || !down || Math.hypot(ev.clientX - down[0], ev.clientY - down[1]) > 4) return;   // a drag turns the view
    const p = hit(ev);
    down = null;
    if (!p) return toast('Click on the part.', 'warn', 2500);
    setTool(null);
    setSetting('seam_x', Math.round(p.x * 100) / 100);
    setSetting('seam_y', Math.round(p.y * 100) / 100);
    setSetting('seam', 'custom');
    toast(`Seam point set at X ${fmt(p.x, 1)}, Y ${fmt(p.y, 1)}. Slice to apply.`, 'info', 4000);
  }, true);
  window.addEventListener('keydown', (ev) => {
    if (!seamTool.mode || ev.target.matches('input, textarea, select')) return;
    if (ev.key === 'Escape') { seamTool.pts = []; setTool(null); }
    else if (ev.key === 'Enter' && seamTool.mode === 'draw') finishDraw();
    else if (ev.key === 'Backspace' && seamTool.mode === 'draw') { seamTool.pts.pop(); updateSeamViz(); ev.preventDefault(); }
  });
}

// ---- printing strategy: the slicing planes drawn on the part (Prepare) ----
// Mouse: drag the ball at the tip of the plane's arrow. The arrow (layer normal) follows the pointer on a sphere around
// the plane centre: its tilt from vertical is the slicing angle, its heading the tilt direction. Shift = 5 deg steps.
const planeDrag = { active: false, hover: false, centre: null, radius: 1, frame: 0 };
function initPlaneDrag() {
  const el = view.renderer.domElement, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  const pick = (ev) => {
    const r = el.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, view.camera);
    return ray;
  };
  const onHandle = (ev) => view.strategy && view.strategy.visible && view.planeHandle &&
    pick(ev).intersectObject(view.planeHandle, false).length > 0;
  const fieldSet = (key, v) => {   // through the settings form, so it is saved and marks the slice out of date
    const inp = $('f_' + key);
    if (inp) { inp.value = v; inp.dispatchEvent(new Event('change', { bubbles: true })); return; }
    app.values[key] = v; persist();
    if (app.result && app.result.source === 'slice') setDirty(true);
  };
  el.addEventListener('pointerdown', (ev) => {
    if (ev.button !== 0 || !onHandle(ev)) return;
    planeDrag.active = true;
    app.history?.mark();
    planeDrag.start = [app.values.slice_angle, app.values.slice_direction];
    view.controls.enabled = false;
    el.setPointerCapture(ev.pointerId);
    el.style.cursor = 'grabbing';
    ev.preventDefault(); ev.stopImmediatePropagation();   // the orbit controls don't see this press
  }, true);
  el.addEventListener('pointermove', (ev) => {
    if (!planeDrag.active) {
      const h = !!onHandle(ev);
      if (h !== planeDrag.hover) {
        planeDrag.hover = h; el.style.cursor = h ? 'grab' : '';
        updateStrategyViz(true);
        if (h) setStatusNote('Drag to tilt and turn the slicing plane (Shift: 5° steps).');
      }
      return;
    }
    // where the pointer ray meets the sphere around the plane centre (or its closest approach)
    const r = pick(ev).ray, c = planeDrag.centre, R = planeDrag.radius;
    const oc = r.origin.clone().sub(c), b = oc.dot(r.direction), disc = b * b - (oc.lengthSq() - R * R);
    const p = disc >= 0 ? r.at(-b - Math.sqrt(disc), new THREE.Vector3()) : r.closestPointToPoint(c, new THREE.Vector3());
    const n = p.sub(c).normalize();
    const step = ev.shiftKey ? 5 : 1;
    let ang = Math.acos(Math.max(-1, Math.min(1, n.z))) * 180 / Math.PI;
    let dir = Math.atan2(n.y, n.x) * 180 / Math.PI;
    ang = Math.min(80, Math.round(ang / step) * step);
    dir = ang < 0.5 ? app.values.slice_direction : Math.round(dir / step) * step;
    if (ang === app.values.slice_angle && dir === app.values.slice_direction) return;
    app.history?.mark();
    app.values.slice_angle = ang; app.values.slice_direction = dir;
    if ($('f_slice_angle')) $('f_slice_angle').value = ang;
    if ($('f_slice_direction')) $('f_slice_direction').value = dir;
    setStatusNote(`Slicing plane ${ang}° · direction ${dir}°`);
    cancelAnimationFrame(planeDrag.frame);
    planeDrag.frame = requestAnimationFrame(() => updateStrategyViz(true));
  }, true);
  const end = (ev) => {
    if (!planeDrag.active) return;
    planeDrag.active = false;
    view.controls.enabled = true;
    el.style.cursor = planeDrag.hover ? 'grab' : '';
    try { el.releasePointerCapture(ev.pointerId); } catch (_) { /* already released */ }
    const [a, d] = [app.values.slice_angle, app.values.slice_direction];
    if (a !== planeDrag.start[0]) fieldSet('slice_angle', a);
    if (d !== planeDrag.start[1]) fieldSet('slice_direction', d);
    updateStrategyViz();
  };
  el.addEventListener('pointerup', end, true);
  el.addEventListener('pointercancel', end, true);
}
const STRATEGY_KEYS = new Set(['print_strategy', 'slice_angle', 'slice_direction', 'base_height', 'layer_height']);
function strategyAxes() {   // slice frame R = Rz(direction) Ry(angle): u = column 0, v = column 1, n = column 2 (layer normal)
  const t = Number(app.values.slice_angle || 0) * Math.PI / 180, p = Number(app.values.slice_direction || 0) * Math.PI / 180;
  return {
    u: new THREE.Vector3(Math.cos(p) * Math.cos(t), Math.sin(p) * Math.cos(t), -Math.sin(t)),
    v: new THREE.Vector3(-Math.sin(p), Math.cos(p), 0),
    n: new THREE.Vector3(Math.cos(p) * Math.sin(t), Math.sin(p) * Math.sin(t), Math.cos(t)),
  };
}
// contour lines where the planes dot(p, n) = d (d in ds) cut the mesh, kept where bed z >= zmin
function meshContours(pos, n, ds, zmin) {
  const out = [];
  const h = new Float32Array(3), P = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
  const A = new THREE.Vector3(), B = new THREE.Vector3(), e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), N = new THREE.Vector3();
  const d0 = ds[0], step = ds.length > 1 ? ds[1] - ds[0] : 1;
  for (let t = 0; t < pos.length; t += 9) {
    for (let k = 0; k < 3; k++) { P[k].set(pos[t + 3 * k], pos[t + 3 * k + 1], pos[t + 3 * k + 2]); h[k] = P[k].dot(n); }
    const lo = Math.min(h[0], h[1], h[2]), hi = Math.max(h[0], h[1], h[2]);
    const i0 = Math.max(0, Math.ceil((lo - d0) / step)), i1 = Math.min(ds.length - 1, Math.floor((hi - d0) / step));
    if (i1 < i0) continue;
    N.crossVectors(e1.subVectors(P[1], P[0]), e2.subVectors(P[2], P[0])).normalize().multiplyScalar(0.4);   // lift off the surface
    for (let i = i0; i <= i1; i++) {
      const d = ds[i];
      let m = 0;
      for (const [a, b] of [[0, 1], [1, 2], [2, 0]]) {
        if ((h[a] - d) * (h[b] - d) < 0) {
          const s = (d - h[a]) / (h[b] - h[a]);
          (m === 0 ? A : B).copy(P[a]).lerp(P[b], s); m++;
        }
      }
      if (m !== 2) continue;
      if (A.z < zmin && B.z < zmin) continue;
      if (A.z < zmin) A.lerp(B, (zmin - A.z) / (B.z - A.z)); else if (B.z < zmin) B.lerp(A, (zmin - B.z) / (A.z - B.z));
      out.push(A.x + N.x, A.y + N.y, A.z + N.z, B.x + N.x, B.y + N.y, B.z + N.z);
    }
  }
  return new Float32Array(out);
}
function updateStrategyViz(quick = false) {
  if (view.strategy) {
    view.scene.remove(view.strategy);
    view.strategy.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    view.strategy = null;
  }
  if (!view.mesh || !app.values || app.values.print_strategy !== 'angled') return;
  const pal = palette(), pos = view.meshPos;
  const { u, v, n } = strategyAxes();
  const base = Math.max(Number(app.values.base_height || 0), 0);
  const g = new THREE.Group();
  // range of the planes over the part above the base
  let hmin = Infinity, hmax = -Infinity;
  const c = new THREE.Vector3(), box = new THREE.Box3();
  for (let i = 0; i < pos.length; i += 3) {
    c.set(pos[i], pos[i + 1], pos[i + 2]); box.expandByPoint(c);
    if (c.z < base) continue;
    const h = c.dot(n); hmin = Math.min(hmin, h); hmax = Math.max(hmax, h);
  }
  if (!(hmax > hmin)) return;
  const lh = Math.max(Number(app.values.layer_height || 1), 0.05);
  const count = Math.min(24, Math.max(2, Math.floor((hmax - hmin) / lh)));
  const ds = []; for (let i = 1; i <= count; i++) ds.push(hmin + (hmax - hmin) * i / (count + 1));
  if (!quick || pos.length < 9 * 150000) {   // contour lines (skipped while dragging very large models)
    const lines = new THREE.BufferGeometry();
    lines.setAttribute('position', new THREE.BufferAttribute(meshContours(pos, n, ds, base), 3));
    g.add(new THREE.LineSegments(lines, new THREE.LineBasicMaterial({ color: pal.plane })));
  }
  // one translucent plane through the middle of the part, with an outline
  const size = box.getSize(new THREE.Vector3()).length() * 0.75;
  const centre = box.getCenter(new THREE.Vector3());
  const mid = (hmin + hmax) / 2;
  const o = centre.clone().addScaledVector(n, mid - centre.dot(n));
  const quad = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshBasicMaterial({
    color: pal.plane, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }));
  const basis = new THREE.Matrix4().makeBasis(u, v, n).setPosition(o);
  quad.applyMatrix4(basis);
  g.add(quad);
  const edge = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(size, size)),
    new THREE.LineBasicMaterial({ color: pal.plane, transparent: true, opacity: 0.7 }));
  edge.applyMatrix4(basis);
  g.add(edge);
  // arrow: the direction the layers grow (layer normal)
  g.add(new THREE.ArrowHelper(n, o, box.getSize(new THREE.Vector3()).length() / 2, pal.plane, size * 0.06, size * 0.035));
  // drag handle: a ball at the arrow tip; drag it to tilt and turn the plane (see initPlaneDrag)
  const handle = new THREE.Mesh(new THREE.SphereGeometry(size * 0.035, 24, 16),
    new THREE.MeshBasicMaterial({ color: planeDrag.hover || planeDrag.active ? pal.base : pal.plane, depthTest: false, transparent: true }));
  const reach = box.getSize(new THREE.Vector3()).length() / 2 + size * 0.06;   // outside the part
  handle.position.copy(o).addScaledVector(n, reach);
  handle.renderOrder = 10;   // drawn last, over the (transparent-pass) model
  g.add(handle);
  view.planeHandle = handle;
  if (!planeDrag.active) { planeDrag.centre = o.clone(); planeDrag.radius = reach; }
  // horizontal base: its top outline on the part
  if (base > 0) {
    const bl = new THREE.BufferGeometry();
    bl.setAttribute('position', new THREE.BufferAttribute(meshContours(pos, new THREE.Vector3(0, 0, 1), [base], -Infinity), 3));
    g.add(new THREE.LineSegments(bl, new THREE.LineBasicMaterial({ color: pal.base })));
    const bq = new THREE.Mesh(new THREE.PlaneGeometry(box.max.x - box.min.x + 40, box.max.y - box.min.y + 40),
      new THREE.MeshBasicMaterial({ color: pal.base, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
    bq.position.set(centre.x, centre.y, base);
    g.add(bq);
  }
  g.renderOrder = 3;
  view.strategy = g;
  view.scene.add(g);
  updateVisibility();
}
function clearMesh() {
  if (view.mesh) { view.scene.remove(view.mesh); view.mesh.children.forEach((m) => m.geometry.dispose()); view.mesh = null; }
  view.parts = {}; view.meshPos = null;
  if (view.selBox) { view.scene.remove(view.selBox); view.selBox.geometry.dispose(); view.selBox = null; }
}
const selectedPart = () => app.model?.parts?.find((p) => p.id === app.selected) || null;
function updateObjectPanel() {
  const m = app.model;
  $('objectEmpty').hidden = !!m; $('objectBody').hidden = !m;
  $('btnSaveProject').disabled = !m;
  const part = selectedPart();
  $('btnDuplicate').disabled = $('btnDelete').disabled = !part;
  if (!m || !part) return;
  // the parts on the bed (a list once there is more than one)
  const list = $('partList');
  list.innerHTML = '';
  list.hidden = m.parts.length < 2;
  m.parts.forEach((p, i) => {
    const row = document.createElement('button');
    row.type = 'button';
    row.className = 'part-row' + (p.id === app.selected ? ' on' : '') + (p.fits ? '' : ' bad');
    row.setAttribute('role', 'option');
    row.setAttribute('aria-selected', p.id === app.selected);
    row.innerHTML = '<span class="part-num"></span><span class="part-name"></span><span class="part-size"></span>';
    row.querySelector('.part-num').textContent = i + 1;
    row.querySelector('.part-name').textContent = p.name;
    row.querySelector('.part-size').textContent = p.size.slice(0, 2).map((v) => fmt(v, 0)).join(' × ');
    row.title = p.fits ? p.name : p.name + ': outside the print area';
    row.addEventListener('click', () => selectPart(p.id));
    list.appendChild(row);
  });
  $('objName').textContent = part.name;
  const s = part.size.map((v) => fmt(v, 1)).join(' × ');
  $('objSize').textContent = `${s} mm` + (part.fits ? '' : '  — outside the print area');
  $('objSize').classList.toggle('bad', !part.fits);
  const t = part.transform;
  ['sx', 'sy', 'sz'].forEach((id, i) => { $(id).value = t.scale[i]; });
  ['rx', 'ry', 'rz'].forEach((id, i) => { $(id).value = t.rotation[i]; });
  $('px').value = fmt((part.min[0] + part.max[0]) / 2, 1).replace(/,/g, '');
  $('py').value = fmt((part.min[1] + part.max[1]) / 2, 1).replace(/,/g, '');
}
function selectPart(id) {
  if (id === app.selected || !view.parts?.[id]) return;
  clearTimeout(transformTimer);
  app.selected = id;
  updateSelection();
  updateObjectPanel();
  if (gizmoUI) gizmoUI.refresh();
}
// a box round the selected part (Prepare, when there is more than one part)
function updateSelection() {
  if (view.selBox) { view.scene.remove(view.selBox); view.selBox.geometry.dispose(); view.selBox.material.dispose(); view.selBox = null; }
  const m = view.parts?.[app.selected];
  if (!m || (app.model?.parts?.length || 0) < 2) return;
  view.selBox = new THREE.BoxHelper(m, cssColor('--accent'));
  view.selBox.material.depthTest = false; view.selBox.material.transparent = true; view.selBox.renderOrder = 7;
  view.scene.add(view.selBox);
  view.selBox.visible = app.mode === 'prepare';
}
let transformTimer = null;
function onTransformInput(e) {
  if ($('lockScale').checked && ['sx', 'sy', 'sz'].includes(e.target.id)) ['sx', 'sy', 'sz'].forEach((id) => { $(id).value = e.target.value; });
  clearTimeout(transformTimer);
  transformTimer = setTimeout(() => applyTransform(true), 350);
}
async function applyTransform(useInputs = false, recenter = false) {
  const part = selectedPart();
  if (!part) return;
  app.history?.mark();
  app.plateBusy = (app.plateBusy || 0) + 1;
  const t = part.transform;
  const body = {
    settings: app.values,
    id: part.id,
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
  } catch (e) { toast(e.message, 'error'); } finally { app.plateBusy--; }
}
// Duplicate / Remove the selected part
async function partAction(action) {
  const part = selectedPart();
  if (!part) return;
  app.history?.mark();
  app.plateBusy = (app.plateBusy || 0) + 1;
  try {
    const res = await (await postJSON('/api/object/' + action, { settings: app.values, id: part.id })).json();
    app.model = res.model;
    if (res.added) app.selected = res.added;
    if (!app.model) {
      clearMesh(); app.selected = null; app.result = null; clearToolpath(); updateObjectPanel(); refreshButtons();
      $('seamPanel').hidden = true; setMode('prepare'); updateVisibility();
      setStatusNote('The bed is empty. Open a model or pick a sample.');
      return;
    }
    await loadMesh();
    updateObjectPanel();
    if (app.result?.source === 'slice') setDirty(true);
  } catch (e) { toast(e.message, 'error'); } finally { app.plateBusy--; }
}
// Undo / redo puts back a saved state: the settings, then the parts on the bed (the server keeps removed parts)
async function applyState(st) {
  const changed = Object.keys(st.values).filter((k) => JSON.stringify(st.values[k]) !== JSON.stringify(app.values[k]));
  if (changed.length) {
    setValues(st.values);
    if (patternUI) patternUI.show();
    if (app.result?.source === 'slice') setDirty(true);
  }
  const cur = (app.model?.parts || []).map((p) => ({ id: p.id, transform: p.transform }));
  const want = st.parts.map((p) => ({ id: p.id, transform: p.transform }));
  if (JSON.stringify(cur) !== JSON.stringify(want)) {
    const res = await (await postJSON('/api/plate/restore', { settings: app.values, parts: want })).json();
    app.model = res.model;
    if (!app.model) {
      clearMesh(); app.selected = null; app.result = null; clearToolpath(); updateObjectPanel(); refreshButtons();
      setMode('prepare');
      return;
    }
    if (!app.model.parts.some((p) => p.id === app.selected)) app.selected = null;
    await loadMesh();
    updateObjectPanel();
    if (gizmoUI) gizmoUI.refresh();
    if (app.result?.source === 'slice') setDirty(true);
  } else if (changed.length && app.mode === 'preview' && app.result?.source === 'slice' &&
             changed.every((k) => k.startsWith('pattern') || k.startsWith('overhang'))) {
    slice({ live: true });   // a surface pattern change: the toolpath follows straight away, as it did
  }
}

// ------------------------------------------------------------------ project files (.yora)
// Save: the parts, every setting and the files they use, in one .yora file. Where the browser can (Chrome, Edge),
// a Save dialog picks the place and Ctrl+S then saves to the same file again; otherwise it is downloaded.
// Shift+Save or Ctrl+Shift+S always asks for a new place.
app.projectHandle = null;
async function saveProject(saveAs = false) {
  if (!app.model) return toast('Nothing to save: open a model first.', 'warn', 3000);
  const name = (app.projectName || app.model.parts[0].name).replace(/\.[^.]+$/, '');
  try {
    let handle = saveAs ? null : app.projectHandle;
    if (!handle && window.showSaveFilePicker) {
      try {
        handle = await window.showSaveFilePicker({ suggestedName: name + '.yora',
          types: [{ description: 'YoRA project', accept: { 'application/zip': ['.yora'] } }] });
      } catch (e) { if (e.name === 'AbortError') return; handle = null; }
    }
    busy(true, 'Saving project…');
    const fileName = handle ? handle.name.replace(/\.yora$/i, '') : name;
    const r = await postJSON('/api/project/save', { settings: app.values, name: fileName });
    const blob = await r.blob();
    if (handle) {
      const w = await handle.createWritable();
      await w.write(blob); await w.close();
      app.projectHandle = handle;
    } else {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob); a.download = fileName + '.yora'; a.click();
      setTimeout(() => URL.revokeObjectURL(a.href), 5000);
    }
    app.projectName = fileName;
    toast(`Project saved: ${fileName}.yora (${fmt(blob.size / 1048576, 1)} MB)`, 'info', 4000);
  } catch (e) { toast('Saving failed: ' + e.message, 'error'); } finally { busy(false); }
}
async function openProject(file) {
  busy(true, 'Opening project…');
  try {
    const res = await (await api('/api/project/open', { method: 'POST', body: await file.arrayBuffer() })).json();
    app.projectName = res.name; app.projectHandle = null;
    app.history?.reset();
    setValues(res.settings);
    app.model = res.model; app.selected = null;
    app.result = null; clearToolpath();
    await loadMesh();
    updateObjectPanel();
    $('dropHint').classList.add('hidden');
    setMode('prepare');
    setView('iso');
    if (matUI && matUI.refresh) await matUI.refresh();
    const n = res.model.parts.length;
    setStatusNote(`${res.name}.yora: ${n} part${n > 1 ? 's' : ''}. Slice to see the toolpath.`);
    (res.notes || []).forEach((note) => toast(note, 'info', 6000));
  } catch (e) { toast('Opening failed: ' + e.message, 'error', 8000); } finally { busy(false); refreshButtons(); }
}

// ------------------------------------------------------------------ slicing
// live: a quick update from the Surface pattern card (Preview): no progress overlay, the view and layer stay put,
// warnings go to the Warnings card only. A request while slicing runs once the current slice is done.
async function slice(opts = {}) {
  const live = !!opts.live;
  if (!app.model) return;
  if (app.busy) { if (live) app.livePending = true; return; }
  if (live) { app.busy = true; refreshButtons(); } else busy(true, 'Slicing…');
  if (patternUI) patternUI.show();
  try {
    await postJSON('/api/slice', { settings: app.values });
    for (;;) {
      await new Promise((r) => setTimeout(r, live ? 60 : 250));
      const st = await (await api('/api/status')).json();
      if (st.code_changed) warnRestart();
      if (st.state === 'running') { if (!live) progress(st.progress, `${st.stage === 'planning' ? 'Planning moves' : 'Slicing'}… ${Math.round(st.progress * 100)}%`); continue; }
      if (st.state === 'error') throw new Error(st.error);
      if (st.state === 'done') { setDirty(false); await showResult(st.result, { live }); break; }
    }
  } catch (e) { toast('Slicing failed: ' + e.message, 'error', 10000); } finally {
    if (live) { app.busy = false; refreshButtons(); } else busy(false);
    refreshButtons();
    if (patternUI) patternUI.show();
    if (app.livePending) { app.livePending = false; slice({ live: true }); }
  }
}
async function showResult(res, opts = {}) {
  app.overhang = null;
  const live = !!opts.live;
  const atTop = Number($('layerRange').value) >= Number($('layerRange').max);
  app.result = res;
  const [e, t] = await Promise.all([
    api('/api/toolpath.bin?kind=extrude').then((r) => r.arrayBuffer()),
    api('/api/toolpath.bin?kind=travel').then((r) => r.arrayBuffer()),
  ]);
  app.ext = new Float32Array(e); app.trv = new Float32Array(t);
  const nl = res.layer_z.length;
  const keep = live && !atTop ? Number($('layerRange').value) : null;
  $('layerRange').max = Math.max(nl - 1, 0);
  $('layerRange').value = keep !== null ? Math.min(keep, nl - 1) : Math.max(nl - 1, 0);
  buildColors();
  if (app.onToolpathChanged) app.onToolpathChanged();
  setMode(app.mode === 'robot' ? 'robot' : 'preview');
  updateLayers();
  renderLegend();
  renderStats();
  if (live) return;
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
  const strat = app.result.stats && app.result.stats.strategy;
  const tilted = strat ? new THREE.Vector3(...strat.normal) : null;
  for (let L = 0; L < nl; L++) {
    const a = eo[L], b = eo[L + 1];
    if (b <= a) { view.beads.push(null); continue; }
    up.set(0, 0, 1);
    if (tilted && L >= strat.flat_layers) up.copy(tilted);      // angled layers: beads lie on the tilted plane
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
  if (level <= 0) return palette().kindNone.clone();
  const t = max <= 1 ? 1 : (level - 1) / (max - 1);
  return rampColor(t, new THREE.Color());
}
// overhang angle (deg) of each extrusion row, loaded when the Overhang colouring is first shown
async function loadOverhang() {
  if (app.overhang || app.overhangLoading) return;
  app.overhangLoading = true;
  try {
    app.overhang = new Float32Array(await (await api('/api/toolpath.bin?kind=overhang')).arrayBuffer());
    buildColors(); updateLayers(); renderLegend();
  } catch (e) { toast('Overhang: ' + e.message, 'error'); } finally { app.overhangLoading = false; }
}
function overhangColor(a, kind, limit, c) {
  if (kind !== 1 && kind !== 2) return c.copy(palette().kindNone);          // only walls have an overhang angle
  if (a > limit) return c.setHex(palette().danger);
  return rampColor(1 - a / Math.max(limit, 1), c);                          // flat on the layer below .. at the limit
}
function buildColors() {
  if (!app.ext) return;
  const mode = $('colorMode').value;
  if (mode === 'overhang' && !app.overhang) loadOverhang();
  const limit = Number(app.values.overhang_limit) || 45;
  const n = app.ext.length / FLOATS;
  const col = new Float32Array(n * 6);
  const maxL = app.result.stats.flow_max_level || 7;
  let smin = Infinity, smax = -Infinity;
  if (mode === 'speed') for (let i = 0; i < n; i++) { const v = app.ext[i * FLOATS + 9]; smin = Math.min(smin, v); smax = Math.max(smax, v); }
  const kindCols = palette().kinds;
  const c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    const o = i * FLOATS;
    if (mode === 'kind') c.copy(kindCols[app.ext[o + 6]] || kindCols[3]);
    else if (mode === 'level') c.copy(levelColor(app.ext[o + 7], maxL));
    else if (mode === 'overhang') { if (app.overhang) overhangColor(app.overhang[i], app.ext[o + 6], limit, c); else c.copy(palette().kindNone); }
    else { const t = smax > smin ? (app.ext[o + 9] - smin) / (smax - smin) : 1; rampColor(t, c); }
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

    if (view.ptsObj) { view.scene.remove(view.ptsObj); view.ptsObj.geometry.dispose(); view.ptsObj = null; }
    $('pointsCount').textContent = '';
    if ($('showPoints').checked) buildPoints(eo[lo], eo[top + 1]);
    if (view.seamPts) { view.scene.remove(view.seamPts); view.seamPts.geometry.dispose(); view.seamPts = null; }
    if ($('showSeams').checked) buildSeamMarks(eo[lo], eo[top + 1]);

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
// Toolpath points (the targets the robot moves through) of segments a..b-1: the start of every contour and the
// end of every move. upto[k] = points up to segment a+k, so the simulation can grow them with the path.
function buildPoints(a, b) {
  const e = app.ext, pos = new Float32Array(Math.max(b - a, 0) * 6), upto = new Uint32Array(Math.max(b - a, 0));
  let n = 0, px = NaN, py = NaN, pz = NaN;
  for (let i = a; i < b; i++) {
    const o = i * FLOATS;
    if (e[o] !== px || e[o + 1] !== py || e[o + 2] !== pz) { pos[n * 3] = e[o]; pos[n * 3 + 1] = e[o + 1]; pos[n * 3 + 2] = e[o + 2]; n++; }
    px = e[o + 3]; py = e[o + 4]; pz = e[o + 5];
    pos[n * 3] = px; pos[n * 3 + 1] = py; pos[n * 3 + 2] = pz; n++;
    upto[i - a] = n;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos.subarray(0, n * 3), 3));
  view.ptsObj = new THREE.Points(g, view.pointsMat);
  view.ptsObj.renderOrder = 2;
  view.ptsObj.userData = { a, upto };
  view.scene.add(view.ptsObj);
  $('pointsCount').textContent = n.toLocaleString();
}
// first point of every outer wall loop of segments a..b-1 (where the loop starts = the seam)
function buildSeamMarks(a, b) {
  const e = app.ext, out = [];
  for (let i = a; i < b; i++) {
    const o = i * FLOATS;
    if (e[o + 6] !== 1) continue;
    const p = (i - 1) * FLOATS;
    const joined = i > a && e[p + 6] === 1 && e[p + 8] === e[o + 8] && e[p + 3] === e[o] && e[p + 4] === e[o + 1] && e[p + 5] === e[o + 2];
    if (!joined) out.push(e[o], e[o + 1], e[o + 2]);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
  view.seamMat = view.seamMat || new THREE.PointsMaterial({ size: 8, sizeAttenuation: false });
  view.seamMat.color.setHex(palette().seam);
  view.seamPts = new THREE.Points(g, view.seamMat);
  view.seamPts.renderOrder = 3;
  view.scene.add(view.seamPts);
}
function applyCut() {
  if (!app.result || !app.ext) return;
  const top = Number($('layerRange').value);
  const lo = $('singleLayer').checked ? top : 0;
  const eo = app.result.extrude_offsets;
  const end = app.cut == null ? eo[top + 1] : firstSegAfter(app.cut, eo[top], eo[top + 1]);
  if (view.extObj && view.extObj.geometry.attributes.instanceStart) view.extObj.geometry.instanceCount = Math.max(0, end - eo[lo]);
  if (view.beads) view.beads.forEach((m, L) => { if (m) m.count = L === top ? Math.max(0, end - eo[top]) : eo[L + 1] - eo[L]; });
  if (view.ptsObj) { const { a, upto } = view.ptsObj.userData, k = end - a; view.ptsObj.geometry.setDrawRange(0, k > 0 ? upto[Math.min(k, upto.length) - 1] : 0); }
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
  const preview = (app.mode === 'preview' || app.mode === 'robot' || app.mode === 'export') && !!app.result;
  if (view.mesh) {
    view.mesh.visible = !preview || $('showModel').checked;
    view.modelMat.opacity = preview ? 0.18 : 1;
    view.modelMat.depthWrite = !preview;
    view.mesh.position.z = preview && app.result.stats?.raft ? app.result.stats.raft.shift : 0;   // the part sits on its raft
  }
  if (view.extObj) view.extObj.visible = preview;
  if (view.beads) view.beads.forEach((m) => { if (m) m.visible = preview && !!m.userData.inRange; });
  if (view.trvObj) view.trvObj.visible = preview && $('showTravel').checked;
  if (view.ptsObj) view.ptsObj.visible = preview;
  if (view.strategy) view.strategy.visible = app.mode === 'prepare' || !app.result;
  if (view.seamViz) view.seamViz.visible = app.mode === 'prepare' || !app.result;
  if (view.selBox) view.selBox.visible = app.mode === 'prepare';
  if (view.seamPts) view.seamPts.visible = preview;
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
    const names = { 1: 'outer wall', 2: 'inner wall', 3: 'sparse infill', 4: 'solid infill', 5: 'brim', 7: 'raft' };
    Object.keys(KIND_LABELS).forEach((k) => { const mm = by[names[k]] || 0; if (mm > 0) item(`var(${KIND_TOKENS[k]})`, KIND_LABELS[k], `${fmt(mm / 1000, 1)} m`); });
  } else if (mode === 'level') {
    const maxL = st.flow_max_level, hist = st.flow_level_hist || [];
    for (let l = maxL; l >= 1; l--) if (hist[l]) item('#' + levelColor(l, maxL).getHexString(), `Level ${l}${l === maxL ? ' (full)' : ''}`, `${hist[l]} moves`);
    const unit = app.values.flow_mode === 'rate' ? 'E/s' : 'E/mm';
    const d = document.createElement('div'); d.className = 'muted';
    d.textContent = `Full scale ${fmt(st.flow_full_scale, 2)} ${unit}`;
    box.appendChild(d);
  } else if (mode === 'overhang') {
    const limit = Number(app.values.overhang_limit) || 45, oh = st.overhang;
    item('var(--danger)', `Past ${fmt(limit, 0)}°`, oh ? `${oh.layers.length} layer${oh.layers.length === 1 ? '' : 's'}` : '');
    item('var(--ramp-0)', `${fmt(limit, 0)}°`, 'the limit');
    item('var(--ramp-2)', `${fmt(limit / 2, 0)}°`);
    item('var(--ramp-4)', '0°', 'on the layer below');
    item('var(--kind-none)', 'Not a wall');
    const d = document.createElement('div'); d.className = 'muted';
    d.textContent = oh ? `Steepest ${fmt(oh.max, 0)}° · limit: Max overhang angle (Material)` : 'Limit: Max overhang angle (Material)';
    box.appendChild(d);
  } else {
    const [a, b] = app.speedRange || [0, 0];
    item('var(--ramp-4)', 'Fast', `${fmt(b, 0)} mm/s`);
    item('var(--ramp-0)', 'Slow', `${fmt(a, 0)} mm/s`);
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
  parts.slice(0, 5).forEach(([k, v]) => { const s = document.createElement('span'); s.innerHTML = `${k} <b></b>`; s.querySelector('b').textContent = v; bar.appendChild(s); });
  if (app.result.source === 'gcode') { const s = document.createElement('span'); s.textContent = '· imported G-code'; bar.appendChild(s); }
  // Preview sidebar: the same numbers as cards
  const card = (k, v, sub = '', wide = false) => `<div class="stat${wide ? ' wide' : ''}"><div class="k">${k}</div><div class="v">${v}</div>${sub ? `<div class="s">${sub}</div>` : ''}</div>`;
  $('summaryGrid').innerHTML = card('Print time', fmtTime(st.time_s), `layers ${fmt(st.layer_time_min / 60, 1)}–${fmt(st.layer_time_max / 60, 1)} min`) +
    card('Material', `${fmt(st.mass_g / 1000, 2)} kg`, `${fmt(st.volume_cm3, 0)} cm³`) +
    card('Layers', st.layers.toLocaleString(), `${st.moves.toLocaleString()} moves`) +
    card('Path', `${fmt(st.extrude_mm / 1000, 1)} m`, `travel ${fmt((st.travel_mm || 0) / 1000, 1)} m`) +
    card('Peak flow', `${fmt(st.max_volumetric_mm3s, 0)} mm³/s`,
      `${fmt(st.max_volumetric_mm3s * 3.6 * app.values.density / 1000, 2)} kg/h · ~${st.io_switches_est} flow level changes`, true) +
    (st.overhang ? card('Steepest overhang', `${fmt(st.overhang.max, 0)}°`,
      st.overhang.layers.length ? `${st.overhang.layers.length} layer${st.overhang.layers.length === 1 ? '' : 's'} past ${fmt(st.overhang.limit, 0)}°` : `within ${fmt(st.overhang.limit, 0)}°`, true) : '');
  $('summarySource').textContent = app.result.source === 'gcode' ? 'imported G-code' : (app.dirty ? 'settings changed: slice again' : '');
  const warns = app.result.warnings || [];
  $('warnCard').hidden = !warns.length;
  $('warnList').innerHTML = '';
  warns.forEach((w) => { const li = document.createElement('li'); li.textContent = w; $('warnList').appendChild(li); });
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
async function exportProgram() {
  if (app.dirty && !confirm('Settings changed since the last slice. Export the old toolpath anyway?')) return;
  const post = app.values.post_name || 'Borunte .act';
  busy(true, `Writing program (${post})…`);
  try {
    const h = await downloadFrom('/api/export/program', { settings: app.values });
    const info = JSON.parse(h.get('X-Export-Info') || '{}');
    const size = info.instructions ? `${info.instructions.toLocaleString()} instructions`
      : info.moves ? `${info.moves.toLocaleString()} moves` + (info.files ? ` · ${info.files.join(', ')}` : '')
        : `${(info.lines || 0).toLocaleString()} lines`;
    toast(`${post}: saved ${h.get('X-Saved-Path')}\n${size}`, 'info', 10000);
    addHistory(post, h.get('X-Saved-Path'), size);
    if (info.robot_checked === false && info.language !== 'gcode') {
      toast('Not robot-checked: run "Check robot path" in the Robot step so the export uses solved tool rotations and a joint move to known start joints.', 'warn', 12000);
    } else if (info.robot_ok === false) toast('The robot check found problems with this job. See the Robot step before running it.', 'error', 12000);
    if ((info.instructions || info.moves || 0) > 10000) {
      toast(`This program is large (${size}). Check your controller's program size limit; raise "Path simplify tolerance" or the line width to reduce points` +
        (info.language === 'kuka_krl' ? ', or split it into files in the post-processor options.' : '.'), 'warn', 15000);
    }
  } catch (e) { toast('Export failed: ' + e.message, 'error', 10000); } finally { busy(false); }
}
function addHistory(post, path, size) {
  const li = document.createElement('li');
  li.innerHTML = '<b></b><span class="muted"></span><span class="path"></span>';
  li.children[0].textContent = post;
  li.children[1].textContent = `${new Date().toLocaleTimeString()} · ${size}`;
  li.children[2].textContent = path || '';
  $('historyList').prepend(li);
  $('historyCard').hidden = false;
}
async function previewCode() {
  try {
    const res = await (await postJSON('/api/post/preview', { settings: app.values, lines: 400 })).json();
    $('codeTitle').textContent = `${app.values.post_name || 'Borunte .act'} · ${res.info.file_in_zip || res.info.file} · first ${res.lines.length} of ${res.total_lines.toLocaleString()} lines`;
    $('codeText').textContent = res.lines.join('\n');
    $('codePanel').hidden = false;
  } catch (e) { toast('Preview failed: ' + e.message, 'error', 8000); }
}
async function exportGcode() {
  try {
    const h = await downloadFrom('/api/export/gcode', { settings: app.values });
    toast('G-code saved: ' + h.get('X-Saved-Path'));
    addHistory('G-code', h.get('X-Saved-Path'), 'for other tools');
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
  $('btnSaveProject').disabled = app.busy || !app.model;
  const has = !!app.result && !app.busy;
  ['btnExportProgram', 'btnPreviewCode', 'btnExportGcode'].forEach((id) => { $(id).disabled = !has; });
  $('tabPreview').disabled = !app.result;
}
// Setup and Robot check share the robot sidebar and the robot in the 3D view; each remembers its own sub-tab
const ROBOT_MODES = ['setup', 'robot'];
const robotPane = { setup: 'setup', robot: 'check' };
function setRobotPane(pane) {
  const side = $('sideRobot');
  side.dataset.pane = pane;
  if (ROBOT_MODES.includes(app.mode)) robotPane[app.mode] = pane;
  side.querySelectorAll('.subtabs button').forEach((x) => x.classList.toggle('on', x.dataset.pane === pane));
  if (pane === 'machine') { fillSetupPost(); if (postUI) postUI.refresh().then(fillSetupPost); }   // the list is loaded by Export
}
function setMode(m) {
  const was = app.mode;
  app.mode = m;
  for (const [tab, mode] of [['tabSetup', 'setup'], ['tabPrepare', 'prepare'], ['tabPreview', 'preview'], ['tabRobot', 'robot'], ['tabExport', 'export']]) {
    $(tab).classList.toggle('active', m === mode);
  }
  $('sidePrepare').hidden = m !== 'prepare';
  $('sidePreview').hidden = m !== 'preview';
  $('sideRobot').hidden = !ROBOT_MODES.includes(m);
  $('sideExport').hidden = m !== 'export';
  if (ROBOT_MODES.includes(m)) { $('sideRobot').dataset.step = m; setRobotPane(robotPane[m]); }
  if (was === 'setup' && m !== 'setup') { try { localStorage.setItem('yora.setupSeen', '1'); } catch (_) { /* storage blocked */ } }
  if (m === 'prepare') updateMachineLine();
  // the "drop a model" hint only on an empty Prepare (in Setup it would hide the robot)
  $('dropHint').classList.toggle('hidden', !!(app.model || app.result) || ROBOT_MODES.includes(m));
  if (m !== 'export') $('codePanel').hidden = true;
  if (robotUI) {
    if (ROBOT_MODES.includes(m)) robotUI.enter();
    else if (ROBOT_MODES.includes(was)) robotUI.leave();
  }
  if (gizmoUI) gizmoUI.refresh();
  if (patternUI) patternUI.show();
  if (m === 'export' && postUI) postUI.refresh();
  updateVisibility();
}

// The app's Python code changed after it started (an update): the page is already new, the server is not, and new
// options would silently not work. Say so once, and keep the note in the status bar.
function warnRestart() {
  if (app.restartWarned) return;
  app.restartWarned = true;
  const msg = 'YoRA was updated while it was running. Close this window and start YoRA Slicer.bat again, or new options will not work.';
  toast(msg, 'warn', 30000);
  const bar = document.createElement('div');
  bar.className = 'restart-note';
  bar.textContent = 'Restart YoRA Slicer.bat to finish the update';
  document.body.appendChild(bar);
}
window.addEventListener('focus', async () => {   // checked again when the window comes back to the front
  if (app.restartWarned || !app.spec.length) return;
  try { if ((await (await api('/api/status')).json()).code_changed) warnRestart(); } catch (_) { /* server gone */ }
});

// ------------------------------------------------------------------ boot
async function boot() {
  initView();
  const spec = await (await api('/api/spec')).json();
  app.spec = spec.spec; app.defaults = spec.defaults;
  if (spec.code_changed) warnRestart();
  setValues(restore());
  await loadProfiles();
  // a reload while the app keeps running: show the parts that are still on the bed
  try {
    const plate = await (await api('/api/plate?settings=' + encodeURIComponent(JSON.stringify(app.values)))).json();
    if (plate.model) {
      app.model = plate.model;
      await loadMesh();
      updateObjectPanel();
      $('dropHint').classList.add('hidden');
    }
  } catch (_) { /* an older server: starts empty */ }
  setView('iso');

  $('fileInput').addEventListener('change', async (e) => { const fs = [...e.target.files]; e.target.value = ''; for (const f of fs) await openFile(f); });
  (await (await api('/api/samples')).json()).forEach((f) => $('sampleSelect').add(new Option(f, f)));
  $('sampleSelect').addEventListener('change', async (e) => {
    const name = e.target.value;
    e.target.value = '';
    if (!name) return;
    const blob = await (await api('/samples/' + encodeURIComponent(name))).blob();
    openFile(new File([blob], name));
  });
  $('btnSlice').addEventListener('click', slice);
  $('btnExportProgram').addEventListener('click', exportProgram);
  $('btnPreviewCode').addEventListener('click', previewCode);
  $('codeClose').addEventListener('click', () => { $('codePanel').hidden = true; });
  $('btnExportGcode').addEventListener('click', exportGcode);
  $('tabSetup').addEventListener('click', () => setMode('setup'));
  $('btnEditSetup').addEventListener('click', () => setMode('setup'));
  $('tabPrepare').addEventListener('click', () => setMode('prepare'));
  $('tabPreview').addEventListener('click', () => app.result && setMode('preview'));
  $('tabRobot').addEventListener('click', () => setMode('robot'));
  $('tabExport').addEventListener('click', () => setMode('export'));
  document.querySelectorAll('[data-go]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.go)));
  // settings: search and Essentials / All
  $('settingsSearch').addEventListener('input', buildForm);
  document.querySelectorAll('#detailSeg button').forEach((b) => {
    b.classList.toggle('on', b.dataset.level === app.detail);
    b.addEventListener('click', () => {
      app.detail = b.dataset.level;
      try { localStorage.setItem('yora.detail', app.detail); } catch (_) { /* storage blocked */ }
      document.querySelectorAll('#detailSeg button').forEach((x) => x.classList.toggle('on', x === b));
      buildForm();
    });
  });
  // robot sub-tabs (Setup: Robot & tool / Bed & controller / Jog; Robot check: Path / Check / Jog)
  document.querySelectorAll('#sideRobot .subtabs button').forEach((b) => b.addEventListener('click', () => setRobotPane(b.dataset.pane)));
  app.hooks.push((f) => { if (['robot_model', 'bed_x', 'bed_y', 'bed_z', 'post_name'].includes(f.key)) updateMachineLine(); });
  app.hooks.push((f) => { if (f.key === 'overhang_limit' && app.ext && $('colorMode').value === 'overhang') { buildColors(); updateLayers(); renderLegend(); } });
  // menu: theme, and the way back to the classic interface
  const theme = (() => { try { return localStorage.getItem('yora.theme') || ''; } catch (_) { return ''; } })();
  document.querySelectorAll('#themeSeg button').forEach((b) => {
    b.classList.toggle('on', b.dataset.theme === theme);
    b.addEventListener('click', () => {
      if (b.dataset.theme) document.documentElement.dataset.theme = b.dataset.theme; else delete document.documentElement.dataset.theme;
      try { localStorage.setItem('yora.theme', b.dataset.theme); } catch (_) { /* storage blocked */ }
      document.querySelectorAll('#themeSeg button').forEach((x) => x.classList.toggle('on', x === b));
      applyTheme();
    });
  });
  // menu: style (clay / flat); only the look changes
  document.querySelectorAll('#styleSeg button').forEach((b) => {
    b.classList.toggle('on', b.dataset.style === document.documentElement.dataset.style);
    b.addEventListener('click', () => {
      document.documentElement.dataset.style = b.dataset.style;
      try { localStorage.setItem('yora.style', b.dataset.style); } catch (_) { /* storage blocked */ }
      document.querySelectorAll('#styleSeg button').forEach((x) => x.classList.toggle('on', x === b));
    });
  });
  $('uiClassic').addEventListener('click', () => {
    try { localStorage.setItem('yora.ui', 'classic'); } catch (_) { /* storage blocked */ }
    location.href = '/classic/';
  });
  robotUI = initRobot({ app, view, $, api, palette, cssColor, postJSON, toast, fieldRow, persist, fmt, fmtTime, updateLayers, fitView: () => setView('fit'),
    mergeValues: (vals) => { app.history?.mark(); setValues({ ...app.values, ...vals }); } });
  postUI = initPostEditor({ app, $, api, postJSON, toast, persist });
  matUI = initMaterials({ app, $, api, postJSON, toast, setSetting });
  gizmoUI = initGizmo({ app, view, $, cssColor, setStatusNote, applyTransform, selectPart, busyTool: () => !!seamTool.mode || planeDrag.active || !!patternUI?.painting() });
  patternUI = initPattern({ app, $, api, toast, setSetting, liveSlice: () => slice({ live: true }), view, palette });
  app.history = initHistory({ app, $, toast, applyState, toolUndo: () => patternUI.undoStroke() });
  $('layerRange').addEventListener('input', () => { app.cut = null; updateLayers(); });
  $('singleLayer').addEventListener('change', updateLayers);
  $('showPoints').addEventListener('change', updateLayers);
  $('showSeams').addEventListener('change', updateLayers);
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
  $('addInput').addEventListener('change', async (e) => { const fs = [...e.target.files]; e.target.value = ''; for (const f of fs) await openFile(f); });
  $('btnDuplicate').addEventListener('click', () => partAction('duplicate'));
  $('btnDelete').addEventListener('click', () => partAction('delete'));
  $('btnSaveProject').addEventListener('click', (e) => saveProject(e.shiftKey));
  window.addEventListener('keydown', (ev) => {
    if ((ev.ctrlKey || ev.metaKey) && ev.key.toLowerCase() === 's') { ev.preventDefault(); saveProject(ev.shiftKey); return; }
    if (ev.key === 'Delete' && app.mode === 'prepare' && app.model && !ev.target.matches('input, textarea, select')) partAction('delete');
  });
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
    (async () => { for (const f of e.dataTransfer.files) await openFile(f); })();   // several models: all on the bed
  });
  refreshButtons();
  buildMachineForm();
  updateMachineLine();
  let seen = true;
  try { seen = !!localStorage.getItem('yora.setupSeen'); } catch (_) { /* storage blocked */ }
  if (!seen) setMode('setup');
}
boot().catch((e) => toast('Startup failed: ' + e.message + '\n(The 3D view needs internet access once to load three.js.)', 'error', 20000));
