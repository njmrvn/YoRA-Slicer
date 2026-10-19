// Robot workspace: robot model (.robot robot model files with 3D links), joint limits,
// tool TCP, bed frame, free rotation about Z, configuration, program events, jog (joint + Cartesian), robot check
// (reach / limits / singularity / configuration / collision) and a time-based simulation.
import * as THREE from 'three';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

const ERR = { 1: 'unreachable', 2: 'joint limit', 4: 'wrist singularity', 8: 'configuration change', 32: 'collision with bed', 64: 'collision with part' };
const POSE_LABELS = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'];
const ROBOT_KEYS = new Set(['robot_model', 'tcp', 'tool_model', 'tool_model_pose', 'tool_model_scale', 'bed_frame', 'tool_rz', 'rotz_range', 'rotz_step', 'sing_margin', 'lock_config',
  'start_joints', 'joint_jump_max', 'check_step', 'floor_margin']);
// simulation speed ratios
const SPEEDS = [[1, '1× real time'], [5, '5× (normal)'], [20, '20×'], [100, '100× (fast)'], [500, '500×']];

// ---------------- kinematics (same model as yora/kinematics.py, so the view needs no server round trip) ----------------
const D2R = Math.PI / 180;
const parse6 = (v) => (String(v ?? '').split(/[\s,;]+/).filter((t) => t !== '').map(Number).concat([0, 0, 0, 0, 0, 0])).slice(0, 6);
function mat(rows) { const m = new THREE.Matrix4(); m.set(...rows.flat()); return m; }
function poseFromXYZRPW([x, y, z, a, b, c]) {   // transl * rotx * roty * rotz, degrees
  return new THREE.Matrix4().makeTranslation(x, y, z)
    .multiply(new THREE.Matrix4().makeRotationX(a * D2R)).multiply(new THREE.Matrix4().makeRotationY(b * D2R))
    .multiply(new THREE.Matrix4().makeRotationZ(c * D2R));
}
function poseToXYZRPW(T) {
  const e = T.elements;   // column-major: R[r][c] = e[c*4 + r]
  const R = (r, c) => e[c * 4 + r];
  const b = Math.asin(Math.max(-1, Math.min(1, R(0, 2))));
  let a, c;
  if (Math.abs(R(0, 2)) < 0.999999) { a = Math.atan2(-R(1, 2), R(2, 2)); c = Math.atan2(-R(0, 1), R(0, 0)); } else { a = Math.atan2(R(2, 1), R(1, 1)); c = 0; }
  return [e[12], e[13], e[14], a / D2R, b / D2R, c / D2R];
}
class Kin {
  constructor(m) {
    this.m = m;
    this.w = m.axes.map((v) => new THREE.Vector3(...v));
    this.p = m.points.map((v) => new THREE.Vector3(...v));
    this.L = m.links_home.map(mat);
    this.M = mat(m.flange_home);
    this.hull = m.hull ? m.hull.map((pts) => pts.map((v) => new THREE.Vector3(...v))) : null;
  }
  frames(q) {   // link frames 1..6 and flange, robot base frame
    const out = [];
    const T = new THREE.Matrix4();
    for (let i = 0; i < 6; i++) {
      const R = new THREE.Matrix4().makeRotationAxis(this.w[i], q[i] * D2R);
      const t = this.p[i].clone().sub(this.p[i].clone().applyMatrix4(R));
      R.setPosition(t);
      T.multiply(R);
      out.push(T.clone().multiply(this.L[i]));
    }
    out.push(T.clone().multiply(this.M));
    return out;
  }
}

export function initRobot(ctx) {
  const { app, view, $, api, postJSON, toast, fieldRow, persist, fmt } = ctx;
  const R = { model: null, kin: null, result: null, sim: null, playing: false, t: 0, stale: true, reach: null, q: null, jogBusy: false };
  app.robot = R;

  // ---------------- 3D objects ----------------
  // group = robot base frame placed in bed coordinates (inverse of the bed frame); links are children posed by FK
  const group = new THREE.Group();
  group.matrixAutoUpdate = false;
  view.scene.add(group);
  app.robotGroup = group;
  const linkObjs = [];          // index 0 = base, 1..6 = links (THREE.Group with meshes, matrix = link frame)
  const linkMats = [];          // per link: materials (for collision highlight)
  for (let i = 0; i < 7; i++) { const g = new THREE.Group(); g.matrixAutoUpdate = false; group.add(g); linkObjs.push(g); linkMats.push([]); }
  const toolObj = new THREE.Group();
  toolObj.matrixAutoUpdate = false;
  group.add(toolObj);
  const toolMat = new THREE.MeshStandardMaterial({ roughness: 0.5, metalness: 0.2 });   // colours: applyColors()
  // skeleton (robots without a 3D model)
  const skel = new THREE.Group();
  view.scene.add(skel);
  const armMat = new LineMaterial({ linewidth: 9, worldUnits: false });
  const armGeom = new LineSegmentsGeometry();
  skel.add(new LineSegments2(armGeom, armMat));
  const jointMat = new THREE.MeshStandardMaterial({ roughness: 0.4 });
  const joints = [];
  for (let i = 0; i < 8; i++) { const m = new THREE.Mesh(new THREE.SphereGeometry(1, 20, 12), jointMat); skel.add(m); joints.push(m); }
  const baseAxes = new THREE.AxesHelper(250);
  baseAxes.visible = app.markers.robot;
  group.add(baseAxes);
  const tcpMarkers = [];     // red tip + X/Y/Z triad at the TCP
  app.markerHooks.push((m) => { baseAxes.visible = m.robot; tcpMarkers.forEach((o) => { o.visible = m.tcp; }); });
  const errGroup = new THREE.Group();
  view.scene.add(errGroup);
  const reachGroup = new THREE.Group();
  view.scene.add(reachGroup);
  const cornerGroup = new THREE.Group();     // Setup: the bed corners checked by "Go to bed corners"
  view.scene.add(cornerGroup);
  armMat.resolution.set(view.renderer.domElement.width, view.renderer.domElement.height);
  new ResizeObserver(() => armMat.resolution.set($('viewport').clientWidth, $('viewport').clientHeight)).observe($('viewport'));

  function cell() {
    const bed = poseFromXYZRPW(parse6(app.values.bed_frame));
    return { bedInv: bed.clone().invert(), tcp: poseFromXYZRPW(parse6(app.values.tcp)) };
  }

  async function loadMesh(name) {
    linkObjs.forEach((g, i) => { g.clear(); linkMats[i] = []; });
    R.hasMesh = false;
    if (!R.model?.has_mesh) return;
    const buf = await (await api('/api/robot/mesh.bin?name=' + encodeURIComponent(name))).arrayBuffer();
    if (R.model?.name !== name) return;
    const n = new DataView(buf).getUint32(0, true);
    const head = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, n)));
    const verts = new Float32Array(buf, 4 + n);
    head.links.forEach((groups, li) => groups.forEach((g) => {
      let geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(verts.slice(g.start * 3, (g.start + g.count) * 3), 3));
      geo = toCreasedNormals(geo, 35 * D2R);
      const mt = new THREE.MeshStandardMaterial({ roughness: 0.55, metalness: 0.15, side: THREE.DoubleSide });
      mt.color.setRGB(g.color[0], g.color[1], g.color[2], THREE.SRGBColorSpace);
      mt.userData.orig = mt.color.clone();   // the robot's own paint (themes may repaint it)
      if (g.color[3] < 0.99) { mt.transparent = true; mt.opacity = g.color[3]; }
      // parts beyond the 6 links (e.g. the weight-compensation cylinder of large KUKA arms) ride on the column
      const owner = li <= 6 ? li : 1;
      linkObjs[owner].add(new THREE.Mesh(geo, mt));
      linkMats[owner].push(mt);
    }));
    R.hasMesh = true;
  }
  // Theme colours of the tool, the skeleton and markers (tokens.css). The robot keeps its own paint in every theme.
  function applyColors() {
    const p = ctx.palette();
    toolMat.color.setHex(p.tool);
    jointMat.color.setHex(p.joint);
    armMat.color.setHex(p.arm);
    toolModelMat.color.setHex(p.tool);
    tipMat.color.setHex(p.tip);
    Object.entries(errMats).forEach(([bit, m]) => m.color.setHex(p.err[bit]));
    if (reachMats.good) { reachMats.good.color.setHex(p.ok); reachMats.bad.color.setHex(p.danger); }
  }
  document.addEventListener('yora-theme', () => { applyColors(); if (R.result && $('rbCharts')) drawCharts(); });

  // Tool head: the imported 3D model placed on the flange (tool_model_pose, scale), or a cone to the TCP.
  // Either way an X/Y/Z triad marks the TCP, so you can see where tool Z points (it must point out of the nozzle).
  const toolModelMat = new THREE.MeshStandardMaterial({ roughness: 0.5, metalness: 0.25, side: THREE.DoubleSide });
  const tipMat = new THREE.MeshBasicMaterial();
  const errMats = {};           // alarm marker materials by error bit
  const reachMats = {};         // reachable / unreachable bed area
  const hitColor = new THREE.Color(), noHit = new THREE.Color(0, 0, 0);
  applyColors();
  const toolCache = {};
  let toolBuild = 0;
  async function buildTool() {
    const id = ++toolBuild;
    const name = app.values.tool_model || '';
    let geo = null;
    if (name) {
      try {
        if (!toolCache[name]) {
          const buf = await (await api('/api/robot/tool.bin?name=' + encodeURIComponent(name))).arrayBuffer();
          const g = new THREE.BufferGeometry();
          g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf), 3));
          toolCache[name] = toCreasedNormals(g, 35 * D2R);
        }
        geo = toolCache[name];
      } catch (e) { toast(`Tool model "${name}": ${e.message}`, 'warn'); }
    }
    if (id !== toolBuild) return;
    toolObj.clear();
    R.toolHull = null;
    const t = cell().tcp;
    const tip = new THREE.Vector3().setFromMatrixPosition(t);
    const len = tip.length();
    const triad = new THREE.AxesHelper(Math.max(40, Math.min(150, len * 0.3)));
    triad.matrixAutoUpdate = false;
    triad.matrix.copy(t);
    toolObj.add(triad);
    tcpMarkers.length = 0;
    tcpMarkers.push(triad);
    if (geo) {
      const sc = Number(app.values.tool_model_scale || 100) / 100;
      const M = poseFromXYZRPW(parse6(app.values.tool_model_pose)).multiply(new THREE.Matrix4().makeScale(sc, sc, sc));
      const m = new THREE.Mesh(geo, toolModelMat);
      m.matrixAutoUpdate = false;
      m.matrix.copy(M);
      toolObj.add(m);
      R.toolHull = supportPoints(geo.attributes.position.array, M);
    } else if (len >= 1) {
      const r = Math.max(8, Math.min(40, len * 0.08));
      const body = new THREE.Mesh(new THREE.CylinderGeometry(r, r * 0.35, len, 24), toolMat);
      body.position.copy(tip.clone().multiplyScalar(0.5));
      body.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), tip.clone().normalize());   // narrow end at the TCP
      toolObj.add(body);
    }
    const tipBall = new THREE.Mesh(new THREE.SphereGeometry(Math.max(4, Math.min(15, len * 0.03)), 16, 10), tipMat);
    tipBall.position.copy(tip);
    toolObj.add(tipBall);
    tcpMarkers.push(tipBall);
    tcpMarkers.forEach((o) => { o.visible = app.markers.tcp; });
    if (R.q) setPose(R.q);
  }
  // Same idea as tool.support_points: the extreme vertex in 150 directions, in the flange frame
  function supportPoints(arr, M) {
    const n = 150, g = Math.PI * (3 - Math.sqrt(5)), dirs = [];
    for (let k = 0; k < n; k++) { const z = 1 - 2 * (k + 0.5) / n, r = Math.sqrt(1 - z * z); dirs.push([r * Math.cos(g * k), r * Math.sin(g * k), z]); }
    const best = new Float64Array(n).fill(-Infinity), idx = new Int32Array(n);
    const e = M.elements, step = 3 * Math.max(1, Math.floor(arr.length / 3 / 60000));
    const P = [];
    for (let i = 0; i < arr.length; i += step) {
      const x = arr[i], y = arr[i + 1], z = arr[i + 2];
      const p = [e[0] * x + e[4] * y + e[8] * z + e[12], e[1] * x + e[5] * y + e[9] * z + e[13], e[2] * x + e[6] * y + e[10] * z + e[14]];
      const pi = P.push(p) - 1;
      for (let k = 0; k < n; k++) { const d = p[0] * dirs[k][0] + p[1] * dirs[k][1] + p[2] * dirs[k][2]; if (d > best[k]) { best[k] = d; idx[k] = pi; } }
    }
    return [...new Set(idx)].map((i) => new THREE.Vector3(...P[i]));
  }

  // Show the robot at joints q (deg). bits = error bits to highlight (collision -> links turn red).
  function setPose(q, bits = null) {
    if (!R.kin || !q) { group.visible = false; skel.visible = false; return; }
    R.q = q;
    const { bedInv } = cell();
    const F = R.kin.frames(q);
    group.matrix.copy(bedInv);
    group.matrixWorldNeedsUpdate = true;
    const inRobot = app.mode === 'robot' || app.mode === 'setup';
    if (R.hasMesh) {
      group.visible = inRobot; skel.visible = false;
      for (let i = 1; i <= 6; i++) { linkObjs[i].matrix.copy(F[i - 1]); linkObjs[i].matrixWorldNeedsUpdate = true; }
    } else {
      group.visible = inRobot; skel.visible = inRobot;
      linkObjs.forEach((g) => g.clear());
      const pts = [new THREE.Vector3(0, 0, 0)].concat(F.slice(0, 6).map((T) => new THREE.Vector3().setFromMatrixPosition(T)));
      pts.push(new THREE.Vector3().setFromMatrixPosition(F[6].clone().multiply(cell().tcp)));
      const pb = pts.map((p) => p.applyMatrix4(bedInv));
      const segs = [];
      for (let i = 0; i < 7; i++) segs.push(pb[i].x, pb[i].y, pb[i].z, pb[i + 1].x, pb[i + 1].y, pb[i + 1].z);
      armGeom.setPositions(segs);
      const sc = Math.max(12, (app.values.bed_x || 1000) / 90);
      pb.forEach((p, i) => { joints[i].position.copy(p); joints[i].scale.setScalar(i === 7 ? sc * 0.5 : sc); });
    }
    toolObj.matrix.copy(F[6]);
    toolObj.matrixWorldNeedsUpdate = true;
    const live = bits === null ? localProblems(q, F) : bits;
    const hit = live & (32 | 64);
    hitColor.setHex(ctx.palette().danger).multiplyScalar(0.55);   // collision: the links glow in the danger colour
    linkMats.forEach((mats, i) => mats.forEach((m) => m.emissive.copy(hit && i >= 2 ? hitColor : noHit)));
    toolModelMat.emissive.copy(hit ? hitColor : noHit);
    updateJogReadout(q, F, live);
  }

  // Same tests as robotpath.Solver.problems / collision, for instant feedback while jogging
  function localProblems(q, F) {
    let bits = 0;
    const m = R.model;
    if (q.some((v, j) => v < m.lower[j] - 1e-6 || v > m.upper[j] + 1e-6)) bits |= 2;
    if (Math.abs(q[4]) < Number(app.values.sing_margin || 0)) bits |= 4;
    if (R.toolHull) {   // tool head model: below the bed surface over the print area
      const { bedInv } = cell();
      const T = bedInv.clone().multiply(F[6]);
      const bx = Number(app.values.bed_x || 0), by = Number(app.values.bed_y || 0);
      const v = new THREE.Vector3();
      for (const p of R.toolHull) { v.copy(p).applyMatrix4(T); if (v.x >= 0 && v.x <= bx && v.y >= 0 && v.y <= by && v.z < -0.5) { bits |= 32; break; } }
    }
    if (R.kin.hull && !(bits & 32)) {
      const { bedInv } = cell();
      const fm = Number(app.values.floor_margin || 0), bx = Number(app.values.bed_x || 0), by = Number(app.values.bed_y || 0);
      const v = new THREE.Vector3();
      for (let i = 2; i <= 6 && !(bits & 32); i++) {
        const T = bedInv.clone().multiply(F[i - 1]);
        for (const p of R.kin.hull[i] || []) {
          v.copy(p).applyMatrix4(T);
          if (v.x >= 0 && v.x <= bx && v.y >= 0 && v.y <= by && v.z < fm) { bits |= 32; break; }
        }
      }
    }
    return bits;
  }

  // ---------------- panel ----------------
  // which sub-tab (Setup / Jog / Path / Check) each section belongs to
  const PANE = { rbModel: 'setup', rbTool: 'setup', rbBed: 'setup', rbJog: 'jog', rbPath: 'path', rbEvents: 'path' };
  function section(title, id, open = true) {
    const det = document.createElement('details');
    det.open = open;
    det.dataset.pane = PANE[id] || 'setup';
    det.innerHTML = `<summary>${title}</summary><div class="fields" id="${id}"></div>`;
    return det;
  }
  function spec(key) { return app.spec.find((f) => f.key === key); }
  function poseEditor(key) {
    const f = spec(key);
    const wrap = document.createElement('div');
    wrap.className = 'pose6';
    const vals = parse6(app.values[key]);
    const head = document.createElement('div');
    head.className = 'pose-head';
    head.innerHTML = `<span class="name">${f.label}<small>${f.help || ''}</small></span>`;
    wrap.appendChild(head);
    const grid = document.createElement('div');
    grid.className = 'pose-grid';
    POSE_LABELS.forEach((lab, i) => {
      const c = document.createElement('label');
      c.innerHTML = `<span>${lab}${i < 3 ? ' mm' : ' °'}</span>`;
      const inp = document.createElement('input');
      inp.type = 'number'; inp.step = 'any'; inp.value = vals[i];
      inp.addEventListener('change', () => {
        vals[i] = Number(inp.value) || 0;
        setValue(key, vals.map((v) => +v.toFixed(4)).join(','));
      });
      c.appendChild(inp);
      grid.appendChild(c);
    });
    wrap.appendChild(grid);
    return wrap;
  }
  function setValue(key, v) {
    app.history?.mark();
    app.values[key] = v;
    persist();
    onRobotSetting(key);
  }

  async function buildPanel() {
    const box = $('robotForm');
    box.innerHTML = '';
    // robot cell: save / load the whole setup (robot, limits, tool, TCP, bed, numbers, position)
    const cellBar = document.createElement('div');
    cellBar.className = 'cell-bar';
    cellBar.dataset.pane = 'setup';
    cellBar.innerHTML = `
      <div class="pick-row"><select id="rbCellSel" aria-label="Saved robot cells"><option value="">Saved robot cells…</option></select>
        <button class="btn small" id="rbCellLoad">Load</button><button class="btn small" id="rbCellSave">Save as…</button></div>
      <div class="row-btns"><span class="muted small" id="rbCellInfo">Robot, tool, TCP, bed frame and position in one file</span>
        <span class="cell-actions"><button class="btn small" id="rbCellExport" title="Download as a .yoracell file (includes the robot and tool models)">Export</button><label class="btn small" title="Load a .yoracell file">Import…<input type="file" id="rbCellFile" accept=".yoracell" hidden></label><button class="btn small" id="rbCellDelete" title="Delete the selected cell">Delete</button></span></div>`;
    box.appendChild(cellBar);
    // 1. robot model
    const s1 = section('Robot model & joint limits', 'rbModel');
    box.appendChild(s1);
    const m = s1.querySelector('.fields');
    m.innerHTML = `
      <div class="field"><label class="name" for="rbModelSel">Robot</label><div class="ctl"><select id="rbModelSel"></select></div></div>
      <div class="pick-row"><select id="rbLibSel" aria-label="Robot files"><option value="">Load robot file (.robot)…</option></select><button class="btn small" id="rbLibBtn">Load</button></div>
      <div class="row-btns"><span class="muted small">Or drop a .robot robot model file</span><label class="btn small">Browse…<input type="file" id="rbFile" accept=".robot" hidden></label></div>
      <div class="muted small" id="rbModelInfo"></div>
      <table class="limits" id="rbLimits"><thead><tr><th></th><th>Lower °</th><th>Upper °</th><th>Used</th></tr></thead><tbody></tbody></table>
      <div class="row-btns"><span class="muted small">Edit the limits to match the controller.</span><button class="btn small" id="rbSaveLimits">Save limits</button></div>`;
    // jog
    const sj = section('Move robot (jog)', 'rbJog');
    sj.querySelector('.fields').innerHTML = `
      <div class="jog-joints" id="rbJogJoints"></div>
      <div class="jog-tcp" id="rbJogTcp"></div>
      <div class="jog-cart">
        <span class="muted small">Tool in bed frame</span>
        <select id="rbJogStep" aria-label="Jog step"><option value="1">1 mm / 1°</option><option value="10" selected>10 mm / 5°</option><option value="50">50 mm / 15°</option><option value="200">200 mm / 45°</option></select>
        <div class="jog-btns">${['X', 'Y', 'Z', 'Rz'].map((a, i) => `<button class="btn small" data-ax="${[0, 1, 2, 5][i]}" data-dir="-1">${a}−</button><button class="btn small" data-ax="${[0, 1, 2, 5][i]}" data-dir="1">${a}+</button>`).join('')}</div>
      </div>
      <div class="row-btns jog-actions"><button class="btn small" id="rbJogHome">Home</button><button class="btn small" id="rbJogStart">Start joints</button><button class="btn small" id="rbJogSet">Use as start joints</button></div>
      <div id="rbJogStatus" class="small"></div>
      <div class="row-btns jog-actions">
        <button class="btn small" id="rbCorners" title="Move the nozzle (pointing down, as when printing) to the four bed corners at the bed and at the full print height, and show which it can reach">Go to bed corners</button>
        <button class="btn small" id="rbBedOrigin" title="Put the bed origin (X 0, Y 0, Z 0 of the print) where the nozzle tip is now. The bed keeps its rotation.">Set bed origin here</button>
      </div>
      <div id="rbCornerResult" class="small"></div>`;
    box.appendChild(sj);
    // 2. tool, 3. bed frame
    const s2 = section('Tool (TCP at the nozzle tip)', 'rbTool');
    const tf = s2.querySelector('.fields');
    const tm = document.createElement('div');
    tm.className = 'pose6';
    tm.innerHTML = `
      <div class="field"><label class="name" for="rbToolSel">Tool 3D model</label><div class="ctl"><select id="rbToolSel"><option value="">None (cone to the TCP)</option></select></div></div>
      <div class="row-btns"><span class="muted small" id="rbToolInfo">STL / OBJ / 3MF of your extruder head</span><label class="btn small">Import model…<input type="file" id="rbToolFile" accept=".stl,.obj,.3mf" hidden></label></div>`;
    tf.appendChild(tm);
    const tmPlace = document.createElement('div');
    tmPlace.className = 'pose6';
    tmPlace.id = 'rbToolPlace';
    tmPlace.appendChild(poseEditor('tool_model_pose'));
    tmPlace.appendChild(fieldRow(spec('tool_model_scale')));
    const tipRow = document.createElement('div');
    tipRow.className = 'row-btns';
    tipRow.innerHTML = '<span class="muted small">Narrow end of the model, tool Z out of the nozzle</span><button class="btn small" id="rbToolTip">Set TCP at nozzle tip</button>';
    tmPlace.appendChild(tipRow);
    tmPlace.hidden = !app.values.tool_model;
    tf.appendChild(tmPlace);
    tf.appendChild(poseEditor('tcp'));
    box.appendChild(s2);
    const s3 = section('Bed frame (user frame at the bed origin)', 'rbBed');
    s3.querySelector('.fields').appendChild(poseEditor('bed_frame'));
    const reachRow = document.createElement('div');
    reachRow.className = 'row-btns';
    reachRow.innerHTML = '<label class="chk"><input type="checkbox" id="rbReach"> Show reachable bed area</label><span class="muted small">nozzle down, at Z = 0</span>';
    s3.querySelector('.fields').appendChild(reachRow);
    box.appendChild(s3);
    // 4. orientation / rotation / configuration
    const s4 = section('Tool orientation, rotation & configuration', 'rbPath');
    const note = document.createElement('div');
    note.className = 'muted small';
    note.textContent = 'Tool orientation: nozzle Z normal to the bed (pointing down).';
    s4.querySelector('.fields').appendChild(note);
    ['tool_rz', 'rotz_range', 'rotz_step', 'sing_margin', 'lock_config', 'start_joints', 'joint_jump_max', 'check_step', 'floor_margin']
      .forEach((k) => s4.querySelector('.fields').appendChild(fieldRow(spec(k))));
    const homeRow = document.createElement('div');
    homeRow.className = 'row-btns';
    homeRow.innerHTML = '<span></span><button class="btn small" id="rbUseHome">Use robot home as start</button>';
    s4.querySelector('.fields').appendChild(homeRow);
    box.appendChild(s4);

    await loadModels();
    $('rbModelSel').addEventListener('change', (e) => { setValue('robot_model', e.target.value); loadModel(); });
    $('rbLibBtn').addEventListener('click', () => loadRobotFile({ path: $('rbLibSel').value }));
    $('rbFile').addEventListener('change', (e) => { if (e.target.files[0]) uploadRobotFile(e.target.files[0]); e.target.value = ''; });
    $('rbSaveLimits').addEventListener('click', saveLimits);
    $('rbReach').checked = !!R.reach;
    $('rbReach').addEventListener('change', toggleReach);
    $('rbUseHome').addEventListener('click', () => {
      if (!R.model) return;
      setValue('start_joints', R.model.home.map((v) => +v.toFixed(3)).join(','));
      buildPanel();
    });
    sj.querySelectorAll('.jog-btns button').forEach((b) => b.addEventListener('click', () => jogCartesian(Number(b.dataset.ax), Number(b.dataset.dir))));
    $('rbJogHome').addEventListener('click', () => jogTo(R.model?.home));
    $('rbJogStart').addEventListener('click', () => jogTo(startJoints()));
    $('rbCorners').addEventListener('click', goToCorners);
    $('rbBedOrigin').addEventListener('click', setBedOriginHere);
    $('rbJogSet').addEventListener('click', () => {
      if (!R.q) return;
      setValue('start_joints', R.q.map((v) => +v.toFixed(3)).join(','));
      buildPanel();
      toast('Start joints set from the jogged robot', 'info', 2500);
    });
    fillLibrary();
    fillTools();
    fillCells();
    $('rbCellLoad').addEventListener('click', () => loadCell($('rbCellSel').value));
    $('rbCellSave').addEventListener('click', saveCell);
    $('rbCellExport').addEventListener('click', exportCell);
    $('rbCellDelete').addEventListener('click', deleteCell);
    $('rbCellFile').addEventListener('change', (e) => { if (e.target.files[0]) importCell(e.target.files[0]); e.target.value = ''; });
    $('rbToolSel').addEventListener('change', (e) => { setValue('tool_model', e.target.value); $('rbToolPlace').hidden = !e.target.value; });
    $('rbToolFile').addEventListener('change', (e) => { if (e.target.files[0]) uploadTool(e.target.files[0]); e.target.value = ''; });
    $('rbToolTip').addEventListener('click', setTcpAtTip);
  }

  // ---------------- robot cells ----------------
  const CELL_KEY = 'yora.cell';
  const currentCell = () => { try { return localStorage.getItem(CELL_KEY) || ''; } catch (_) { return ''; } };
  const setCurrentCell = (n) => { try { localStorage.setItem(CELL_KEY, n); } catch (_) { /* storage blocked */ } };
  async function fillCells() {
    try {
      const { cells } = await (await api('/api/cells')).json();
      const sel = $('rbCellSel');
      cells.forEach((n) => sel.add(new Option(n, n)));
      if (cells.includes(currentCell())) sel.value = currentCell();
    } catch (_) { /* no cells yet */ }
  }
  async function applyCell(cell, how) {
    R.pendingJoints = cell.joints || null;
    Object.keys(toolCache).forEach((k) => delete toolCache[k]);
    setCurrentCell(cell.name);
    ctx.mergeValues(cell.settings);          // rebuilds the panel; the robot is posed when its model has loaded
    const miss = (cell.missing || []).length ? ` Missing: ${cell.missing.join(', ')} (load them, or import the .yoracell from the PC it was saved on).` : '';
    toast(`Robot cell "${cell.name}" ${how}.${miss}`, miss ? 'warn' : 'info', miss ? 12000 : 4000);
  }
  async function loadCell(name) {
    if (!name) return toast('Pick a saved cell first.', 'warn');
    try { await applyCell(await (await api('/api/cell?name=' + encodeURIComponent(name))).json(), 'loaded'); } catch (e) { toast('Loading the cell failed: ' + e.message, 'error'); }
  }
  async function saveCell() {
    const name = prompt('Save the robot cell as', $('rbCellSel').value || currentCell() || 'My robot cell');
    if (!name) return;
    try {
      const cell = await (await postJSON('/api/cell', { name, settings: app.values, joints: R.q })).json();
      setCurrentCell(cell.name);
      await buildPanel();
      $('rbCellInfo').textContent = `Saved "${cell.name}" ${cell.saved}`;
      toast(`Robot cell "${cell.name}" saved (profiles/cells)`, 'info', 4000);
    } catch (e) { toast('Saving the cell failed: ' + e.message, 'error'); }
  }
  async function exportCell() {
    const name = $('rbCellSel').value;
    if (!name) return toast('Save the cell first, then pick it to export.', 'warn');
    try {
      const blob = await (await api('/api/cell/export?name=' + encodeURIComponent(name))).blob();
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = name + '.yoracell';
      a.click();
      setTimeout(() => URL.revokeObjectURL(a.href), 5000);
    } catch (e) { toast('Export failed: ' + e.message, 'error'); }
  }
  async function importCell(file) {
    try {
      const res = await api('/api/cell/import', { method: 'POST', body: await file.arrayBuffer() });
      await applyCell(await res.json(), 'imported');
    } catch (e) { toast('Import failed: ' + e.message, 'error', 10000); }
  }
  async function deleteCell() {
    const name = $('rbCellSel').value;
    if (!name || !confirm(`Delete the saved robot cell "${name}"? (The robot and tool models stay.)`)) return;
    try {
      await postJSON('/api/cell/delete', { name });
      if (currentCell() === name) setCurrentCell('');
      await buildPanel();
    } catch (e) { toast(e.message, 'error'); }
  }

  async function fillTools() {
    try {
      const { tools } = await (await api('/api/robot/tools')).json();
      const sel = $('rbToolSel');
      tools.forEach((n) => sel.add(new Option(n, n)));
      sel.value = tools.includes(app.values.tool_model) ? app.values.tool_model : '';
    } catch (_) { /* keep "None" */ }
  }
  async function uploadTool(file) {
    try {
      const res = await (await api('/api/robot/tool_upload?name=' + encodeURIComponent(file.name), { method: 'POST', body: await file.arrayBuffer() })).json();
      delete toolCache[res.name];
      setValue('tool_model', res.name);
      await buildPanel();
      $('rbToolInfo').textContent = `${res.name}: ${res.triangles.toLocaleString()} triangles, ${res.size.map((v) => fmt(v, 0)).join(' × ')}`;
      toast(res.hint || 'Tool model imported. Place it on the flange if needed, then click "Set TCP at nozzle tip".', res.hint ? 'warn' : 'info', 9000);
    } catch (e) { toast('Tool import failed: ' + e.message, 'error', 10000); }
  }
  async function setTcpAtTip() {
    try {
      const res = await (await postJSON('/api/robot/tool_tip', { settings: app.values })).json();
      setValue('tcp', res.tcp.join(','));
      buildPanel();
      toast(`TCP set at the nozzle tip: ${res.tcp.map((v) => fmt(v, 1)).join(', ')}. The blue Z arrow must point out of the nozzle.`, 'info', 8000);
    } catch (e) { toast(e.message, 'error'); }
  }

  async function loadModels() {
    const { models } = await (await api('/api/robot/models')).json();
    const sel = $('rbModelSel');
    sel.innerHTML = '';
    if (!models.length) sel.add(new Option('(load a .robot robot model file)', ''));
    models.forEach((n) => sel.add(new Option(n, n)));
    if (models.includes(app.values.robot_model)) sel.value = app.values.robot_model;
    else if (models.length) { sel.value = models[0]; app.values.robot_model = models[0]; persist(); }
    await loadModel();
  }
  async function loadModel() {
    const name = $('rbModelSel').value;
    if (!name) { R.model = null; R.kin = null; renderLimits(); buildJog(); setPose(null); return; }
    try {
      R.model = await (await api('/api/robot/model?name=' + encodeURIComponent(name))).json();
      R.kin = new Kin(R.model);
      const parts = [R.model.source || ''];
      parts.push(R.model.has_mesh ? '3D model loaded' : 'no 3D model (load the .robot file to see the real arm)');
      $('rbModelInfo').textContent = parts.filter(Boolean).join(' · ');
      await loadMesh(name);
    } catch (e) { R.model = null; R.kin = null; toast(e.message, 'error'); }
    renderLimits();
    buildJog();
    buildTool();
    if (R.pendingJoints) { setPose(R.pendingJoints.slice()); R.pendingJoints = null; } else if (!(R.sim && !R.stale)) setPose(startJoints());
    ctx.fitView?.();
  }
  async function fillLibrary() {
    try {
      const { files } = await (await api('/api/robot/library')).json();
      const sel = $('rbLibSel');
      files.forEach((f) => {
        const o = new Option(`${f.name}${f.axes !== 6 ? ` (${f.axes} axes)` : ''} — ${f.path.replace(/^.*\\(Downloads|Desktop|Documents)\\/, '$1\\')}`, f.path);
        o.disabled = f.axes !== 6;
        o.title = f.path;
        sel.add(o);
      });
    } catch (_) { /* no library: browse still works */ }
  }
  async function afterLoad(res) {
    toast(`Loaded "${res.name}" from its .robot file`, 'info', 5000);
    app.values.robot_model = res.name; persist();
    markStale();
    await loadModels();
  }
  async function loadRobotFile(body) {
    if (!body.path) return toast('Pick a robot file first.', 'warn');
    try { await afterLoad(await (await postJSON('/api/robot/load_file', body)).json()); } catch (e) { toast('Loading the robot failed: ' + e.message, 'error', 10000); }
  }
  async function uploadRobotFile(file) {
    try {
      const res = await api('/api/robot/upload?name=' + encodeURIComponent(file.name), { method: 'POST', body: await file.arrayBuffer() });
      await afterLoad(await res.json());
    } catch (e) { toast('Loading the robot failed: ' + e.message, 'error', 10000); }
  }
  app.loadRobotFile = uploadRobotFile;
  function renderLimits() {
    const tb = $('rbLimits')?.querySelector('tbody');
    if (!tb) return;
    tb.innerHTML = '';
    if (!R.model) return;
    const used = R.result?.summary;
    for (let j = 0; j < 6; j++) {
      const tr = document.createElement('tr');
      const u = used ? `${fmt(used.joint_min[j], 0)}…${fmt(used.joint_max[j], 0)}` : '–';
      const tight = used && used.limit_margin_min[j] < 10;
      tr.innerHTML = `<th>J${j + 1}</th><td><input type="number" step="any" value="${R.model.lower[j]}"></td>` +
        `<td><input type="number" step="any" value="${R.model.upper[j]}"></td><td class="${tight ? 'bad' : 'muted'}">${u}</td>`;
      tb.appendChild(tr);
    }
  }
  async function saveLimits() {
    if (!R.model) return;
    const rows = [...$('rbLimits').querySelectorAll('tbody tr')];
    const lower = rows.map((r) => Number(r.querySelectorAll('input')[0].value));
    const upper = rows.map((r) => Number(r.querySelectorAll('input')[1].value));
    try {
      await postJSON('/api/robot/limits', { name: R.model.name, lower, upper });
      R.model.lower = lower; R.model.upper = upper;
      buildJog();
      markStale();
      toast('Joint limits saved', 'info', 2500);
    } catch (e) { toast(e.message, 'error'); }
  }
  // ---------------- jog ----------------
  function startJoints() {
    if (!R.model) return null;
    const v = String(app.values.start_joints || '').trim();
    const q = v ? v.split(/[\s,;]+/).map(Number) : [];
    return q.length === 6 && q.every(Number.isFinite) ? q : R.model.home.slice();
  }
  function buildJog() {
    const box = $('rbJogJoints');
    if (!box) return;
    box.innerHTML = '';
    if (!R.model) { box.innerHTML = '<div class="muted small">Pick a robot first.</div>'; return; }
    for (let j = 0; j < 6; j++) {
      const row = document.createElement('div');
      row.className = 'jog-row';
      row.innerHTML = `<span>J${j + 1}</span><input type="range" min="${R.model.lower[j]}" max="${R.model.upper[j]}" step="0.1" aria-label="J${j + 1}">` +
        `<input type="number" step="0.1" aria-label="J${j + 1} degrees">`;
      const [rng, num] = row.querySelectorAll('input');
      const set = (v) => { stopPlay(); const q = (R.q || startJoints()).slice(); q[j] = Number(v); setPose(q); };
      rng.addEventListener('input', () => set(rng.value));
      num.addEventListener('change', () => set(num.value));
      box.appendChild(row);
    }
  }
  function updateJogReadout(q, F, bits) {
    const rows = $('rbJogJoints')?.querySelectorAll('.jog-row');
    if (rows?.length === 6) rows.forEach((row, j) => {
      const [rng, num] = row.querySelectorAll('input');
      if (document.activeElement !== rng) rng.value = q[j];
      if (document.activeElement !== num) num.value = (+q[j]).toFixed(2);
      row.classList.toggle('bad', q[j] < R.model.lower[j] || q[j] > R.model.upper[j]);
    });
    const tcpBox = $('rbJogTcp');
    if (tcpBox) {
      const { bedInv, tcp } = cell();
      const p = poseToXYZRPW(bedInv.clone().multiply(F[6]).multiply(tcp));
      tcpBox.innerHTML = POSE_LABELS.map((l, i) => `<span><small>${l}</small>${fmt(p[i], i < 3 ? 1 : 1)}</span>`).join('');
      R.tcpBed = p;
    }
    const st = $('rbJogStatus');
    if (st) {
      const names = Object.entries(ERR).filter(([b]) => bits & Number(b)).map(([, n]) => n);
      st.innerHTML = names.length ? `<span class="bad">${names.join(' · ')}</span>` : '<span class="muted">OK: within limits, no collision with the bed</span>';
    }
  }
  function jogTo(q) { if (q) { stopPlay(); setPose(q.slice()); } }
  async function jogCartesian(axis, dir) {
    if (!R.model || !R.q || R.jogBusy) return;
    stopPlay();
    const step = Number($('rbJogStep').value);
    const pose = (R.tcpBed || []).slice();
    if (pose.length !== 6) return;
    if (axis === 5) {   // rotate about the tool Z axis: post-multiply by Rz
      const T = poseFromXYZRPW(pose).multiply(new THREE.Matrix4().makeRotationZ(dir * { 1: 1, 10: 5, 50: 15, 200: 45 }[step] * D2R));
      pose.splice(0, 6, ...poseToXYZRPW(T));
    } else pose[axis] += dir * step;
    R.jogBusy = true;
    try {
      const res = await (await postJSON('/api/robot/jog', { settings: app.values, joints: R.q, pose })).json();
      if (!res.ok) toast('The robot cannot reach that pose', 'warn', 2000);
      else setPose(res.joints);
    } catch (e) { toast(e.message, 'error'); } finally { R.jogBusy = false; }
  }

  // Setup: visit the bed corners (bed surface, then full print height) with the nozzle down; mark each corner
  // green (reachable without problems) or red, and list the problems
  async function goToCorners() {
    if (!R.model) return toast('Pick a robot first.', 'warn');
    stopPlay();
    cornerGroup.clear();
    const box = $('rbCornerResult');
    box.textContent = 'Checking the bed corners…';
    let res;
    try { res = await (await postJSON('/api/robot/corners', { settings: app.values, joints: R.q || startJoints() })).json(); } catch (e) { box.textContent = ''; return toast(e.message, 'error'); }
    const pal = ctx.palette(), r = Math.max(8, (Number(app.values.bed_x) || 1000) / 80);
    const names = ['front left', 'front right', 'back right', 'back left'];
    res.corners.forEach((c) => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 10), new THREE.MeshBasicMaterial({ color: c.ok ? pal.ok : pal.danger, depthTest: false, transparent: true, opacity: 0.85 }));
      m.position.set(...c.point); m.renderOrder = 15;
      cornerGroup.add(m);
    });
    const bad = res.corners.filter((c) => !c.ok);
    box.innerHTML = '<div class="corner-list">' + res.corners.map((c, i) => `<span class="${c.ok ? '' : 'bad'}" title="${c.ok ? 'reachable' : c.problems.join(', ')}">` +
      `${names[i % 4]}<br>${i < 4 ? 'bed' : 'top'}</span>`).join('') + '</div>' +
      `<div class="${bad.length ? 'bad' : 'muted'}">${bad.length ? `${bad.length} of 8 corners have problems (hover a corner to see which).` : 'All 8 corners can be reached, at the bed and at the full print height.'}</div>`;
    for (const c of res.corners) {   // show the robot at each corner in turn
      if (!c.joints) continue;
      setPose(c.joints.slice());
      await new Promise((ok) => setTimeout(ok, 650));
      if (!['setup', 'robot'].includes(app.mode)) break;
    }
  }
  // Setup: the bed origin goes where the nozzle tip is (in the robot base frame); its rotation stays as it is
  function setBedOriginHere() {
    if (!R.kin || !R.q) return toast('Pick a robot and jog the nozzle to the bed corner first.', 'warn');
    const F = R.kin.frames(R.q);
    const tip = new THREE.Vector3().setFromMatrixPosition(F[6].clone().multiply(cell().tcp));
    const old = parse6(app.values.bed_frame);
    const neu = [tip.x, tip.y, tip.z, old[3], old[4], old[5]].map((v) => +v.toFixed(2));
    const f = (v) => v.slice(0, 3).map((x) => fmt(x, 1)).join(', ');
    if (!confirm(`Put the bed origin at the nozzle tip?

Bed frame X, Y, Z: ${f(old)}  →  ${f(neu)} mm
(rotation ${old.slice(3).map((x) => fmt(x, 1)).join(', ')}° stays)`)) return;
    setValue('bed_frame', neu.join(','));
    R.pendingJoints = R.q.slice();          // the robot stays where it is while the panel is rebuilt
    buildPanel();
    cornerGroup.clear();
    toast('Bed origin set at the nozzle tip. The print area now starts here.', 'info', 5000);
  }

  // ---------------- reach ----------------
  async function toggleReach() {
    reachGroup.clear();
    R.reach = null;
    if (!$('rbReach').checked) return;
    try {
      const res = await (await postJSON('/api/robot/reach', { settings: app.values, n: 25 })).json();
      R.reach = res;
      const dx = res.x[1] - res.x[0], dy = res.y[1] - res.y[0];
      const good = reachMats.good = new THREE.MeshBasicMaterial({ color: ctx.palette().ok, transparent: true, opacity: 0.22, depthWrite: false });
      const bad = reachMats.bad = new THREE.MeshBasicMaterial({ color: ctx.palette().danger, transparent: true, opacity: 0.28, depthWrite: false });
      const geo = new THREE.PlaneGeometry(dx, dy);
      res.ok.forEach((row, j) => row.forEach((ok, i) => {
        const m = new THREE.Mesh(geo, ok ? good : bad);
        m.position.set(res.x[i], res.y[j], 0.6);
        reachGroup.add(m);
      }));
      const n = res.ok.flat().filter(Boolean).length;
      toast(`Reachable at Z=0: ${Math.round(100 * n / res.ok.flat().length)}% of the print area`, 'info', 4000);
    } catch (e) { toast(e.message, 'error'); }
  }

  // ---------------- check ----------------
  function markStale() {
    R.stale = true;
    errGroup.clear();
    if (R.result) $('rbStatus').innerHTML = '<span class="badge warn">Settings changed — check again</span>';
    if (!R.playing && R.model) setPose(R.q || startJoints());
  }
  function onRobotSetting(key) {
    if (!ROBOT_KEYS.has(key)) return;
    if (key === 'tcp' || key.startsWith('tool_model')) buildTool();
    markStale();
    if (key === 'bed_frame' || key === 'tcp' || key === 'robot_model') { if ($('rbReach')?.checked) toggleReach(); }
  }
  app.hooks.push((f) => onRobotSetting(f.key));
  app.onToolpathChanged = () => { R.sim = null; R.result = null; stopPlay(); markStale(); renderResult(); };

  async function check() {
    if (!app.result) return toast('Slice a model or import G-code first.', 'warn');
    if (!R.model) return toast('Pick or load a robot model first.', 'warn');
    $('rbCheck').disabled = true;
    try {
      await postJSON('/api/robot/check', { settings: app.values });
      for (;;) {
        await new Promise((r) => setTimeout(r, 300));
        const st = await (await api('/api/robot/status')).json();
        if (st.state === 'running') { $('rbStatus').innerHTML = `<span class="badge">Solving joints… ${Math.round(st.progress * 100)}%</span>`; continue; }
        if (st.state === 'error') throw new Error(st.error);
        if (st.state === 'done') { await showResult(st.result); break; }
      }
    } catch (e) { $('rbStatus').innerHTML = ''; toast('Robot check failed: ' + e.message, 'error', 10000); } finally { $('rbCheck').disabled = false; }
  }
  async function showResult(res) {
    R.result = res; R.stale = false;
    const buf = await (await api('/api/robot/sim.bin')).arrayBuffer();
    R.sim = new Float32Array(buf);
    R.simN = R.sim.length / 9;
    R.simT = R.simN ? R.sim[(R.simN - 1) * 9] : 0;
    $('rbPlay').max = R.simT;
    $('rbPlay').step = Math.max(R.simT / 5000, 0.01);
    $('rbPlay').value = 0;
    $('playBar').hidden = false;
    showTime(0);
    drawErrors(res.error_points);
    renderResult();
    renderLimits();
  }
  function drawErrors(points) {
    errGroup.clear();
    const s = Math.max(6, (app.values.bed_x || 1000) / 140);
    const geo = new THREE.SphereGeometry(s, 10, 8);
    const mats = {};
    points.forEach(([x, y, z, e]) => {
      const bit = [1, 2, 32, 64, 4, 8].find((b) => e & b) || 1;
      mats[bit] = mats[bit] || (errMats[bit] = errMats[bit] || new THREE.MeshBasicMaterial({ color: ctx.palette().err[bit] }));
      const m = new THREE.Mesh(geo, mats[bit]);
      m.position.set(x, y, z);
      errGroup.add(m);
    });
  }
  function renderResult() {
    const box = $('rbResult');
    const sm = R.result?.summary;
    if (!sm) { box.innerHTML = '<div class="muted small">Run the check to solve the robot joints for every move, then play the simulation.</div>'; $('playBar').hidden = true; return; }
    const pct = Math.round(sm.valid_ratio * 1000) / 10;
    $('rbStatus').innerHTML = sm.ok ? '<span class="badge ok">✓ Robot can print this job</span>'
      : `<span class="badge bad">${sm.issues_total.toLocaleString()} moves with problems (${pct}% valid)</span>`;
    const cfg = sm.config;
    const rows = [
      ['Configuration', `${cfg.rear ? 'Rear' : 'Front'} · elbow ${cfg.lower_elbow ? 'down' : 'up'} · wrist ${cfg.wrist_flip ? 'flip' : 'no flip'}${sm.locked ? ' (locked)' : ''}`],
      ['Start joints', sm.start_joints.map((v) => fmt(v, 1)).join(', ')],
      ['Closest to wrist singularity', `|J5| = ${fmt(sm.j5_min_abs, 1)}°`],
      ['J6 travel (wind-up)', `${fmt(sm.j6_travel, 1)}°`],
      ['Tool rotation used', sm.rz_min === sm.rz_max ? `${fmt(sm.rz_min, 0)}° (constant)` : `${fmt(sm.rz_min, 0)}…${fmt(sm.rz_max, 0)}° · ${sm.rz_changes} changes`],
      ['Smallest limit margin', sm.limit_margin_min.map((v, j) => `J${j + 1} ${fmt(v, 0)}°`).join(' · ')],
      ['Collision check', R.model?.has_mesh ? 'robot 3D links vs bed and printed part' : 'joint positions vs bed plane (load the .robot file for a 3D check)'],
      ['Cycle time (path)', ctx.fmtTime(sm.time_s)],
    ];
    const counts = Object.entries(sm.counts).filter(([, n]) => n > 0).map(([k, n]) => `${k}: ${n}`).join(' · ');
    box.innerHTML = `<table class="kv">${rows.map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`).join('')}</table>` +
      (counts ? `<div class="bad small">${counts}</div>` : '') +
      '<div class="charts" id="rbCharts"></div>' +
      (R.result.issues.length ? '<div class="issues" id="rbIssues"></div>' : '');
    drawCharts();
    if (R.result.issues.length) {
      const list = $('rbIssues');
      list.innerHTML = '<div class="muted small">Problems (click to inspect)</div>';
      R.result.issues.slice(0, 60).forEach((it) => {
        const b = document.createElement('button');
        b.className = 'issue';
        b.textContent = `Layer ${it.layer} · (${it.x}, ${it.y}, ${it.z}) · ${it.problems.join(', ')}`;
        b.addEventListener('click', () => jumpToMove(it.move, it.layer - 1));
        list.appendChild(b);
      });
    }
  }
  function drawCharts() {
    const box = $('rbCharts');
    if (!box) return;
    box.innerHTML = '';
    const pc = ctx.palette().chart;
    const tr = R.result.trace;
    const sm = R.result.summary;
    for (let j = 0; j < 6; j++) {
      const c = document.createElement('canvas');
      c.width = 330; c.height = 46;
      c.title = `J${j + 1}`;
      box.appendChild(c);
      const g = c.getContext('2d');
      const lo = sm.limit_lower[j], hi = sm.limit_upper[j];
      const y = (v) => 4 + (1 - (v - lo) / (hi - lo)) * (c.height - 8);
      g.fillStyle = pc.bg; g.fillRect(0, 0, c.width, c.height);
      g.strokeStyle = pc.limit; g.setLineDash([4, 3]); g.lineWidth = 1;
      [lo, hi].forEach((v) => { g.beginPath(); g.moveTo(0, y(v)); g.lineTo(c.width, y(v)); g.stroke(); });
      g.setLineDash([]);
      g.strokeStyle = pc.line; g.lineWidth = 1.5; g.beginPath();
      tr.joints.forEach((q, i) => { const x = 28 + i / Math.max(tr.joints.length - 1, 1) * (c.width - 32); i ? g.lineTo(x, y(q[j])) : g.moveTo(x, y(q[j])); });
      g.stroke();
      g.fillStyle = pc.err;
      tr.err.forEach((e, i) => { if (e) { const x = 28 + i / Math.max(tr.joints.length - 1, 1) * (c.width - 32); g.fillRect(x - 1, 2, 2, c.height - 4); } });
      g.fillStyle = pc.text; g.font = '11px Segoe UI'; g.fillText(`J${j + 1}`, 4, 14);
    }
  }

  // ---------------- simulation (time-based, joints interpolated between samples) ----------------
  function sampleAt(t) {   // last sample with time <= t
    let lo = 0, hi = R.simN - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (R.sim[mid * 9] <= t) lo = mid; else hi = mid - 1; }
    return lo;
  }
  function showTime(t) {
    if (!R.sim || !R.simN) return;
    R.t = Math.max(0, Math.min(t, R.simT));
    const k = sampleAt(R.t);
    const k1 = Math.min(k + 1, R.simN - 1);
    const t0 = R.sim[k * 9], t1 = R.sim[k1 * 9];
    const a = t1 > t0 ? (R.t - t0) / (t1 - t0) : 0;
    const q = [];
    for (let j = 0; j < 6; j++) q.push(R.sim[k * 9 + 1 + j] * (1 - a) + R.sim[k1 * 9 + 1 + j] * a);
    const bits = R.sim[k * 9 + 7] | 0;
    setPose(q, bits);
    const move = R.sim[k * 9 + 8] | 0;
    const tr = R.result.trace;
    let li = 0;
    for (let i = 0; i < tr.move.length && tr.move[i] <= move; i++) li = i;
    const layer = tr.layer[li] ?? 0;
    const names = Object.entries(ERR).filter(([b]) => bits & Number(b)).map(([, n]) => n);
    $('playLabel').innerHTML = `${clock(R.t)} / ${clock(R.simT)} · layer ${layer + 1} · move ${move.toLocaleString()}` +
      (names.length ? ` · <span class="bad">${names.join(', ')}</span>` : '');
    // Toolpath grows behind the nozzle: everything up to the move the robot has reached (interpolated between samples)
    const moveNow = Math.floor(move + a * ((R.sim[k1 * 9 + 8] | 0) - move));
    if ($('playFollow').checked) app.showProgress?.(moveNow);
    else app.showProgress?.(null);
  }
  function clock(s) {
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = Math.floor(s % 60);
    return (h ? `${h}:${String(m).padStart(2, '0')}` : `${m}`) + `:${String(x).padStart(2, '0')}`;
  }
  function jumpToMove(move, layer) {
    if (!R.sim) return;
    stopPlay();
    let k = 0;
    for (let i = 0; i < R.simN; i++) { if ((R.sim[i * 9 + 8] | 0) <= move) k = i; else break; }
    const t = R.sim[k * 9];
    $('rbPlay').value = t;
    showTime(t);
    if (app.result && !$('playFollow').checked) { $('layerRange').value = layer; ctx.updateLayers(); }
  }
  let lastT = 0;
  function tick(now) {
    if (!R.playing) return;
    const dt = Math.min((now - lastT) / 1000, 0.1);
    lastT = now;
    let t = R.t + dt * Number($('playSpeed').value);
    if (t >= R.simT) { t = R.simT; stopPlay(); }
    $('rbPlay').value = t;
    showTime(t);
    if (R.playing) requestAnimationFrame(tick);
  }
  function stopPlay() { R.playing = false; $('playBtn').textContent = '▶'; }
  $('playSpeed').innerHTML = SPEEDS.map(([v, l]) => `<option value="${v}"${v === 5 ? ' selected' : ''}>${l}</option>`).join('');
  $('playBtn').addEventListener('click', () => {
    if (!R.sim) return;
    R.playing = !R.playing;
    $('playBtn').textContent = R.playing ? '❚❚' : '▶';
    if (R.playing) { if (R.t >= R.simT) R.t = 0; lastT = performance.now(); requestAnimationFrame(tick); }
  });
  $('rbPlay').addEventListener('input', () => { stopPlay(); showTime(Number($('rbPlay').value)); });
  $('playFollow').addEventListener('change', () => { if (R.sim) showTime(R.t); });
  $('rbCheck').addEventListener('click', check);

  return {
    enter() {
      errGroup.visible = app.mode === 'robot'; reachGroup.visible = true; cornerGroup.visible = true;
      if (!$('robotForm').childElementCount) buildPanel(); else setPose(R.q || startJoints());
      $('playBar').hidden = !R.sim || app.mode !== 'robot';
      if (R.sim && !R.stale) showTime(R.t);
    },
    leave() { group.visible = false; skel.visible = false; errGroup.visible = false; reachGroup.visible = false; cornerGroup.visible = false; $('playBar').hidden = true; stopPlay(); app.showProgress?.(null); },
    rebuild() { if ($('robotForm').childElementCount) buildPanel(); markStale(); },
  };
}
