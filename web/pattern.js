// Surface pattern card (Preview): pick the pattern, drag its sliders, choose a picture, paint where it goes. The
// toolpath is planned again as you go (the server reuses the sliced layers, so only the moves are recalculated).
// Up to four pattern layers: the chips at the top choose the layer that the whole card edits. Layer 1 is the
// pattern_* settings, layers 2-4 live in pattern_layers (JSON list, each its own pattern_* values).
import * as THREE from 'three';

export function initPattern(ctx) {
  const { app, $, api, toast, setSetting, liveSlice, view, palette } = ctx;
  const WAVES = ['sine', 'zigzag', 'spikes'];
  // key, label (or per pattern), slider min / max / step, shown for these patterns (null = all)
  const ROWS = [
    ['pattern_amplitude', 'Depth', 0, 20, 0.1, null],
    ['pattern_wavelength', { image: 'Detail' }, 2, 200, 1, null],
    ['pattern_waves', 'Waves / lap', 0, 400, 1, [...WAVES, 'bumps', 'noise']],
    ['pattern_shift', { bumps: 'Shift / row' }, 0, 100, 5, [...WAVES, 'bumps']],
    ['pattern_spike_width', { bumps: 'Bump width' }, 2, 100, 1, ['spikes', 'bumps']],
    ['pattern_row_height', 'Row height', 2, 200, 1, ['bumps', 'noise']],
    ['pattern_seed', 'Variation', 0, 50, 1, ['noise']],
    ['pattern_image_angle', 'Centre at', -180, 180, 5, ['image']],
    ['pattern_image_span', 'Width', 10, 360, 5, ['image']],
    ['pattern_slope_limit', 'Skip slopes over', 0, 90, 1, null],
  ];
  const DEFAULT_LABEL = { pattern_wavelength: 'Wavelength', pattern_shift: 'Shift / layer', pattern_spike_width: 'Spike width' };
  const spec = (key) => app.spec.find((f) => f.key === key) || {};
  let timer = null;

  // ---------------------------------------------------------------- pattern layers
  const MAX_LAYERS = 4;
  const COLORS = [null, '#2bb3a3', '#e0a030', '#8a7cff'];   // layer 1 uses the seam colour of the theme
  const layerColor = (n) => COLORS[n - 1] || palette().seam;
  let cur = 1;                                              // the layer the card edits
  const isLayerKey = (k) => k.startsWith('pattern') && k !== 'pattern_layers';
  const layerDefaults = () => Object.fromEntries(Object.entries(app.defaults).filter(([k]) => isLayerKey(k)));
  const extra = () => {
    try { const l = JSON.parse(app.values.pattern_layers || '[]'); return Array.isArray(l) ? l : []; } catch (_) { return []; }
  };
  const count = () => 1 + Math.min(extra().length, MAX_LAYERS - 1);
  // the values of layer n (all settings, its own pattern_* values)
  const valsOf = (n) => (n === 1 ? app.values : Object.assign({}, app.values, layerDefaults(), extra()[n - 2] || {}));
  const vals = () => valsOf(cur);
  const saveExtra = (list) => setSetting('pattern_layers', list.length ? JSON.stringify(list) : '');
  // set pattern_* values of the current layer
  function put(obj) {
    if (cur === 1) { for (const [k, v] of Object.entries(obj)) setSetting(k, v); return; }
    const list = extra();
    list[cur - 2] = Object.assign({}, list[cur - 2] || {}, obj);
    saveExtra(list);
  }
  const replan = (now) => { clearTimeout(timer); timer = setTimeout(liveSlice, now ? 0 : 250); };   // sliders: wait until they rest
  function change(key, v, now = false) {
    put({ [key]: v });
    replan(now);
    show();
  }
  function selectLayer(n) {
    if (paintTool.finish && paint.mode) paintTool.finish();
    cur = Math.max(1, Math.min(n, count()));
    show();
  }
  function addLayer(copy = false) {
    const list = extra();
    if (list.length >= MAX_LAYERS - 1) return;
    let d;
    if (copy) {
      const v = vals();
      d = Object.fromEntries(Object.keys(layerDefaults()).map((k) => [k, v[k]]));
      if (cur === 1) d.pattern_blend = 'cover';
      d.pattern_hidden = false;
    } else {
      // a new layer starts with no area: Fill is ready to pick where it goes
      d = { pattern: 'sine', pattern_amplitude: app.defaults.pattern_amplitude, pattern_wavelength: app.defaults.pattern_wavelength,
        pattern_paint_mode: 'painted', pattern_blend: 'cover' };
    }
    list.push(d);
    saveExtra(list);
    cur = list.length + 1;
    replan(true);
    show();
    if (!copy && paintTool.start) paintTool.start('fill');
  }
  function deleteLayer() {
    if (paint.mode && paintTool.cancel) paintTool.cancel();
    const list = extra();
    if (cur === 1) {
      if (!list.length) { put({ pattern: 'none' }); replan(true); show(); return; }
      // layer 2 becomes layer 1
      const v = Object.assign(layerDefaults(), list.shift());
      for (const k of Object.keys(layerDefaults())) setSetting(k, v[k]);
    } else {
      list.splice(cur - 2, 1);
      cur -= 1;
    }
    saveExtra(list);
    replan(true);
    show();
  }
  function showLayers() {
    const box = $('patLayers');
    const n = count();
    if (cur > n) cur = n;
    box.innerHTML = '';
    for (let i = 1; i <= n; i++) {
      const v = valsOf(i);
      const b = el('button', { type: 'button', title: `Layer ${i}: edit it with the card below` });
      b.setAttribute('role', 'tab');
      b.setAttribute('aria-selected', String(i === cur));
      b.classList.toggle('on', i === cur);
      b.classList.toggle('off', !!v.pattern_hidden);
      const dot = el('span', { className: 'dot' });
      dot.style.background = layerColor(i);
      const kind = v.pattern === 'none' ? 'none' : v.pattern;
      b.append(dot, document.createTextNode(`${i} · ${kind.charAt(0).toUpperCase()}${kind.slice(1)}`));
      b.addEventListener('click', () => selectLayer(i));
      box.appendChild(b);
    }
    if (n < MAX_LAYERS && app.values.pattern !== 'none') {
      const add = el('button', { type: 'button', textContent: '+ Layer', title: 'Another pattern on part of the wall: pick its area with Fill' });
      add.addEventListener('click', () => addLayer(false));
      box.appendChild(add);
    }
    const tools = $('patLayerTools');
    tools.hidden = n === 1 && app.values.pattern === 'none';
    const v = vals();
    $('patBlend').hidden = cur === 1;
    $('patBlend').value = v.pattern_blend || 'cover';
    $('patHide').textContent = v.pattern_hidden ? 'Show' : 'Hide';
    $('patDup').disabled = n >= MAX_LAYERS;
    $('patDel').hidden = n === 1;
  }
  const el = (tag, props = {}) => Object.assign(document.createElement(tag), props);

  function numberRow(key, min, max, step) {
    const f = spec(key);
    const row = el('div', { className: 'pat-row' });
    row.dataset.key = key;
    const name = el('label', { htmlFor: 'pat_' + key, title: f.help || '' });
    const range = el('input', { type: 'range', min, max, step, id: 'pat_' + key });
    const num = el('input', { type: 'number', step: 'any' });
    if (f.min !== undefined) num.min = f.min;
    if (f.max !== undefined) num.max = f.max;
    const clamp = (v) => {
      if (!Number.isFinite(v)) v = app.defaults[key];
      if (f.min !== undefined) v = Math.max(f.min, v);
      if (f.max !== undefined) v = Math.min(f.max, v);
      return f.type === 'int' ? Math.round(v) : v;
    };
    range.addEventListener('input', () => { num.value = range.value; change(key, clamp(Number(range.value))); });
    num.addEventListener('change', () => { const v = clamp(Number(num.value)); num.value = v; range.value = v; change(key, v, true); });
    const ctl = el('div', { className: 'pat-num' });
    ctl.append(num, el('span', { className: 'unit', textContent: f.unit || '' }));
    row.append(name, range, ctl);
    return row;
  }
  function selectRow(key, label, options) {
    const row = el('div', { className: 'pat-row' });
    row.dataset.key = key;
    const sel = el('select', { id: 'pat_' + key });
    options.forEach(([v, t]) => sel.add(new Option(t, v)));
    sel.addEventListener('change', () => change(key, sel.value, true));
    row.append(el('label', { htmlFor: sel.id, textContent: label, title: spec(key).help || '' }), sel);
    return row;
  }

  function build() {
    const box = $('patRows');
    box.innerHTML = '';
    for (const [key, , min, max, step] of ROWS) box.appendChild(numberRow(key, min, max, step));
    box.appendChild(selectRow('pattern_image_raise', 'Raised', [['dark', 'dark parts'], ['light', 'light parts']]));
    const tile = el('div', { className: 'pat-row' });
    tile.dataset.key = 'pattern_image_tile';
    const chk = el('label', { className: 'chk' });
    const cb = el('input', { type: 'checkbox', id: 'pat_pattern_image_tile' });
    cb.addEventListener('change', () => change('pattern_image_tile', cb.checked, true));
    chk.append(cb, document.createTextNode(' Repeat all the way round'));
    tile.append(el('span'), chk);
    box.appendChild(tile);
    box.appendChild(selectRow('pattern_direction', 'Direction', (spec('pattern_direction').options || ['outward', 'centered', 'inward']).map((o) => [o, o])));
    // a pattern the running server does not know (it is older than this page) cannot be chosen
    const known = spec('pattern').options || [];
    $('patType').querySelectorAll('button').forEach((b) => {
      if (known.length && !known.includes(b.dataset.pattern)) { b.disabled = true; b.title = 'Restart YoRA Slicer.bat to use this pattern'; }
    });
    $('patType').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
      change('pattern', b.dataset.pattern, true);
      if (b.dataset.pattern === 'image') loadPictures();
    }));
    $('patPicture').addEventListener('change', () => change('pattern_image', $('patPicture').value, true));
    $('patUpload').addEventListener('change', async (ev) => {
      const file = ev.target.files[0];
      ev.target.value = '';
      if (!file) return;
      try {
        const res = await (await api('/api/pattern/image?name=' + encodeURIComponent(file.name), { method: 'POST', body: await file.arrayBuffer() })).json();
        await loadPictures();
        change('pattern_image', res.name, true);
      } catch (e) { toast('Picture: ' + e.message, 'error'); }
    });
    const brushRow = numberRow('pattern_brush', 1, 100, 1);
    brushRow.querySelector('label').textContent = 'Brush size';
    $('patPaint').insertBefore(brushRow, $('patPaint').querySelector('.row-btns'));
    const fillRow = numberRow('pattern_fill_angle', 1, 90, 1);
    fillRow.querySelector('label').textContent = 'Fill angle';
    fillRow.hidden = true;
    $('patPaint').insertBefore(fillRow, $('patPaint').querySelector('.row-btns'));
    $('patPaintMode').addEventListener('change', () => change('pattern_paint_mode', $('patPaintMode').value, true));
    const blendRow = numberRow('pattern_edge_blend', 0, 50, 1);
    blendRow.querySelector('label').textContent = 'Edge blend';
    $('patPaint').insertBefore(blendRow, $('patPaint').querySelector('.row-btns'));
    $('patBlend').addEventListener('change', () => change('pattern_blend', $('patBlend').value, true));
    $('patHide').addEventListener('click', () => change('pattern_hidden', !vals().pattern_hidden, true));
    $('patDup').addEventListener('click', () => addLayer(true));
    $('patDel').addEventListener('click', deleteLayer);
    initPaint();
  }

  let pictures = null;
  async function loadPictures() {
    try { pictures = await (await api('/api/patterns')).json(); } catch (e) { pictures = []; }
    const sel = $('patPicture');
    sel.innerHTML = '';
    sel.add(new Option(pictures.length ? 'Choose a picture' : 'No pictures yet: upload one', ''));
    pictures.forEach((n) => sel.add(new Option(n, n)));
    show();
  }

  function show() {
    const panel = $('patternPanel');
    panel.hidden = !(app.result && app.result.source === 'slice');
    updatePaintViz();
    if (panel.hidden) return;
    if (!$('patRows').children.length) build();
    showLayers();
    const v = vals();
    const kind = v.pattern || 'none', on = kind !== 'none';
    $('patType').querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.pattern === kind));
    for (const [key, label, , , , kinds] of ROWS) {
      const row = $('patRows').querySelector(`[data-key="${key}"]`);
      row.hidden = !on || (kinds && !kinds.includes(kind));
      row.querySelector('label').textContent = typeof label === 'string' ? label : (label[kind] || DEFAULT_LABEL[key]);
      const [range, num] = [row.querySelector('input[type=range]'), row.querySelector('input[type=number]')];
      if (document.activeElement !== range) range.value = v[key];
      if (document.activeElement !== num) num.value = v[key];
    }
    for (const key of ['pattern_image_raise', 'pattern_image_tile']) $('patRows').querySelector(`[data-key="${key}"]`).hidden = kind !== 'image';
    $('pat_pattern_image_raise').value = v.pattern_image_raise;
    $('pat_pattern_image_tile').checked = !!v.pattern_image_tile;
    $('pat_pattern_direction').value = v.pattern_direction;
    $('pat_pattern_direction').closest('.pat-row').hidden = !on;
    // picture
    $('patImage').hidden = kind !== 'image';
    if (kind === 'image') {
      if (pictures === null) { pictures = []; loadPictures(); }
      const sel = $('patPicture');
      if (v.pattern_image && ![...sel.options].some((o) => o.value === v.pattern_image)) sel.add(new Option(v.pattern_image + ' (missing)', v.pattern_image));
      sel.value = v.pattern_image || '';
      const thumb = $('patThumb');
      thumb.hidden = !v.pattern_image;
      const src = v.pattern_image ? '/api/pattern/image?name=' + encodeURIComponent(v.pattern_image) : '';
      if (thumb.dataset.src !== src) { thumb.dataset.src = src; if (src) thumb.src = src; }
    }
    // paint
    $('patPaint').hidden = !on;
    $('patPaintMode').value = v.pattern_paint_mode;
    $('patPaint').querySelector('[data-key="pattern_edge_blend"]').hidden = v.pattern_paint_mode === 'everywhere';
    for (const key of ['pattern_brush', 'pattern_fill_angle', 'pattern_edge_blend']) {
      for (const inp of $('patPaint').querySelectorAll(`[data-key="${key}"] input`)) if (document.activeElement !== inp) inp.value = v[key];
    }
    $('patPaint').querySelector('[data-key="pattern_brush"]').hidden = paint.mode === 'fill';
    $('patPaint').querySelector('[data-key="pattern_fill_angle"]').hidden = paint.mode !== 'fill';
    const n = paint.mode ? paint.pts.length : parsePts(v.pattern_paint).length;
    const nf = countFaces(paint.mode ? paint.faces : parseFaces(v.pattern_faces));
    $('patPaintCount').textContent = [n ? `${n} painted points` : '', nf ? `${nf} picked triangles` : ''].filter(Boolean).join(' · ');
    $('patPaintHint').textContent = paint.mode ? HINTS[paint.mode]
      : (v.pattern_paint_mode !== 'everywhere' && !n && !nf ? 'Press Paint or Fill, then mark the part.' : '');
    $('patState').textContent = app.busy ? 'updating…' : '';
    // the wave count chosen for each outline and the wavelength it gives (from the last slice)
    const rep = (cur === 1 ? app.result?.stats?.pattern : app.result?.stats?.pattern_layers?.[String(cur)]) || [];
    const info = $('patWaves');
    info.hidden = !on || kind === 'image' || !rep.length;
    if (!info.hidden) {
      const f = (v) => (Math.round(v * 10) / 10).toString();
      const main = rep[0];
      info.textContent = `${main.n} waves per lap · wavelength ${f(main.wavelength_min)}–${f(main.wavelength_max)} mm` +
        (rep.length > 1 ? ` · ${rep.length - 1} more outline${rep.length > 2 ? 's' : ''} (holes, other parts) with their own count` : '');
      info.title = rep.map((r, i) => `${i ? 'Outline ' + (i + 1) : 'Main outline'}: ${r.n} waves, ${f(r.wavelength_min)}–${f(r.wavelength_max)} mm, ${r.layers} layers`).join(String.fromCharCode(10));
    }
  }

  // ---------------------------------------------------------------- painting on the part
  const HINTS = {
    paint: 'Left-drag on the part to paint. Right-drag turns the view. Enter or Done finishes, Esc cancels.',
    erase: 'Left-drag over painted areas to rub them out. Enter or Done finishes, Esc cancels.',
    fill: 'Click a surface to pick it, Shift+click to unpick. Right-drag turns the view. Enter or Done finishes, Esc cancels.',
  };
  // pts: brush points (bed position); faces: {part id: Set of triangle indices} picked with Fill
  const paint = { mode: null, pts: [], faces: {}, undo: [], before: null, active: false, showModelWas: null };
  const paintTool = {};   // start / finish / cancel, set by initPaint
  const parseFaces = (text) => {
    const out = {};
    for (const part of String(text || '').split(';')) {
      const [pid, rng] = part.split(':');
      if (rng === undefined || !Number.isFinite(Number(pid))) continue;
      const set = new Set();
      for (const r of rng.split(',')) {
        const [a, b] = r.split('-').map(Number);
        if (!Number.isFinite(a)) continue;
        for (let i = a; i <= (Number.isFinite(b) ? b : a); i++) set.add(i);
      }
      if (set.size) out[Number(pid)] = set;
    }
    return out;
  };
  const formatFaces = (faces) => Object.keys(faces).map(Number).sort((a, b) => a - b).filter((pid) => faces[pid].size).map((pid) => {
    const idx = [...faces[pid]].sort((a, b) => a - b), runs = [];
    for (let i = 0; i < idx.length; i++) {
      let j = i;
      while (j + 1 < idx.length && idx[j + 1] === idx[j] + 1) j++;
      runs.push(i === j ? String(idx[i]) : `${idx[i]}-${idx[j]}`);
      i = j;
    }
    return `${pid}:${runs.join(',')}`;
  }).join('; ');
  const countFaces = (faces) => Object.values(faces).reduce((n, s) => n + s.size, 0);
  const copyFaces = (faces) => Object.fromEntries(Object.entries(faces).map(([k, s]) => [k, new Set(s)]));
  const snap = () => ({ pts: paint.pts.slice(), faces: copyFaces(paint.faces) });
  const back = (u) => { paint.pts = u.pts; paint.faces = u.faces; };
  const parsePts = (text) => String(text || '').split(';').map((p) => p.split(',').map(Number))
    .filter((p) => p.length >= 3 && p.every(Number.isFinite)).map((p) => p.slice(0, 3));
  const brush = () => Math.max(Number(vals().pattern_brush) || 15, 0.5);

  function updatePaintViz(hover = null) {
    if (view.paintViz) {
      view.scene.remove(view.paintViz);
      view.paintViz.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
      view.paintViz = null;
    }
    if (!app.values || !view.mesh) return;
    const g = new THREE.Group();
    // every layer's area: the layer being edited in its colour, the others grey
    for (let n = 1; n <= count(); n++) {
      const v = valsOf(n), mine = n === cur;
      const editing = mine && paint.mode;
      if (!editing && (!v.pattern || v.pattern === 'none' || v.pattern_paint_mode === 'everywhere' || v.pattern_hidden)) continue;
      drawArea(g, editing ? paint.pts : parsePts(v.pattern_paint), editing ? paint.faces : parseFaces(v.pattern_faces),
        mine ? layerColor(n) : '#8a8f98', mine ? (paint.mode ? 1 : 0.6) : 0.35, Math.max(Number(v.pattern_brush) || 15, 0.5));
    }
    const col = layerColor(cur), r = brush();
    if (hover) {   // the brush under the pointer
      const ring = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14),
        new THREE.MeshBasicMaterial({ color: col, wireframe: true, transparent: true, opacity: paint.mode === 'erase' ? 0.9 : 0.5, depthTest: false }));
      ring.position.set(...hover);
      ring.renderOrder = 9;
      g.add(ring);
    }
    if (!g.children.length) return;
    g.position.z = view.mesh.position.z;    // the part sits on its raft in the preview
    view.paintViz = g;
    view.scene.add(g);
  }

  // one layer's painted points and picked surfaces; strength 0..1 scales the opacity
  function drawArea(g, pts, faces, col, strength, r) {
    for (const [pid, set] of Object.entries(faces)) {   // picked surfaces: the part's own triangles, tinted
      const part = view.parts && view.parts[pid];
      if (!part || !set.size) continue;
      const src = part.geometry.attributes.position.array, n = src.length / 9;
      const pos = new Float32Array(set.size * 9);
      let k = 0;
      for (const i of set) if (i < n) { pos.set(src.subarray(i * 9, i * 9 + 9), k * 9); k++; }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(pos.subarray(0, k * 9), 3));
      const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.5 * strength,
        side: THREE.DoubleSide, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }));
      mesh.renderOrder = 7;
      g.add(mesh);
    }
    if (pts.length) {
      const mat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.35 * strength, depthWrite: false });
      const inst = new THREE.InstancedMesh(new THREE.SphereGeometry(r, 14, 10), mat, pts.length);
      const m = new THREE.Matrix4();
      pts.forEach((p, i) => inst.setMatrixAt(i, m.makeTranslation(p[0], p[1], p[2])));
      inst.renderOrder = 8;
      g.add(inst);
    }
  }

  function initPaint() {
    const canvas = view.renderer.domElement, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const defaults = { ...view.controls.mouseButtons };
    const hitFace = (ev) => {
      if (!view.mesh) return null;
      const r = canvas.getBoundingClientRect();
      ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, view.camera);
      view.modelMat.side = THREE.DoubleSide;
      return ray.intersectObject(view.mesh, true)[0] || null;
    };
    const hit = (ev) => {
      const h = hitFace(ev);
      return h ? [h.point.x, h.point.y, h.point.z - view.mesh.position.z] : null;   // model position (no raft)
    };
    const setTool = (m) => {
      paint.mode = m;
      $('patPaintBtn').classList.toggle('on', m === 'paint');
      $('patEraseBtn').classList.toggle('on', m === 'erase');
      $('patFillBtn').classList.toggle('on', m === 'fill');
      $('patPaintBtn').textContent = m === 'paint' ? 'Done' : 'Paint';
      $('patEraseBtn').textContent = m === 'erase' ? 'Done' : 'Erase';
      $('patFillBtn').textContent = m === 'fill' ? 'Done' : 'Fill';
      $('patPaintUndo').hidden = !m;
      view.controls.mouseButtons = m ? { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE } : { ...defaults };
      canvas.style.cursor = m ? 'crosshair' : '';
      // the part has to be visible to paint on it
      const sm = $('showModel');
      if (m && paint.showModelWas === null) {
        paint.showModelWas = sm.checked;
        if (!sm.checked) { sm.checked = true; sm.dispatchEvent(new Event('change', { bubbles: true })); }
      } else if (!m && paint.showModelWas !== null) {
        if (sm.checked !== paint.showModelWas) { sm.checked = paint.showModelWas; sm.dispatchEvent(new Event('change', { bubbles: true })); }
        paint.showModelWas = null;
      }
      show();
    };
    const start = (m) => {
      if (paint.mode === m) return finish();
      if (!paint.mode) {
        paint.pts = parsePts(vals().pattern_paint);
        paint.faces = parseFaces(vals().pattern_faces);
        paint.before = snap(); paint.undo = [];
      }
      setTool(m);
    };
    const finish = () => {
      const text = paint.pts.map((p) => p.map((c) => c.toFixed(1)).join(',')).join('; ');
      const ftext = formatFaces(paint.faces);
      const v = vals();
      const changed = text !== (v.pattern_paint || ''), fchanged = ftext !== (v.pattern_faces || '');
      setTool(null);
      if (!changed && !fchanged) return;
      const upd = {};
      if ((text || ftext) && v.pattern_paint_mode === 'everywhere') upd.pattern_paint_mode = 'painted';
      if (changed) upd.pattern_paint = text;
      if (fchanged) upd.pattern_faces = ftext;
      put(upd);
      replan(true);
      show();
    };
    const cancel = () => { if (paint.before) back(paint.before); setTool(null); };
    Object.assign(paintTool, { start, finish, cancel });
    const dab = (p) => {
      if (paint.mode === 'paint') {
        const last = paint.pts[paint.pts.length - 1];
        if (paint.active && last && Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) < brush() / 3) return;
        paint.pts.push(p);
      } else {
        const r = brush();
        paint.pts = paint.pts.filter((q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) > r);
        // picked triangles whose middle is under the brush
        for (const [pid, set] of Object.entries(paint.faces)) {
          const pos = view.parts?.[pid]?.geometry.attributes.position.array;
          if (!pos) continue;
          for (const i of [...set]) {
            const o = i * 9;
            const cx = (pos[o] + pos[o + 3] + pos[o + 6]) / 3, cy = (pos[o + 1] + pos[o + 4] + pos[o + 7]) / 3, cz = (pos[o + 2] + pos[o + 5] + pos[o + 8]) / 3;
            if (Math.hypot(p[0] - cx, p[1] - cy, p[2] - cz) <= r) set.delete(i);
          }
        }
      }
    };
    const fill = async (ev) => {
      const h = hitFace(ev);
      if (!h || h.faceIndex == null) return;
      const pid = h.object.userData.id;
      try {
        const res = await (await api('/api/object/fill', { method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ id: pid, tri: h.faceIndex, angle: Number(vals().pattern_fill_angle) || 30 }) })).json();
        if (paint.mode !== 'fill') return;
        paint.undo.push(snap());
        const set = paint.faces[pid] || (paint.faces[pid] = new Set());
        for (const i of res.faces) { if (ev.shiftKey) set.delete(i); else set.add(i); }
        show();
      } catch (e) { toast('Fill: ' + e.message, 'error'); }
    };
    $('patPaintBtn').addEventListener('click', () => start('paint'));
    $('patEraseBtn').addEventListener('click', () => start('erase'));
    $('patFillBtn').addEventListener('click', () => start('fill'));
    $('patPaintUndo').addEventListener('click', () => { if (paint.undo.length) { back(paint.undo.pop()); show(); } });
    $('patPaintClear').addEventListener('click', () => {
      if (paint.mode) { paint.undo.push(snap()); paint.pts = []; paint.faces = {}; show(); return; }
      const v = vals();
      if (v.pattern_paint || v.pattern_faces) { put({ pattern_paint: '', pattern_faces: '' }); replan(true); show(); }
    });
    canvas.addEventListener('pointerdown', (ev) => {
      if (!paint.mode || ev.button !== 0) return;
      if (paint.mode === 'fill') { ev.stopImmediatePropagation(); fill(ev); return; }
      const p = hit(ev);
      if (!p) return;
      paint.undo.push(snap());
      dab(p);
      paint.active = true;
      canvas.setPointerCapture(ev.pointerId);
      updatePaintViz(p);
      ev.stopImmediatePropagation();
    }, true);
    canvas.addEventListener('pointermove', (ev) => {
      if (!paint.mode || paint.mode === 'fill') return;
      const p = hit(ev);
      if (p && paint.active) dab(p);
      cancelAnimationFrame(paint.frame);
      paint.frame = requestAnimationFrame(() => { updatePaintViz(p); if (paint.active) $('patPaintCount').textContent = `${paint.pts.length} painted points`; });
    }, true);
    canvas.addEventListener('pointerup', (ev) => {
      if (!paint.active) return;
      paint.active = false;
      try { canvas.releasePointerCapture(ev.pointerId); } catch (_) { /* released */ }
      show();
    }, true);
    window.addEventListener('keydown', (ev) => {
      if (!paint.mode || ev.target.matches('input, textarea, select')) return;
      if (ev.key === 'Escape') cancel();
      else if (ev.key === 'Enter') finish();
    });
  }

  app.hooks.push((f) => { if (f.key && f.key.startsWith('pattern')) show(); });
  // Ctrl+Z while painting takes back the last stroke (true = handled)
  const undoStroke = () => { if (!paint.mode) return false; if (paint.undo.length) { back(paint.undo.pop()); show(); } return true; };
  return { show, painting: () => !!paint.mode, undoStroke };
}
