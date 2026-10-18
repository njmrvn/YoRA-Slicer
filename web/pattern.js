// Surface pattern card (Preview): pick the pattern, drag its sliders, choose a picture, paint where it goes. The
// toolpath is planned again as you go (the server reuses the sliced layers, so only the moves are recalculated).
import * as THREE from 'three';

export function initPattern(ctx) {
  const { app, $, api, toast, setSetting, liveSlice, view, palette } = ctx;
  const WAVES = ['sine', 'zigzag', 'spikes'];
  // key, label (or per pattern), slider min / max / step, shown for these patterns (null = all)
  const ROWS = [
    ['pattern_amplitude', 'Depth', 0, 20, 0.1, null],
    ['pattern_wavelength', { image: 'Detail' }, 2, 200, 1, null],
    ['pattern_shift', { bumps: 'Shift / row' }, 0, 100, 5, [...WAVES, 'bumps']],
    ['pattern_spike_width', { bumps: 'Bump width' }, 2, 100, 1, ['spikes', 'bumps']],
    ['pattern_row_height', 'Row height', 2, 200, 1, ['bumps', 'noise']],
    ['pattern_seed', 'Variation', 0, 50, 1, ['noise']],
    ['pattern_image_angle', 'Centre at', -180, 180, 5, ['image']],
    ['pattern_image_span', 'Width', 10, 360, 5, ['image']],
  ];
  const DEFAULT_LABEL = { pattern_wavelength: 'Wavelength', pattern_shift: 'Shift / layer', pattern_spike_width: 'Spike width' };
  const spec = (key) => app.spec.find((f) => f.key === key) || {};
  let timer = null;

  function change(key, v, now = false) {
    setSetting(key, v);
    clearTimeout(timer);
    timer = setTimeout(liveSlice, now ? 0 : 250);   // sliders: wait until they rest a moment
    show();
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
    $('patPaintMode').addEventListener('change', () => change('pattern_paint_mode', $('patPaintMode').value, true));
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
    const v = app.values;
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
    for (const inp of $('patPaint').querySelectorAll('[data-key="pattern_brush"] input')) if (document.activeElement !== inp) inp.value = v.pattern_brush;
    const n = paint.mode ? paint.pts.length : parsePts(v.pattern_paint).length;
    $('patPaintCount').textContent = n ? `${n} painted points` : '';
    $('patPaintHint').textContent = paint.mode ? HINTS[paint.mode]
      : (v.pattern_paint_mode !== 'everywhere' && !n ? 'Press Paint, then paint on the part.' : '');
    $('patState').textContent = app.busy ? 'updating…' : '';
  }

  // ---------------------------------------------------------------- painting on the part
  const HINTS = {
    paint: 'Left-drag on the part to paint. Right-drag turns the view. Enter or Done finishes, Esc cancels.',
    erase: 'Left-drag over painted areas to rub them out. Enter or Done finishes, Esc cancels.',
  };
  const paint = { mode: null, pts: [], undo: [], before: null, active: false, showModelWas: null };
  const parsePts = (text) => String(text || '').split(';').map((p) => p.split(',').map(Number))
    .filter((p) => p.length >= 3 && p.every(Number.isFinite)).map((p) => p.slice(0, 3));
  const brush = () => Math.max(Number(app.values.pattern_brush) || 15, 0.5);

  function updatePaintViz(hover = null) {
    if (view.paintViz) {
      view.scene.remove(view.paintViz);
      view.paintViz.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
      view.paintViz = null;
    }
    const v = app.values || {};
    const pts = paint.mode ? paint.pts : parsePts(v.pattern_paint);
    const shown = paint.mode || (v.pattern && v.pattern !== 'none' && v.pattern_paint_mode !== 'everywhere');
    if (!shown || (!pts.length && !hover) || !view.mesh) return;
    const g = new THREE.Group();
    const col = palette().seam, r = brush();
    if (pts.length) {
      const mat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: paint.mode ? 0.35 : 0.12, depthWrite: false });
      const inst = new THREE.InstancedMesh(new THREE.SphereGeometry(r, 14, 10), mat, pts.length);
      const m = new THREE.Matrix4();
      pts.forEach((p, i) => inst.setMatrixAt(i, m.makeTranslation(p[0], p[1], p[2])));
      inst.renderOrder = 8;
      g.add(inst);
    }
    if (hover) {   // the brush under the pointer
      const ring = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14),
        new THREE.MeshBasicMaterial({ color: col, wireframe: true, transparent: true, opacity: paint.mode === 'erase' ? 0.9 : 0.5, depthTest: false }));
      ring.position.set(...hover);
      ring.renderOrder = 9;
      g.add(ring);
    }
    g.position.z = view.mesh.position.z;    // the part sits on its raft in the preview
    view.paintViz = g;
    view.scene.add(g);
  }

  function initPaint() {
    const canvas = view.renderer.domElement, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const defaults = { ...view.controls.mouseButtons };
    const hit = (ev) => {
      if (!view.mesh) return null;
      const r = canvas.getBoundingClientRect();
      ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, view.camera);
      view.mesh.material.side = THREE.DoubleSide;
      const h = ray.intersectObject(view.mesh, false)[0];
      return h ? [h.point.x, h.point.y, h.point.z - view.mesh.position.z] : null;   // model position (no raft)
    };
    const setTool = (m) => {
      paint.mode = m;
      $('patPaintBtn').classList.toggle('on', m === 'paint');
      $('patEraseBtn').classList.toggle('on', m === 'erase');
      $('patPaintBtn').textContent = m === 'paint' ? 'Done' : 'Paint';
      $('patEraseBtn').textContent = m === 'erase' ? 'Done' : 'Erase';
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
      if (!paint.mode) { paint.pts = parsePts(app.values.pattern_paint); paint.before = paint.pts.slice(); paint.undo = []; }
      setTool(m);
    };
    const finish = () => {
      const text = paint.pts.map((p) => p.map((c) => c.toFixed(1)).join(',')).join('; ');
      const changed = text !== (app.values.pattern_paint || '');
      setTool(null);
      if (!changed) return;
      if (text && app.values.pattern_paint_mode === 'everywhere') setSetting('pattern_paint_mode', 'painted');
      change('pattern_paint', text, true);
    };
    const cancel = () => { paint.pts = paint.before || []; setTool(null); };
    const dab = (p) => {
      if (paint.mode === 'paint') {
        const last = paint.pts[paint.pts.length - 1];
        if (paint.active && last && Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) < brush() / 3) return;
        paint.pts.push(p);
      } else {
        const r = brush();
        paint.pts = paint.pts.filter((q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]) > r);
      }
    };
    $('patPaintBtn').addEventListener('click', () => start('paint'));
    $('patEraseBtn').addEventListener('click', () => start('erase'));
    $('patPaintUndo').addEventListener('click', () => { if (paint.undo.length) { paint.pts = paint.undo.pop(); show(); } });
    $('patPaintClear').addEventListener('click', () => {
      if (paint.mode) { paint.undo.push(paint.pts.slice()); paint.pts = []; show(); return; }
      if (app.values.pattern_paint) change('pattern_paint', '', true);
    });
    canvas.addEventListener('pointerdown', (ev) => {
      if (!paint.mode || ev.button !== 0) return;
      const p = hit(ev);
      if (!p) return;
      paint.undo.push(paint.pts.slice());
      dab(p);
      paint.active = true;
      canvas.setPointerCapture(ev.pointerId);
      updatePaintViz(p);
      ev.stopImmediatePropagation();
    }, true);
    canvas.addEventListener('pointermove', (ev) => {
      if (!paint.mode) return;
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
  return { show, painting: () => !!paint.mode };
}
