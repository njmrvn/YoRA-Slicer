// Undo / redo (Ctrl+Z, Ctrl+Y or Ctrl+Shift+Z, and the arrows in the top bar). A step holds the state before and
// after a change: every setting (including the seam line and the painted pattern areas) and the parts on the bed
// (which ones, in which order, and their scale, rotation and position). Changes that follow each other quickly
// (a slider being dragged, a part being moved) become one step.
export function initHistory(ctx) {
  const { app, $, toast, applyState } = ctx;
  const MAX = 100;
  const H = { undo: [], redo: [], pending: null, timer: 0, restoring: false };
  const clone = (v) => JSON.parse(JSON.stringify(v));
  const snap = () => ({
    values: clone(app.values),
    parts: (app.model?.parts || []).map((p) => ({ id: p.id, name: p.name, transform: clone(p.transform) })),
  });
  const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);

  // what a step changed, for the button tips and the "Undo: ..." note
  function describe(a, b) {
    const ids = (s) => s.parts.map((p) => p.id);
    if (b.parts.length > a.parts.length) return b.parts.length - a.parts.length > 1 ? 'add parts' : `add ${b.parts.find((p) => !ids(a).includes(p.id))?.name || 'part'}`;
    if (b.parts.length < a.parts.length) return `remove ${a.parts.find((p) => !ids(b).includes(p.id))?.name || 'part'}`;
    const moved = b.parts.find((p, i) => !same(p.transform, a.parts[i]?.transform));
    if (moved) {
      const o = a.parts.find((p) => p.id === moved.id)?.transform || {};
      const what = !same(o.scale, moved.transform.scale) ? 'scale' : !same(o.rotation, moved.transform.rotation) ? 'rotate' : 'move';
      return `${what} ${moved.name}`;
    }
    const keys = Object.keys(b.values).filter((k) => !same(a.values[k], b.values[k]));
    const label = (k) => (app.spec.find((f) => f.key === k)?.label || k).toLowerCase();
    if (keys.includes('pattern_paint')) return 'paint';
    if (keys.includes('seam_path') || keys.includes('seam_x')) return 'seam';
    return keys.length > 2 ? `${keys.length} settings` : keys.map(label).join(', ') || 'change';
  }

  function refresh() {
    const u = H.undo[H.undo.length - 1], r = H.redo[H.redo.length - 1];
    $('btnUndo').disabled = !u && !H.pending;
    $('btnRedo').disabled = !r;
    $('btnUndo').title = u ? `Undo ${u.label} (Ctrl+Z)` : 'Undo (Ctrl+Z)';
    $('btnRedo').title = r ? `Redo ${r.label} (Ctrl+Y)` : 'Redo (Ctrl+Y)';
  }
  // call before changing anything; the step is closed a moment after the last change
  function mark() {
    if (H.restoring || !app.spec.length) return;
    if (!H.pending) H.pending = { before: snap() };
    clearTimeout(H.timer);
    H.timer = setTimeout(commit, 450);
    refresh();
  }
  function commit() {
    clearTimeout(H.timer);
    if (!H.pending) return;
    if (app.plateBusy > 0) { H.timer = setTimeout(commit, 150); return; }   // wait for the server's answer
    const before = H.pending.before, after = snap();
    H.pending = null;
    if (!same(before, after)) {
      H.undo.push({ before, after, label: describe(before, after) });
      if (H.undo.length > MAX) H.undo.shift();
      H.redo = [];
    }
    refresh();
  }
  async function go(from, to, which) {
    commit();
    if (H.restoring) return;
    const step = from.pop();
    if (!step) return;
    to.push(step);
    H.restoring = true;
    refresh();
    try {
      await applyState(which === 'undo' ? step.before : step.after);
      toast(`${which === 'undo' ? 'Undo' : 'Redo'}: ${step.label}`, 'info', 1800);
    } catch (e) { toast(`${which === 'undo' ? 'Undo' : 'Redo'} failed: ${e.message}`, 'error'); } finally { H.restoring = false; refresh(); }
  }
  const undo = () => go(H.undo, H.redo, 'undo');
  const redo = () => go(H.redo, H.undo, 'redo');
  function reset() { clearTimeout(H.timer); H.undo = []; H.redo = []; H.pending = null; refresh(); }

  $('btnUndo').addEventListener('click', undo);
  $('btnRedo').addEventListener('click', redo);
  window.addEventListener('keydown', (ev) => {
    if (!(ev.ctrlKey || ev.metaKey) || ev.altKey) return;
    const k = ev.key.toLowerCase();
    if (k !== 'z' && k !== 'y') return;
    const el = ev.target instanceof Element ? ev.target : null;
    if (el && el.matches('input[type=text], input[type=number], input[type=search], textarea, [contenteditable]')) return;   // the box's own undo
    if (ctx.toolUndo && k === 'z' && !ev.shiftKey && ctx.toolUndo()) { ev.preventDefault(); return; }               // a stroke while painting
    ev.preventDefault();
    if (k === 'y' || ev.shiftKey) redo(); else undo();
  });
  refresh();
  return { mark, undo, redo, reset, commit, get restoring() { return H.restoring; } };
}
