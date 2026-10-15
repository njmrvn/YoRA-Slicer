// Move / rotate the part with the mouse (Prepare). Double-click the part: arrows move it along X / Y, the square in
// the middle moves it freely on the bed, the arcs turn it about Z (on the bed) or tip it about X / Y. The part follows
// the mouse while dragging; on release the Object panel's position and rotation are set, as if typed. Shift snaps to
// 10 mm / 15 deg. A click on empty space (or Esc) hides the handles.
import * as THREE from 'three';

export function initGizmo(ctx) {
  const { app, view, $, cssColor, setStatusNote, applyTransform } = ctx;
  const el = view.renderer.domElement;
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  const G = { group: null, handles: [], active: false, drag: null, hover: null, centre: new THREE.Vector3(), size: 100 };
  const AX = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };

  const pick = (ev) => {
    const r = el.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, view.camera);
    return ray;
  };
  const hitMesh = (ev) => view.mesh && pick(ev).intersectObject(view.mesh, false).length > 0;
  const hitHandle = (ev) => {
    if (!G.group) return null;
    const h = pick(ev).intersectObjects(G.handles, false)[0];
    return h ? h.object.userData.handle : null;
  };

  // ---------------- the handles ----------------
  function material(token) {
    return new THREE.MeshBasicMaterial({ color: cssColor(token), depthTest: false, transparent: true, opacity: 0.9 });
  }
  function build() {
    clear();
    if (!view.mesh) return;
    const box = new THREE.Box3().setFromObject(view.mesh);
    const size = box.getSize(new THREE.Vector3());
    G.centre.copy(box.getCenter(new THREE.Vector3()));
    G.base = new THREE.Vector3(G.centre.x, G.centre.y, box.min.z);
    G.size = Math.max(size.x, size.y, 40);
    const g = new THREE.Group();
    const t = Math.max(2, G.size * 0.012);
    const R = G.size * 0.5 + 25;                     // rotate ring radius
    const S = R + t * 3, L = Math.max(40, R * 0.45); // arrows start outside the ring, so no handle hides another
    const arrow = (axis, token) => {
      const m = material(token), dir = AX[axis];
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(t, t, L, 12), m);
      const head = new THREE.Mesh(new THREE.ConeGeometry(t * 3.2, t * 9, 20), m);
      const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      shaft.quaternion.copy(q); head.quaternion.copy(q);
      shaft.position.copy(G.base).addScaledVector(dir, S + L / 2);
      head.position.copy(G.base).addScaledVector(dir, S + L + t * 4);
      // a fat invisible grip so the thin arrow is easy to catch
      const grip = new THREE.Mesh(new THREE.CylinderGeometry(t * 4, t * 4, L + t * 9, 8), new THREE.MeshBasicMaterial({ visible: false }));
      grip.quaternion.copy(q); grip.position.copy(G.base).addScaledVector(dir, S + (L + t * 9) / 2);
      [shaft, head, grip].forEach((o) => { o.userData.handle = 'move-' + axis; o.renderOrder = 20; g.add(o); });
      return [shaft, head, grip];
    };
    const ring = (axis, token, radius, arc, centre) => {
      const m = material(token);
      const geo = new THREE.TorusGeometry(radius, t * 0.9, 8, 96, arc);
      const torus = new THREE.Mesh(geo, m);
      const grip = new THREE.Mesh(new THREE.TorusGeometry(radius, t * 4, 6, 64, arc), new THREE.MeshBasicMaterial({ visible: false }));
      // a torus lies in its own XY plane with the arc on the +Y side: turn its normal onto the rotation axis and,
      // for the X / Y arcs, its +Y onto world +Z, so those half arcs stand upright over the part
      const basis = axis === 'z' ? null
        : axis === 'x' ? new THREE.Matrix4().makeBasis(AX.y, AX.z, AX.x)
          : new THREE.Matrix4().makeBasis(AX.x.clone().negate(), AX.z, AX.y);
      const q = basis ? new THREE.Quaternion().setFromRotationMatrix(basis) : new THREE.Quaternion();
      [torus, grip].forEach((o) => { o.quaternion.copy(q); o.position.copy(centre); o.userData.handle = 'rot-' + axis; o.renderOrder = 20; g.add(o); });
      return [torus, grip];
    };
    const free = new THREE.Mesh(new THREE.PlaneGeometry(t * 10, t * 10), material('--text'));
    free.material.opacity = 0.35; free.material.side = THREE.DoubleSide;
    free.position.copy(G.base).add(new THREE.Vector3(t * 7, t * 7, 0.5));
    free.userData.handle = 'move-xy'; free.renderOrder = 20; g.add(free);
    arrow('x', '--axis-x'); arrow('y', '--axis-y');
    ring('z', '--axis-z', R, Math.PI * 2, G.base);
    ring('x', '--axis-x', R * 0.85, Math.PI, G.centre);
    ring('y', '--axis-y', R * 0.85, Math.PI, G.centre);
    G.handles = [];
    g.traverse((o) => { if (o.userData.handle) G.handles.push(o); });
    G.group = g;
    view.scene.add(g);
    setHover(null);
  }
  function clear() {
    if (!G.group) return;
    view.scene.remove(G.group);
    G.group.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    G.group = null; G.handles = [];
  }
  function setHover(name) {
    G.hover = name;
    G.handles.forEach((o) => { if (o.material.visible !== false) o.material.opacity = !name || o.userData.handle === name ? 0.95 : 0.35; });
    el.style.cursor = name ? (name.startsWith('rot') ? 'grab' : 'move') : '';
  }
  function show() { G.active = true; build(); setStatusNote('Drag an arrow or the square to move the part, an arc to turn it (Shift: 10 mm / 15° steps). Click empty space to finish.'); }
  function hide() { G.active = false; G.drag = null; clear(); el.style.cursor = ''; }

  // ---------------- dragging ----------------
  function planeHit(r, normal, point) {
    const pl = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, point);
    return r.ray.intersectPlane(pl, new THREE.Vector3());
  }
  function angleOf(p, axis, pivot) {
    const d = p.clone().sub(pivot);
    return axis === 'x' ? Math.atan2(d.z, d.y) : axis === 'y' ? Math.atan2(d.x, d.z) : Math.atan2(d.y, d.x);
  }
  function startDrag(ev, handle) {
    const [kind, axis] = handle.split('-');
    const pivot = kind === 'rot' && axis !== 'z' ? G.centre.clone() : G.base.clone();
    const normal = kind === 'rot' ? AX[axis] : AX.z;
    const p = planeHit(pick(ev), normal, pivot);
    if (!p) return false;
    G.drag = { kind, axis, pivot, normal, start: p, a0: kind === 'rot' ? angleOf(p, axis, pivot) : 0, delta: new THREE.Vector3(), angle: 0 };
    view.controls.enabled = false;
    el.setPointerCapture(ev.pointerId);
    return true;
  }
  function moveDrag(ev) {
    const d = G.drag;
    const p = planeHit(pick(ev), d.normal, d.pivot);
    if (!p) return;
    if (d.kind === 'move') {
      const v = p.clone().sub(d.start);
      v.z = 0;
      if (d.axis !== 'xy') { const a = AX[d.axis], k = v.dot(a); v.copy(a).multiplyScalar(k); }   // along the arrow only
      const step = ev.shiftKey ? 10 : 1;
      v.x = Math.round(v.x / step) * step; v.y = Math.round(v.y / step) * step;
      d.delta.copy(v);
      setStatusNote(d.axis === 'xy' ? `Move X ${fmtMm(v.x)}, Y ${fmtMm(v.y)}` : `Move ${d.axis.toUpperCase()} ${fmtMm(v[d.axis])}`);
    } else {
      let a = (angleOf(p, d.axis, d.pivot) - d.a0) * 180 / Math.PI;
      a = ((a + 540) % 360) - 180;
      const step = ev.shiftKey ? 15 : 1;
      d.angle = Math.round(a / step) * step;
      setStatusNote(`Rotate ${d.axis.toUpperCase()} ${d.angle > 0 ? '+' : ''}${d.angle}°`);
    }
    preview();
  }
  const fmtMm = (v) => `${v > 0 ? '+' : ''}${Math.round(v * 10) / 10} mm`;
  function preview() {   // show the part where it will go: rotate about the pivot, then shift
    const d = G.drag, m = view.mesh;
    if (!m) return;
    const q = d.kind === 'rot' ? new THREE.Quaternion().setFromAxisAngle(AX[d.axis], d.angle * Math.PI / 180) : new THREE.Quaternion();
    m.quaternion.copy(q);
    m.position.copy(d.pivot).sub(d.pivot.clone().applyQuaternion(q)).add(d.delta);
    if (G.group) { G.group.position.copy(d.delta); }
  }
  async function endDrag(ev) {
    if (G.drag) moveDrag(ev);                       // the release point counts too
    const d = G.drag;
    G.drag = null;
    view.controls.enabled = true;
    try { el.releasePointerCapture(ev.pointerId); } catch (_) { /* released */ }
    if (!d || (!d.delta.lengthSq() && !d.angle)) { resetPreview(); return; }
    if (d.kind === 'move') {
      $('px').value = Math.round((Number($('px').value) + d.delta.x) * 10) / 10;
      $('py').value = Math.round((Number($('py').value) + d.delta.y) * 10) / 10;
    } else {
      // new rotation = turn about the world axis, after the current one; the server applies X, then Y, then Z
      // (fixed axes), i.e. R = Rz Ry Rx = three.js Euler order 'ZYX'
      const cur = ['rx', 'ry', 'rz'].map((id) => (Number($(id).value) || 0) * Math.PI / 180);
      const R = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(cur[0], cur[1], cur[2], 'ZYX'));
      const turn = new THREE.Matrix4().makeRotationAxis(AX[d.axis], d.angle * Math.PI / 180);
      const e = new THREE.Euler().setFromRotationMatrix(turn.multiply(R), 'ZYX');
      const deg = (r) => { let v = Math.round(r * 1800 / Math.PI) / 10; if (Object.is(v, -0)) v = 0; return v; };
      $('rx').value = deg(e.x); $('ry').value = deg(e.y); $('rz').value = deg(e.z);
    }
    await applyTransform(true);
    resetPreview();
    if (G.active) build();
  }
  function resetPreview() {
    if (view.mesh) { view.mesh.quaternion.identity(); view.mesh.position.set(0, 0, view.mesh.position.z && app.mode !== 'prepare' ? view.mesh.position.z : 0); }
    if (G.group) G.group.position.set(0, 0, 0);
  }

  // ---------------- mouse ----------------
  let down = null;
  el.addEventListener('dblclick', (ev) => {
    if (app.mode !== 'prepare' || !view.mesh || (ctx.busyTool && ctx.busyTool())) return;
    if (hitMesh(ev)) show();
  });
  el.addEventListener('pointerdown', (ev) => {
    down = [ev.clientX, ev.clientY];
    if (!G.active || ev.button !== 0) return;
    const h = hitHandle(ev);
    if (h && startDrag(ev, h)) { ev.preventDefault(); ev.stopImmediatePropagation(); }
  }, true);
  el.addEventListener('pointermove', (ev) => {
    if (G.drag) { moveDrag(ev); return; }
    if (G.active) { const h = hitHandle(ev); if (h !== G.hover) setHover(h); }
  }, true);
  el.addEventListener('pointerup', (ev) => {
    if (G.drag) { endDrag(ev); return; }
    // a click (no drag) on empty space hides the handles; a drag turns the view as usual
    if (G.active && down && Math.hypot(ev.clientX - down[0], ev.clientY - down[1]) < 4 && !hitHandle(ev) && !hitMesh(ev)) hide();
  }, true);
  window.addEventListener('keydown', (ev) => { if (ev.key === 'Escape' && G.active && !G.drag) hide(); });

  return {
    hide,
    refresh() { if (G.active) { if (app.mode === 'prepare' && view.mesh) build(); else hide(); } },
  };
}
