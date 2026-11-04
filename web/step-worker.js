// Reads a STEP file away from the page, so a large one does not freeze it. OpenCascade (web/vendor/occt) turns the CAD
// surfaces into triangles. Answer: an ArrayBuffer for /api/model?format=cad: uint32 n, n uint32 CAD face numbers,
// then n * 9 float32 vertices.
importScripts('vendor/occt/occt-import-js.js');

const TESSELLATION = { linearUnit: 'millimeter', linearDeflectionType: 'absolute_value', linearDeflection: 0.1, angularDeflection: 0.35 };

self.onmessage = async (ev) => {
  try {
    const occt = await self.occtimportjs({ locateFile: (name) => 'vendor/occt/' + name });
    const res = occt.ReadStepFile(new Uint8Array(ev.data), TESSELLATION);
    if (!res.success) throw new Error('This file could not be read as STEP.');
    let n = 0;
    for (const m of res.meshes) n += m.index.array.length / 3;
    const body = new ArrayBuffer(4 + 4 * n + 36 * n);
    new Uint32Array(body, 0, 1)[0] = n;
    const faces = new Uint32Array(body, 4, n), tris = new Float32Array(body, 4 + 4 * n, 9 * n);
    let t = 0, id = 0;
    for (const m of res.meshes) {
      const pos = m.attributes.position.array, idx = m.index.array, count = idx.length / 3;
      for (let k = 0; k < count; k++) {
        for (let c = 0; c < 3; c++) {
          const v = idx[3 * k + c] * 3;
          tris[9 * (t + k) + 3 * c] = pos[v];
          tris[9 * (t + k) + 3 * c + 1] = pos[v + 1];
          tris[9 * (t + k) + 3 * c + 2] = pos[v + 2];
        }
      }
      faces.fill(id, t, t + count);               // triangles of no CAD face stay together
      id += 1;
      for (const b of m.brep_faces) { faces.fill(id, t + b.first, t + b.last + 1); id += 1; }
      t += count;
    }
    self.postMessage({ body }, [body]);
  } catch (e) {
    self.postMessage({ error: e.message || String(e) });
  }
};
