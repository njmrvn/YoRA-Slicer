// STEP files are read in the page (web/step-worker.js, with OpenCascade in web/vendor/occt). Returns the body for
// /api/model?format=cad: the triangles and which CAD face each belongs to. A new worker per file gives its memory back.
export async function readStep(file) {
  const data = await file.arrayBuffer();
  const worker = new Worker('step-worker.js');
  try {
    return await new Promise((resolve, reject) => {
      worker.onerror = (e) => reject(new Error('The STEP reader could not start: ' + (e.message || 'unknown error')));
      worker.onmessage = (ev) => (ev.data.error ? reject(new Error(ev.data.error)) : resolve(ev.data.body));
      worker.postMessage(data, [data]);
    });
  } finally {
    worker.terminate();
  }
}
