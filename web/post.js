// Post-processor picker (Export step) and editor window: language, options and event templates.
export function initPostEditor(ctx) {
  const { app, $, api, postJSON, toast, persist } = ctx;
  const S = { list: [], meta: null, cur: null, origName: null, event: 'program_start', dirty: false };

  async function loadMeta() {
    const res = await (await api('/api/posts')).json();
    S.list = res.posts;
    S.meta = res;
    return res;
  }
  const langLabel = (k) => S.meta?.languages[k]?.label || k;

  // ---------------- Export step: picker ----------------
  async function refresh() {
    try { await loadMeta(); } catch (e) { toast('Post-processors: ' + e.message, 'error'); return; }
    const sel = $('postSelect');
    sel.innerHTML = '';
    const groups = { true: 'Built in', false: 'Yours' };
    for (const b of [true, false]) {
      const items = S.list.filter((p) => p.builtin === b);
      if (!items.length) continue;
      const og = document.createElement('optgroup');
      og.label = groups[b];
      items.forEach((p) => og.appendChild(new Option(`${p.name}`, p.name)));
      sel.appendChild(og);
    }
    if (!S.list.some((p) => p.name === app.values.post_name)) { app.values.post_name = 'Borunte .act'; persist(); }
    sel.value = app.values.post_name;
    await showInfo();
  }
  async function showInfo() {
    const name = app.values.post_name;
    $('btnExportLabel').textContent = `Export ${name}`;
    try {
      const d = await (await api('/api/post?name=' + encodeURIComponent(name))).json();
      const used = Object.entries(d.events).filter(([, v]) => v.trim()).length;
      $('postInfo').innerHTML = '<div><span class="lang"></span></div><div class="desc"></div><div class="muted small ev"></div>';
      $('postInfo').querySelector('.lang').textContent = langLabel(d.language);
      $('postInfo').querySelector('.desc').textContent = d.description || '';
      $('postInfo').querySelector('.ev').textContent = used ? `${used} event${used > 1 ? 's' : ''} set up` : 'No events set up';
    } catch (e) { $('postInfo').textContent = e.message; }
    const rp = app.robot?.result;
    $('exportRobotNote').className = 'note' + (rp && !rp.summary?.ok ? ' warn' : '');
    $('exportRobotNote').textContent = !app.result ? 'Slice a model or import G-code first.'
      : !rp ? 'Tip: run "Check robot path" in the Robot step first, so the program uses the solved tool rotations and starts with a joint move to known joints.'
        : rp.summary?.ok ? 'Robot check passed: the program uses its tool rotations and start joints.'
          : 'The robot check found problems with this job. Review the Robot step before running it.';
  }
  $('postSelect').addEventListener('change', (e) => { app.history?.mark(); app.values.post_name = e.target.value; persist(); showInfo(); });
  $('btnPostEdit').addEventListener('click', () => open(app.values.post_name));

  // ---------------- editor window ----------------
  const dlg = $('postDialog');
  async function open(name) {
    await loadMeta();
    renderList();
    await select(name || S.list[0].name);
    dlg.showModal();
  }
  function renderList() {
    const box = $('peList');
    box.innerHTML = '';
    S.list.forEach((p) => {
      const b = document.createElement('button');
      b.className = 'pe-item' + (S.cur && p.name === S.origName ? ' on' : '');
      b.innerHTML = '<span class="n"></span><span class="l"></span>';
      b.querySelector('.n').textContent = p.name;
      b.querySelector('.l').textContent = (p.builtin ? 'Built in · ' : '') + langLabel(p.language);
      b.addEventListener('click', () => { if (confirmDiscard()) select(p.name); });
      box.appendChild(b);
    });
  }
  function confirmDiscard() { return !S.dirty || confirm('Discard the changes to this post-processor?'); }
  async function select(name) {
    const d = await (await api('/api/post?name=' + encodeURIComponent(name))).json();
    load(d, d.builtin ? null : d.name);
  }
  function load(d, origName) {
    S.cur = JSON.parse(JSON.stringify(d));
    S.origName = origName ?? d.name;
    S.dirty = false;
    renderList();
    renderEditor();
  }
  function renderEditor() {
    const d = S.cur;
    const ro = !!d.builtin;
    document.querySelector('.pe-main').dataset.builtin = ro ? '1' : '';
    $('peBuiltin').hidden = !ro;
    $('peName').value = d.name;
    $('peName').readOnly = ro;
    $('peDesc').value = d.description || '';
    $('peDesc').readOnly = ro;
    const lang = $('peLang');
    lang.innerHTML = '';
    Object.entries(S.meta.languages).forEach(([k, v]) => lang.add(new Option(v.label, k)));
    lang.value = d.language;
    lang.disabled = ro;
    $('peDelete').disabled = ro || !S.origName;
    $('peSave').disabled = ro;
    renderOptions();
    renderEvents();
    $('pePreviewOut').hidden = true;
    check();
  }
  function renderOptions() {
    const box = $('peOptions');
    box.innerHTML = '';
    (S.meta.languages[S.cur.language].options || []).forEach((o) => {
      const lab = document.createElement('label');
      lab.innerHTML = '<span></span>';
      lab.firstChild.textContent = o.label + (o.unit ? ` (${o.unit})` : '');
      let input;
      if (o.type === 'choice') {
        input = document.createElement('select');
        o.options.forEach((v) => input.add(new Option(v, v)));
      } else {
        input = document.createElement('input');
        input.type = o.type === 'text' ? 'text' : 'number';
        if (o.type !== 'text') { input.step = o.type === 'int' ? '1' : 'any'; if (o.min !== undefined) input.min = o.min; if (o.max !== undefined) input.max = o.max; }
      }
      input.value = S.cur.options[o.key] ?? o.default;
      input.disabled = !!S.cur.builtin;
      input.addEventListener('change', () => {
        S.cur.options[o.key] = o.type === 'text' || o.type === 'choice' ? input.value : Number(input.value);
        touch();
      });
      lab.appendChild(input);
      if (o.help) { const h = document.createElement('span'); h.className = 'help'; h.textContent = o.help; lab.appendChild(h); }
      box.appendChild(lab);
    });
    $('peRawNote').textContent = S.meta.languages[S.cur.language].raw
      ? '· any other line is copied into the program as is (raw code).'
      : '· raw code lines are not possible in this language.';
  }
  function renderEvents() {
    const nav = $('peEventList');
    nav.innerHTML = '';
    S.meta.events.forEach(([key, label]) => {
      const b = document.createElement('button');
      b.className = 'pe-event' + (key === S.event ? ' on' : '') + ((S.cur.events[key] || '').trim() ? ' has' : '');
      b.innerHTML = '<span></span><span class="dot"></span>';
      b.firstChild.textContent = label;
      b.addEventListener('click', () => { S.event = key; renderEvents(); });
      nav.appendChild(b);
    });
    const ev = S.meta.events.find(([k]) => k === S.event);
    $('peEventHelp').textContent = ev ? ev[2] : '';
    $('peEventText').value = S.cur.events[S.event] || '';
    $('peEventText').readOnly = !!S.cur.builtin;
    const chips = $('pePlaceholders');
    chips.innerHTML = '<b>Placeholders</b> ';
    Object.entries(S.meta.placeholders).forEach(([k, v]) => {
      const c = document.createElement('button');
      c.className = 'chip';
      c.textContent = `{${k}}`;
      c.title = v;
      c.addEventListener('click', () => insert(`{${k}}`));
      chips.appendChild(c);
    });
  }
  function insert(text) {
    const ta = $('peEventText');
    if (ta.readOnly) return;
    const a = ta.selectionStart, b = ta.selectionEnd;
    ta.value = ta.value.slice(0, a) + text + ta.value.slice(b);
    ta.selectionStart = ta.selectionEnd = a + text.length;
    ta.focus();
    ta.dispatchEvent(new Event('input'));
  }
  let checkTimer = 0;
  function touch() {
    S.dirty = true;
    clearTimeout(checkTimer);
    checkTimer = setTimeout(check, 250);
  }
  async function check() {
    try {
      const { problems } = await (await postJSON('/api/post/check', S.cur)).json();
      $('peProblems').innerHTML = '';
      problems.forEach((p) => { const d = document.createElement('div'); d.textContent = '• ' + p; $('peProblems').appendChild(d); });
    } catch (_) { /* server busy: checked again on save */ }
  }
  $('peEventText').addEventListener('input', (e) => {
    if (S.cur.builtin) return;
    S.cur.events[S.event] = e.target.value;
    document.querySelectorAll('.pe-event').forEach((b, i) => b.classList.toggle('has', !!(S.cur.events[S.meta.events[i][0]] || '').trim()));
    touch();
  });
  $('peName').addEventListener('input', (e) => { S.cur.name = e.target.value; touch(); });
  $('peDesc').addEventListener('input', (e) => { S.cur.description = e.target.value; touch(); });
  $('peLang').addEventListener('change', (e) => { S.cur.language = e.target.value; S.cur.options = {}; renderOptions(); touch(); });
  $('peNew').addEventListener('click', () => {
    if (!confirmDiscard()) return;
    load({ name: 'My post-processor', language: 'kuka_krl', options: {}, events: {}, description: '', builtin: false }, '');
    S.dirty = true;
    $('peName').select();
  });
  $('peDup').addEventListener('click', () => {
    const d = JSON.parse(JSON.stringify(S.cur));
    d.name = `${d.name} (my copy)`;
    d.builtin = false;
    load(d, '');
    S.dirty = true;
    $('peName').select();
  });
  $('peDelete').addEventListener('click', async () => {
    if (!S.origName || !confirm(`Delete the post-processor "${S.origName}"?`)) return;
    try {
      await postJSON('/api/post/delete', { name: S.origName });
      if (app.values.post_name === S.origName) { app.values.post_name = 'Borunte .act'; persist(); }
      await loadMeta();
      await select(S.list[0].name);
      refresh();
    } catch (e) { toast(e.message, 'error'); }
  });
  $('peSave').addEventListener('click', async () => {
    try {
      const saved = await (await postJSON('/api/post', S.cur)).json();
      if (S.origName && S.origName !== saved.name) await postJSON('/api/post/delete', { name: S.origName });   // renamed
      app.values.post_name = saved.name;
      persist();
      await loadMeta();
      load(Object.assign(saved, { builtin: false }), saved.name);
      refresh();
      toast(`Post-processor "${saved.name}" saved and selected for export`, 'info', 4000);
    } catch (e) { toast('Save failed: ' + e.message, 'error', 8000); }
  });
  $('pePreview').addEventListener('click', async () => {
    const out = $('pePreviewOut');
    try {
      const res = await (await postJSON('/api/post/preview', { settings: app.values, post: S.cur, lines: 120 })).json();
      out.textContent = res.lines.join('\n') + (res.total_lines > res.lines.length ? `\n… ${res.total_lines - res.lines.length} more lines` : '');
    } catch (e) { out.textContent = 'Preview not available: ' + e.message; }
    out.hidden = false;
  });
  $('peClose').addEventListener('click', () => { if (confirmDiscard()) dlg.close(); });
  dlg.addEventListener('cancel', (e) => { if (!confirmDiscard()) e.preventDefault(); });

  return { refresh, open };
}
