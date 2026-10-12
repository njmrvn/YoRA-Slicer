// Material card (Prepare) and the material editor: name, brand, type, colour, heating zones (one, or several with
// their own temperatures), bed temperature, density, flow, drying and notes. Saved in profiles/materials/.
export function initMaterials(ctx) {
  const { app, $, api, postJSON, toast, setSetting } = ctx;
  const S = { list: [], types: [], editing: null };   // editing: the name the edited material was loaded under
  const MAX_ZONES = 12;

  const find = (name) => S.list.find((m) => m.name === name) || null;
  const el = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text != null) e.textContent = text; return e; };

  async function refresh() {
    try {
      const res = await (await api('/api/materials')).json();
      S.list = res.materials; S.types = res.types;
    } catch (e) { toast('Materials: ' + e.message, 'error'); return; }
    const sel = $('matSelect');
    sel.innerHTML = '';
    sel.add(new Option('No material selected', ''));
    for (const [label, builtin] of [['Your materials', false], ['Examples', true]]) {
      const items = S.list.filter((m) => !!m.builtin === builtin);
      if (!items.length) continue;
      const og = document.createElement('optgroup');
      og.label = label;
      items.forEach((m) => og.appendChild(new Option(m.brand ? `${m.name} · ${m.brand}` : m.name, m.name)));
      sel.appendChild(og);
    }
    $('mTypes').innerHTML = '';
    S.types.forEach((t) => $('mTypes').appendChild(new Option(t)));
    show();
  }

  // summary of the selected material under the picker
  function show() {
    const name = app.values.material || '';
    const sel = $('matSelect');
    if (sel.options.length) sel.value = find(name) ? name : '';
    const m = find(name);
    $('matEdit').disabled = !m;
    const box = $('matInfo');
    box.innerHTML = '';
    if (!m) {
      box.appendChild(el('div', 'muted small', name ? `"${name}" is no longer in the library.` : 'Pick a material, or New to add your own.'));
      return;
    }
    const head = el('div', 'mat-head');
    const sw = el('span', 'mat-swatch');
    if (m.color) sw.style.background = m.color; else sw.classList.add('none');
    head.append(sw, el('span', 'mat-sub', [m.brand, m.type].filter(Boolean).join(' · ') || 'No brand / type'));
    box.appendChild(head);
    const zones = el('div', 'mat-zones-view');
    m.zones.forEach((z, i) => {
      const chip = el('span', 'mat-zone');
      chip.append(el('span', 'muted', m.zones.length > 1 ? z.name : 'Print temp'), el('b', null, `${z.temp} °C`));
      zones.appendChild(chip);
      if (i < m.zones.length - 1) zones.appendChild(el('span', 'mat-arrow', '›'));
    });
    box.appendChild(zones);
    const facts = [];
    if (m.bed_temp > 0) facts.push(`Bed ${m.bed_temp} °C`);
    facts.push(`Density ${m.density} g/cm³`);
    if (m.flow > 0) facts.push(`Flow ${m.flow} %`);
    if (m.drying_temp > 0) facts.push(`Dry ${m.drying_temp} °C${m.drying_hours ? ` · ${m.drying_hours} h` : ''}`);
    box.appendChild(el('div', 'muted small', facts.join('  ·  ')));
    if (m.notes) box.appendChild(el('div', 'mat-notes-view small', m.notes));
  }

  // choosing a material: remember it and apply its density (and flow, if it has one)
  function choose(name) {
    setSetting('material', name);
    const m = find(name);
    if (m) {
      setSetting('density', m.density);
      if (m.flow > 0) setSetting('flow', m.flow);
      toast(`Material: ${m.name}. Density${m.flow > 0 ? ' and flow' : ''} set from the material.`, 'info', 3500);
    }
    show();
  }

  // ---------------- editor ----------------
  function zoneMode() { return $('mZoneMode').dataset.mode; }
  function setZoneMode(mode) {
    $('mZoneMode').dataset.mode = mode;
    $('mZoneMode').querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.mode === mode));
    const rows = readZones();
    if (mode === 'single' && rows.length > 1) renderZones([rows[rows.length - 1]]);          // keep the nozzle zone
    else if (mode === 'multi' && rows.length < 2) renderZones([{ name: 'Feed', temp: rows[0] ? rows[0].temp - 20 : 190 }, ...(rows.length ? rows : [{ name: 'Nozzle', temp: 210 }])]);
    else renderZones(rows);
  }
  function readZones() {
    return [...$('mZones').querySelectorAll('.mat-zone-row')].map((r) => ({
      name: r.querySelector('.zn').value.trim(), temp: Number(r.querySelector('.zt').value) }));
  }
  function renderZones(zones) {
    const box = $('mZones');
    box.innerHTML = '';
    const multi = zoneMode() === 'multi';
    zones.forEach((z, i) => {
      const row = el('div', 'mat-zone-row');
      const num = el('span', 'mat-zone-num', multi ? String(i + 1) : '');
      const name = el('input', 'zn'); name.type = 'text'; name.value = z.name; name.placeholder = multi ? `Zone ${i + 1}` : 'Heater';
      name.setAttribute('aria-label', 'Zone name');
      const temp = el('input', 'zt'); temp.type = 'number'; temp.min = 0; temp.max = 600; temp.step = 1; temp.value = z.temp;
      temp.setAttribute('aria-label', 'Temperature');
      const unit = el('span', 'muted', '°C');
      const del = el('button', 'btn small ghost', '×'); del.type = 'button'; del.title = 'Remove this zone';
      del.hidden = !multi || zones.length <= 2;
      del.addEventListener('click', () => { const zs = readZones(); zs.splice(i, 1); renderZones(zs); });
      row.append(num, name, temp, unit, del);
      box.appendChild(row);
    });
    $('mAddZone').hidden = !multi || zones.length >= MAX_ZONES;
  }

  function open(m, asCopy) {
    const builtin = !!(m && m.builtin);
    S.editing = m && !builtin && !asCopy ? m.name : null;
    $('matTitle').textContent = !m ? 'New material' : (builtin || asCopy ? 'New material from ' + m.name : 'Edit material');
    $('matBuiltinNote').hidden = !builtin;
    const v = m || { name: '', brand: '', type: '', color: '#cccccc', zones: [{ name: 'Nozzle', temp: 210 }], bed_temp: 0,
      density: app.values.density || 1.2, flow: 0, drying_temp: 0, drying_hours: 0, notes: '' };
    $('mName').value = builtin || asCopy ? `${v.name} (mine)` : v.name;
    $('mBrand').value = v.brand; $('mType').value = v.type; $('mColor').value = v.color || '#cccccc';
    $('mBed').value = v.bed_temp; $('mDensity').value = v.density; $('mFlow').value = v.flow;
    $('mDryT').value = v.drying_temp; $('mDryH').value = v.drying_hours; $('mNotes').value = v.notes;
    $('mZoneMode').dataset.mode = v.zones.length > 1 ? 'multi' : 'single';
    $('mZoneMode').querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.mode === $('mZoneMode').dataset.mode));
    renderZones(v.zones.map((z) => ({ ...z })));
    $('mDelete').hidden = !S.editing;
    $('matDialog').showModal();
    $('mName').focus();
  }

  async function save() {
    const zones = readZones();
    if (zones.some((z) => !Number.isFinite(z.temp) || z.temp <= 0)) return toast('Give every heating zone a temperature.', 'warn');
    const m = { name: $('mName').value.trim(), brand: $('mBrand').value, type: $('mType').value, color: $('mColor').value,
      zones, bed_temp: Number($('mBed').value) || 0, density: Number($('mDensity').value) || 1.2, flow: Number($('mFlow').value) || 0,
      drying_temp: Number($('mDryT').value) || 0, drying_hours: Number($('mDryH').value) || 0, notes: $('mNotes').value };
    if (!m.name) return toast('Give the material a name.', 'warn');
    if (find(m.name) && m.name !== S.editing && !confirm(`"${m.name}" already exists. Replace it?`)) return;
    try {
      const saved = await (await postJSON('/api/material', { material: m, old_name: S.editing })).json();
      $('matDialog').close();
      await refresh();
      choose(saved.name);
      toast(`Material "${saved.name}" saved.`, 'info', 3000);
    } catch (e) { toast(e.message, 'error'); }
  }

  async function remove() {
    if (!S.editing || !confirm(`Delete the material "${S.editing}"?`)) return;
    try {
      await postJSON('/api/material/delete', { name: S.editing });
      $('matDialog').close();
      if (app.values.material === S.editing) setSetting('material', '');
      await refresh();
    } catch (e) { toast(e.message, 'error'); }
  }

  $('matSelect').addEventListener('change', () => choose($('matSelect').value));
  $('matNew').addEventListener('click', () => open(null));
  $('matEdit').addEventListener('click', () => { const m = find(app.values.material); if (m) open(m); });
  $('mZoneMode').querySelectorAll('button').forEach((b) => b.addEventListener('click', () => setZoneMode(b.dataset.mode)));
  $('mAddZone').addEventListener('click', () => {
    const zs = readZones();
    const last = zs[zs.length - 1];
    zs.push({ name: `Zone ${zs.length + 1}`, temp: last ? last.temp : 210 });
    renderZones(zs);
  });
  $('mSave').addEventListener('click', save);
  $('mCancel').addEventListener('click', () => $('matDialog').close());
  $('mDelete').addEventListener('click', remove);

  refresh();
  return { refresh, show };
}
