"""The dashboard page, embedded as a string.

Embedded rather than installed as a data file for one reason: on exam day the
page must load even if the workspace was rebuilt with ``--symlink-install`` and
a share directory went stale. A Python string cannot go missing.

No CDN, no build step, no framework -- the robot's WiFi at school is not a
dependency you want in a graded demo.
"""

INDEX_HTML = r"""<!doctype html>
<html lang="en" data-theme="dark">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LIMO Pro — People Census</title>
<style>
  :root{
    --bg:#0d1117; --panel:#161b22; --panel-2:#1c2431; --line:#2a3341;
    --ink:#e6edf3; --muted:#8b949e;
    --ok:#3fb950; --warn:#d29922; --bad:#f85149; --accent:#58a6ff; --violet:#bc8cff;
    --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);
       font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
  header{display:flex;align-items:baseline;gap:16px;padding:14px 20px;
         border-bottom:1px solid var(--line);background:var(--panel)}
  header h1{margin:0;font-size:16px;font-weight:650;letter-spacing:.2px}
  header .sub{color:var(--muted);font-size:12px;font-family:var(--mono)}
  #conn{margin-left:auto;font-size:12px;font-family:var(--mono);color:var(--bad)}
  #conn.up{color:var(--ok)}

  .wrap{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(0,1fr);
        gap:14px;padding:14px;max-width:1500px;margin:0 auto}
  @media (max-width:900px){.wrap{grid-template-columns:1fr}}

  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
        overflow:hidden}
  .card h2{margin:0;padding:10px 14px;font-size:12px;font-weight:600;
           text-transform:uppercase;letter-spacing:.8px;color:var(--muted);
           border-bottom:1px solid var(--line);background:var(--panel-2)}
  .card .body{padding:14px}

  .tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
  .tile{background:var(--panel-2);border:1px solid var(--line);border-radius:8px;
        padding:12px}
  .tile .k{font-size:11px;color:var(--muted);text-transform:uppercase;
           letter-spacing:.6px}
  .tile .v{font-size:28px;font-weight:700;font-variant-numeric:tabular-nums;
           line-height:1.2;margin-top:2px}
  .tile .v small{font-size:13px;color:var(--muted);font-weight:500}

  #cam{width:100%;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
  #mapcanvas{width:100%;display:block;background:#0a0e14;border-radius:6px}

  .state{display:inline-block;padding:3px 10px;border-radius:99px;font-size:12px;
         font-weight:650;font-family:var(--mono)}
  .state.PATROLLING{background:rgba(88,166,255,.15);color:var(--accent)}
  .state.APPROACHING{background:rgba(188,140,255,.15);color:var(--violet)}
  .state.DWELLING{background:rgba(63,185,80,.15);color:var(--ok)}
  .state.HOLDING{background:rgba(248,81,73,.15);color:var(--bad)}
  .state.DONE{background:rgba(63,185,80,.2);color:var(--ok)}
  .state.IDLE,.state.LOCALIZING{background:rgba(139,148,158,.15);color:var(--muted)}
  .state.FAULT{background:rgba(248,81,73,.25);color:var(--bad)}

  .detail{color:var(--muted);font-size:13px;margin-top:8px;font-family:var(--mono)}

  table{width:100%;border-collapse:collapse;font-size:13px}
  th{text-align:left;color:var(--muted);font-weight:600;font-size:11px;
     text-transform:uppercase;letter-spacing:.6px;padding:6px 8px;
     border-bottom:1px solid var(--line)}
  td{padding:7px 8px;border-bottom:1px solid rgba(42,51,65,.6);
     font-variant-numeric:tabular-nums}
  tr:last-child td{border-bottom:none}
  .pill{display:inline-block;padding:1px 8px;border-radius:99px;font-size:11px;
        font-weight:600}
  .pill.v{background:rgba(63,185,80,.16);color:var(--ok)}
  .pill.p{background:rgba(210,153,34,.16);color:var(--warn)}
  .empty{color:var(--muted);font-size:13px;padding:16px 8px;text-align:center}

  .bar{height:6px;background:var(--panel-2);border-radius:99px;overflow:hidden;
       margin-top:10px}
  .bar > i{display:block;height:100%;background:var(--ok);width:0%;
           transition:width .4s ease}

  .fpswarn{color:var(--bad)} .fpsok{color:var(--ok)}
  .btns{display:flex;gap:8px;padding:0 14px 14px}
  button{flex:1;background:var(--panel-2);color:var(--ink);border:1px solid var(--line);
         border-radius:7px;padding:9px;font-size:13px;font-weight:600;cursor:pointer}
  button:hover{border-color:var(--accent);color:var(--accent)}
</style>
</head>
<body>
<header>
  <h1>LIMO Pro — Classroom People Census</h1>
  <span class="sub" id="model">—</span>
  <span id="conn">offline</span>
</header>

<div class="wrap">
  <div style="display:flex;flex-direction:column;gap:14px;min-width:0">
    <div class="card">
      <h2>Live perception</h2>
      <img id="cam" src="/stream.mjpg" alt="camera stream">
    </div>
    <div class="card">
      <h2>Map</h2>
      <div class="body"><canvas id="mapcanvas" width="900" height="500"></canvas></div>
    </div>
  </div>

  <div style="display:flex;flex-direction:column;gap:14px;min-width:0">
    <div class="card">
      <h2>Count</h2>
      <div class="body">
        <div class="tiles">
          <div class="tile"><div class="k">People</div><div class="v" id="t-count">0</div></div>
          <div class="tile"><div class="k">Visited</div><div class="v" id="t-visited">0</div></div>
          <div class="tile"><div class="k">Inference</div><div class="v" id="t-fps">0<small> fps</small></div></div>
          <div class="tile"><div class="k">Pipeline</div><div class="v" id="t-pfps">0<small> fps</small></div></div>
        </div>
        <div class="bar"><i id="progress"></i></div>
      </div>
    </div>

    <div class="card">
      <h2>Mission</h2>
      <div class="body">
        <span class="state IDLE" id="state">IDLE</span>
        <div class="detail" id="detail">—</div>
        <table style="margin-top:12px">
          <tr><td>waypoint</td><td id="m-wp" style="text-align:right">—</td></tr>
          <tr><td>goals sent / failed</td><td id="m-goals" style="text-align:right">—</td></tr>
          <tr><td>replans</td><td id="m-replan" style="text-align:right">—</td></tr>
          <tr><td>elapsed</td><td id="m-time" style="text-align:right">—</td></tr>
        </table>
      </div>
      <div class="btns">
        <button onclick="cmd('start')">Start</button>
        <button onclick="cmd('stop')">Stop</button>
        <button onclick="cmd('reset')">Reset count</button>
      </div>
    </div>

    <div class="card">
      <h2>People</h2>
      <div class="body" style="padding:0">
        <table id="people">
          <thead><tr><th>#</th><th>x, y (m)</th><th>dist</th><th>conf</th><th>obs</th><th></th></tr></thead>
          <tbody></tbody>
        </table>
        <div class="empty" id="nopeople">no one detected yet</div>
      </div>
    </div>
  </div>
</div>

<script>
let last = null;

function cmd(what){ fetch('/api/'+what, {method:'POST'}); }

function fmtTime(s){
  s = Math.floor(s||0);
  return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');
}

async function poll(){
  try{
    const r = await fetch('/api/state', {cache:'no-store'});
    const d = await r.json();
    last = d;
    render(d);
    document.getElementById('conn').textContent = 'live';
    document.getElementById('conn').className = 'up';
  }catch(e){
    document.getElementById('conn').textContent = 'offline';
    document.getElementById('conn').className = '';
  }
}

function render(d){
  document.getElementById('t-count').textContent   = d.confirmed_count;
  document.getElementById('t-visited').textContent = d.visited_count;

  const f = document.getElementById('t-fps');
  f.innerHTML = (d.inference_fps||0).toFixed(0) + '<small> fps</small>';
  f.className = 'v ' + ((d.inference_fps||0) >= 50 ? 'fpsok' : 'fpswarn');
  document.getElementById('t-pfps').innerHTML =
      (d.pipeline_fps||0).toFixed(0) + '<small> fps</small>';

  document.getElementById('model').textContent =
      (d.backend||'—') + ' · ' + (d.model_name||'—') +
      ' · ' + (d.inference_ms||0).toFixed(1) + ' ms';

  const st = document.getElementById('state');
  st.textContent = d.state; st.className = 'state ' + d.state;
  document.getElementById('detail').textContent = d.detail || '—';
  document.getElementById('m-wp').textContent =
      d.current_waypoint >= 0 ? (d.current_waypoint+1)+' / '+d.total_waypoints
                              : '— / '+d.total_waypoints;
  document.getElementById('m-goals').textContent = d.nav_goals_sent+' / '+d.nav_goals_failed;
  document.getElementById('m-replan').textContent = d.replans;
  document.getElementById('m-time').textContent = fmtTime(d.mission_elapsed_s);

  const pct = d.confirmed_count ? (100*d.visited_count/d.confirmed_count) : 0;
  document.getElementById('progress').style.width = pct.toFixed(0)+'%';

  const tb = document.querySelector('#people tbody');
  tb.innerHTML = '';
  (d.people||[]).forEach(p => {
    const tr = document.createElement('tr');
    tr.innerHTML =
      '<td>'+p.id+'</td>'+
      '<td>'+p.x.toFixed(2)+', '+p.y.toFixed(2)+'</td>'+
      '<td>'+(p.distance>=0 ? p.distance.toFixed(2)+' m' : '—')+'</td>'+
      '<td>'+p.confidence.toFixed(2)+'</td>'+
      '<td>'+p.observations+'</td>'+
      '<td>'+(p.visited ? '<span class="pill v">visited</span>'
                        : '<span class="pill p">pending</span>')+'</td>';
    tb.appendChild(tr);
  });
  document.getElementById('nopeople').style.display =
      (d.people||[]).length ? 'none' : 'block';

  drawMap(d);
}

function drawMap(d){
  const c = document.getElementById('mapcanvas');
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H);

  const pts = [];
  (d.people||[]).forEach(p => pts.push([p.x,p.y]));
  (d.waypoints||[]).forEach(w => pts.push([w[0],w[1]]));
  if(d.robot) pts.push([d.robot[0], d.robot[1]]);
  if(!pts.length){
    ctx.fillStyle='#8b949e'; ctx.font='13px monospace';
    ctx.fillText('waiting for map-frame data…', 16, 26); return;
  }

  let minx=Math.min(...pts.map(p=>p[0])), maxx=Math.max(...pts.map(p=>p[0]));
  let miny=Math.min(...pts.map(p=>p[1])), maxy=Math.max(...pts.map(p=>p[1]));
  const pad = 1.5;
  minx-=pad; maxx+=pad; miny-=pad; maxy+=pad;
  const s = Math.min(W/(maxx-minx), H/(maxy-miny));
  // ROS map frame is x-right / y-up; canvas y grows downwards.
  const X = x => (x-minx)*s;
  const Y = y => H-(y-miny)*s;

  ctx.strokeStyle='#1c2431'; ctx.lineWidth=1;
  for(let gx=Math.ceil(minx); gx<=maxx; gx++){
    ctx.beginPath(); ctx.moveTo(X(gx),0); ctx.lineTo(X(gx),H); ctx.stroke();
  }
  for(let gy=Math.ceil(miny); gy<=maxy; gy++){
    ctx.beginPath(); ctx.moveTo(0,Y(gy)); ctx.lineTo(W,Y(gy)); ctx.stroke();
  }

  ctx.strokeStyle='#30475e'; ctx.lineWidth=1.5; ctx.setLineDash([5,5]);
  ctx.beginPath();
  (d.waypoints||[]).forEach((w,i)=>{ i? ctx.lineTo(X(w[0]),Y(w[1])) : ctx.moveTo(X(w[0]),Y(w[1])); });
  ctx.stroke(); ctx.setLineDash([]);

  (d.waypoints||[]).forEach((w,i)=>{
    ctx.fillStyle = (i===d.current_waypoint) ? '#58a6ff' : '#30475e';
    ctx.beginPath(); ctx.arc(X(w[0]),Y(w[1]),6,0,7); ctx.fill();
    ctx.fillStyle='#8b949e'; ctx.font='11px monospace';
    ctx.fillText('W'+(i+1), X(w[0])+9, Y(w[1])-7);
  });

  (d.people||[]).forEach(p=>{
    ctx.fillStyle = p.visited ? '#3fb950' : '#d29922';
    ctx.beginPath(); ctx.arc(X(p.x),Y(p.y),11,0,7); ctx.fill();
    ctx.fillStyle='#0d1117'; ctx.font='bold 11px monospace';
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(String(p.id), X(p.x), Y(p.y));
    ctx.textAlign='start'; ctx.textBaseline='alphabetic';
  });

  if(d.robot){
    const [rx,ry,ryaw] = d.robot;
    ctx.save();
    ctx.translate(X(rx),Y(ry)); ctx.rotate(-ryaw);
    ctx.fillStyle='#58a6ff';
    ctx.beginPath(); ctx.moveTo(14,0); ctx.lineTo(-9,8); ctx.lineTo(-9,-8);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }
}

setInterval(poll, 250);
poll();
</script>
</body>
</html>
"""
