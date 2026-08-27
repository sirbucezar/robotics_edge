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
  header{display:flex;align-items:baseline;gap:16px;padding:8px 16px;
         border-bottom:1px solid var(--line);background:var(--panel)}
  header h1{margin:0;font-size:16px;font-weight:650;letter-spacing:.2px}
  header .sub{color:var(--muted);font-size:12px;font-family:var(--mono)}
  #conn{margin-left:auto;font-size:12px;font-family:var(--mono);color:var(--bad)}
  #conn.up{color:var(--ok)}

  /* One screen, no scrolling: perception and map share the top row, the
     numbers sit underneath. The camera feed is deliberately small -- it is
     evidence that detection works, not the subject of the page. */
  .wrap{display:flex;flex-direction:column;gap:8px;padding:8px 12px;
        max-width:1600px;margin:0 auto}
  /* Live perception takes almost the full width; the map is a small locator
     in the top-right, ~80% of the feed panel's former width. The feed is what
     an examiner watches, the map only answers "where is it right now". */
  .toprow{display:grid;grid-template-columns:minmax(0,760px) minmax(220px,1fr);
          gap:12px;align-items:start}
  /* Headline numbers sit beside the feed, stacked, so a viewer reads camera
     and count in one glance without the eye travelling down the page. */
  .toprow .tiles{grid-template-columns:1fr}
  .feedwrap{position:relative}
  /* The map is a picture-in-picture locator, not a peer panel: overlaying it
     keeps one thing to look at instead of two, and the corner it sits in is
     ceiling and wall, never the floor where detections happen. */
  .mapover{position:absolute;top:8px;right:8px;width:230px;
           background:rgba(13,17,23,.86);border:1px solid var(--line);
           border-radius:8px;padding:6px 6px 2px;backdrop-filter:blur(2px)}
  .mapover .lab{font:10px var(--mono);color:var(--muted);
                text-transform:uppercase;letter-spacing:.7px;padding:0 2px 4px}
  @media (max-width:800px){.mapover{position:static;width:auto;margin:10px}}
  .botrow{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
  @media (max-width:1000px){.toprow,.botrow{grid-template-columns:1fr}}

  .legend{display:flex;gap:14px;flex-wrap:wrap;padding:5px 12px;
          border-top:1px solid var(--line);font-size:12px;font-family:var(--mono)}
  .legend span{display:flex;align-items:center;gap:6px;color:var(--muted)}
  .legend i{width:11px;height:11px;border-radius:3px;display:inline-block}

  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
        overflow:hidden}
  .card h2{margin:0;padding:6px 12px;font-size:11px;font-weight:600;
           text-transform:uppercase;letter-spacing:.8px;color:var(--muted);
           border-bottom:1px solid var(--line);background:var(--panel-2)}
  .card .body{padding:10px 12px}

  .tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
  .tile{background:var(--panel-2);border:1px solid var(--line);border-radius:8px;
        padding:8px 10px}
  .tile .k{font-size:11px;color:var(--muted);text-transform:uppercase;
           letter-spacing:.6px}
  .tile .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums;
           line-height:1.15;margin-top:1px}
  .tile .v small{font-size:13px;color:var(--muted);font-weight:500}

  /* 4/3 made the panel tall and the lower third is bare floor. 1.9:1 removes
     ~30% of the height; object-fit:cover crops rather than letterboxes, so the
     detections stay full size instead of shrinking. */
  /* The camera is 640x480, so show it at 4:3 and never crop or stretch it --
     a squashed feed misrepresents where a box sits in the frame. Width is
     capped instead, which keeps the panel from dominating a wide screen while
     leaving the four headline numbers visible without scrolling. */
  #cam{width:100%;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
  .feedcard{max-width:760px}
  #mapcanvas{width:100%;height:auto;display:block}
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
  td{padding:4px 8px;border-bottom:1px solid rgba(42,51,65,.6);
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
  .btns{display:flex;gap:8px;padding:0 12px 10px}
  button{flex:1;background:var(--panel-2);color:var(--ink);border:1px solid var(--line);
         border-radius:7px;padding:6px;font-size:12px;font-weight:600;cursor:pointer}
  button:hover{border-color:var(--accent);color:var(--accent)}
  /* Deliberately the loudest control on the page. It cancels every nav2 goal
     and floods zero velocities -- the one button someone may need to find in
     a hurry while the robot is moving. */
  #estop{background:var(--bad);color:#fff;border-color:var(--bad);
         font-size:14px;letter-spacing:.6px;padding:11px}
  #estop:hover{background:#ff6b61;color:#fff;border-color:#ff6b61}
</style>
</head>
<body>
<style>.teleop-live{color:#7fd6a2}.capture-live{color:#ffd166}</style>
<header>
  <h1>LIMO Pro — Classroom People Census</h1>
  <span class="sub" id="model">—</span>
  <span id="conn">offline</span>
</header>

<div class="wrap">
  <div class="toprow">
    <div class="card feedcard">
      <h2>Live perception</h2>
      <div class="feedwrap">
        <img id="cam" src="/stream.mjpg" alt="camera stream">
        <div class="mapover">
          <div class="lab">position</div>
          <canvas id="mapcanvas" width="230" height="185"></canvas>
        </div>
      </div>
      <div class="legend">
        <span><i style="background:#50c878"></i>person</span>
        <span><i style="background:#ebbe3c"></i>chair</span>
        <span><i style="background:#f05a5a"></i>other</span>
      </div>
    </div>

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
  </div>

  <div class="botrow">
    <div class="card">
      <h2>Mission</h2>
      <div class="body">
        <span class="state IDLE" id="state">IDLE</span>
        <div class="detail" id="detail">&mdash;</div>
        <table style="margin-top:12px">
          <tr><td>waypoint</td><td id="m-wp" style="text-align:right">&mdash;</td></tr>
          <tr><td>goals sent / failed</td><td id="m-goals" style="text-align:right">&mdash;</td></tr>
          <tr><td>replans</td><td id="m-replan" style="text-align:right">&mdash;</td></tr>
          <tr><td>elapsed</td><td id="m-time" style="text-align:right">&mdash;</td></tr>
        </table>
      </div>
      <div class="btns">
        <button onclick="cmd('start')">Start</button>
        <button onclick="cmd('reset');TRAIL.length=0">Reset</button>
      </div>
      <div class="btns">
        <button id="estop" onclick="cmd('estop')">EMERGENCY STOP</button>
      </div>
    </div>

    <div class="card" style="grid-column:span 2">
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

/* Where the robot has actually been, in map coordinates. Preset waypoints were
   removed: they described a route nobody is following any more and made the
   map look authoritative about a plan that does not exist. A home glyph plus
   the real travelled path says only what is true. */
const TRAIL = [];
const TRAIL_MAX = 4000;
function pushTrail(r){
  if(!r) return;
  const p = TRAIL[TRAIL.length-1];
  if(p && Math.hypot(p[0]-r[0], p[1]-r[1]) < 0.05) return;  // 5 cm dedupe
  TRAIL.push([r[0], r[1]]);
  if(TRAIL.length > TRAIL_MAX) TRAIL.shift();
}

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
    renderCapture(d);
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

  pushTrail(d.robot);

  const pts = [[0,0]];                       // home is always in frame
  (d.people||[]).forEach(p => pts.push([p.x,p.y]));
  TRAIL.forEach(t => pts.push(t));
  if(d.robot) pts.push([d.robot[0], d.robot[1]]);

  let minx=Math.min(...pts.map(p=>p[0])), maxx=Math.max(...pts.map(p=>p[0]));
  let miny=Math.min(...pts.map(p=>p[1])), maxy=Math.max(...pts.map(p=>p[1]));
  const pad = 1.0;
  minx-=pad; maxx+=pad; miny-=pad; maxy+=pad;
  // Keep metres square in both axes, or the room shears as the trail grows.
  const s = Math.min(W/(maxx-minx), H/(maxy-miny));
  const cx = (minx+maxx)/2, cy = (miny+maxy)/2;
  const X = x => W/2 + (x-cx)*s;
  const Y = y => H/2 - (y-cy)*s;

  ctx.strokeStyle='#1c2431'; ctx.lineWidth=1;
  for(let gx=Math.ceil(minx); gx<=maxx; gx++){
    ctx.beginPath(); ctx.moveTo(X(gx),0); ctx.lineTo(X(gx),H); ctx.stroke();
  }
  for(let gy=Math.ceil(miny); gy<=maxy; gy++){
    ctx.beginPath(); ctx.moveTo(0,Y(gy)); ctx.lineTo(W,Y(gy)); ctx.stroke();
  }

  /* path actually driven */
  if(TRAIL.length > 1){
    ctx.strokeStyle='#58a6ff'; ctx.lineWidth=2; ctx.globalAlpha=0.75;
    ctx.beginPath();
    TRAIL.forEach((t,i)=> i ? ctx.lineTo(X(t[0]),Y(t[1])) : ctx.moveTo(X(t[0]),Y(t[1])));
    ctx.stroke(); ctx.globalAlpha=1;
  }

  /* home glyph at the origin -- which IS the taped start spot, because
     cartographer was restarted with the robot standing on it */
  const hx = X(0), hy = Y(0);
  ctx.strokeStyle='#8b949e'; ctx.lineWidth=1.6;
  ctx.beginPath();
  ctx.moveTo(hx-7, hy+1); ctx.lineTo(hx, hy-6); ctx.lineTo(hx+7, hy+1);
  ctx.stroke();
  ctx.strokeRect(hx-5, hy+1, 10, 7);
  ctx.fillStyle='#8b949e'; ctx.font='10px monospace';
  ctx.fillText('home', hx-13, hy+21);

  (d.people||[]).forEach(p=>{
    ctx.fillStyle = p.visited ? '#3fb950' : '#d29922';
    ctx.beginPath(); ctx.arc(X(p.x),Y(p.y),10,0,7); ctx.fill();
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
    ctx.beginPath(); ctx.moveTo(13,0); ctx.lineTo(-8,7); ctx.lineTo(-8,-7);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  ctx.fillStyle='#8b949e'; ctx.font='10px monospace';
  ctx.fillText('1 m grid', 6, H-6);
}

setInterval(poll, 250);

/* ---- WASD teleop -------------------------------------------------------
   Held keys drive; releasing stops. We resend at 10 Hz rather than once per
   keypress because the robot deadman-stops if commands go stale -- that is
   what makes a closed tab or a dropped connection safe rather than a runaway.
------------------------------------------------------------------------- */
const TELEOP_KEYS = {w:0, a:0, s:0, d:0};
let teleopOn = false;

function teleopVector(){
  const vx = (TELEOP_KEYS.w ? 1 : 0) - (TELEOP_KEYS.s ? 1 : 0);
  const wz = (TELEOP_KEYS.a ? 1 : 0) - (TELEOP_KEYS.d ? 1 : 0);
  return [vx, wz];
}

async function sendTeleop(){
  if(!teleopOn) return;
  const [vx, wz] = teleopVector();
  if(vx === 0 && wz === 0) return;      // idle: stay off /cmd_vel for nav2
  try{
    await fetch(`/api/teleop?vx=${vx * TELEOP_SPEED}&wz=${wz * TELEOP_TURN}`,
                {method: 'POST'});
  }catch(e){ /* transient network drop; the deadman handles it */ }
}

const TELEOP_SPEED = 0.35;
const TELEOP_TURN  = 0.9;

function setTeleopKey(ev, down){
  const k = ev.key.toLowerCase();
  if(!(k in TELEOP_KEYS)) return;
  if(document.activeElement && /INPUT|TEXTAREA/.test(document.activeElement.tagName)) return;
  ev.preventDefault();
  TELEOP_KEYS[k] = down ? 1 : 0;
  const badge = document.getElementById('teleop-badge');
  if(badge){
    const [vx, wz] = teleopVector();
    badge.textContent = (vx||wz) ? `driving  vx=${vx}  wz=${wz}` : 'idle';
    badge.className = (vx||wz) ? 'teleop-live' : '';
  }
  if(down) sendTeleop();
}

/* ---- dataset capture keys ---------------------------------------------
   O = sitting in the seat, E = seat empty, P = standing, B = bag on the seat.
   B files as chair_empty deliberately: it is a hard negative, so a rucksack
   on a chair does not read as a student.

   Every capture counts down first. The operator is alone in the room and has
   to walk from the laptop to the chair before the frames are taken.
------------------------------------------------------------------------- */
const CAPTURE_KEYS = {o: 'chair_occupied', e: 'chair_empty',
                      p: 'person_standing', b: 'chair_empty'};

async function sendCapture(label){
  try{
    const r = await fetch(`/api/capture?label=${label}`, {method: 'POST'});
    if(r.status === 409) setCaptureBadge('busy -- one capture at a time');
  }catch(err){ setCaptureBadge('capture request failed'); }
}

function setCaptureBadge(text, live){
  const b = document.getElementById('capture-badge');
  if(!b) return;
  b.textContent = text;
  b.className = live ? 'capture-live' : '';
}

function onCaptureKey(ev){
  const k = ev.key.toLowerCase();
  if(!(k in CAPTURE_KEYS)) return;
  if(document.activeElement && /INPUT|TEXTAREA/.test(document.activeElement.tagName)) return;
  ev.preventDefault();
  sendCapture(CAPTURE_KEYS[k]);
}

window.addEventListener('keydown', onCaptureKey);

window.addEventListener('keydown', e => setTeleopKey(e, true));
window.addEventListener('keyup',   e => setTeleopKey(e, false));
/* Losing focus must release everything -- otherwise an alt-tab while holding
   W leaves a key stuck down in our own state. */
window.addEventListener('blur', () => {
  for(const k in TELEOP_KEYS) TELEOP_KEYS[k] = 0;
  const badge = document.getElementById('teleop-badge');
  if(badge){ badge.textContent = 'idle'; badge.className = ''; }
});

setInterval(sendTeleop, 100);
teleopOn = true;

poll();
</script>
</body>
</html>
"""
