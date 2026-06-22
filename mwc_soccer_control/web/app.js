const $ = (id) => document.getElementById(id);
const field = $("field");
const fieldCtx = field.getContext("2d");
const chart = $("chart");
const chartCtx = chart.getContext("2d");

let latest = null;
let lastEventAt = 0;
const history = { ball: [], velocity: [] };
const penaltyRobotTrail = [];
let previousMode = null;
let previousState = null;

const penaltyStages = [
  "NAVIGATE_TO_POINT", "START_BALL_TRACK", "ALIGN",
  "STOP_BALL_TRACK", "READY_KICK", "KICK", "FINISH"
];
const goalkeeperStages = ["TRACK_BALL"];

function modeOf(data) {
  return data.behavior?.mode || data.command?.mode || data.navigation?.mode || "IDLE";
}

function stateOf(data) {
  return data.behavior?.state || "IDLE";
}

function effectiveVelocity(data) {
  if (data.velocity) return data.velocity;
  const values = data.behavior?.values || {};
  if (Number.isFinite(Number(values.vx)) ||
      Number.isFinite(Number(values.vy)) ||
      Number.isFinite(Number(values.wz))) {
    return {
      vx: finite(values.vx),
      vy: finite(values.vy),
      wz: finite(values.wz),
      dry_run: true,
    };
  }
  return null;
}

function finite(value, fallback = 0) {
  return Number.isFinite(Number(value)) ? Number(value) : fallback;
}

function fixed(value, digits = 2, suffix = "") {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function resize(canvas, ctx) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return rect;
}

function setConnection(online) {
  $("connection-dot").className = `dot ${online ? "online" : "offline"}`;
  $("connection-text").textContent = online ? "实时数据已连接" : "连接已断开";
}

function render(data) {
  latest = data;
  const mode = modeOf(data);
  const state = stateOf(data);
  const perception = data.perception;
  const velocity = effectiveVelocity(data);

  if (mode !== previousMode ||
      (state === "NAVIGATE_TO_POINT" && previousState !== "NAVIGATE_TO_POINT")) {
    penaltyRobotTrail.length = 0;
  }
  previousMode = mode;
  previousState = state;

  $("mode").textContent = mode;
  $("state").textContent = state;
  $("confidence").textContent = perception ? `${Math.round(perception.ball_confidence * 100)}%` : "—";
  $("velocity").textContent = velocity
    ? `(${fixed(velocity.vx)}, ${fixed(velocity.vy)}, ${fixed(velocity.wz)})`
    : "—";

  renderPipeline(mode, state, data.behavior);
  renderTelemetry(data);
  renderHealth(data);
  renderEvents(data.events || []);
  renderFreshness(data);
  renderField(data);

  const now = Date.now();
  if (now - lastEventAt > 90) {
    history.ball.push(perception ? finite(perception.ball.x) : null);
    history.velocity.push(velocity ? finite(velocity.vy) : null);
    if (history.ball.length > 120) history.ball.shift();
    if (history.velocity.length > 120) history.velocity.shift();
    lastEventAt = now;
  }
  renderChart();
}

function renderPipeline(mode, state, behavior) {
  const stages = mode === "GOALKEEPER" ? goalkeeperStages : penaltyStages;
  const index = stages.indexOf(state);
  $("pipeline").innerHTML = stages.map((stage, i) => {
    const cls = i === index ? "current" : (index >= 0 && i < index ? "done" : "");
    return `<div class="stage ${cls}">${stage}</div>`;
  }).join("");
  $("progress").style.width = `${Math.max(0, Math.min(100, finite(behavior?.progress) * 100))}%`;
  $("detail").textContent = behavior?.detail || "尚未收到行为状态";
  const badge = $("active-badge");
  badge.textContent = state === "ERROR" ? "异常" : (behavior?.active ? "运行中" : "待机");
  badge.className = `badge ${state === "ERROR" ? "error" : (behavior?.active ? "" : "muted")}`;
}

function renderTelemetry(data) {
  const p = data.perception;
  const v = effectiveVelocity(data);
  const values = data.behavior?.values || {};
  const mode = modeOf(data);
  const items = mode === "GOALKEEPER" ? [
    ["图像 X", fixed(p?.ball.x, 3)],
    ["图像 Y", fixed(p?.ball.y, 3)],
    ["横移 vy", fixed(v?.vy, 3, " m/s")],
    ["死区", `±${fixed(data.config.goalkeeper_center_deadband_x, 3)}`],
    ["球有效", p?.image_has_ball && p?.transform_valid ? "是" : "否"],
    ["帧坐标", p?.ball.frame || "—"],
  ] : [
    ["相对距离", p?.image_has_ball ? fixed(Math.hypot(p.ball.x, p.ball.y), 3, " m") : "—"],
    ["相对角度", p?.image_has_ball ? fixed(Math.atan2(p.ball.y, p.ball.x) * 180 / Math.PI, 1, "°") : "—"],
    ["X 误差", fixed(values.x_err, 3, " m")],
    ["Y 误差", fixed(values.y_err, 3, " m")],
    ["目标距离", fixed(Math.hypot(data.config.align_target_ball_x_m, data.config.align_target_ball_y_m), 3, " m")],
    ["目标角度", fixed(Math.atan2(data.config.align_target_ball_y_m, data.config.align_target_ball_x_m) * 180 / Math.PI, 1, "°")],
  ];
  $("telemetry").innerHTML = items.map(
    ([label, value]) => `<div><span>${label}</span><strong>${value}</strong></div>`
  ).join("");
}

function renderHealth(data) {
  const stale = (age, limit) => age === null || age > limit;
  const rows = [
    ["行为状态", data.behavior_age_s, 2.0, Boolean(data.behavior)],
    ["足球感知", data.perception_age_s, data.config.perception_stale_s, Boolean(data.perception)],
    ["导航状态", data.navigation_age_s, data.config.nav_stale_s, Boolean(data.navigation)],
    ["速度指令", data.velocity_age_s, 2.0, Boolean(data.velocity)],
  ];
  $("health").innerHTML = rows.map(([name, age, limit, present]) => {
    const bad = stale(age, limit);
    const cls = !present ? "error" : (bad ? "warn" : "");
    const text = age === null ? "未收到" : `${fixed(age, 2, " s")}`;
    return `<div class="health-row"><span>${name}</span><span>${text}</span><i class="health-state ${cls}"></i></div>`;
  }).join("");
}

function renderEvents(events) {
  $("events").innerHTML = events.slice().reverse().map(event => `
    <div class="event">
      <small>${event.time} · ${event.mode}</small>
      <strong>${event.state}</strong>
      <p title="${escapeHtml(event.detail)}">${escapeHtml(event.detail)}</p>
    </div>
  `).join("") || '<p class="detail">尚无状态切换事件</p>';
}

function escapeHtml(text) {
  return String(text || "").replace(/[&<>"']/g, char => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#039;"
  }[char]));
}

function renderFreshness(data) {
  const badge = $("perception-freshness");
  const age = data.perception_age_s;
  const valid = data.perception?.image_has_ball && data.perception?.transform_valid;
  if (age === null) {
    badge.textContent = "无感知";
    badge.className = "badge muted";
  } else if (age > data.config.perception_stale_s || !valid) {
    badge.textContent = `感知异常 · ${fixed(age, 1, "s")}`;
    badge.className = "badge warn";
  } else {
    badge.textContent = `实时 · ${fixed(age, 1, "s")}`;
    badge.className = "badge";
  }
}

function baseCanvas(ctx, rect) {
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#0a2117";
  ctx.fillRect(0, 0, rect.width, rect.height);
}

function renderField(data) {
  const rect = resize(field, fieldCtx);
  baseCanvas(fieldCtx, rect);
  if (modeOf(data) === "GOALKEEPER") {
    $("view-title").textContent = "守门 · 相机归一化视图";
    drawGoalkeeper(data, rect);
    $("legend").innerHTML = '<span style="--legend:#59f28f">足球</span><span style="--legend:#ffd166">中心死区</span><span style="--legend:#62d9ff">横移速度</span>';
  } else {
    $("view-title").textContent = "点球 · 固定球 / 机器人相对移动";
    drawPenalty(data, rect);
    $("legend").innerHTML = '<span style="--legend:#59f28f">固定点球</span><span style="--legend:#effff4">机器人</span><span style="--legend:#c4ff67">目标站位</span><span style="--legend:#62d9ff">相对移动轨迹</span><span style="--legend:#ffd166">射门目标</span>';
  }
}

function drawPitch(ctx, x, y, w, h) {
  ctx.fillStyle = "#103c27";
  ctx.fillRect(x, y, w, h);
  ctx.strokeStyle = "rgba(220,255,232,.42)";
  ctx.lineWidth = 1;
  ctx.strokeRect(x, y, w, h);
  ctx.beginPath();
  ctx.moveTo(x, y + h / 2); ctx.lineTo(x + w, y + h / 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(x + w / 2, y + h / 2, Math.min(w, h) * .1, 0, Math.PI * 2);
  ctx.stroke();
}

function drawPenalty(data, rect) {
  const ctx = fieldCtx;
  const pad = 28;
  const x = pad, y = pad, w = rect.width - pad * 2, h = rect.height - pad * 2;
  drawPitch(ctx, x, y, w, h);

  const centerX = x + w / 2;
  const ballX = centerX;
  const ballY = y + h * .48;
  const metersToPixels = Math.min(w * .34 / 1.1, h * .38 / 1.2);
  const robotFromBall = (forward, lateral) => ({
    x: ballX + lateral * metersToPixels,
    y: ballY + forward * metersToPixels,
  });

  ctx.strokeStyle = "rgba(255,255,255,.15)";
  ctx.setLineDash([5, 7]);
  ctx.beginPath(); ctx.moveTo(centerX, y + 15); ctx.lineTo(centerX, y + h - 18); ctx.stroke();
  ctx.setLineDash([]);

  const goalY = y + 20;
  const goalW = w * .52;
  ctx.lineWidth = 5;
  ctx.strokeStyle = "#effff4";
  ctx.beginPath();
  ctx.moveTo(centerX - goalW / 2, goalY + 18);
  ctx.lineTo(centerX - goalW / 2, goalY);
  ctx.lineTo(centerX + goalW / 2, goalY);
  ctx.lineTo(centerX + goalW / 2, goalY + 18);
  ctx.stroke();

  const goalTarget = finite(data.command?.goal_target);
  const targetX = centerX + goalTarget * goalW / 2;
  ctx.fillStyle = "#ffd166";
  ctx.beginPath(); ctx.arc(targetX, goalY + 4, 6, 0, Math.PI * 2); ctx.fill();

  ctx.strokeStyle = "rgba(255,209,102,.38)";
  ctx.setLineDash([6, 6]);
  ctx.beginPath(); ctx.moveTo(ballX, ballY); ctx.lineTo(targetX, goalY + 4); ctx.stroke();
  ctx.setLineDash([]);

  const target = robotFromBall(
    data.config.align_target_ball_x_m,
    data.config.align_target_ball_y_m
  );
  const tolW = data.config.align_y_tolerance_m * metersToPixels * 2;
  const tolH = data.config.align_x_tolerance_m * metersToPixels * 2;
  ctx.fillStyle = "rgba(196,255,103,.12)";
  ctx.strokeStyle = "#c4ff67";
  ctx.lineWidth = 1;
  ctx.fillRect(target.x - tolW / 2, target.y - tolH / 2, tolW, tolH);
  ctx.strokeRect(target.x - tolW / 2, target.y - tolH / 2, tolW, tolH);
  drawCross(ctx, target.x, target.y, "#c4ff67");
  drawRobot(ctx, target.x, target.y, "#c4ff67", .32);

  const ball = data.perception?.ball;
  let robot = null;
  if (ball && data.perception.image_has_ball) {
    robot = robotFromBall(finite(ball.x), finite(ball.y));
    const last = penaltyRobotTrail[penaltyRobotTrail.length - 1];
    if (!last || Math.hypot(last.x - robot.x, last.y - robot.y) > 1.5) {
      penaltyRobotTrail.push(robot);
      if (penaltyRobotTrail.length > 160) penaltyRobotTrail.shift();
    }

    if (penaltyRobotTrail.length > 1) {
      ctx.strokeStyle = "rgba(98,217,255,.58)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      penaltyRobotTrail.forEach((entry, index) => {
        if (index === 0) ctx.moveTo(entry.x, entry.y);
        else ctx.lineTo(entry.x, entry.y);
      });
      ctx.stroke();
      ctx.lineWidth = 1;
    }

    ctx.strokeStyle = "rgba(89,242,143,.35)";
    ctx.beginPath(); ctx.moveTo(robot.x, robot.y - 18); ctx.lineTo(ballX, ballY); ctx.stroke();
    drawRobot(ctx, robot.x, robot.y);
  } else {
    ctx.fillStyle = "rgba(239,255,244,.62)";
    ctx.font = "12px system-ui";
    ctx.textAlign = "center";
    ctx.fillText(
      stateOf(data) === "NAVIGATE_TO_POINT"
        ? "导航至点球点前目标站位…"
        : "等待球相对坐标",
      centerX,
      y + h - 30
    );
    ctx.textAlign = "left";
  }
  drawBall(ctx, ballX, ballY);

  const velocity = effectiveVelocity(data);
  if (velocity && robot) {
    drawArrow(
      ctx,
      robot.x,
      robot.y,
      -finite(velocity.vy) * 55,
      -finite(velocity.vx) * 55,
      "#62d9ff"
    );
  }
  ctx.fillStyle = "rgba(239,255,244,.65)";
  ctx.font = "11px system-ui";
  ctx.fillText("球固定在点球点", x + 10, y + h - 10);
  ctx.fillText("机器人位置由相对球坐标反算", x + w - 166, y + h - 10);
}

function drawGoalkeeper(data, rect) {
  const ctx = fieldCtx;
  const pad = 34;
  const x = pad, y = pad, w = rect.width - pad * 2, h = rect.height - pad * 2;
  ctx.fillStyle = "#0c291c";
  ctx.fillRect(x, y, w, h);
  ctx.strokeStyle = "rgba(220,255,232,.35)";
  ctx.strokeRect(x, y, w, h);

  const centerX = x + w / 2;
  const deadband = data.config.goalkeeper_center_deadband_x;
  const deadbandW = deadband * w;
  ctx.fillStyle = "rgba(255,209,102,.10)";
  ctx.fillRect(centerX - deadbandW / 2, y, deadbandW, h);
  ctx.strokeStyle = "#ffd166";
  ctx.setLineDash([5, 6]);
  ctx.beginPath(); ctx.moveTo(centerX, y); ctx.lineTo(centerX, y + h); ctx.stroke();
  ctx.setLineDash([]);

  for (let i = 1; i < 4; i++) {
    ctx.strokeStyle = "rgba(255,255,255,.07)";
    ctx.beginPath();
    ctx.moveTo(x, y + h * i / 4); ctx.lineTo(x + w, y + h * i / 4);
    ctx.stroke();
  }

  const ball = data.perception?.ball;
  if (ball && data.perception.image_has_ball) {
    const bx = centerX + Math.max(-1, Math.min(1, finite(ball.x))) * w / 2;
    const by = y + h / 2 - Math.max(-1, Math.min(1, finite(ball.y))) * h / 2;
    drawBall(ctx, bx, by);
    ctx.fillStyle = "#effff4";
    ctx.font = "12px system-ui";
    ctx.fillText(`(${fixed(ball.x, 2)}, ${fixed(ball.y, 2)})`, bx + 13, by - 10);
  }

  drawRobot(ctx, centerX, y + h - 30);
  const vy = finite(effectiveVelocity(data)?.vy);
  drawArrow(ctx, centerX, y + h - 30, vy * 90, 0, "#62d9ff");
  ctx.fillStyle = "rgba(239,255,244,.62)";
  ctx.font = "11px system-ui";
  ctx.fillText("-1 左", x + 8, y + h - 9);
  ctx.fillText("+1 右", x + w - 42, y + h - 9);
}

function drawRobot(ctx, x, y, color = "#effff4", alpha = 1) {
  ctx.save();
  ctx.translate(x, y);
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.fillRect(-11, -18, 22, 26);
  ctx.fillStyle = "#62d9ff";
  ctx.beginPath(); ctx.moveTo(0, -28); ctx.lineTo(-7, -17); ctx.lineTo(7, -17); ctx.closePath(); ctx.fill();
  ctx.restore();
}

function drawBall(ctx, x, y) {
  ctx.shadowColor = "#59f28f";
  ctx.shadowBlur = 18;
  ctx.fillStyle = "#59f28f";
  ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.fill();
  ctx.shadowBlur = 0;
  ctx.fillStyle = "#082014";
  ctx.beginPath(); ctx.arc(x + 2, y - 2, 3, 0, Math.PI * 2); ctx.fill();
}

function drawCross(ctx, x, y, color) {
  ctx.strokeStyle = color;
  ctx.beginPath();
  ctx.moveTo(x - 8, y); ctx.lineTo(x + 8, y);
  ctx.moveTo(x, y - 8); ctx.lineTo(x, y + 8);
  ctx.stroke();
}

function drawArrow(ctx, x, y, dx, dy, color) {
  if (Math.hypot(dx, dy) < 2) return;
  const ex = x + dx, ey = y + dy;
  const angle = Math.atan2(dy, dx);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 9 * Math.cos(angle - .5), ey - 9 * Math.sin(angle - .5));
  ctx.lineTo(ex - 9 * Math.cos(angle + .5), ey - 9 * Math.sin(angle + .5));
  ctx.closePath(); ctx.fill();
  ctx.lineWidth = 1;
}

function renderChart() {
  const rect = resize(chart, chartCtx);
  chartCtx.clearRect(0, 0, rect.width, rect.height);
  chartCtx.fillStyle = "rgba(5,15,10,.45)";
  chartCtx.fillRect(0, 0, rect.width, rect.height);
  chartCtx.strokeStyle = "rgba(255,255,255,.08)";
  chartCtx.beginPath(); chartCtx.moveTo(0, rect.height / 2); chartCtx.lineTo(rect.width, rect.height / 2); chartCtx.stroke();
  plot(history.ball, "#59f28f", rect, 1.2);
  plot(history.velocity, "#62d9ff", rect, 1.2);
  chartCtx.font = "10px system-ui";
  chartCtx.fillStyle = "#59f28f"; chartCtx.fillText("球 X", 9, 14);
  chartCtx.fillStyle = "#62d9ff"; chartCtx.fillText("vy", 48, 14);
}

function plot(values, color, rect, range) {
  if (values.length < 2) return;
  chartCtx.strokeStyle = color;
  chartCtx.lineWidth = 1.5;
  chartCtx.beginPath();
  let drawing = false;
  values.forEach((value, i) => {
    if (value === null) { drawing = false; return; }
    const x = i / 119 * rect.width;
    const y = rect.height / 2 - Math.max(-range, Math.min(range, value)) / range * rect.height * .4;
    if (!drawing) { chartCtx.moveTo(x, y); drawing = true; } else chartCtx.lineTo(x, y);
  });
  chartCtx.stroke();
}

const source = new EventSource("/events");
source.onopen = () => setConnection(true);
source.onmessage = (event) => {
  try { render(JSON.parse(event.data)); } catch (error) { console.error(error); }
};
source.onerror = () => setConnection(false);

window.addEventListener("resize", () => {
  if (latest) renderField(latest);
  renderChart();
});
