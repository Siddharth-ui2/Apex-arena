// Simulation constants (same as your React version)
const TRACK_LENGTH = 4200; // meters
const DT = 0.05;
const OVERTAKE_MIN_GAP = 3.0;
const OVERTAKE_BASE_SUCCESS = 0.85;
const PIT_STOP_TIME = 12.0;
const PIT_THRESHOLD_ENERGY = 0.22;
const MECH_FAILURE_PER_LAP = 0.001;

// helper conversions
const mps = kmh => kmh * 1000 / 3600;
const kmh = mpsVal => mpsVal * 3.6;

// ---------------- AgentState ----------------
class AgentState {
  constructor(config){
    this.id = config.id;
    this.name = config.name;
    this.color = config.color;
    this.maxSpeed = config.maxSpeed;
    this.accel = config.accel;
    this.braking = config.braking;
    this.energyCapacity = config.energyCapacity;
    this.energy = (config.energy === undefined) ? config.energyCapacity : config.energy;
    this.aggression = config.aggression;
    this.reliability = config.reliability;
    this.pos = 0;
    this.lapsCompleted = 0;
    this.speed = 0;
    this.status = 'running';
    this.pitTimeLeft = 0;
    this.totalTime = 0;
    this.currentLapStartTime = 0;
    this.lastLapTime = null;
    this.bestLapTime = null;
    this.penalties = 0;
  }

  clone(){
    const cfg = {
      id:this.id, name:this.name, color:this.color, maxSpeed:this.maxSpeed,
      accel:this.accel, braking:this.braking, energyCapacity:this.energyCapacity,
      energy:this.energy, aggression:this.aggression, reliability:this.reliability
    };
    const c = new AgentState(cfg);
    c.pos = this.pos; c.lapsCompleted = this.lapsCompleted; c.speed = this.speed;
    c.status = this.status; c.pitTimeLeft = this.pitTimeLeft; c.totalTime = this.totalTime;
    c.currentLapStartTime = this.currentLapStartTime; c.lastLapTime = this.lastLapTime;
    c.bestLapTime = this.bestLapTime; c.penalties = this.penalties;
    return c;
  }
}

// ---------------- Controller ----------------
const simpleController = (agent, worldTime) => {
  if (agent.status !== 'running') return [0, false];

  const energyFrac = agent.energy / agent.energyCapacity;
  let targetSpeed = agent.maxSpeed * (0.8 + 0.2 * energyFrac);
  targetSpeed *= (1.0 + 0.15 * (agent.aggression - 0.5));
  const wantPit = energyFrac < PIT_THRESHOLD_ENERGY;

  if (agent.speed < targetSpeed) {
    return [Math.min(agent.accel, (targetSpeed - agent.speed) / DT), wantPit];
  } else {
    return [-Math.min(agent.braking, (agent.speed - targetSpeed) / DT), wantPit];
  }
};

// ---------------- Simulator ----------------
class Simulator {
  constructor(agents, trackLength, laps, dt, seed){
    this.agents = agents.map(a => a.clone());
    this.trackLength = trackLength;
    this.lapsToRun = laps;
    this.dt = dt;
    this.t = 0;
    this.seed = seed || Date.now();
    this.rng = this.seededRandom(this.seed);
  }

  seededRandom(seed){
    let s = seed;
    return () => { s = Math.sin(s) * 10000; return s - Math.floor(s); };
  }

  step(){
    const actions = {};
    for (const a of this.agents) {
      const [acc, wantPit] = simpleController(a, this.t);
      actions[a.id] = [acc, wantPit];
    }

    for (const a of this.agents) {
      if (a.status === 'finished' || a.status === 'retired') continue;

      if (a.status === 'pitting') {
        a.pitTimeLeft -= this.dt;
        a.totalTime += this.dt;
        if (a.pitTimeLeft <= 0) {
          a.status = 'running';
          a.energy = a.energyCapacity;
        }
        continue;
      }

      const [acc, wantPit] = actions[a.id];
      if (wantPit && a.status === 'running') {
        a.status = 'pitting';
        a.pitTimeLeft = PIT_STOP_TIME;
        a.totalTime += this.dt;
        continue;
      }

      a.speed = Math.max(0, Math.min(a.maxSpeed, a.speed + acc * this.dt));
      a.pos += a.speed * this.dt;
      a.totalTime += this.dt;

      const drain = (a.speed / Math.max(1e-6, a.maxSpeed)) * (this.dt * 0.005 * a.energyCapacity);
      a.energy = Math.max(0, a.energy - drain);

      if (a.pos >= this.trackLength) {
        a.pos -= this.trackLength;
        a.lapsCompleted += 1;
        const lapTime = this.t + this.dt - a.currentLapStartTime;
        a.lastLapTime = lapTime;
        if (a.bestLapTime === null || lapTime < a.bestLapTime) a.bestLapTime = lapTime;
        a.currentLapStartTime = this.t + this.dt;

        if (this.rng() < MECH_FAILURE_PER_LAP * (1.0 - a.reliability + 0.1)) {
          a.status = 'retired';
          continue;
        }
      }

      if (a.energy <= 0 && a.status === 'running') {
        a.speed = Math.min(a.speed, a.maxSpeed * 0.4);
        if (this.rng() < 0.02) a.status = 'retired';
      }
    }

    this.resolveInteractions();
    this.t += this.dt;

    for (const a of this.agents) {
      if (a.lapsCompleted >= this.lapsToRun && a.status === 'running') a.status = 'finished';
    }
  }

  resolveInteractions(){
    const distanceCovered = a => a.lapsCompleted * this.trackLength + a.pos;
    const ordering = [...this.agents].sort((a,b)=> distanceCovered(b)-distanceCovered(a));

    for (let idx=0; idx < ordering.length-1; idx++){
      const leader = ordering[idx];
      const follower = ordering[idx+1];
      if (leader.status !== 'running' || follower.status !== 'running') continue;

      const leaderDist = distanceCovered(leader);
      const followerDist = distanceCovered(follower);
      const gap = leaderDist - followerDist;

      if (gap > 0 && gap <= 30) {
        if (follower.speed > leader.speed * 1.003) {
          let pSuccess = OVERTAKE_BASE_SUCCESS * (0.5 + 0.5 * follower.aggression) * (1.0 - 0.02 * Math.max(0, gap - 3));
          pSuccess = Math.max(0.05, Math.min(0.98, pSuccess));

          if (this.rng() < pSuccess) {
            const extra = Math.min(1.5 * (follower.speed - leader.speed) * this.dt, gap + 0.1);
            follower.pos += extra;
            if (this.rng() < (0.03 * (1.0 + follower.aggression))) {
              leader.penalties += 2.0;
            }
            follower.energy = Math.max(0, follower.energy - 0.002 * follower.energyCapacity);
          } else {
            follower.pos = Math.max(0, follower.pos - 0.5);
            follower.speed = Math.max(0, follower.speed * 0.9);
            if (this.rng() < 0.01 * follower.aggression) {
              follower.penalties += 5.0;
              if (this.rng() < 0.05) follower.status = 'retired';
            }
          }
        }
      }
    }
  }

  getLeaderboard(){
    const distanceCovered = a => a.lapsCompleted * this.trackLength + a.pos;
    const sorted = [...this.agents].sort((a,b)=> distanceCovered(b)-distanceCovered(a));
    const leader = sorted[0];
    return sorted.map((a, idx) => {
      const distGap = distanceCovered(leader) - distanceCovered(a);
      const refSpeed = Math.max(leader.speed, 1);
      const gap = distGap / refSpeed;
      return { position: idx+1, agent: a, gap };
    });
  }
}

// ---------------- Default agents ----------------
function createDefaultAgents(){
  const names = ['Hamilton','Verstappen','Leclerc','Sainz','Norris','Piastri','Russell','Alonso'];
  const colors = ['#00D2BE','#1E41FF','#DC0000','#DC0000','#FF8700','#FF8700','#00D2BE','#006F62'];
  return names.map((name,i) => new AgentState({
    id: `A${i+1}`,
    name,
    color: colors[i],
    maxSpeed: mps(220 + (i % 4) * 8 + i * 2),
    accel: 4.5 + (i % 3) * 0.5,
    braking: 6.0 + (i % 2) * 0.8,
    energyCapacity: 100,
    energy: 100 * (0.8 + 0.2 * (1 - i/7)),
    aggression: Math.max(0.25, Math.min(0.95, 0.4 + 0.6 * ((i % 5) / 4))),
    reliability: 0.995 - (i % 6) * 0.0005
  }));
}

// ---------------- UI + App logic ----------------
const startBtn = document.getElementById('startBtn');
const pauseBtn = document.getElementById('pauseBtn');
const resetBtn = document.getElementById('resetBtn');
const lapsInput = document.getElementById('lapsInput');
const timeDisplay = document.getElementById('timeDisplay');
const leaderboardBody = document.getElementById('leaderboardBody');
const statusBanner = document.getElementById('statusBanner');
const footerText = document.getElementById('footerText');

let agents = createDefaultAgents();
let laps = parseInt(lapsInput.value,10) || 10;
let simulator = null;
let animRef = null;
let isRunning = false;

function formatTime(seconds){
  const mins = Math.floor(seconds / 60);
  const secs = (seconds % 60).toFixed(1);
  const padded = secs.toString().padStart(4,'0');
  return `${mins}:${padded}`;
}

function renderLeaderboard(leaderboard){
  leaderboardBody.innerHTML = '';
  leaderboard.forEach(({position, agent, gap}) => {
    const tr = document.createElement('tr');

    // Pos
    const posTd = document.createElement('td');
    posTd.innerHTML = position === 1 ? '🥇' : (position === 2 ? '🥈' : (position === 3 ? '🥉' : position));
    posTd.style.fontWeight = '700';
    tr.appendChild(posTd);

    // Driver
    const driverTd = document.createElement('td');
    const driverDiv = document.createElement('div');
    driverDiv.className = 'driver-cell';
    const dot = document.createElement('div');
    dot.className = 'driver-dot';
    dot.style.backgroundColor = agent.color;
    const nameSpan = document.createElement('span');
    nameSpan.textContent = agent.name;
    nameSpan.style.fontWeight = '600';
    driverDiv.appendChild(dot);
    driverDiv.appendChild(nameSpan);
    driverTd.appendChild(driverDiv);
    tr.appendChild(driverTd);

    // Laps
    const lapsTd = document.createElement('td');
    lapsTd.className = 'center';
    lapsTd.textContent = `${agent.lapsCompleted}/${laps}`;
    tr.appendChild(lapsTd);

    // Status
    const statusTd = document.createElement('td');
    statusTd.className = 'center';
    const statusSpan = document.createElement('span');
    statusSpan.textContent = agent.status.toUpperCase();
    statusSpan.style.padding = '4px 8px';
    statusSpan.style.borderRadius = '6px';
    statusSpan.style.fontSize = '12px';
    statusSpan.style.fontWeight = '700';
    if (agent.status === 'running') {
      statusSpan.style.background = 'linear-gradient(90deg,#16a34a,#059669)';
      statusSpan.style.color = '#02111b';
    } else if (agent.status === 'pitting') {
      statusSpan.style.background = 'linear-gradient(90deg,#f59e0b,#d97706)';
      statusSpan.style.color = '#111827';
    } else if (agent.status === 'finished') {
      statusSpan.style.background = 'linear-gradient(90deg,#3b82f6,#06b6d4)';
      statusSpan.style.color = '#02111b';
    } else {
      statusSpan.style.background = 'linear-gradient(90deg,#ef4444,#dc2626)';
      statusSpan.style.color = '#111827';
    }
    statusTd.appendChild(statusSpan);
    tr.appendChild(statusTd);

    // Speed
    const speedTd = document.createElement('td');
    speedTd.className = 'right';
    speedTd.textContent = `${Math.round(kmh(agent.speed))} km/h`;
    speedTd.style.fontFamily = 'monospace';
    tr.appendChild(speedTd);

    // Energy
    const energyTd = document.createElement('td');
    energyTd.className = 'right';
    const eb = document.createElement('div'); eb.className = 'energy-bar';
    const fill = document.createElement('div'); fill.className = 'energy-fill';
    const pct = Math.round((agent.energy / agent.energyCapacity) * 100);
    fill.style.width = pct + '%';
    eb.appendChild(fill);
    const pctSpan = document.createElement('span'); pctSpan.textContent = `${pct}%`; pctSpan.className = 'small';
    pctSpan.style.marginLeft = '8px';
    const wrapper = document.createElement('div'); wrapper.style.display = 'flex'; wrapper.style.justifyContent='flex-end'; wrapper.style.alignItems='center'; wrapper.appendChild(eb); wrapper.appendChild(pctSpan);
    energyTd.appendChild(wrapper);
    tr.appendChild(energyTd);

    // Best lap
    const bestTd = document.createElement('td');
    bestTd.className = 'right';
    bestTd.style.fontFamily = 'monospace';
    bestTd.textContent = agent.bestLapTime ? formatTime(agent.bestLapTime) : '--';
    tr.appendChild(bestTd);

    // Gap
    const gapTd = document.createElement('td');
    gapTd.className = 'right small';
    gapTd.style.fontFamily = 'monospace';
    gapTd.textContent = position === 1 ? 'Leader' : `+${gap.toFixed(1)}s`;
    tr.appendChild(gapTd);

    leaderboardBody.appendChild(tr);
  });
}

function updateFooter(){
  footerText.textContent = `Simulating ${agents.length} agents • Track: ${(TRACK_LENGTH / 1000).toFixed(1)}km • Physics timestep: ${DT}s`;
}

// ---------------- Controls handlers ----------------
startBtn.addEventListener('click', () => {
  if (isRunning) return;
  laps = Math.max(1, parseInt(lapsInput.value,10) || 1);
  simulator = new Simulator(agents, TRACK_LENGTH, laps, DT, Date.now());
  isRunning = true;
  startBtn.classList.add('hidden');
  pauseBtn.classList.remove('hidden');
  lapsInput.disabled = true;
  statusBanner.classList.add('hidden');
  runLoop();
});

pauseBtn.addEventListener('click', () => {
  isRunning = false;
  startBtn.classList.remove('hidden');
  pauseBtn.classList.add('hidden');
  lapsInput.disabled = false;
  if (animRef) cancelAnimationFrame(animRef);
});

resetBtn.addEventListener('click', () => {
  isRunning = false;
  if (animRef) cancelAnimationFrame(animRef);
  agents = createDefaultAgents();
  simulator = null;
  startBtn.classList.remove('hidden');
  pauseBtn.classList.add('hidden');
  lapsInput.disabled = false;
  statusBanner.classList.add('hidden');
  renderLeaderboard((new Simulator(agents, TRACK_LENGTH, parseInt(lapsInput.value,10)||10, DT, Date.now())).getLeaderboard());
  timeDisplay.textContent = '0:0.0';
  updateFooter();
});

// initialize view
renderLeaderboard((new Simulator(agents, TRACK_LENGTH, parseInt(lapsInput.value,10)||10, DT, Date.now())).getLeaderboard());
updateFooter();
timeDisplay.textContent = '0:0.0';

// ---------------- Simulation loop ----------------
function runLoop(){
  if (!simulator) return;
  const simulateFrame = () => {
    if (!isRunning) return;
    // run multiple steps per frame for speed
    for (let i = 0; i < 5; i++){
      simulator.step();
      const allDone = simulator.agents.every(a => a.status === 'finished' || a.status === 'retired');
      if (allDone){
        isRunning = false;
        startBtn.classList.remove('hidden');
        pauseBtn.classList.add('hidden');
        lapsInput.disabled = false;
        statusBanner.classList.remove('hidden');
        renderLeaderboard(simulator.getLeaderboard());
        timeDisplay.textContent = formatTime(simulator.t);
        return;
      }
    }

    renderLeaderboard(simulator.getLeaderboard());
    timeDisplay.textContent = formatTime(simulator.t);
    animRef = requestAnimationFrame(simulateFrame);
  };

  animRef = requestAnimationFrame(simulateFrame);
}
