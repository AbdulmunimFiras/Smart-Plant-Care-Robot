import React, { useState, useMemo, useEffect, useRef } from 'react';
import { Droplet, Sun, Thermometer, Wind, Leaf, Activity, Cpu, Play, Pause, Globe, BarChart3, GitCompare, History, Sparkles, Terminal, Clock, ShieldCheck, AlertTriangle, AlertOctagon, BookOpen } from 'lucide-react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, Legend, CartesianGrid } from 'recharts';

// ============ FUZZY HELPERS (exact Arduino match) ============
const triangle = (x, a, b, c) => {
  if (x <= a || x >= c) return 0;
  if (x === b) return 1;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
};
const trapezoid = (x, a, b, c, d) => {
  if (x <= a || x >= d) return 0;
  if (x >= b && x <= c) return 1;
  if (x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
};

const fuzzyServoAngle = (light) => {
  const l_low = trapezoid(light, 0, 0, 20, 40);
  const l_med = triangle(light, 30, 50, 70);
  const l_high = trapezoid(light, 60, 80, 100, 100);
  const num = l_low * 0 + l_med * 45 + l_high * 90;
  const den = l_low + l_med + l_high;
  return { angle: den === 0 ? 0 : Math.round(num / den), l_low, l_med, l_high };
};

const fuzzyPumpDuration = (moisture, temp, airHumidity) => {
  const m_dry = trapezoid(moisture, 0, 0, 20, 40);
  const m_normal = triangle(moisture, 30, 50, 70);
  const m_wet = trapezoid(moisture, 60, 80, 100, 100);
  const t_cold = trapezoid(temp, 0, 0, 18, 25);
  const t_warm = triangle(temp, 22, 28, 34);
  const t_hot = trapezoid(temp, 30, 35, 50, 50);
  const off = m_wet;
  const shortP = Math.min(m_normal, t_cold);
  const medP = Math.max(Math.min(m_dry, t_cold), Math.min(m_normal, t_warm));
  const longP = Math.max(Math.min(m_dry, t_warm), Math.min(m_dry, t_hot));
  const num = off * 0 + shortP * 1500 + medP * 3000 + longP * 5000;
  const den = off + shortP + medP + longP;
  if (den === 0) return { time: 0, memberships: { m_dry, m_normal, m_wet, t_cold, t_warm, t_hot }, rules: { off, shortP, medP, longP }, hFactor: 1 };
  let baseTime = num / den;
  const h_dry = trapezoid(airHumidity, 0, 0, 30, 50);
  const h_normal = triangle(airHumidity, 40, 60, 80);
  const h_humid = trapezoid(airHumidity, 70, 90, 100, 100);
  const hFactor = (h_dry * 1.3 + h_normal * 1.0 + h_humid * 0.7) / (h_dry + h_normal + h_humid + 0.001);
  return {
    time: Math.round(baseTime * hFactor),
    memberships: { m_dry, m_normal, m_wet, t_cold, t_warm, t_hot, h_dry, h_normal, h_humid },
    rules: { off, shortP, medP, longP },
    hFactor
  };
};

const classicServoAngle = (light) => light >= 70 ? 90 : light >= 30 ? 45 : 0;
const classicPumpDuration = (moisture) => moisture < 40 ? 3000 : 0;

// ============ TRANSLATIONS ============
const t = {
  en: {
    title: "Smart Plant Care", subtitle: "Fuzzy Logic Intelligence",
    inputs: "Sensor Inputs", moisture: "Soil Moisture", light: "Light Level", temp: "Temperature", humidity: "Air Humidity",
    scenarios: "Test Scenarios", desert: "Desert · dry, hot, bright", rainy: "Rainy · dry soil, humid air",
    optimal: "Optimal · well-watered", heatwave: "Heatwave · extreme stress",
    liveView: "Live Environment", lcd: "LCD Display 16×2", aiOutput: "Inference Output",
    servoAngle: "Servo Angle", pumpDuration: "Pump Duration", humidityFactor: "Humidity factor",
    membership: "Membership μ(x)", rules: "Rule Strength", fuzzyRules: "Active Fuzzy Rules",
    effective: "Effective Light", status: "Pump Status", active: "ACTIVE", idle: "IDLE",
    triggerPump: "Trigger Pump", watering: "Watering…", noWater: "No watering needed",
    autoDemo: "Run Demo", stopDemo: "Stop Demo",
    membershipTab: "Membership", compareTab: "Fuzzy vs Classic", historyTab: "Telemetry",
    classic: "Classic (if-else)", fuzzy: "Fuzzy AI", waterSaved: "Water Saved",
    dry: "dry", normal: "normal", wet: "wet", cold: "cold", warm: "warm", hot: "hot",
    low: "low", med: "med", high: "high", humid: "humid",
    plantHealth: "Plant Vitality", excellent: "Thriving", good: "Healthy", warning: "Stressed", critical: "Critical",
    serial: "Serial Monitor", uptime: "Uptime",
    safe: "Stable", warn: "Watch", danger: "Critical",
    rOff: "Soil is wet → no watering needed",
    rShort: "Soil okay + cold air → short watering",
    rMed: "Dry soil + cold OR normal soil + warm → medium watering",
    rLong: "Dry soil + warm/hot air → long watering",
    rServoLow: "Low light → no shading needed (0°)",
    rServoMed: "Medium light → partial shading (45°)",
    rServoHigh: "High light → full shading (90°)",
    designedBy: "Fuzzy Inference · Arduino Uno · Polarized Shading System"
  },
  ar: {
    title: "نظام رعاية النبات الذكي", subtitle: "ذكاء المنطق الفازي",
    inputs: "مدخلات الحساسات", moisture: "رطوبة التربة", light: "مستوى الضوء", temp: "درجة الحرارة", humidity: "رطوبة الجو",
    scenarios: "سيناريوهات الاختبار", desert: "صحراء · جاف، حار، مشرق", rainy: "ممطر · تربة جافة وجو رطب",
    optimal: "مثالي · نبتة مروية جيداً", heatwave: "موجة حر · إجهاد شديد",
    liveView: "العرض المباشر", lcd: "شاشة LCD 16×2", aiOutput: "مخرجات الاستدلال",
    servoAngle: "زاوية السيرفو", pumpDuration: "مدة المضخة", humidityFactor: "معامل الرطوبة",
    membership: "دالة الانتماء μ(x)", rules: "قوة القواعد", fuzzyRules: "القواعد الفازية النشطة",
    effective: "الضوء الفعّال", status: "حالة المضخة", active: "نشطة", idle: "خاملة",
    triggerPump: "تشغيل المضخة", watering: "جاري الري…", noWater: "لا حاجة للري",
    autoDemo: "تشغيل العرض", stopDemo: "إيقاف العرض",
    membershipTab: "الانتماء", compareTab: "فازي مقابل تقليدي", historyTab: "السجل المباشر",
    classic: "تقليدي (if-else)", fuzzy: "فازي AI", waterSaved: "ماء موفّر",
    dry: "جاف", normal: "متوسط", wet: "رطب", cold: "بارد", warm: "دافئ", hot: "حار",
    low: "منخفض", med: "متوسط", high: "مرتفع", humid: "رطب",
    plantHealth: "حيوية النبتة", excellent: "مزدهرة", good: "بصحة جيدة", warning: "مُجهَدة", critical: "حرجة",
    serial: "شاشة Serial Monitor", uptime: "وقت التشغيل",
    safe: "مستقر", warn: "تنبّه", danger: "حرج",
    rOff: "التربة رطبة → لا حاجة للري",
    rShort: "تربة عادية + جو بارد → ري قصير",
    rMed: "تربة جافة + بارد أو عادية + دافئ → ري متوسط",
    rLong: "تربة جافة + جو دافئ/حار → ري طويل",
    rServoLow: "ضوء منخفض → لا تظليل (0°)",
    rServoMed: "ضوء متوسط → تظليل جزئي (45°)",
    rServoHigh: "ضوء قوي → تظليل كامل (90°)",
    designedBy: "استدلال فازي · أردوينو أونو · نظام تظليل بالأوراق المستقطبة"
  }
};

// ============ SLIDER ============
const Slider = ({ icon: Icon, label, value, onChange, min = 0, max = 100, unit = "%", color, accent }) => {
  const pct = ((value - min) / (max - min)) * 100;
  return (
    <div className="space-y-2.5 group">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2.5">
          <div className="w-7 h-7 rounded-md flex items-center justify-center transition"
            style={{ background: `${color}15`, border: `1px solid ${color}30` }}>
            <Icon size={13} style={{ color }} />
          </div>
          <span className="text-[13px] tracking-wide text-stone-300">{label}</span>
        </div>
        <div className="flex items-baseline gap-1">
          <span className="font-mono text-base font-semibold tabular-nums" style={{ color }}>{value}</span>
          <span className="text-[10px] text-stone-500">{unit}</span>
        </div>
      </div>
      <div className="relative h-1.5">
        <div className="absolute inset-0 rounded-full bg-stone-800/60" />
        <div className="absolute inset-y-0 left-0 rounded-full transition-all duration-300"
          style={{
            width: `${pct}%`,
            background: `linear-gradient(90deg, ${color}40, ${color})`,
            boxShadow: `0 0 12px ${color}50`
          }} />
        <input type="range" min={min} max={max} value={value} onChange={(e) => onChange(Number(e.target.value))}
          className="absolute inset-0 w-full h-full opacity-0 cursor-pointer" />
        <div className="absolute top-1/2 -translate-y-1/2 w-3.5 h-3.5 rounded-full pointer-events-none transition-all duration-200"
          style={{
            left: `calc(${pct}% - 7px)`,
            background: '#fafaf9',
            boxShadow: `0 0 0 2px ${color}, 0 0 16px ${color}80`
          }} />
      </div>
    </div>
  );
};

// ============ MEMBERSHIP BAR ============
const MembershipBar = ({ label, value, color }) => (
  <div className="flex items-center gap-2.5 text-[11px]">
    <span className="w-14 text-stone-500 lowercase tracking-wide">{label}</span>
    <div className="flex-1 h-1.5 bg-stone-800/70 rounded-full overflow-hidden">
      <div className="h-full transition-all duration-500 rounded-full"
        style={{
          width: `${value * 100}%`,
          background: `linear-gradient(90deg, ${color}80, ${color})`,
          boxShadow: value > 0.1 ? `0 0 8px ${color}80` : 'none'
        }} />
    </div>
    <span className="w-9 text-end font-mono tabular-nums text-stone-400">{value.toFixed(2)}</span>
  </div>
);

// ============ MEMBERSHIP GRAPH ============
const MembershipGraph = ({ title, currentValue, sets, max = 100, unit = "%" }) => {
  const width = 340;
  const height = 140;
  const padding = { top: 18, right: 18, bottom: 28, left: 32 };
  const plotW = width - padding.left - padding.right;
  const plotH = height - padding.top - padding.bottom;
  const xScale = (x) => padding.left + (x / max) * plotW;
  const yScale = (y) => padding.top + (1 - y) * plotH;
  const generatePath = (fn) => {
    const points = [];
    for (let x = 0; x <= max; x += max / 100) {
      points.push(`${xScale(x)},${yScale(fn(x))}`);
    }
    return `M${xScale(0)},${yScale(0)} L${points.join(' L')} L${xScale(max)},${yScale(0)} Z`;
  };
  return (
    <div className="relative bg-stone-900/40 backdrop-blur-sm border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-br from-stone-50/[0.02] to-transparent pointer-events-none" />
      <div className="relative flex items-center justify-between mb-3">
        <span className="text-[11px] text-stone-400 uppercase tracking-[0.2em] font-medium">{title}</span>
        <span className="text-[11px] font-mono font-semibold text-stone-200 tabular-nums">x = {currentValue}{unit}</span>
      </div>
      <svg viewBox={`0 0 ${width} ${height}`} className="w-full relative" style={{ direction: 'ltr' }}>
        {[0, 0.25, 0.5, 0.75, 1].map(y => (
          <line key={y} x1={padding.left} y1={yScale(y)} x2={width - padding.right} y2={yScale(y)} stroke="#1c1917" strokeWidth="0.5" />
        ))}
        {[0, 0.5, 1].map(y => (
          <text key={y} x={padding.left - 6} y={yScale(y) + 3} textAnchor="end" fontSize="9" fill="#57534e" fontFamily="JetBrains Mono">{y}</text>
        ))}
        {[0, max / 2, max].map(x => (
          <text key={x} x={xScale(x)} y={height - 8} textAnchor="middle" fontSize="9" fill="#57534e" fontFamily="JetBrains Mono">{x}{unit}</text>
        ))}
        <line x1={padding.left} y1={yScale(0)} x2={width - padding.right} y2={yScale(0)} stroke="#44403c" strokeWidth="1" />
        <line x1={padding.left} y1={padding.top} x2={padding.left} y2={yScale(0)} stroke="#44403c" strokeWidth="1" />
        {sets.map((s, i) => (
          <path key={i} d={generatePath(s.fn)} fill={s.color} fillOpacity="0.18" stroke={s.color} strokeWidth="1.5" />
        ))}
        <line x1={xScale(currentValue)} y1={padding.top} x2={xScale(currentValue)} y2={yScale(0)}
          stroke="#fbbf24" strokeWidth="1.5" strokeDasharray="3,3" opacity="0.8" />
        <circle cx={xScale(currentValue)} cy={yScale(0)} r="3.5" fill="#fbbf24" />
        {sets.map((s, i) => {
          const v = s.fn(currentValue);
          if (v < 0.05) return null;
          return <circle key={i} cx={xScale(currentValue)} cy={yScale(v)} r="3.5" fill={s.color} stroke="#0c0a09" strokeWidth="1.5" />;
        })}
        <g transform={`translate(${padding.left + 8}, ${padding.top + 4})`}>
          {sets.map((s, i) => (
            <g key={i} transform={`translate(${i * 64}, 0)`}>
              <rect width="8" height="8" fill={s.color} fillOpacity="0.4" stroke={s.color} rx="1" />
              <text x="12" y="7" fontSize="9" fill="#a8a29e" fontFamily="JetBrains Mono">{s.label}</text>
            </g>
          ))}
        </g>
      </svg>
    </div>
  );
};

// ============ MAIN APP ============
export default function PlantCareSimulator() {
  const [moisture, setMoisture] = useState(35);
  const [light, setLight] = useState(75);
  const [temp, setTemp] = useState(30);
  const [airHumidity, setAirHumidity] = useState(40);
  const [pumpActive, setPumpActive] = useState(false);
  const [lcdScreen, setLcdScreen] = useState(0);
  const [lang, setLang] = useState('ar');
  const [activeTab, setActiveTab] = useState('membership');
  const [autoDemo, setAutoDemo] = useState(false);
  const [history, setHistory] = useState([]);
  const [waterSaved, setWaterSaved] = useState(0);
  const [serialLog, setSerialLog] = useState([]);
  const [uptime, setUptime] = useState(0);
  const serialRef = useRef(null);

  const L = t[lang];
  const isRTL = lang === 'ar';

  const servoData = useMemo(() => fuzzyServoAngle(light), [light]);
  const servoAngle = servoData.angle;
  const pumpResult = useMemo(() => fuzzyPumpDuration(moisture, temp, airHumidity), [moisture, temp, airHumidity]);
  const pumpTime = pumpResult.time;
  const m = pumpResult.memberships || {};
  const classicServo = classicServoAngle(light);
  const classicPump = classicPumpDuration(moisture);

  useEffect(() => {
    const diff = classicPump - pumpTime;
    if (diff > 0) setWaterSaved(prev => prev + diff / 1000);
  }, [moisture, temp, airHumidity]);

  useEffect(() => {
    const id = setInterval(() => setLcdScreen(s => (s + 1) % 2), 2000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
    const id = setInterval(() => setUptime(u => u + 1), 1000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
    const id = setInterval(() => {
      setHistory(prev => [...prev, {
        t: prev.length, servo: servoAngle, pump: pumpTime / 100,
        moisture, light, temp, humidity: airHumidity
      }].slice(-30));
    }, 1500);
    return () => clearInterval(id);
  }, [servoAngle, pumpTime, moisture, light, temp, airHumidity]);

  useEffect(() => {
    const emit = () => {
      const ts = new Date().toLocaleTimeString('en-US', { hour12: false });
      const lines = [
        { ts, type: 'data', text: `Moisture: ${moisture}%` },
        { ts, type: 'data', text: `Light: ${light}%` },
        { ts, type: 'data', text: `Temp: ${temp.toFixed(1)} C` },
        { ts, type: 'data', text: `Humid: ${airHumidity}%` },
        { ts, type: 'output', text: `>> Fuzzy Servo Angle: ${servoAngle} deg` },
        { ts, type: 'output', text: `>> Fuzzy Pump Time: ${pumpTime} ms` },
      ];
      if (pumpTime > 500) {
        lines.push({ ts, type: 'pump', text: 'Pump: ON' });
        lines.push({ ts, type: 'pump', text: 'Pump: OFF' });
      }
      lines.push({ ts, type: 'sep', text: '─────────────────' });
      setSerialLog(prev => [...prev, ...lines].slice(-60));
    };
    emit();
    const id = setInterval(emit, 2500);
    return () => clearInterval(id);
  }, [moisture, light, temp, airHumidity, servoAngle, pumpTime]);

  useEffect(() => {
    if (serialRef.current) serialRef.current.scrollTop = serialRef.current.scrollHeight;
  }, [serialLog]);

  useEffect(() => {
    if (!autoDemo) return;
    const scenarios = [
      { moisture: 15, light: 85, temp: 35, airHumidity: 25 },
      { moisture: 20, light: 40, temp: 15, airHumidity: 80 },
      { moisture: 75, light: 60, temp: 25, airHumidity: 55 },
      { moisture: 30, light: 95, temp: 40, airHumidity: 20 },
      { moisture: 50, light: 50, temp: 28, airHumidity: 60 }
    ];
    let idx = 0;
    const apply = () => {
      const s = scenarios[idx % scenarios.length];
      const startVals = { moisture, light, temp, airHumidity };
      const steps = 20;
      let step = 0;
      const interp = setInterval(() => {
        step++;
        const k = step / steps;
        setMoisture(Math.round(startVals.moisture + (s.moisture - startVals.moisture) * k));
        setLight(Math.round(startVals.light + (s.light - startVals.light) * k));
        setTemp(Math.round(startVals.temp + (s.temp - startVals.temp) * k));
        setAirHumidity(Math.round(startVals.airHumidity + (s.airHumidity - startVals.airHumidity) * k));
        if (step >= steps) clearInterval(interp);
      }, 40);
      idx++;
    };
    apply();
    const id = setInterval(apply, 4000);
    return () => clearInterval(id);
  }, [autoDemo]);

  const triggerPump = () => {
    if (pumpTime < 500) return;
    setPumpActive(true);
    setTimeout(() => setPumpActive(false), Math.min(pumpTime, 5000));
  };

  const effectiveLight = light * (1 - servoAngle / 90);
  const sunIntensity = light / 100;

  const plantHealthScore = (() => {
    let s = 100;
    if (moisture < 25) s -= 30;
    else if (moisture > 85) s -= 15;
    if (temp > 38 || temp < 10) s -= 25;
    if (effectiveLight > 60) s -= 15;
    if (airHumidity < 25) s -= 10;
    return Math.max(0, s);
  })();
  const healthLabel = plantHealthScore >= 80 ? L.excellent : plantHealthScore >= 60 ? L.good : plantHealthScore >= 35 ? L.warning : L.critical;
  const healthColor = plantHealthScore >= 80 ? '#34d399' : plantHealthScore >= 60 ? '#a3e635' : plantHealthScore >= 35 ? '#fbbf24' : '#f87171';

  const statusLevel = plantHealthScore >= 70 ? 'safe' : plantHealthScore >= 35 ? 'warn' : 'danger';

  const fmtUptime = (s) => {
    const m = Math.floor(s / 60), sec = s % 60;
    return `${String(m).padStart(2, '0')}:${String(sec).padStart(2, '0')}`;
  };

  // Sky gradient based on light - more cinematic
  const skyGradient = `linear-gradient(180deg,
    rgba(${20 + sunIntensity * 80}, ${30 + sunIntensity * 100}, ${50 + sunIntensity * 80}, 1) 0%,
    rgba(${10 + sunIntensity * 30}, ${15 + sunIntensity * 40}, ${25 + sunIntensity * 35}, 1) 60%,
    rgba(8, 8, 8, 1) 100%)`;

  const moistureGraph = [
    { label: L.dry, color: '#d97706', fn: x => trapezoid(x, 0, 0, 20, 40) },
    { label: L.normal, color: '#34d399', fn: x => triangle(x, 30, 50, 70) },
    { label: L.wet, color: '#60a5fa', fn: x => trapezoid(x, 60, 80, 100, 100) }
  ];
  const tempGraph = [
    { label: L.cold, color: '#22d3ee', fn: x => trapezoid(x, 0, 0, 18, 25) },
    { label: L.warm, color: '#fbbf24', fn: x => triangle(x, 22, 28, 34) },
    { label: L.hot, color: '#f87171', fn: x => trapezoid(x, 30, 35, 50, 50) }
  ];
  const lightGraph = [
    { label: L.low, color: '#78716c', fn: x => trapezoid(x, 0, 0, 20, 40) },
    { label: L.med, color: '#fde047', fn: x => triangle(x, 30, 50, 70) },
    { label: L.high, color: '#fbbf24', fn: x => trapezoid(x, 60, 80, 100, 100) }
  ];
  const humidityGraph = [
    { label: L.dry, color: '#fbbf24', fn: x => trapezoid(x, 0, 0, 30, 50) },
    { label: L.normal, color: '#34d399', fn: x => triangle(x, 40, 60, 80) },
    { label: L.humid, color: '#22d3ee', fn: x => trapezoid(x, 70, 90, 100, 100) }
  ];

  const activeRules = [
    { key: 'rOff', strength: pumpResult.rules?.off || 0, color: '#78716c' },
    { key: 'rShort', strength: pumpResult.rules?.shortP || 0, color: '#a3e635' },
    { key: 'rMed', strength: pumpResult.rules?.medP || 0, color: '#fbbf24' },
    { key: 'rLong', strength: pumpResult.rules?.longP || 0, color: '#f87171' }
  ];
  const activeServoRules = [
    { key: 'rServoLow', strength: servoData.l_low, color: '#78716c' },
    { key: 'rServoMed', strength: servoData.l_med, color: '#fde047' },
    { key: 'rServoHigh', strength: servoData.l_high, color: '#fbbf24' }
  ];

  const fontStack = isRTL
    ? "'Tajawal', system-ui, sans-serif"
    : "'Inter Tight', system-ui, sans-serif";

  const displayFont = isRTL
    ? "'Tajawal', system-ui, sans-serif"
    : "'Fraunces', 'Cormorant Garamond', Georgia, serif";

  return (
    <>
      <link href="https://fonts.googleapis.com/css2?family=Tajawal:wght@300;400;500;700;900&family=Fraunces:ital,opsz,wght@0,9..144,300..900;1,9..144,300..900&family=Inter+Tight:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500;600&display=swap" rel="stylesheet" />

      <div className="min-h-screen text-stone-100 relative overflow-hidden"
        style={{
          fontFamily: fontStack,
          direction: isRTL ? 'rtl' : 'ltr',
          background: '#080807'
        }}>

        {/* AMBIENT BACKGROUND - aurora mesh */}
        <div className="fixed inset-0 pointer-events-none opacity-60">
          <div className="absolute top-0 left-1/4 w-[600px] h-[600px] rounded-full"
            style={{ background: 'radial-gradient(circle, rgba(52,211,153,0.15) 0%, transparent 70%)', filter: 'blur(60px)' }} />
          <div className="absolute top-1/3 right-0 w-[500px] h-[500px] rounded-full"
            style={{ background: 'radial-gradient(circle, rgba(251,191,36,0.08) 0%, transparent 70%)', filter: 'blur(80px)' }} />
          <div className="absolute bottom-0 left-0 w-[700px] h-[500px] rounded-full"
            style={{ background: 'radial-gradient(circle, rgba(34,211,238,0.06) 0%, transparent 70%)', filter: 'blur(80px)' }} />
        </div>

        {/* GRAIN TEXTURE */}
        <div className="fixed inset-0 pointer-events-none opacity-[0.025] mix-blend-overlay"
          style={{
            backgroundImage: `url("data:image/svg+xml,%3Csvg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='n'%3E%3CfeTurbulence baseFrequency='0.9'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23n)'/%3E%3C/svg%3E")`
          }} />

        <div className="relative max-w-[1440px] mx-auto px-5 md:px-8 py-6 md:py-8">

          {/* HEADER */}
          <header className="mb-8 flex items-end justify-between flex-wrap gap-4 pb-6 border-b border-stone-800/60">
            <div className="space-y-2">
              <div className="flex items-center gap-2 text-[10px] uppercase tracking-[0.3em] text-emerald-400/80">
                <span className="w-6 h-px bg-emerald-400/60" />
                <Leaf size={10} />
                <span>{L.subtitle}</span>
              </div>
              <h1 className="leading-none"
                style={{ fontFamily: displayFont, fontWeight: isRTL ? 900 : 400 }}>
                <span className="text-3xl md:text-5xl text-stone-100 tracking-tight">
                  {L.title}
                </span>
                {!isRTL && (
                  <span className="text-3xl md:text-5xl italic text-emerald-400/90 ms-2"
                    style={{ fontFamily: displayFont, fontWeight: 300, fontStyle: 'italic' }}>
                    studio
                  </span>
                )}
              </h1>
              <div className="flex items-center gap-4 text-[11px] text-stone-500 pt-1" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                <span className="flex items-center gap-1.5">
                  <span className="relative flex h-1.5 w-1.5">
                    <span className="absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75 animate-ping" />
                    <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-emerald-500" />
                  </span>
                  <span className="text-emerald-400/90">LIVE</span>
                </span>
                <span className="text-stone-700">·</span>
                <span className="flex items-center gap-1.5">
                  <Clock size={10} />
                  {fmtUptime(uptime)}
                </span>
                <span className="text-stone-700">·</span>
                <span>ARDUINO UNO R3</span>
              </div>
            </div>

            <div className="flex items-center gap-2">
              <button onClick={() => setAutoDemo(!autoDemo)}
                className={`group relative flex items-center gap-2 px-4 py-2.5 rounded-full text-[12px] font-medium transition-all duration-300 ${
                  autoDemo
                    ? 'bg-emerald-500 text-emerald-950 shadow-[0_0_30px_rgba(52,211,153,0.4)]'
                    : 'bg-stone-900/80 text-stone-300 border border-stone-800 hover:border-emerald-700/60 hover:text-emerald-300'
                }`}>
                {autoDemo ? <Pause size={13} /> : <Play size={13} />}
                <span>{autoDemo ? L.stopDemo : L.autoDemo}</span>
              </button>
              <button onClick={() => setLang(lang === 'en' ? 'ar' : 'en')}
                className="flex items-center gap-2 px-4 py-2.5 rounded-full text-[12px] font-medium transition-all duration-300 bg-stone-900/80 text-stone-300 border border-stone-800 hover:border-stone-600 hover:text-stone-100">
                <Globe size={13} />
                <span>{lang === 'en' ? 'العربية' : 'English'}</span>
              </button>
            </div>
          </header>

          {/* STATUS PILLS */}
          <div className="flex gap-2 mb-6 flex-wrap">
            {[
              { id: 'safe', icon: ShieldCheck, label: L.safe, color: '#34d399', glow: 'rgba(52,211,153,0.4)' },
              { id: 'warn', icon: AlertTriangle, label: L.warn, color: '#fbbf24', glow: 'rgba(251,191,36,0.4)' },
              { id: 'danger', icon: AlertOctagon, label: L.danger, color: '#f87171', glow: 'rgba(248,113,113,0.4)' }
            ].map(p => {
              const active = statusLevel === p.id;
              return (
                <div key={p.id}
                  className={`flex items-center gap-2 px-3.5 py-1.5 rounded-full text-[11px] font-medium transition-all duration-500 ${
                    active ? '' : 'opacity-25'
                  }`}
                  style={{
                    background: active ? `${p.color}15` : 'rgba(28,25,23,0.5)',
                    border: `1px solid ${active ? p.color + '60' : '#1c1917'}`,
                    color: active ? p.color : '#57534e',
                    boxShadow: active ? `0 0 24px ${p.glow}` : 'none'
                  }}>
                  <p.icon size={12} />
                  <span className="tracking-wide">{p.label}</span>
                </div>
              );
            })}
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-12 gap-5">

            {/* === LEFT COLUMN === */}
            <div className="lg:col-span-3 space-y-4">

              {/* Sensor Inputs */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-6 overflow-hidden">
                <div className="absolute top-0 inset-x-0 h-px bg-gradient-to-r from-transparent via-emerald-500/40 to-transparent" />
                <div className="absolute inset-0 bg-gradient-to-br from-stone-50/[0.02] to-transparent pointer-events-none" />
                <div className="relative">
                  <div className="flex items-center gap-2 mb-5 text-stone-500 text-[10px] uppercase tracking-[0.25em]">
                    <Activity size={11} />
                    <span>{L.inputs}</span>
                  </div>
                  <div className="space-y-5">
                    <Slider icon={Droplet} label={L.moisture} value={moisture} onChange={setMoisture} color="#60a5fa" />
                    <Slider icon={Sun} label={L.light} value={light} onChange={setLight} color="#fbbf24" />
                    <Slider icon={Thermometer} label={L.temp} value={temp} onChange={setTemp} min={0} max={50} unit="°C" color="#f87171" />
                    <Slider icon={Wind} label={L.humidity} value={airHumidity} onChange={setAirHumidity} color="#22d3ee" />
                  </div>
                </div>
              </div>

              {/* Plant Vitality */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="absolute inset-0 opacity-30 pointer-events-none"
                  style={{ background: `radial-gradient(circle at 30% 0%, ${healthColor}30, transparent 60%)` }} />
                <div className="relative">
                  <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-3">{L.plantHealth}</div>
                  <div className="flex items-baseline justify-between mb-3">
                    <span className="text-2xl tracking-tight" style={{ color: healthColor, fontFamily: displayFont, fontWeight: isRTL ? 700 : 400, fontStyle: isRTL ? 'normal' : 'italic' }}>
                      {healthLabel}
                    </span>
                    <span className="text-xs font-mono text-stone-500 tabular-nums" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                      {plantHealthScore}<span className="text-stone-700">/100</span>
                    </span>
                  </div>
                  <div className="relative h-1 bg-stone-800/60 rounded-full overflow-hidden">
                    <div className="absolute inset-y-0 left-0 rounded-full transition-all duration-700"
                      style={{
                        width: `${plantHealthScore}%`,
                        background: `linear-gradient(90deg, ${healthColor}80, ${healthColor})`,
                        boxShadow: `0 0 12px ${healthColor}80`
                      }} />
                  </div>
                </div>
              </div>

              {/* Water Saved */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-blue-900/40 rounded-2xl p-5 overflow-hidden">
                <div className="absolute inset-0 opacity-50 pointer-events-none"
                  style={{ background: 'radial-gradient(circle at 70% 100%, rgba(59,130,246,0.2), transparent 60%)' }} />
                <div className="relative">
                  <div className="flex items-center gap-2 text-blue-400/90 text-[10px] uppercase tracking-[0.25em] mb-2">
                    <Sparkles size={11} />
                    <span>{L.waterSaved}</span>
                  </div>
                  <div className="flex items-baseline gap-1.5">
                    <span className="text-4xl tracking-tight text-blue-300 tabular-nums"
                      style={{ fontFamily: displayFont, fontWeight: 300 }}>
                      {waterSaved.toFixed(1)}
                    </span>
                    <span className="text-xs text-blue-500/60 uppercase tracking-wider" style={{ fontFamily: "'JetBrains Mono', monospace" }}>sec</span>
                  </div>
                  <div className="text-[10px] text-stone-500 mt-1.5 tracking-wide">vs {L.classic}</div>
                </div>
              </div>

              {/* Test Scenarios */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-3">{L.scenarios}</div>
                <div className="space-y-1.5">
                  {[
                    { label: L.desert, vals: [15, 85, 35, 25], hue: '#fbbf24' },
                    { label: L.rainy, vals: [20, 40, 15, 80], hue: '#22d3ee' },
                    { label: L.optimal, vals: [75, 60, 25, 55], hue: '#34d399' },
                    { label: L.heatwave, vals: [30, 95, 40, 20], hue: '#f87171' }
                  ].map((s, i) => (
                    <button key={i} onClick={() => {
                      setMoisture(s.vals[0]); setLight(s.vals[1]); setTemp(s.vals[2]); setAirHumidity(s.vals[3]);
                    }}
                      className="group w-full text-start text-[12px] px-3 py-2.5 rounded-lg transition-all duration-300 border border-stone-800/60 hover:border-stone-600 bg-stone-900/30 hover:bg-stone-800/50 flex items-center gap-2.5">
                      <span className="w-1 h-1 rounded-full transition-all duration-300 group-hover:w-3"
                        style={{ background: s.hue, boxShadow: `0 0 6px ${s.hue}` }} />
                      <span className="text-stone-300 group-hover:text-stone-100">{s.label}</span>
                    </button>
                  ))}
                </div>
              </div>
            </div>

            {/* === CENTER COLUMN === */}
            <div className="lg:col-span-6 space-y-5">

              {/* Live Visualization */}
              <div className="relative rounded-2xl overflow-hidden border border-stone-800/80">
                <div className="absolute top-4 left-5 z-10 flex items-center gap-2" style={{ direction: isRTL ? 'rtl' : 'ltr' }}>
                  <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse" />
                  <span className="text-[10px] text-stone-300 uppercase tracking-[0.25em] font-medium">
                    {L.liveView}
                  </span>
                </div>
                <div className="absolute top-4 right-5 z-10 flex items-center gap-1.5 bg-black/40 backdrop-blur-md border border-stone-700/50 rounded-full px-2.5 py-1 text-[10px] text-emerald-300/90"
                  style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                  <span className="w-1 h-1 rounded-full bg-emerald-400" />
                  <span>UNO R3</span>
                </div>

                <div style={{ background: skyGradient, transition: 'background 1.2s ease' }}>
                  <svg viewBox="0 0 400 340" className="w-full h-80">
                    <defs>
                      <radialGradient id="sunGrad">
                        <stop offset="0%" stopColor="#fef3c7" stopOpacity={sunIntensity} />
                        <stop offset="50%" stopColor="#fde047" stopOpacity={sunIntensity * 0.9} />
                        <stop offset="100%" stopColor="#f59e0b" stopOpacity={sunIntensity * 0.6} />
                      </radialGradient>
                      <linearGradient id="sheetGrad" x1="0%" y1="0%" x2="100%" y2="100%">
                        <stop offset="0%" stopColor="#1e293b" stopOpacity="0.9" />
                        <stop offset="100%" stopColor="#0f172a" stopOpacity="0.95" />
                      </linearGradient>
                      <linearGradient id="lightBeam" x1="0%" y1="0%" x2="0%" y2="100%">
                        <stop offset="0%" stopColor="#fde047" stopOpacity={effectiveLight / 200} />
                        <stop offset="100%" stopColor="#fde047" stopOpacity="0" />
                      </linearGradient>
                      <radialGradient id="sunHalo">
                        <stop offset="0%" stopColor="#fde047" stopOpacity={sunIntensity * 0.5} />
                        <stop offset="100%" stopColor="#fde047" stopOpacity="0" />
                      </radialGradient>
                      <radialGradient id="potShadow" cx="50%" cy="50%">
                        <stop offset="0%" stopColor="#000" stopOpacity="0.6" />
                        <stop offset="100%" stopColor="#000" stopOpacity="0" />
                      </radialGradient>
                      <linearGradient id="potGrad" x1="0%" y1="0%" x2="0%" y2="100%">
                        <stop offset="0%" stopColor="#a16207" />
                        <stop offset="50%" stopColor="#78350f" />
                        <stop offset="100%" stopColor="#451a03" />
                      </linearGradient>
                    </defs>

                    {/* Stars (only when low light) */}
                    {sunIntensity < 0.4 && [...Array(20)].map((_, i) => {
                      const x = (i * 47) % 380 + 10;
                      const y = (i * 31) % 80 + 10;
                      return <circle key={i} cx={x} cy={y} r="0.5" fill="#fff" opacity={(0.4 - sunIntensity) * 0.7} />;
                    })}

                    {/* Sun halo glow */}
                    <circle cx="60" cy="55" r="90" fill="url(#sunHalo)" />
                    {/* Sun core */}
                    <circle cx="60" cy="55" r={22 + sunIntensity * 4} fill="url(#sunGrad)">
                      <animate attributeName="r" values={`${22 + sunIntensity * 3};${22 + sunIntensity * 5};${22 + sunIntensity * 3}`} dur="3s" repeatCount="indefinite" />
                    </circle>
                    {/* Sun rays */}
                    {sunIntensity > 0.2 && [...Array(12)].map((_, i) => {
                      const angle = (i * 30) * Math.PI / 180;
                      const x1 = 60 + Math.cos(angle) * (28 + sunIntensity * 4);
                      const y1 = 55 + Math.sin(angle) * (28 + sunIntensity * 4);
                      const x2 = 60 + Math.cos(angle) * (40 + sunIntensity * 14);
                      const y2 = 55 + Math.sin(angle) * (40 + sunIntensity * 14);
                      return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2} stroke="#fde047" strokeWidth="1.5" opacity={sunIntensity * 0.9} strokeLinecap="round" />;
                    })}

                    {/* Light beam to plant */}
                    <rect x="160" y="20" width="80" height="100" fill="url(#lightBeam)" />

                    {/* Polarized Sheet 1 (fixed) */}
                    <g transform="translate(200, 130)">
                      <rect x="-50" y="-3" width="100" height="6" fill="url(#sheetGrad)" stroke="#475569" strokeWidth="0.8" rx="2" />
                      {[...Array(20)].map((_, i) => (
                        <line key={i} x1={-48 + i * 5} y1="-3" x2={-48 + i * 5} y2="3" stroke="#64748b" strokeWidth="0.4" opacity="0.6" />
                      ))}
                    </g>

                    {/* Polarized Sheet 2 (rotating) */}
                    <g transform={`translate(200, 110) rotate(${servoAngle})`} style={{ transition: 'transform 0.6s cubic-bezier(0.4, 0, 0.2, 1)' }}>
                      <rect x="-50" y="-3" width="100" height="6" fill="url(#sheetGrad)" stroke="#475569" strokeWidth="0.8" rx="2" />
                      {[...Array(20)].map((_, i) => (
                        <line key={i} x1={-48 + i * 5} y1="-3" x2={-48 + i * 5} y2="3" stroke="#64748b" strokeWidth="0.4" opacity="0.6" />
                      ))}
                    </g>

                    {/* Servo */}
                    <g transform="translate(265, 100)">
                      <rect x="0" y="0" width="35" height="25" fill="#1f2937" stroke="#374151" strokeWidth="1" rx="3" />
                      <circle cx="17" cy="12" r="6" fill="#374151" />
                      <circle cx="17" cy="12" r="2.5" fill="#9ca3af" />
                      <circle cx="17" cy="12" r="1" fill="#fbbf24" opacity={servoAngle > 0 ? 1 : 0.3} />
                      <text x="17" y="40" textAnchor="middle" fill="#94a3b8" fontSize="7" fontFamily="JetBrains Mono">SERVO</text>
                    </g>

                    {/* Pot shadow */}
                    <ellipse cx="200" cy="312" rx="45" ry="6" fill="url(#potShadow)" />

                    {/* Plant pot */}
                    <path d="M 165 270 L 180 310 L 220 310 L 235 270 Z" fill="url(#potGrad)" stroke="#451a03" strokeWidth="1" />
                    <ellipse cx="200" cy="270" rx="35" ry="6" fill={moisture > 60 ? "#1f2937" : moisture > 30 ? "#451a03" : "#78350f"} />
                    {/* Soil moisture indicator */}
                    {moisture > 30 && (
                      <ellipse cx="200" cy="270" rx="32" ry="4" fill={moisture > 60 ? "#0c4a6e" : "#451a03"} opacity={moisture / 100} />
                    )}

                    {/* Plant */}
                    <g style={{ transformOrigin: '200px 270px', animation: 'sway 4s ease-in-out infinite' }}>
                      <path d="M 200 270 Q 200 240 200 200" stroke="#15803d" strokeWidth="2.5" fill="none" />
                      {(() => {
                        const health = moisture < 25 ? '#a16207' : moisture > 80 ? '#15803d' : '#22c55e';
                        const lightStress = effectiveLight > 70 ? 0.7 : 1;
                        return (
                          <>
                            <ellipse cx="180" cy="240" rx="15" ry="8" fill={health} opacity={lightStress} transform="rotate(-30 180 240)" />
                            <ellipse cx="220" cy="230" rx="15" ry="8" fill={health} opacity={lightStress} transform="rotate(30 220 230)" />
                            <ellipse cx="185" cy="215" rx="13" ry="7" fill={health} opacity={lightStress} transform="rotate(-45 185 215)" />
                            <ellipse cx="215" cy="205" rx="13" ry="7" fill={health} opacity={lightStress} transform="rotate(45 215 205)" />
                            <ellipse cx="200" cy="195" rx="12" ry="6" fill={health} opacity={lightStress} />
                          </>
                        );
                      })()}
                    </g>

                    {/* Water drops */}
                    {pumpActive && [...Array(8)].map((_, i) => (
                      <g key={i}>
                        <ellipse cx={195 + (i % 4) * 4} cy="245" rx="1.5" ry="2.5" fill="#60a5fa">
                          <animate attributeName="cy" values="245;272" dur={`${0.7 + (i % 3) * 0.15}s`} repeatCount="indefinite" begin={`${i * 0.08}s`} />
                          <animate attributeName="opacity" values="1;0" dur={`${0.7 + (i % 3) * 0.15}s`} repeatCount="indefinite" begin={`${i * 0.08}s`} />
                        </ellipse>
                      </g>
                    ))}

                    {/* Pump */}
                    <g transform="translate(330, 250)">
                      <rect x="0" y="0" width="40" height="40"
                        fill={pumpActive ? "#1e3a8a" : "#1f2937"}
                        stroke={pumpActive ? "#60a5fa" : "#374151"} strokeWidth="1.2" rx="4">
                        {pumpActive && <animate attributeName="stroke-opacity" values="1;0.4;1" dur="0.6s" repeatCount="indefinite" />}
                      </rect>
                      <circle cx="20" cy="20" r="8" fill="none" stroke={pumpActive ? "#93c5fd" : "#6b7280"} strokeWidth="1.5">
                        {pumpActive && <animateTransform attributeName="transform" type="rotate" from="0 20 20" to="360 20 20" dur="1s" repeatCount="indefinite" />}
                      </circle>
                      <line x1="20" y1="14" x2="20" y2="20" stroke={pumpActive ? "#93c5fd" : "#6b7280"} strokeWidth="1.5" />
                      <text x="20" y="55" textAnchor="middle" fill="#94a3b8" fontSize="7" fontFamily="JetBrains Mono">PUMP</text>
                    </g>

                    {/* Servo angle label */}
                    <g transform="translate(200, 175)">
                      <rect x="-22" y="-9" width="44" height="16" rx="8" fill="#022c22" stroke="#10b981" strokeWidth="0.8" opacity="0.9" />
                      <text x="0" y="2" textAnchor="middle" fill="#34d399" fontSize="9" fontWeight="bold" fontFamily="JetBrains Mono">θ {servoAngle}°</text>
                    </g>
                  </svg>
                  <style>{`@keyframes sway { 0%,100% { transform: rotate(-1deg); } 50% { transform: rotate(1deg); } }`}</style>
                </div>

                <div className="grid grid-cols-3 gap-px bg-stone-800/40 border-t border-stone-800/80">
                  <div className="bg-stone-900/60 backdrop-blur px-4 py-3">
                    <div className="text-[9px] text-stone-500 uppercase tracking-[0.2em] mb-1">{L.effective}</div>
                    <div className="text-amber-300 text-lg tabular-nums" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                      {Math.round(effectiveLight)}<span className="text-xs text-stone-600">%</span>
                    </div>
                  </div>
                  <div className="bg-stone-900/60 backdrop-blur px-4 py-3">
                    <div className="text-[9px] text-stone-500 uppercase tracking-[0.2em] mb-1">{L.servoAngle}</div>
                    <div className="text-emerald-300 text-lg tabular-nums" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                      {servoAngle}<span className="text-xs text-stone-600">°</span>
                    </div>
                  </div>
                  <div className="bg-stone-900/60 backdrop-blur px-4 py-3">
                    <div className="text-[9px] text-stone-500 uppercase tracking-[0.2em] mb-1">{L.status}</div>
                    <div className={`text-lg tabular-nums ${pumpActive ? 'text-blue-300' : 'text-stone-500'}`} style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                      {pumpActive ? L.active : L.idle}
                    </div>
                  </div>
                </div>
              </div>

              {/* LCD Display */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="flex items-center justify-between mb-3">
                  <span className="text-[10px] text-stone-500 uppercase tracking-[0.25em]">{L.lcd}</span>
                  <div className="flex items-center gap-1.5">
                    {[0, 1].map(i => (
                      <span key={i} className="w-1.5 h-1.5 rounded-full transition-all"
                        style={{ background: lcdScreen === i ? '#34d399' : '#44403c' }} />
                    ))}
                  </div>
                </div>
                <div className="bg-emerald-950/80 border border-stone-800 rounded-lg p-4 relative overflow-hidden"
                  style={{ direction: 'ltr', boxShadow: 'inset 0 0 30px rgba(16,185,129,0.15)' }}>
                  <div className="absolute inset-0 pointer-events-none"
                    style={{
                      backgroundImage: 'repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,0,0,0.15) 2px, rgba(0,0,0,0.15) 3px)'
                    }} />
                  <div className="relative font-mono text-emerald-300" style={{ fontFamily: "'Courier New', monospace", textShadow: '0 0 8px rgba(52,211,153,0.6)' }}>
                    {lcdScreen === 0 ? (
                      <>
                        <div className="text-base tracking-wider">M:{moisture}% L:{light}%</div>
                        <div className="text-base tracking-wider">Servo:{servoAngle}°</div>
                      </>
                    ) : (
                      <>
                        <div className="text-base tracking-wider">Temp: {temp.toFixed(1)} C</div>
                        <div className="text-base tracking-wider">Humid:{airHumidity}%</div>
                      </>
                    )}
                  </div>
                </div>
                <button onClick={triggerPump} disabled={pumpActive || pumpTime < 500}
                  className={`mt-3.5 w-full py-2.5 rounded-lg text-[12px] font-medium transition-all duration-300 ${
                    pumpActive
                      ? 'bg-blue-500/15 text-blue-300 border border-blue-700/50 cursor-not-allowed'
                      : pumpTime < 500
                      ? 'bg-stone-900/60 text-stone-600 border border-stone-800/60 cursor-not-allowed'
                      : 'bg-emerald-500/10 text-emerald-300 border border-emerald-700/50 hover:bg-emerald-500/20 hover:border-emerald-500/60 hover:shadow-[0_0_24px_rgba(52,211,153,0.25)]'
                  }`}>
                  {pumpActive
                    ? `${L.watering} (${pumpTime}ms)`
                    : pumpTime < 500
                    ? L.noWater
                    : `${isRTL ? '◀' : '▶'} ${L.triggerPump} · ${pumpTime}ms`}
                </button>
              </div>

              {/* Serial Monitor */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="flex items-center justify-between mb-3">
                  <div className="flex items-center gap-2 text-stone-500 text-[10px] uppercase tracking-[0.25em]">
                    <Terminal size={11} />
                    <span>{L.serial}</span>
                  </div>
                  <div className="flex items-center gap-1.5 text-[10px] text-emerald-400/80" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                    <span className="w-1 h-1 rounded-full bg-emerald-400 animate-pulse" />
                    <span>9600 BAUD</span>
                  </div>
                </div>
                <div ref={serialRef} className="bg-black/60 border border-stone-800/60 rounded-lg p-3 h-44 overflow-y-auto"
                  style={{ fontFamily: "'JetBrains Mono', monospace", fontSize: '10.5px', direction: 'ltr' }}>
                  {serialLog.map((line, i) => (
                    <div key={i} className={`leading-relaxed ${
                      line.type === 'data' ? 'text-emerald-400/90'
                      : line.type === 'output' ? 'text-amber-300 font-medium'
                      : line.type === 'pump' ? 'text-blue-300 font-medium'
                      : 'text-stone-700'
                    }`}>
                      <span className="text-stone-700">[{line.ts}]</span> {line.text}
                    </div>
                  ))}
                </div>
              </div>
            </div>

            {/* === RIGHT COLUMN === */}
            <div className="lg:col-span-3 space-y-4">

              {/* Inference Output */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-6 overflow-hidden">
                <div className="absolute top-0 inset-x-0 h-px bg-gradient-to-r from-transparent via-emerald-500/40 to-transparent" />
                <div className="absolute inset-0 bg-gradient-to-br from-emerald-500/[0.04] to-transparent pointer-events-none" />
                <div className="relative">
                  <div className="flex items-center gap-2 mb-5 text-stone-500 text-[10px] uppercase tracking-[0.25em]">
                    <Cpu size={11} />
                    <span>{L.aiOutput}</span>
                  </div>
                  <div className="space-y-5">
                    <div>
                      <div className="text-[10px] text-stone-500 mb-2 uppercase tracking-wider">{L.servoAngle}</div>
                      <div className="flex items-baseline gap-1.5">
                        <span className="text-5xl tabular-nums tracking-tight text-emerald-300"
                          style={{ fontFamily: displayFont, fontWeight: 300 }}>
                          {servoAngle}
                        </span>
                        <span className="text-xl text-emerald-500/50">°</span>
                      </div>
                    </div>
                    <div className="border-t border-stone-800/80 pt-4">
                      <div className="text-[10px] text-stone-500 mb-2 uppercase tracking-wider">{L.pumpDuration}</div>
                      <div className="flex items-baseline gap-1.5">
                        <span className="text-5xl tabular-nums tracking-tight text-blue-300"
                          style={{ fontFamily: displayFont, fontWeight: 300 }}>
                          {pumpTime}
                        </span>
                        <span className="text-xs text-blue-500/50 uppercase tracking-wider" style={{ fontFamily: "'JetBrains Mono', monospace" }}>ms</span>
                      </div>
                      {pumpResult.hFactor && (
                        <div className="text-[10px] text-stone-500 mt-2" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                          {L.humidityFactor}: ×{pumpResult.hFactor.toFixed(2)}
                        </div>
                      )}
                    </div>
                  </div>
                </div>
              </div>

              {/* Active Fuzzy Rules */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="flex items-center gap-2 text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-3">
                  <BookOpen size={11} />
                  <span>{L.fuzzyRules}</span>
                </div>

                <div className="text-[10px] text-stone-600 mb-2 mt-3 uppercase tracking-wider">{L.pumpDuration}</div>
                <div className="space-y-1">
                  {activeRules.map((r, i) => (
                    <div key={i} className={`flex items-start gap-2 px-2.5 py-2 rounded-lg text-[11px] transition-all ${
                      r.strength > 0.05 ? 'bg-stone-800/40' : 'opacity-25'
                    }`}>
                      <span className="font-mono text-stone-500 w-12 shrink-0 tabular-nums" style={{ direction: 'ltr', fontFamily: "'JetBrains Mono', monospace" }}>
                        μ={r.strength.toFixed(2)}
                      </span>
                      <span className="shrink-0 mt-0.5" style={{ color: r.color }}>●</span>
                      <span className="text-stone-300 leading-snug flex-1">{L[r.key]}</span>
                    </div>
                  ))}
                </div>

                <div className="text-[10px] text-stone-600 mb-2 mt-4 uppercase tracking-wider">{L.servoAngle}</div>
                <div className="space-y-1">
                  {activeServoRules.map((r, i) => (
                    <div key={i} className={`flex items-start gap-2 px-2.5 py-2 rounded-lg text-[11px] transition-all ${
                      r.strength > 0.05 ? 'bg-stone-800/40' : 'opacity-25'
                    }`}>
                      <span className="font-mono text-stone-500 w-12 shrink-0 tabular-nums" style={{ direction: 'ltr', fontFamily: "'JetBrains Mono', monospace" }}>
                        μ={r.strength.toFixed(2)}
                      </span>
                      <span className="shrink-0 mt-0.5" style={{ color: r.color }}>●</span>
                      <span className="text-stone-300 leading-snug flex-1">{L[r.key]}</span>
                    </div>
                  ))}
                </div>
              </div>

              {/* Membership Bars */}
              <div className="relative bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-5 overflow-hidden">
                <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-3">{L.membership}</div>

                <div className="text-[10px] text-stone-600 mt-2 mb-2 uppercase tracking-wider">{L.moisture}</div>
                <div className="space-y-1.5">
                  <MembershipBar label={L.dry} value={m.m_dry || 0} color="#d97706" />
                  <MembershipBar label={L.normal} value={m.m_normal || 0} color="#34d399" />
                  <MembershipBar label={L.wet} value={m.m_wet || 0} color="#60a5fa" />
                </div>
                <div className="text-[10px] text-stone-600 mt-3 mb-2 uppercase tracking-wider">{L.temp}</div>
                <div className="space-y-1.5">
                  <MembershipBar label={L.cold} value={m.t_cold || 0} color="#22d3ee" />
                  <MembershipBar label={L.warm} value={m.t_warm || 0} color="#fbbf24" />
                  <MembershipBar label={L.hot} value={m.t_hot || 0} color="#f87171" />
                </div>
                <div className="text-[10px] text-stone-600 mt-3 mb-2 uppercase tracking-wider">{L.humidity}</div>
                <div className="space-y-1.5">
                  <MembershipBar label={L.dry} value={m.h_dry || 0} color="#fbbf24" />
                  <MembershipBar label={L.normal} value={m.h_normal || 0} color="#34d399" />
                  <MembershipBar label={L.humid} value={m.h_humid || 0} color="#22d3ee" />
                </div>
              </div>
            </div>
          </div>

          {/* === BOTTOM TABS === */}
          <div className="mt-8">
            <div className="flex gap-1 mb-5 border-b border-stone-800/80">
              {[
                { id: 'membership', icon: BarChart3, label: L.membershipTab },
                { id: 'compare', icon: GitCompare, label: L.compareTab },
                { id: 'history', icon: History, label: L.historyTab }
              ].map(tab => (
                <button key={tab.id} onClick={() => setActiveTab(tab.id)}
                  className={`relative flex items-center gap-2 px-5 py-3 text-[12px] font-medium transition-all duration-300 ${
                    activeTab === tab.id ? 'text-emerald-300' : 'text-stone-500 hover:text-stone-300'
                  }`}>
                  <tab.icon size={13} />
                  <span>{tab.label}</span>
                  {activeTab === tab.id && (
                    <span className="absolute bottom-0 left-2 right-2 h-px bg-gradient-to-r from-transparent via-emerald-400 to-transparent" />
                  )}
                </button>
              ))}
            </div>

            {activeTab === 'membership' && (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <MembershipGraph title={L.moisture} currentValue={moisture} sets={moistureGraph} />
                <MembershipGraph title={L.temp} currentValue={temp} sets={tempGraph} max={50} unit="°C" />
                <MembershipGraph title={L.light} currentValue={light} sets={lightGraph} />
                <MembershipGraph title={L.humidity} currentValue={airHumidity} sets={humidityGraph} />
              </div>
            )}

            {activeTab === 'compare' && (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-6">
                  <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-4">{L.servoAngle}</div>
                  <div className="space-y-5">
                    <div>
                      <div className="flex items-baseline justify-between mb-2">
                        <span className="text-stone-400 text-[12px]">{L.classic}</span>
                        <div className="flex items-baseline gap-1">
                          <span className="text-3xl tabular-nums text-stone-300" style={{ fontFamily: displayFont, fontWeight: 300 }}>{classicServo}</span>
                          <span className="text-stone-600">°</span>
                        </div>
                      </div>
                      <div className="h-1 bg-stone-800/60 rounded-full overflow-hidden">
                        <div className="h-full bg-gradient-to-r from-stone-600 to-stone-500 transition-all duration-500" style={{ width: `${(classicServo / 90) * 100}%` }} />
                      </div>
                    </div>
                    <div>
                      <div className="flex items-baseline justify-between mb-2">
                        <span className="text-emerald-300 text-[12px] flex items-center gap-1.5">
                          {L.fuzzy}
                          <Sparkles size={11} className="text-emerald-400" />
                        </span>
                        <div className="flex items-baseline gap-1">
                          <span className="text-3xl tabular-nums text-emerald-300" style={{ fontFamily: displayFont, fontWeight: 300 }}>{servoAngle}</span>
                          <span className="text-emerald-500/50">°</span>
                        </div>
                      </div>
                      <div className="h-1 bg-stone-800/60 rounded-full overflow-hidden">
                        <div className="h-full transition-all duration-500"
                          style={{ width: `${(servoAngle / 90) * 100}%`, background: 'linear-gradient(90deg, #34d39980, #34d399)', boxShadow: '0 0 10px rgba(52,211,153,0.6)' }} />
                      </div>
                    </div>
                  </div>
                </div>
                <div className="bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-6">
                  <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em] mb-4">{L.pumpDuration}</div>
                  <div className="space-y-5">
                    <div>
                      <div className="flex items-baseline justify-between mb-2">
                        <span className="text-stone-400 text-[12px]">{L.classic}</span>
                        <div className="flex items-baseline gap-1">
                          <span className="text-3xl tabular-nums text-stone-300" style={{ fontFamily: displayFont, fontWeight: 300 }}>{classicPump}</span>
                          <span className="text-xs text-stone-600 uppercase">ms</span>
                        </div>
                      </div>
                      <div className="h-1 bg-stone-800/60 rounded-full overflow-hidden">
                        <div className="h-full bg-gradient-to-r from-stone-600 to-stone-500 transition-all duration-500" style={{ width: `${(classicPump / 5000) * 100}%` }} />
                      </div>
                    </div>
                    <div>
                      <div className="flex items-baseline justify-between mb-2">
                        <span className="text-blue-300 text-[12px] flex items-center gap-1.5">
                          {L.fuzzy}
                          <Sparkles size={11} className="text-blue-400" />
                        </span>
                        <div className="flex items-baseline gap-1">
                          <span className="text-3xl tabular-nums text-blue-300" style={{ fontFamily: displayFont, fontWeight: 300 }}>{pumpTime}</span>
                          <span className="text-xs text-blue-500/50 uppercase">ms</span>
                        </div>
                      </div>
                      <div className="h-1 bg-stone-800/60 rounded-full overflow-hidden">
                        <div className="h-full transition-all duration-500"
                          style={{ width: `${(pumpTime / 5000) * 100}%`, background: 'linear-gradient(90deg, #60a5fa80, #60a5fa)', boxShadow: '0 0 10px rgba(96,165,250,0.6)' }} />
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'history' && (
              <div className="bg-stone-900/40 backdrop-blur-md border border-stone-800/80 rounded-2xl p-6">
                <div className="flex items-baseline justify-between mb-4">
                  <div className="text-stone-500 text-[10px] uppercase tracking-[0.25em]">{L.historyTab}</div>
                  <div className="text-[10px] text-stone-600" style={{ fontFamily: "'JetBrains Mono', monospace" }}>
                    {history.length} samples
                  </div>
                </div>
                <div style={{ width: '100%', height: 290, direction: 'ltr' }}>
                  <ResponsiveContainer>
                    <LineChart data={history}>
                      <CartesianGrid stroke="#1c1917" strokeDasharray="3 3" />
                      <XAxis dataKey="t" stroke="#57534e" fontSize={10} fontFamily="JetBrains Mono" />
                      <YAxis stroke="#57534e" fontSize={10} fontFamily="JetBrains Mono" />
                      <Tooltip contentStyle={{ backgroundColor: '#0c0a09', border: '1px solid #292524', borderRadius: 8, fontFamily: 'JetBrains Mono', fontSize: 11 }} />
                      <Legend wrapperStyle={{ fontFamily: 'JetBrains Mono', fontSize: 11 }} />
                      <Line type="monotone" dataKey="servo" stroke="#34d399" strokeWidth={2} dot={false} name="Servo (°)" />
                      <Line type="monotone" dataKey="pump" stroke="#60a5fa" strokeWidth={2} dot={false} name="Pump (×100 ms)" />
                      <Line type="monotone" dataKey="moisture" stroke="#d97706" strokeWidth={1.2} strokeDasharray="3 3" dot={false} name="Moisture (%)" />
                      <Line type="monotone" dataKey="light" stroke="#fbbf24" strokeWidth={1.2} strokeDasharray="3 3" dot={false} name="Light (%)" />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
              </div>
            )}
          </div>

          {/* FOOTER */}
          <footer className="mt-10 pt-6 border-t border-stone-800/60">
            <div className="flex items-center justify-between flex-wrap gap-3 text-[10px] text-stone-600 tracking-[0.2em] uppercase"
              style={{ fontFamily: "'JetBrains Mono', monospace" }}>
              <div className="flex items-center gap-2">
                <Leaf size={10} className="text-emerald-600/60" />
                <span>{L.designedBy}</span>
              </div>
              <div className="flex items-center gap-3">
                <span>v2.0</span>
                <span className="text-stone-800">·</span>
                <span>MAMDANI INFERENCE</span>
              </div>
            </div>
          </footer>
        </div>
      </div>
    </>
  );
}
