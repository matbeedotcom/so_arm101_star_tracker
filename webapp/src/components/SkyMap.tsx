import { useEffect, useMemo, useRef, useState } from "react";
import { computeBodyPositions, type BodyPosition } from "../sky";

export type SkySelection =
  | { kind: "body"; name: string; alt: number; az: number }
  | { kind: "point"; alt: number; az: number };

const VIEWBOX = 400;
const PAD = 28;
const R = (VIEWBOX - PAD * 2) / 2;
const CX = VIEWBOX / 2;
const CY = VIEWBOX / 2;

const KIND_STYLE: Record<BodyPosition["kind"], { fill: string; radius: number }> = {
  sun:    { fill: "#ffd060", radius: 7 },
  moon:   { fill: "#e6ecff", radius: 6 },
  planet: { fill: "#ff9d6b", radius: 4.5 },
  star:   { fill: "#f3f7ff", radius: 3 },
};

function project(alt: number, az: number) {
  const r = Math.max(0, Math.min(1, (90 - alt) / 90));
  const azR = (az * Math.PI) / 180;
  return {
    x: CX + R * r * Math.sin(azR),
    y: CY - R * r * Math.cos(azR),
  };
}

function unproject(svgX: number, svgY: number) {
  const dx = (svgX - CX) / R;
  const dy = -(svgY - CY) / R;
  const r = Math.sqrt(dx * dx + dy * dy);
  if (r > 1) return null;
  const alt = 90 - r * 90;
  let az = (Math.atan2(dx, dy) * 180) / Math.PI;
  if (az < 0) az += 360;
  return { alt, az };
}

export function SkyMap({
  observerLat,
  observerLon,
  stars,
  planets,
  currentAlt,
  currentAz,
  targetAlt,
  targetAz,
  selected,
  onSelect,
}: {
  observerLat: number;
  observerLon: number;
  stars: string[];
  planets: string[];
  currentAlt: number | null;
  currentAz: number | null;
  targetAlt: number | null;
  targetAz: number | null;
  selected: SkySelection | null;
  onSelect: (s: SkySelection | null) => void;
}) {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 20000);
    return () => clearInterval(id);
  }, []);

  const [hover, setHover] = useState<BodyPosition | null>(null);
  const [filter, setFilter] = useState("");
  const svgRef = useRef<SVGSVGElement>(null);

  const allNames = useMemo(() => [...planets, ...stars], [planets, stars]);
  const bodies = useMemo(
    () => computeBodyPositions(allNames, observerLat, observerLon, now),
    [allNames, observerLat, observerLon, now],
  );

  const visible = bodies.filter((b) => b.alt >= 0);
  const below = bodies.filter((b) => b.alt < 0);

  const lf = filter.trim().toLowerCase();
  const matches = (name: string) => !lf || name.includes(lf);

  function svgPoint(evt: React.MouseEvent<SVGSVGElement>) {
    const svg = svgRef.current;
    if (!svg) return null;
    const pt = svg.createSVGPoint();
    pt.x = evt.clientX;
    pt.y = evt.clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return null;
    const inv = ctm.inverse();
    const p = pt.matrixTransform(inv);
    return { x: p.x, y: p.y };
  }

  function pickBody(svgX: number, svgY: number): BodyPosition | null {
    let best: BodyPosition | null = null;
    let bestDist = 14;
    for (const b of visible) {
      const p = project(b.alt, b.az);
      const d = Math.hypot(p.x - svgX, p.y - svgY);
      if (d < bestDist) {
        bestDist = d;
        best = b;
      }
    }
    return best;
  }

  function handleClick(evt: React.MouseEvent<SVGSVGElement>) {
    const p = svgPoint(evt);
    if (!p) return;
    const body = pickBody(p.x, p.y);
    if (body) {
      onSelect({ kind: "body", name: body.name, alt: body.alt, az: body.az });
      return;
    }
    const aa = unproject(p.x, p.y);
    if (!aa) {
      onSelect(null);
      return;
    }
    onSelect({ kind: "point", alt: aa.alt, az: aa.az });
  }

  function handleMove(evt: React.MouseEvent<SVGSVGElement>) {
    const p = svgPoint(evt);
    if (!p) return;
    setHover(pickBody(p.x, p.y));
  }

  const selectedPos = (() => {
    if (!selected) return null;
    if (selected.kind === "body") {
      const b = bodies.find((x) => x.name === selected.name);
      if (b) return project(b.alt, b.az);
    }
    if (selected.alt >= 0) return project(selected.alt, selected.az);
    return null;
  })();

  const currentPos =
    currentAlt != null && currentAz != null && currentAlt >= -2
      ? project(Math.max(0, currentAlt), currentAz)
      : null;
  const targetPos =
    targetAlt != null && targetAz != null && targetAlt >= 0
      ? project(targetAlt, targetAz)
      : null;

  return (
    <div className="sky-map">
      <div className="sky-map-toolbar">
        <input
          className="sky-filter"
          placeholder="Filter (e.g. mars, vega)…"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
        />
        <span className="muted sky-time">
          {now.toLocaleTimeString()} · lat {observerLat.toFixed(2)}°, lon {observerLon.toFixed(2)}°
        </span>
      </div>

      <svg
        ref={svgRef}
        className="sky-svg"
        viewBox={`0 0 ${VIEWBOX} ${VIEWBOX}`}
        onClick={handleClick}
        onMouseMove={handleMove}
        onMouseLeave={() => setHover(null)}
      >
        <defs>
          <radialGradient id="sky-bg" cx="50%" cy="50%" r="50%">
            <stop offset="0%" stopColor="#0e1a36" />
            <stop offset="100%" stopColor="#04060d" />
          </radialGradient>
        </defs>

        <circle cx={CX} cy={CY} r={R} fill="url(#sky-bg)" stroke="#2a3760" strokeWidth={1.5} />

        {[30, 60].map((alt) => (
          <circle
            key={alt}
            cx={CX}
            cy={CY}
            r={R * ((90 - alt) / 90)}
            fill="none"
            stroke="#1f2a44"
            strokeDasharray="2 4"
          />
        ))}

        {[0, 90, 180, 270].map((az) => {
          const a = project(0, az);
          return (
            <line
              key={az}
              x1={CX}
              y1={CY}
              x2={a.x}
              y2={a.y}
              stroke="#1a2440"
              strokeDasharray="2 5"
            />
          );
        })}

        {[
          { az: 0,   label: "N" },
          { az: 90,  label: "E" },
          { az: 180, label: "S" },
          { az: 270, label: "W" },
        ].map(({ az, label }) => {
          const p = project(-3, az);
          return (
            <text
              key={label}
              x={p.x}
              y={p.y + 4}
              textAnchor="middle"
              className={`sky-cardinal sky-cardinal-${label.toLowerCase()}`}
            >
              {label}
            </text>
          );
        })}

        {visible.map((b) => {
          if (!matches(b.name)) return null;
          const p = project(b.alt, b.az);
          const style = KIND_STYLE[b.kind];
          const isHover = hover?.name === b.name;
          const isSel = selected?.kind === "body" && selected.name === b.name;
          return (
            <g key={b.name} className="sky-body" pointerEvents="none">
              <circle
                cx={p.x}
                cy={p.y}
                r={style.radius + (isHover || isSel ? 2 : 0)}
                fill={style.fill}
                opacity={lf && !matches(b.name) ? 0.15 : 0.95}
              />
              {(isHover || isSel || (lf && matches(b.name))) && (
                <text
                  x={p.x + style.radius + 4}
                  y={p.y + 3}
                  className="sky-label"
                >
                  {b.name}
                </text>
              )}
            </g>
          );
        })}

        {currentPos && (
          <g pointerEvents="none">
            <circle cx={currentPos.x} cy={currentPos.y} r={9} fill="none" stroke="#6db4ff" strokeWidth={1.2} />
            <line x1={currentPos.x - 12} y1={currentPos.y} x2={currentPos.x + 12} y2={currentPos.y} stroke="#6db4ff" strokeWidth={0.8} />
            <line x1={currentPos.x} y1={currentPos.y - 12} x2={currentPos.x} y2={currentPos.y + 12} stroke="#6db4ff" strokeWidth={0.8} />
          </g>
        )}

        {targetPos && (
          <circle cx={targetPos.x} cy={targetPos.y} r={11} fill="none" stroke="#4ad08a" strokeWidth={1.5} strokeDasharray="3 3" pointerEvents="none" />
        )}

        {selectedPos && (
          <g pointerEvents="none">
            <circle cx={selectedPos.x} cy={selectedPos.y} r={13} fill="none" stroke="#ffb84d" strokeWidth={1.5} />
            <circle cx={selectedPos.x} cy={selectedPos.y} r={2.5} fill="#ffb84d" />
          </g>
        )}
      </svg>

      <div className="sky-info">
        {selected ? (
          selected.kind === "body" ? (
            <span>
              <strong>{selected.name}</strong>
              <span className="muted"> · alt {selected.alt.toFixed(1)}°, az {selected.az.toFixed(1)}°</span>
            </span>
          ) : (
            <span>
              <strong>Region</strong>
              <span className="muted"> · alt {selected.alt.toFixed(1)}°, az {selected.az.toFixed(1)}°</span>
            </span>
          )
        ) : (
          <span className="muted">Click a dot to pick a body — or anywhere on the dome to pick a region.</span>
        )}
      </div>

      {below.length > 0 && (
        <details className="sky-below">
          <summary>{below.length} below horizon</summary>
          <div className="sky-below-list">
            {below
              .filter((b) => matches(b.name))
              .map((b) => (
                <span key={b.name} className="sky-below-item" title={`alt ${b.alt.toFixed(1)}°, az ${b.az.toFixed(1)}°`}>
                  {b.name}
                </span>
              ))}
          </div>
        </details>
      )}
    </div>
  );
}
