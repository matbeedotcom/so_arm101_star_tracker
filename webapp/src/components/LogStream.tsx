import { useEffect, useRef } from "react";

const severityClass = (line: string): string => {
  if (line.startsWith("E ")) return "l-e";
  if (line.startsWith("W ")) return "l-w";
  if (line.startsWith("D ")) return "l-d";
  return "l-i";
};

export function LogStream({ lines, onClear }: { lines: string[]; onClear: () => void }) {
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    // Stick to bottom only if user hasn't scrolled away.
    const stuck = el.scrollHeight - el.scrollTop - el.clientHeight < 30;
    if (stuck) el.scrollTop = el.scrollHeight;
  }, [lines]);

  return (
    <div className="card">
      <h2>
        Log <span style={{ float: "right" }}>
          <button onClick={onClear} style={{ padding: "2px 8px", fontSize: 11 }}>clear</button>
        </span>
      </h2>
      <div className="log" ref={ref}>
        {lines.length === 0 ? (
          <span className="faint">(empty)</span>
        ) : (
          lines.map((line, i) => (
            <div key={i} className={severityClass(line)}>{line}</div>
          ))
        )}
      </div>
    </div>
  );
}
