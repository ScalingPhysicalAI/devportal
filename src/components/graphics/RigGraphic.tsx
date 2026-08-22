// Abstract line-art of an articulated arm / manipulator — a stylized
// stand-in for Buildo, not product photography.
export function RigGraphic({ className }: { className?: string }) {
  return (
    <svg
      viewBox="0 0 480 480"
      fill="none"
      className={className}
      aria-hidden="true"
    >
      <circle cx="240" cy="240" r="200" stroke="#b89c72" strokeOpacity="0.12" />
      <circle cx="240" cy="240" r="150" stroke="#b89c72" strokeOpacity="0.16" />
      <circle cx="240" cy="240" r="100" stroke="#b89c72" strokeOpacity="0.2" />

      {/* base */}
      <rect x="200" y="360" width="80" height="20" rx="2" stroke="#e8e2d6" strokeOpacity="0.5" />
      <line x1="240" y1="360" x2="240" y2="300" stroke="#e8e2d6" strokeOpacity="0.6" strokeWidth="2" />

      {/* joint 1 */}
      <circle cx="240" cy="290" r="12" fill="#050505" stroke="#b89c72" strokeWidth="2" />
      <line
        x1="240"
        y1="290"
        x2="180"
        y2="200"
        stroke="#e8e2d6"
        strokeOpacity="0.6"
        strokeWidth="2"
      />

      {/* joint 2 */}
      <circle cx="180" cy="200" r="10" fill="#050505" stroke="#b89c72" strokeWidth="2" />
      <line
        x1="180"
        y1="200"
        x2="260"
        y2="140"
        stroke="#e8e2d6"
        strokeOpacity="0.6"
        strokeWidth="2"
      />

      {/* joint 3 / end effector */}
      <circle cx="260" cy="140" r="8" fill="#050505" stroke="#b89c72" strokeWidth="2" />
      <path
        d="M260 140 L285 118 M260 140 L285 150"
        stroke="#b89c72"
        strokeWidth="2"
        strokeLinecap="round"
      />

      {/* telemetry ticks */}
      {Array.from({ length: 24 }).map((_, i) => {
        const angle = (i / 24) * Math.PI * 2;
        const r1 = 200;
        const r2 = i % 3 === 0 ? 190 : 196;
        const x1 = 240 + r1 * Math.cos(angle);
        const y1 = 240 + r1 * Math.sin(angle);
        const x2 = 240 + r2 * Math.cos(angle);
        const y2 = 240 + r2 * Math.sin(angle);
        return (
          <line
            key={i}
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            stroke="#b89c72"
            strokeOpacity="0.4"
          />
        );
      })}
    </svg>
  );
}
