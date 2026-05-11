// Lightweight client-side astronomy for the sky-map picker.
//
// - Stars: ICRS RA/Dec lookup (matches goto/config.py STAR_CATALOG).
// - Sun / Moon / planets: Paul Schlyter's low-precision orbital elements
//   (https://stjarnhimlen.se/comp/ppcomp.html). Accuracy is ~1° for inner
//   planets and a few arc-minutes for Sun/Moon — plenty for visual picking;
//   the Pi resolves the actual goto via astropy.

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;

export interface AltAz { alt: number; az: number; }
export interface RaDec { ra: number; dec: number; }

// RA/Dec in degrees (J2000), mirrors goto/config.py:STAR_CATALOG.
export const STAR_CATALOG_RADEC: Record<string, RaDec> = {
  polaris:    { ra:  37.9545, dec:  89.2641 },
  sirius:     { ra: 101.2872, dec: -16.7161 },
  vega:       { ra: 279.2347, dec:  38.7836 },
  arcturus:   { ra: 213.9153, dec:  19.1824 },
  betelgeuse: { ra:  88.7929, dec:   7.4071 },
  rigel:      { ra:  78.6345, dec:  -8.2017 },
  capella:    { ra:  79.1723, dec:  45.9980 },
  procyon:    { ra: 114.8255, dec:   5.2250 },
  altair:     { ra: 297.6958, dec:   8.8683 },
  deneb:      { ra: 310.3580, dec:  45.2803 },
  antares:    { ra: 247.3519, dec: -26.4320 },
  spica:      { ra: 201.2982, dec: -11.1613 },
  aldebaran:  { ra:  68.9802, dec:  16.5093 },
  regulus:    { ra: 152.0930, dec:  11.9672 },
  castor:     { ra: 113.6496, dec:  31.8883 },
  pollux:     { ra: 116.3289, dec:  28.0262 },
  fomalhaut:  { ra: 344.4127, dec: -29.6222 },
  canopus:    { ra:  95.9879, dec: -52.6957 },
};

export const SOLAR_SYSTEM = [
  "sun", "moon", "mercury", "venus", "mars",
  "jupiter", "saturn", "uranus", "neptune",
] as const;

function julianDate(d: Date): number {
  return d.getTime() / 86400000 + 2440587.5;
}
function daysSince2000(d: Date): number {
  return julianDate(d) - 2451545.0;
}
function norm360(x: number): number {
  const r = x % 360;
  return r < 0 ? r + 360 : r;
}

// Greenwich Mean Sidereal Time in degrees.
function gmstDeg(d: Date): number {
  const D = daysSince2000(d);
  const hours = ((18.697374558 + 24.06570982441908 * D) % 24 + 24) % 24;
  return hours * 15;
}

export function lstDeg(d: Date, lonDeg: number): number {
  return norm360(gmstDeg(d) + lonDeg);
}

export function raDecToAltAz(ra: number, dec: number, lat: number, lst: number): AltAz {
  const ha = norm360(lst - ra) * D2R;
  const dec_r = dec * D2R;
  const lat_r = lat * D2R;
  const sinAlt = Math.sin(dec_r) * Math.sin(lat_r) + Math.cos(dec_r) * Math.cos(lat_r) * Math.cos(ha);
  const alt = Math.asin(Math.max(-1, Math.min(1, sinAlt)));
  const cosAz = (Math.sin(dec_r) - Math.sin(alt) * Math.sin(lat_r)) / (Math.cos(alt) * Math.cos(lat_r));
  let az = Math.acos(Math.max(-1, Math.min(1, cosAz)));
  if (Math.sin(ha) > 0) az = 2 * Math.PI - az;
  return { alt: alt * R2D, az: az * R2D };
}

// ── Schlyter orbital elements (epoch 2000-01-01 00:00 UT) ────────────────────

interface Elements {
  N0: number; dN: number;   // long. of ascending node (deg)
  i0: number; di: number;   // inclination (deg)
  w0: number; dW: number;   // arg. of perihelion (deg)
  a0: number; dA: number;   // semi-major axis (AU; Earth radii for Moon)
  e0: number; dE: number;   // eccentricity
  M0: number; dM: number;   // mean anomaly (deg)
}

const SUN: Elements = {
  N0:   0,        dN: 0,
  i0:   0,        di: 0,
  w0: 282.9404,   dW: 4.70935e-5,
  a0:   1.0,      dA: 0,
  e0:   0.016709, dE: -1.151e-9,
  M0: 356.0470,   dM: 0.9856002585,
};
const MOON: Elements = {
  N0: 125.1228,   dN: -0.0529538083,
  i0:   5.1454,   di: 0,
  w0: 318.0634,   dW: 0.1643573223,
  a0:  60.2666,   dA: 0,
  e0:   0.054900, dE: 0,
  M0: 115.3654,   dM: 13.0649929509,
};
const MERCURY: Elements = {
  N0:  48.3313,   dN: 3.24587e-5,
  i0:   7.0047,   di: 5.00e-8,
  w0:  29.1241,   dW: 1.01444e-5,
  a0:   0.387098, dA: 0,
  e0:   0.205635, dE: 5.59e-10,
  M0: 168.6562,   dM: 4.0923344368,
};
const VENUS: Elements = {
  N0:  76.6799,   dN: 2.46590e-5,
  i0:   3.3946,   di: 2.75e-8,
  w0:  54.8910,   dW: 1.38374e-5,
  a0:   0.723330, dA: 0,
  e0:   0.006773, dE: -1.302e-9,
  M0:  48.0052,   dM: 1.6021302244,
};
const MARS: Elements = {
  N0:  49.5574,   dN: 2.11081e-5,
  i0:   1.8497,   di: -1.78e-8,
  w0: 286.5016,   dW: 2.92961e-5,
  a0:   1.523688, dA: 0,
  e0:   0.093405, dE: 2.516e-9,
  M0:  18.6021,   dM: 0.5240207766,
};
const JUPITER: Elements = {
  N0: 100.4542,   dN: 2.76854e-5,
  i0:   1.3030,   di: -1.557e-7,
  w0: 273.8777,   dW: 1.64505e-5,
  a0:   5.20256,  dA: 0,
  e0:   0.048498, dE: 4.469e-9,
  M0:  19.8950,   dM: 0.0830853001,
};
const SATURN: Elements = {
  N0: 113.6634,   dN: 2.38980e-5,
  i0:   2.4886,   di: -1.081e-7,
  w0: 339.3939,   dW: 2.97661e-5,
  a0:   9.55475,  dA: 0,
  e0:   0.055546, dE: -9.499e-9,
  M0: 316.9670,   dM: 0.0334442282,
};
const URANUS: Elements = {
  N0:  74.0005,   dN: 1.3978e-5,
  i0:   0.7733,   di: 1.9e-8,
  w0:  96.6612,   dW: 3.0565e-5,
  a0:  19.18171,  dA: -1.55e-8,
  e0:   0.047318, dE: 7.45e-9,
  M0: 142.5905,   dM: 0.011725806,
};
const NEPTUNE: Elements = {
  N0: 131.7806,   dN: 3.0173e-5,
  i0:   1.7700,   di: -2.55e-7,
  w0: 272.8461,   dW: -6.027e-6,
  a0:  30.05826,  dA: 3.313e-8,
  e0:   0.008606, dE: 2.15e-9,
  M0: 260.2471,   dM: 0.005995147,
};

const PLANET_ELEMENTS: Record<string, Elements> = {
  mercury: MERCURY, venus: VENUS, mars: MARS,
  jupiter: JUPITER, saturn: SATURN, uranus: URANUS, neptune: NEPTUNE,
};

// Solve Kepler's equation, return eccentric anomaly E (degrees).
function eccentricAnomaly(M: number, e: number): number {
  const Mr = M * D2R;
  let E = M + (e * R2D) * Math.sin(Mr) * (1 + e * Math.cos(Mr));
  for (let k = 0; k < 6; k++) {
    const Er = E * D2R;
    const dE = (E - (e * R2D) * Math.sin(Er) - M) / (1 - e * Math.cos(Er));
    E -= dE;
    if (Math.abs(dE) < 1e-6) break;
  }
  return E;
}

// Heliocentric ecliptic coords (for Moon: geocentric).
function eclXYZ(el: Elements, d: number): { x: number; y: number; z: number } {
  const N = el.N0 + el.dN * d;
  const i = el.i0 + el.di * d;
  const w = el.w0 + el.dW * d;
  const a = el.a0 + el.dA * d;
  const e = el.e0 + el.dE * d;
  const M = norm360(el.M0 + el.dM * d);
  const E = eccentricAnomaly(M, e);
  const Er = E * D2R;
  const xv = a * (Math.cos(Er) - e);
  const yv = a * Math.sqrt(1 - e * e) * Math.sin(Er);
  const v = Math.atan2(yv, xv);
  const r = Math.sqrt(xv * xv + yv * yv);
  const Nr = N * D2R, ir = i * D2R, wr = w * D2R;
  const vw = v + wr;
  return {
    x: r * (Math.cos(Nr) * Math.cos(vw) - Math.sin(Nr) * Math.sin(vw) * Math.cos(ir)),
    y: r * (Math.sin(Nr) * Math.cos(vw) + Math.cos(Nr) * Math.sin(vw) * Math.cos(ir)),
    z: r * Math.sin(vw) * Math.sin(ir),
  };
}

function obliquity(d: number): number {
  return (23.4393 - 3.563e-7 * d) * D2R;
}

function eclToEqRaDec(x: number, y: number, z: number, oblecl: number): RaDec {
  const xe = x;
  const ye = y * Math.cos(oblecl) - z * Math.sin(oblecl);
  const ze = y * Math.sin(oblecl) + z * Math.cos(oblecl);
  return {
    ra: norm360(Math.atan2(ye, xe) * R2D),
    dec: Math.atan2(ze, Math.sqrt(xe * xe + ye * ye)) * R2D,
  };
}

export function bodyRaDec(name: string, when: Date): RaDec | null {
  const d = daysSince2000(when);
  const ob = obliquity(d);

  if (name === "sun") {
    const s = eclXYZ(SUN, d);
    return eclToEqRaDec(s.x, s.y, s.z, ob);
  }
  if (name === "moon") {
    const m = eclXYZ(MOON, d);
    return eclToEqRaDec(m.x, m.y, m.z, ob);
  }
  const el = PLANET_ELEMENTS[name];
  if (!el) {
    const star = STAR_CATALOG_RADEC[name];
    return star ?? null;
  }
  const p = eclXYZ(el, d);
  const s = eclXYZ(SUN, d);
  return eclToEqRaDec(p.x + s.x, p.y + s.y, p.z + s.z, ob);
}

export interface BodyPosition {
  name: string;
  kind: "star" | "sun" | "moon" | "planet";
  alt: number;
  az: number;
}

export function computeBodyPositions(
  names: string[],
  lat: number,
  lon: number,
  when: Date,
): BodyPosition[] {
  const lst = lstDeg(when, lon);
  const out: BodyPosition[] = [];
  for (const name of names) {
    const rd = bodyRaDec(name, when);
    if (!rd) continue;
    const aa = raDecToAltAz(rd.ra, rd.dec, lat, lst);
    let kind: BodyPosition["kind"];
    if (name === "sun") kind = "sun";
    else if (name === "moon") kind = "moon";
    else if (name in PLANET_ELEMENTS) kind = "planet";
    else kind = "star";
    out.push({ name, kind, alt: aa.alt, az: aa.az });
  }
  return out;
}
