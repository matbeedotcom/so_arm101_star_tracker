"""Celestial target resolution and alt/az computation."""

from astropy.coordinates import EarthLocation, AltAz, SkyCoord, get_body
from astropy.time import Time
import astropy.units as u

from .config import (
    OBSERVER_LAT, OBSERVER_LON, OBSERVER_HEIGHT,
    STAR_CATALOG, SOLAR_SYSTEM,
)


def get_observer():
    """Return EarthLocation for the configured observer position."""
    return EarthLocation(
        lat=OBSERVER_LAT * u.deg,
        lon=OBSERVER_LON * u.deg,
        height=OBSERVER_HEIGHT * u.m,
    )


def resolve_target(target_name):
    """Resolve a target name to a (kind, data) tuple.

    Returns:
        ('body', name) for solar system objects
        ('star', SkyCoord) for catalog or resolved stars
        ('fixed', (alt, az)) for fixed coordinates
        None if unresolvable
    """
    name = target_name.lower().strip()

    if name in SOLAR_SYSTEM:
        return ('body', name)
    if name in STAR_CATALOG:
        ra, dec = STAR_CATALOG[name]
        return ('star', SkyCoord(ra=ra, dec=dec, frame='icrs'))

    # Try astropy name resolver (requires internet)
    try:
        coord = SkyCoord.from_name(target_name)
        return ('star', coord)
    except Exception:
        pass

    return None


def compute_altaz(target_info, when=None):
    """Compute Alt/Az for a resolved target at ``when`` (Time, default now)."""
    kind, data = target_info

    if kind == 'fixed':
        return data  # (alt, az) tuple — time-invariant

    obs = get_observer()
    t = when if when is not None else Time.now()
    altaz_frame = AltAz(obstime=t, location=obs)

    if kind == 'body':
        body = get_body(data, t, obs)
        body_altaz = body.transform_to(altaz_frame)
    else:
        body_altaz = data.transform_to(altaz_frame)

    return body_altaz.alt.deg, body_altaz.az.deg


def next_rise_time(target_info, min_alt=10.0, search_hours=48,
                   coarse_minutes=10, refine_seconds=60):
    """Find when ``target_info`` next reaches at least ``min_alt`` degrees.

    Returns a dict ``{iso, alt, az, minutes_from_now}`` or ``None`` if the
    target stays below ``min_alt`` for the entire search window.

    Two-phase search to keep astropy overhead bounded:
      1. coarse sweep at ``coarse_minutes`` resolution to find the first
         step where altitude crosses ``min_alt`` going up
      2. bisection on that interval down to ``refine_seconds`` resolution

    Fixed (alt, az) targets are time-invariant — returns None if below.
    """
    kind, _ = target_info
    if kind == 'fixed':
        alt_now, _ = compute_altaz(target_info)
        return None if alt_now < min_alt else {
            'iso': Time.now().utc.isot, 'alt': alt_now, 'az': _,
            'minutes_from_now': 0,
        }

    now = Time.now()
    step = coarse_minutes / 60.0  # hours
    n_steps = int(search_hours / step) + 1

    prev_alt = None
    cross_lo = None
    cross_hi = None

    for i in range(n_steps):
        t = now + (i * step) * u.hour
        alt, az = compute_altaz(target_info, when=t)
        if prev_alt is not None and prev_alt < min_alt <= alt:
            cross_lo = (i - 1) * step
            cross_hi = i * step
            break
        prev_alt = alt

    if cross_lo is None:
        return None  # never rises in window

    # Bisection refine to ~refine_seconds precision.
    lo_h, hi_h = cross_lo, cross_hi
    target_precision = refine_seconds / 3600.0
    while (hi_h - lo_h) > target_precision:
        mid_h = (lo_h + hi_h) / 2.0
        mid_alt, _ = compute_altaz(target_info, when=now + mid_h * u.hour)
        if mid_alt >= min_alt:
            hi_h = mid_h
        else:
            lo_h = mid_h

    rise_t = now + hi_h * u.hour
    alt, az = compute_altaz(target_info, when=rise_t)
    return {
        'iso': rise_t.utc.isot + 'Z' if not rise_t.utc.isot.endswith('Z') else rise_t.utc.isot,
        'alt': alt,
        'az': az,
        'minutes_from_now': round(hi_h * 60),
    }
