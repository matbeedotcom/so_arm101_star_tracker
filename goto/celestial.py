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


def compute_altaz(target_info):
    """Compute current Alt/Az for a resolved target.

    Handles all target types including fixed coordinates.
    Returns (alt_deg, az_deg).
    """
    kind, data = target_info

    if kind == 'fixed':
        return data  # (alt, az) tuple

    obs = get_observer()
    now = Time.now()
    altaz_frame = AltAz(obstime=now, location=obs)

    if kind == 'body':
        body = get_body(data, now, obs)
        body_altaz = body.transform_to(altaz_frame)
    else:
        body_altaz = data.transform_to(altaz_frame)

    return body_altaz.alt.deg, body_altaz.az.deg
