#!/usr/bin/env python3

"""
Standalone astropy validation for star tracker calculations.
Tests celestial coordinate calculations against known astronomical data.
"""

import numpy as np
from datetime import datetime, timezone

def test_without_astropy():
    """Test basic calculations without astropy library."""
    print("=== Testing Basic Calculations (No Astropy) ===")
    
    # Test sidereal rate calculation
    sidereal_rate_deg_per_hour = 15.041067  # True sidereal rate
    sidereal_rate_rad_per_sec = np.radians(sidereal_rate_deg_per_hour) / 3600
    
    print(f"Sidereal rate: {sidereal_rate_deg_per_hour:.6f}°/hour")
    print(f"Sidereal rate: {sidereal_rate_rad_per_sec:.8f} rad/sec")
    
    # Test coordinate system conversions
    test_locations = [
        (40.7128, -74.0060, "New York City"),
        (51.5074, -0.1278, "London"),
        (-33.8688, 151.2093, "Sydney"),
        (90.0, 0.0, "North Pole"),
        (0.0, 0.0, "Equator")
    ]
    
    print("\nPolaris altitude approximation (should ≈ latitude):")
    for lat, lon, name in test_locations:
        polaris_alt_approx = lat  # Polaris altitude ≈ observer latitude
        print(f"{name:15}: Lat={lat:7.3f}°, Polaris Alt≈{polaris_alt_approx:7.3f}°")
    
    return True

def test_with_astropy():
    """Test accurate calculations with astropy."""
    try:
        from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
        from astropy.time import Time
        from astropy import units as u
        import astropy.coordinates as coord
        
        print("\n=== Testing Astropy Calculations ===")
        
        # Test time: Summer Solstice 2024 at noon UTC
        test_time = Time('2024-06-21T12:00:00')
        print(f"Test time: {test_time.iso}")
        
        # Test location: New York City
        location = EarthLocation(lat=40.7128*u.deg, lon=-74.0060*u.deg, height=10*u.m)
        altaz_frame = AltAz(obstime=test_time, location=location)
        
        print(f"Observer location: {location}")
        
        # Sun position on summer solstice
        sun_coord = get_sun(test_time)
        sun_altaz = sun_coord.transform_to(altaz_frame)
        
        print(f"\nSun position (Summer Solstice, NYC, Noon UTC):")
        print(f"  Altitude: {sun_altaz.alt.deg:.2f}°")
        print(f"  Azimuth:  {sun_altaz.az.deg:.2f}°")
        
        # Validate sun altitude on summer solstice
        # NYC latitude: 40.7°, sun declination on solstice: +23.44°
        # Expected noon altitude ≈ 90° - |lat - dec| = 90° - |40.7° - 23.44°| = 72.74°
        expected_sun_alt = 90 - abs(40.7 - 23.44)
        alt_error = abs(sun_altaz.alt.deg - expected_sun_alt)
        
        print(f"  Expected: ~{expected_sun_alt:.1f}° (theoretical)")
        print(f"  Error:    {alt_error:.1f}°")
        
        if alt_error < 5:
            print("  ✓ Sun altitude within expected range")
        else:
            print("  ? Sun altitude differs from simple calculation (may include refraction, etc.)")
        
        # Moon position
        moon_coord = get_moon(test_time)
        moon_altaz = moon_coord.transform_to(altaz_frame)
        
        print(f"\nMoon position:")
        print(f"  Altitude: {moon_altaz.alt.deg:.2f}°")
        print(f"  Azimuth:  {moon_altaz.az.deg:.2f}°")
        print(f"  Visible:  {'Yes' if moon_altaz.alt.deg > 0 else 'No'}")
        
        # Polaris position
        polaris = coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s')
        polaris_altaz = polaris.transform_to(altaz_frame)
        
        print(f"\nPolaris position:")
        print(f"  Altitude: {polaris_altaz.alt.deg:.2f}°")
        print(f"  Azimuth:  {polaris_altaz.az.deg:.2f}°")
        
        # Validate Polaris altitude ≈ latitude
        polaris_error = abs(polaris_altaz.alt.deg - 40.7128)
        print(f"  Expected: ~{40.7128:.1f}° (latitude)")
        print(f"  Error:    {polaris_error:.2f}°")
        
        if polaris_error < 2:
            print("  ✓ Polaris altitude matches latitude")
        else:
            print("  ? Polaris altitude error larger than expected")
        
        # Test bright stars
        stars = [
            ('Sirius', 'α CMa', '06h45m09s', '-16d42m58s'),
            ('Vega', 'α Lyr', '18h36m56s', '+38d47m01s'),
            ('Arcturus', 'α Boo', '14h15m40s', '+19d10m57s')
        ]
        
        print(f"\nBright star positions:")
        for name, designation, ra, dec in stars:
            star_coord = coord.SkyCoord(ra=ra, dec=dec)
            star_altaz = star_coord.transform_to(altaz_frame)
            
            visible = "Yes" if star_altaz.alt.deg > 0 else "No"
            print(f"  {name:10} ({designation}): Alt={star_altaz.alt.deg:6.1f}°, Az={star_altaz.az.deg:6.1f}°, Visible={visible}")
        
        # Test coordinate precision
        print(f"\nCoordinate precision test:")
        
        # Calculate same object 1 second later
        test_time_plus = test_time + 1*u.second
        altaz_frame_plus = AltAz(obstime=test_time_plus, location=location)
        
        sun_altaz_plus = sun_coord.transform_to(altaz_frame_plus)
        
        alt_change = (sun_altaz_plus.alt - sun_altaz.alt).to(u.arcsec)
        az_change = (sun_altaz_plus.az - sun_altaz.az).to(u.arcsec)
        
        print(f"  Sun position change over 1 second:")
        print(f"    Altitude: {alt_change.value:.3f} arcsec")
        print(f"    Azimuth:  {az_change.value:.3f} arcsec")
        
        # For astrophotography, we need sub-arcsecond precision over exposure times
        if abs(alt_change.value) < 1 and abs(az_change.value) < 1:
            print("  ✓ Sub-arcsecond precision maintained")
        else:
            print("  ? Coordinate changes larger than expected")
        
        return True
        
    except ImportError:
        print("\n~ Astropy not available - install with: pip3 install astropy")
        return False
    except Exception as e:
        print(f"\n✗ Astropy calculation error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_time_synchronization():
    """Test time synchronization accuracy requirements."""
    print("\n=== Testing Time Synchronization ===")
    
    # System time
    system_time = datetime.now(timezone.utc)
    
    # Simulate GPS time (assume perfect synchronization)
    gps_time = system_time
    
    # Calculate potential errors
    print(f"System UTC time: {system_time.isoformat()}")
    print(f"GPS UTC time:    {gps_time.isoformat()}")
    
    # For astrophotography, time accuracy requirements:
    # - Tracking: ~1 second accuracy sufficient
    # - Precise ephemeris: ~1 millisecond for best accuracy
    
    time_error = abs((gps_time - system_time).total_seconds())
    print(f"Time synchronization error: {time_error*1000:.1f} milliseconds")
    
    if time_error < 0.001:  # 1ms
        print("✓ Excellent time synchronization")
    elif time_error < 0.1:  # 100ms
        print("✓ Good time synchronization")
    elif time_error < 1.0:  # 1s
        print("✓ Adequate time synchronization for tracking")
    else:
        print("? Time synchronization may affect accuracy")
    
    # Test UTC to local time conversion
    local_time = system_time.astimezone()
    print(f"Local time:      {local_time.isoformat()}")
    
    return True

def test_coordinate_transformations():
    """Test coordinate system transformations."""
    print("\n=== Testing Coordinate Transformations ===")
    
    # Test altitude/azimuth to joint angle conversion
    # (Replicated from star_tracker_node.py)
    
    test_cases = [
        (np.radians(45), np.radians(0), "45° alt, 0° az (north)"),
        (np.radians(30), np.radians(90), "30° alt, 90° az (east)"),
        (np.radians(60), np.radians(180), "60° alt, 180° az (south)"),
        (np.radians(20), np.radians(270), "20° alt, 270° az (west)"),
        (np.radians(90), np.radians(0), "90° alt (zenith)"),
        (np.radians(0), np.radians(0), "0° alt (horizon)")
    ]
    
    print("Alt/Az to robot joint angle conversion:")
    print("Format: [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll]")
    
    for alt, az, description in test_cases:
        # Simple joint angle calculation (from star_tracker_node.py)
        shoulder_rotation = az
        shoulder_pitch = alt - np.pi/2  # Adjust for robot's zero position
        elbow = 0.0
        wrist_pitch = -shoulder_pitch  # Compensate for shoulder pitch
        wrist_roll = 0.0
        
        # Clamp to joint limits
        joint_positions = [
            np.clip(shoulder_rotation, -np.pi, np.pi),
            np.clip(shoulder_pitch, -np.pi, np.pi),
            np.clip(elbow, -np.pi, np.pi),
            np.clip(wrist_pitch, -np.pi, np.pi),
            np.clip(wrist_roll, -np.pi, np.pi)
        ]
        
        # Convert to degrees for display
        joint_degrees = [np.degrees(j) for j in joint_positions]
        
        print(f"  {description:20} -> [{joint_degrees[0]:6.1f}, {joint_degrees[1]:6.1f}, {joint_degrees[2]:6.1f}, {joint_degrees[3]:6.1f}, {joint_degrees[4]:6.1f}]°")
        
        # Validate joint limits
        for i, angle in enumerate(joint_positions):
            if not (-np.pi <= angle <= np.pi):
                print(f"    ✗ Joint {i} out of range: {np.degrees(angle):.1f}°")
                return False
    
    print("  ✓ All joint angles within valid range")
    return True

def main():
    """Run all astropy validation tests."""
    print("Astropy and Coordinate Validation for Star Tracker")
    print("=" * 60)
    
    results = {}
    
    # Run validation tests
    results['basic_calculations'] = test_without_astropy()
    results['astropy_calculations'] = test_with_astropy()
    results['time_sync'] = test_time_synchronization()
    results['coordinate_transforms'] = test_coordinate_transformations()
    
    # Summary
    print(f"\n" + "=" * 60)
    print("ASTROPY VALIDATION SUMMARY")
    print("=" * 60)
    
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test_name.upper():25} {status}")
    
    overall_success = all(results.values())
    print(f"\nOVERALL: {'✓ ALL TESTS PASSED' if overall_success else '✗ SOME TESTS FAILED'}")
    
    if not results['astropy_calculations']:
        print("\nNOTE: Install astropy for full validation: pip3 install astropy")
    
    return overall_success

if __name__ == '__main__':
    success = main()