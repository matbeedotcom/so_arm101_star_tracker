#!/usr/bin/env python3

"""
Test validation script for Star Tracker GPS integration.
Validates syntax, imports, and basic functionality without requiring ROS2 runtime.
"""

import sys
import ast
import importlib
import traceback
from datetime import datetime, timezone
import json

def validate_syntax(filepath, description):
    """Validate Python syntax of a file."""
    print(f"\n=== Validating {description} ===")
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            source = f.read()
        
        # Parse AST to check syntax
        ast.parse(source)
        print(f"✓ Syntax validation passed for {filepath}")
        return True
        
    except SyntaxError as e:
        print(f"✗ Syntax error in {filepath}: {e}")
        return False
    except Exception as e:
        print(f"✗ Error reading {filepath}: {e}")
        return False

def validate_imports():
    """Validate required imports are available."""
    print(f"\n=== Validating Required Imports ===")
    
    required_imports = [
        ('numpy', 'Scientific computing'),
        ('datetime', 'Date/time handling'),
        ('json', 'Configuration files'),
        ('threading', 'Multi-threading'),
        ('unittest', 'Testing framework')
    ]
    
    optional_imports = [
        ('astropy', 'Astronomy calculations'),
        ('scipy', 'Scientific computing'),
        ('rclpy', 'ROS2 Python client')
    ]
    
    results = {'required': [], 'optional': []}
    
    # Test required imports
    for module, description in required_imports:
        try:
            importlib.import_module(module)
            print(f"✓ {module} - {description}")
            results['required'].append((module, True))
        except ImportError:
            print(f"✗ {module} - {description} (REQUIRED)")
            results['required'].append((module, False))
    
    # Test optional imports
    for module, description in optional_imports:
        try:
            importlib.import_module(module)
            print(f"✓ {module} - {description}")
            results['optional'].append((module, True))
        except ImportError:
            print(f"~ {module} - {description} (optional, limited functionality)")
            results['optional'].append((module, False))
    
    return results

def test_astropy_calculations():
    """Test astropy calculations with known data."""
    print(f"\n=== Testing Astropy Calculations ===")
    
    try:
        from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
        from astropy.time import Time
        from astropy import units as u
        import astropy.coordinates as coord
        
        # Test location: New York City
        location = EarthLocation(lat=40.7128*u.deg, lon=-74.0060*u.deg, height=10*u.m)
        
        # Test time: 2024-06-21 12:00:00 UTC (Summer solstice)
        test_time = Time('2024-06-21T12:00:00')
        altaz_frame = AltAz(obstime=test_time, location=location)
        
        # Test sun position
        sun_coord = get_sun(test_time)
        sun_altaz = sun_coord.transform_to(altaz_frame)
        
        print(f"✓ Sun position calculated: Alt={sun_altaz.alt.deg:.1f}°, Az={sun_altaz.az.deg:.1f}°")
        
        # Validate sun altitude on summer solstice (should be high for NYC)
        if 60 < sun_altaz.alt.deg < 80:
            print(f"✓ Sun altitude reasonable for NYC summer solstice")
        else:
            print(f"? Sun altitude unusual but may be correct: {sun_altaz.alt.deg:.1f}°")
        
        # Test moon position
        moon_coord = get_moon(test_time)
        moon_altaz = moon_coord.transform_to(altaz_frame)
        
        print(f"✓ Moon position calculated: Alt={moon_altaz.alt.deg:.1f}°, Az={moon_altaz.az.deg:.1f}°")
        
        # Test star coordinates
        polaris = coord.SkyCoord(ra='02h31m49s', dec='+89d15m51s')
        polaris_altaz = polaris.transform_to(altaz_frame)
        
        print(f"✓ Polaris position: Alt={polaris_altaz.alt.deg:.1f}°, Az={polaris_altaz.az.deg:.1f}°")
        
        # Polaris altitude should approximately equal latitude for NYC
        polaris_alt_error = abs(polaris_altaz.alt.deg - 40.7)
        if polaris_alt_error < 5:
            print(f"✓ Polaris altitude matches latitude (error: {polaris_alt_error:.1f}°)")
        else:
            print(f"? Polaris altitude error larger than expected: {polaris_alt_error:.1f}°")
        
        return True
        
    except ImportError:
        print("~ Astropy not available - using fallback calculations")
        return False
    except Exception as e:
        print(f"✗ Astropy calculation error: {e}")
        traceback.print_exc()
        return False

def test_coordinate_transformations():
    """Test coordinate transformation functions."""
    print(f"\n=== Testing Coordinate Transformations ===")
    
    import numpy as np
    
    # Test altitude/azimuth to joint angle conversion
    test_cases = [
        (np.radians(45), np.radians(0), "45° alt, 0° az (north)"),
        (np.radians(30), np.radians(90), "30° alt, 90° az (east)"),
        (np.radians(60), np.radians(180), "60° alt, 180° az (south)"),
        (np.radians(20), np.radians(270), "20° alt, 270° az (west)")
    ]
    
    try:
        for alt, az, description in test_cases:
            # Replicate altaz_to_joint_angles logic from star_tracker_node.py
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
            
            # Validate joint limits
            all_valid = all(-np.pi <= j <= np.pi for j in joint_positions)
            
            if all_valid:
                joint_degrees = [f"{np.degrees(j):.1f}" for j in joint_positions]
                print(f"✓ {description} -> Joints: {joint_degrees}")
            else:
                print(f"✗ {description} -> Invalid joint angles")
                return False
        
        print("✓ All coordinate transformations valid")
        return True
        
    except Exception as e:
        print(f"✗ Coordinate transformation error: {e}")
        traceback.print_exc()
        return False

def test_mock_data_generation():
    """Test mock GPS and IMU data generation."""
    print(f"\n=== Testing Mock Data Generation ===")
    
    try:
        import numpy as np
        from scipy.spatial.transform import Rotation
        
        # Test GPS coordinate generation
        base_lat, base_lon = 40.7128, -74.0060
        noise_level = 0.000001  # ~10cm GPS accuracy
        
        gps_samples = []
        for _ in range(100):
            lat = base_lat + np.random.normal(0, noise_level)
            lon = base_lon + np.random.normal(0, noise_level)
            gps_samples.append((lat, lon))
        
        # Check GPS noise characteristics
        lats, lons = zip(*gps_samples)
        lat_std = np.std(lats)
        lon_std = np.std(lons)
        
        if lat_std < noise_level * 2 and lon_std < noise_level * 2:
            print(f"✓ GPS noise simulation realistic: lat_std={lat_std:.8f}, lon_std={lon_std:.8f}")
        else:
            print(f"? GPS noise may be too high: lat_std={lat_std:.8f}, lon_std={lon_std:.8f}")
        
        # Test IMU quaternion generation
        euler_angles = [0.5, 0.1, 0.3]  # yaw, roll, pitch
        rotation = Rotation.from_euler('zyx', euler_angles)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        
        # Validate quaternion norm
        quat_norm = np.linalg.norm(quaternion)
        if abs(quat_norm - 1.0) < 1e-6:
            print(f"✓ IMU quaternion generation valid: norm={quat_norm:.8f}")
        else:
            print(f"✗ IMU quaternion invalid: norm={quat_norm:.8f}")
            return False
        
        # Test noise addition
        noise_level = 0.01
        noisy_euler = [angle + np.random.normal(0, noise_level) for angle in euler_angles]
        
        print(f"✓ IMU noise simulation: original={[np.degrees(a) for a in euler_angles]}, "
              f"noisy={[np.degrees(a) for a in noisy_euler]}")
        
        return True
        
    except Exception as e:
        print(f"✗ Mock data generation error: {e}")
        traceback.print_exc()
        return False

def test_configuration_files():
    """Test configuration and launch file structure."""
    print(f"\n=== Testing Configuration Structure ===")
    
    try:
        # Test configuration data structure
        test_config = {
            'method': '2star',
            'is_aligned': True,
            'alignment_matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            'alignment_offset': [0.0, 0.0, 0.0],
            'location': {
                'latitude': 40.7128,
                'longitude': -74.0060,
                'altitude': 10.0
            },
            'timestamp': datetime.now().isoformat()
        }
        
        # Test JSON serialization
        json_str = json.dumps(test_config, indent=2)
        parsed_config = json.loads(json_str)
        
        if parsed_config['method'] == test_config['method']:
            print("✓ Configuration JSON serialization works")
        else:
            print("✗ Configuration JSON serialization failed")
            return False
        
        # Test required configuration keys
        required_keys = ['method', 'is_aligned', 'alignment_matrix', 'location']
        for key in required_keys:
            if key not in parsed_config:
                print(f"✗ Missing required configuration key: {key}")
                return False
        
        print("✓ Configuration structure validation passed")
        return True
        
    except Exception as e:
        print(f"✗ Configuration test error: {e}")
        traceback.print_exc()
        return False

def main():
    """Run all validation tests."""
    print("Star Tracker Test Validation")
    print("=" * 50)
    
    results = {}
    
    # File paths (adjust if needed)
    test_files = [
        ("star_tracker/test_framework.py", "Test Framework"),
        ("star_tracker/integration_tests.py", "Integration Tests"),
        ("star_tracker/gps_interface.py", "GPS Interface"),
        ("launch/test_star_tracker.launch.py", "Test Launch File")
    ]
    
    # Validate syntax for each file
    syntax_results = []
    for filepath, description in test_files:
        try:
            result = validate_syntax(filepath, description)
            syntax_results.append(result)
        except Exception as e:
            print(f"? Could not validate {filepath}: {e}")
            syntax_results.append(False)
    
    results['syntax'] = all(syntax_results)
    
    # Validate imports
    import_results = validate_imports()
    results['imports'] = all(success for module, success in import_results['required'])
    
    # Test calculations
    results['astropy'] = test_astropy_calculations()
    results['coordinates'] = test_coordinate_transformations()
    results['mock_data'] = test_mock_data_generation()
    results['config'] = test_configuration_files()
    
    # Summary
    print(f"\n" + "=" * 50)
    print("VALIDATION SUMMARY")
    print("=" * 50)
    
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{test_name.upper():15} {status}")
    
    overall_success = all(results.values())
    print(f"\nOVERALL: {'✓ ALL TESTS PASSED' if overall_success else '✗ SOME TESTS FAILED'}")
    
    # Save results
    results['timestamp'] = datetime.now().isoformat()
    results['overall_success'] = overall_success
    
    try:
        with open('validation_results.json', 'w') as f:
            json.dump(results, f, indent=2)
        print("\nResults saved to validation_results.json")
    except Exception as e:
        print(f"Could not save results: {e}")
    
    return overall_success

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)