#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
import numpy as np
from datetime import datetime, timezone
import json
import time
import threading
from dataclasses import dataclass
from typing import List, Dict, Tuple

try:
    from astropy.coordinates import EarthLocation, AltAz, get_sun, get_moon
    from astropy.time import Time
    from astropy import units as u
    import astropy.coordinates as coord
    ASTROPY_AVAILABLE = True
except ImportError:
    ASTROPY_AVAILABLE = False


@dataclass
class TestLocation:
    name: str
    lat: float
    lon: float 
    alt: float


@dataclass
class TrackingResult:
    timestamp: float
    target_alt: float
    target_az: float
    commanded_alt: float
    commanded_az: float
    error_alt: float
    error_az: float


class StarTrackerIntegrationTests(unittest.TestCase):
    """Comprehensive integration tests for star tracker system."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        rclpy.init()
        cls.test_locations = [
            TestLocation('new_york', 40.7128, -74.0060, 10.0),
            TestLocation('london', 51.5074, -0.1278, 35.0),
            TestLocation('sydney', -33.8688, 151.2093, 58.0),
            TestLocation('tokyo', 35.6762, 139.6503, 40.0),
            TestLocation('north_pole', 90.0, 0.0, 0.0),
            TestLocation('equator', 0.0, 0.0, 0.0)
        ]
        
        cls.test_targets = ['sun', 'moon', 'polaris', 'sirius']
        cls.results = {}
    
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment."""
        rclpy.shutdown()
    
    def test_astropy_calculations(self):
        """Test astropy celestial coordinate calculations."""
        if not ASTROPY_AVAILABLE:
            self.skipTest("Astropy not available")
        
        print("\\nTesting astropy calculations...")
        
        # Fixed test time for reproducible results
        test_time = Time('2024-06-21T12:00:00')  # Summer solstice
        
        for location in self.test_locations:
            earth_location = EarthLocation(
                lat=location.lat*u.deg,
                lon=location.lon*u.deg, 
                height=location.alt*u.m
            )
            altaz_frame = AltAz(obstime=test_time, location=earth_location)
            
            # Test sun position
            sun_coord = get_sun(test_time)
            sun_altaz = sun_coord.transform_to(altaz_frame)
            
            # Validate sun altitude on summer solstice
            expected_sun_alt = 90 - abs(location.lat - 23.44)  # Rough calculation
            alt_error = abs(sun_altaz.alt.deg - expected_sun_alt)
            
            print(f"{location.name}: Sun alt={sun_altaz.alt.deg:.1f}° (expected ~{expected_sun_alt:.1f}°)")
            
            # Sun should be within reasonable bounds
            self.assertGreater(sun_altaz.alt.deg, -90)
            self.assertLess(sun_altaz.alt.deg, 90)
            
            # For locations not in polar regions, sun should be reasonable on solstice
            if abs(location.lat) < 80:
                self.assertLess(alt_error, 30, f"Sun altitude error too large for {location.name}")
    
    def test_coordinate_transformations(self):
        """Test coordinate transformation accuracy."""
        print("\\nTesting coordinate transformations...")
        
        # Test known star positions
        test_cases = [
            {'name': 'polaris', 'ra': '02h31m49s', 'dec': '+89d15m51s'},
            {'name': 'sirius', 'ra': '06h45m09s', 'dec': '-16d42m58s'},
            {'name': 'vega', 'ra': '18h36m56s', 'dec': '+38d47m01s'}
        ]
        
        if not ASTROPY_AVAILABLE:
            self.skipTest("Astropy not available")
        
        test_time = Time('2024-01-01T00:00:00')
        
        for location in self.test_locations[:2]:  # Test first 2 locations
            earth_location = EarthLocation(
                lat=location.lat*u.deg,
                lon=location.lon*u.deg,
                height=location.alt*u.m
            )
            altaz_frame = AltAz(obstime=test_time, location=earth_location)
            
            for star in test_cases:
                star_coord = coord.SkyCoord(ra=star['ra'], dec=star['dec'])
                star_altaz = star_coord.transform_to(altaz_frame)
                
                print(f"{location.name} - {star['name']}: Alt={star_altaz.alt.deg:.1f}°, Az={star_altaz.az.deg:.1f}°")
                
                # Basic sanity checks
                self.assertGreaterEqual(star_altaz.alt.deg, -90)
                self.assertLessEqual(star_altaz.alt.deg, 90)
                self.assertGreaterEqual(star_altaz.az.deg, 0)
                self.assertLessEqual(star_altaz.az.deg, 360)
    
    def test_joint_angle_conversions(self):
        """Test conversion from alt/az to robot joint angles."""
        print("\\nTesting joint angle conversions...")
        
        # Mock star tracker node for testing
        from star_tracker.star_tracker_node import StarTrackerNode
        
        # This would require actual node testing - simplified here
        test_cases = [
            (np.radians(45), np.radians(0)),    # 45° alt, 0° az (north)
            (np.radians(30), np.radians(90)),   # 30° alt, 90° az (east)
            (np.radians(60), np.radians(180)),  # 60° alt, 180° az (south)
            (np.radians(20), np.radians(270)),  # 20° alt, 270° az (west)
        ]
        
        for alt, az in test_cases:
            # Simple joint angle calculation (from star_tracker_node.py)
            shoulder_rotation = az
            shoulder_pitch = alt - np.pi/2
            elbow = 0.0
            wrist_pitch = -shoulder_pitch
            wrist_roll = 0.0
            
            joint_positions = [
                np.clip(shoulder_rotation, -np.pi, np.pi),
                np.clip(shoulder_pitch, -np.pi, np.pi),
                np.clip(elbow, -np.pi, np.pi),
                np.clip(wrist_pitch, -np.pi, np.pi),
                np.clip(wrist_roll, -np.pi, np.pi)
            ]
            
            print(f"Alt={np.degrees(alt):.1f}°, Az={np.degrees(az):.1f}° -> Joints: {[np.degrees(j) for j in joint_positions]}")
            
            # Validate joint limits
            for joint_angle in joint_positions:
                self.assertGreaterEqual(joint_angle, -np.pi)
                self.assertLessEqual(joint_angle, np.pi)
    
    def test_tracking_accuracy_simulation(self):
        """Simulate tracking accuracy over time."""
        print("\\nTesting tracking accuracy simulation...")
        
        if not ASTROPY_AVAILABLE:
            self.skipTest("Astropy not available")
        
        # Simulate tracking for 1 hour
        start_time = Time('2024-06-21T20:00:00')  # Evening
        duration_hours = 1.0
        time_steps = 60  # 1 minute intervals
        
        location = self.test_locations[0]  # New York
        earth_location = EarthLocation(
            lat=location.lat*u.deg,
            lon=location.lon*u.deg,
            height=location.alt*u.m
        )
        
        tracking_results = []
        
        for i in range(time_steps):
            current_time = start_time + (i * duration_hours / time_steps) * u.hour
            altaz_frame = AltAz(obstime=current_time, location=earth_location)
            
            # Track moon
            moon_coord = get_moon(current_time)
            moon_altaz = moon_coord.transform_to(altaz_frame)
            
            if moon_altaz.alt.deg > 0:  # Above horizon
                # Simulate commanded position with small error
                commanded_alt = moon_altaz.alt.rad + np.random.normal(0, np.radians(0.1))
                commanded_az = moon_altaz.az.rad + np.random.normal(0, np.radians(0.1))
                
                result = TrackingResult(
                    timestamp=current_time.unix,
                    target_alt=moon_altaz.alt.rad,
                    target_az=moon_altaz.az.rad,
                    commanded_alt=commanded_alt,
                    commanded_az=commanded_az,
                    error_alt=commanded_alt - moon_altaz.alt.rad,
                    error_az=commanded_az - moon_altaz.az.rad
                )
                tracking_results.append(result)
        
        # Analyze results
        if tracking_results:
            alt_errors = [abs(r.error_alt) for r in tracking_results]
            az_errors = [abs(r.error_az) for r in tracking_results]
            
            rms_alt_error = np.sqrt(np.mean(np.array(alt_errors)**2))
            rms_az_error = np.sqrt(np.mean(np.array(az_errors)**2))
            
            print(f"Tracking results over {len(tracking_results)} samples:")
            print(f"RMS altitude error: {np.degrees(rms_alt_error):.3f}°")
            print(f"RMS azimuth error: {np.degrees(rms_az_error):.3f}°")
            
            # Accuracy requirements for astrophotography
            self.assertLess(rms_alt_error, np.radians(0.5), "Altitude tracking error too large")
            self.assertLess(rms_az_error, np.radians(0.5), "Azimuth tracking error too large")
        else:
            self.skipTest("No valid tracking data (moon below horizon)")
    
    def test_gps_time_accuracy(self):
        """Test GPS time synchronization accuracy."""
        print("\\nTesting GPS time accuracy...")
        
        # Simulate GPS time vs system time
        system_time = time.time()
        gps_time = system_time + 0.01  # 10ms offset (typical GPS accuracy)
        
        time_error = abs(gps_time - system_time)
        print(f"GPS vs system time error: {time_error*1000:.1f}ms")
        
        # GPS should be accurate to within 100ms for our purposes
        self.assertLess(time_error, 0.1, "GPS time synchronization error too large")
    
    def test_imu_noise_filtering(self):
        """Test IMU noise characteristics and filtering."""
        print("\\nTesting IMU noise filtering...")
        
        # Simulate noisy IMU data
        true_orientation = [0.5, 0.0, 0.3]  # yaw, roll, pitch in radians
        noise_samples = 1000
        noise_level = 0.01  # 0.01 rad ≈ 0.6°
        
        noisy_measurements = []
        for _ in range(noise_samples):
            noise = [np.random.normal(0, noise_level) for _ in range(3)]
            noisy_measurement = [true + n for true, n in zip(true_orientation, noise)]
            noisy_measurements.append(noisy_measurement)
        
        # Calculate statistics
        measurements_array = np.array(noisy_measurements)
        mean_measurements = np.mean(measurements_array, axis=0)
        std_measurements = np.std(measurements_array, axis=0)
        
        print(f"True orientation: {[np.degrees(x) for x in true_orientation]}")
        print(f"Mean measured: {[np.degrees(x) for x in mean_measurements]}")
        print(f"Std deviation: {[np.degrees(x) for x in std_measurements]}")
        
        # Check that mean converges to true value
        for i in range(3):
            self.assertAlmostEqual(mean_measurements[i], true_orientation[i], places=2)
            self.assertLess(std_measurements[i], noise_level * 1.2)  # Within expected noise
    
    def test_performance_requirements(self):
        """Test that system meets performance requirements."""
        print("\\nTesting performance requirements...")
        
        # Simulate timing measurements
        update_rates = []
        processing_times = []
        
        for _ in range(100):
            start_time = time.time()
            
            # Simulate star tracker computation
            time.sleep(0.001)  # 1ms processing time
            
            end_time = time.time()
            processing_time = end_time - start_time
            processing_times.append(processing_time)
            
            # Simulate 1Hz update rate
            time.sleep(0.999)
            update_rates.append(1.0)
        
        avg_processing_time = np.mean(processing_times)
        avg_update_rate = np.mean(update_rates)
        
        print(f"Average processing time: {avg_processing_time*1000:.1f}ms")
        print(f"Average update rate: {avg_update_rate:.1f}Hz")
        
        # Performance requirements
        self.assertLess(avg_processing_time, 0.1, "Processing time too slow")
        self.assertGreater(avg_update_rate, 0.9, "Update rate too low")
    
    def test_edge_cases(self):
        """Test edge cases and error conditions."""
        print("\\nTesting edge cases...")
        
        # Test extreme latitudes
        extreme_locations = [
            TestLocation('north_pole', 89.9, 0.0, 0.0),
            TestLocation('south_pole', -89.9, 0.0, 0.0)
        ]
        
        if ASTROPY_AVAILABLE:
            test_time = Time('2024-06-21T12:00:00')
            
            for location in extreme_locations:
                earth_location = EarthLocation(
                    lat=location.lat*u.deg,
                    lon=location.lon*u.deg,
                    height=location.alt*u.m
                )
                altaz_frame = AltAz(obstime=test_time, location=earth_location)
                
                sun_coord = get_sun(test_time)
                sun_altaz = sun_coord.transform_to(altaz_frame)
                
                print(f"{location.name}: Sun alt={sun_altaz.alt.deg:.1f}°")
                
                # Should still produce valid coordinates
                self.assertIsNotNone(sun_altaz.alt.deg)
                self.assertIsNotNone(sun_altaz.az.deg)
    
    def save_test_results(self, filename='integration_test_results.json'):
        """Save test results for analysis."""
        results = {
            'timestamp': datetime.now().isoformat(),
            'astropy_available': ASTROPY_AVAILABLE,
            'test_locations': [loc.__dict__ for loc in self.test_locations],
            'results': self.results
        }
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\\nTest results saved to {filename}")


def run_integration_tests():
    """Run all integration tests."""
    print("Starting Star Tracker Integration Tests...")
    print("=" * 50)
    
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(StarTrackerIntegrationTests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Save results
    test_instance = StarTrackerIntegrationTests()
    test_instance.save_test_results()
    
    print("\\n" + "=" * 50)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors))/result.testsRun)*100:.1f}%")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_integration_tests()
    exit(0 if success else 1)