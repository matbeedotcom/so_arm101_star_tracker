#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header, String, Float64, Bool
import serial
import time
from datetime import datetime, timezone
import threading
import re


class GPSInterface(Node):
    """
    ROS2 interface for Adafruit Ultimate GPS v3 breakout board.
    Parses NMEA sentences and publishes GPS data for star tracking.
    """
    
    def __init__(self):
        super().__init__('gps_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')  # Default UART on RPi
        self.declare_parameter('baud_rate', 9600)  # GPS default baud rate
        self.declare_parameter('update_rate', 1.0)  # Hz for publishing
        self.declare_parameter('timeout', 2.0)  # Serial timeout
        self.declare_parameter('enable_pps', False)  # Use PPS signal
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.enable_pps = self.get_parameter('enable_pps').value
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.time_pub = self.create_publisher(TimeReference, 'gps/time', 10)
        self.speed_pub = self.create_publisher(Float64, 'gps/speed', 10)
        self.track_pub = self.create_publisher(Float64, 'gps/track', 10)
        self.satellites_pub = self.create_publisher(Float64, 'gps/satellites', 10)
        self.hdop_pub = self.create_publisher(Float64, 'gps/hdop', 10)
        self.status_pub = self.create_publisher(String, 'gps/status', 10)
        self.fix_status_pub = self.create_publisher(Bool, 'gps/has_fix', 10)
        
        # GPS data storage
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'speed': 0.0,
            'track': 0.0,
            'satellites': 0,
            'hdop': 99.0,
            'fix_quality': 0,
            'utc_time': None,
            'date': None,
            'has_fix': False
        }
        
        # Initialize serial connection
        self.serial_conn = None
        self.running = False
        
        if not self.init_gps():
            self.get_logger().error('Failed to initialize GPS')
            return
        
        # Start GPS reading thread
        self.gps_thread = threading.Thread(target=self.gps_reader_thread)
        self.gps_thread.daemon = True
        self.running = True
        self.gps_thread.start()
        
        # Timer for publishing GPS data
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_gps_data)
        
        self.get_logger().info(f'GPS interface initialized on {self.serial_port}')
    
    def init_gps(self):
        """Initialize GPS serial connection."""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.timeout
            )
            
            # Wait for GPS to initialize
            time.sleep(2)
            
            # Send initialization commands to GPS
            self.send_gps_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')  # Set NMEA sentences
            self.send_gps_command('PMTK220,1000')  # Set update rate to 1Hz
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'GPS initialization failed: {e}')
            return False
    
    def send_gps_command(self, command):
        """Send command to GPS with checksum."""
        if not self.serial_conn:
            return
        
        # Calculate NMEA checksum
        checksum = 0
        for char in command:
            checksum ^= ord(char)
        
        # Format and send command
        full_command = f'${command}*{checksum:02X}\r\n'
        self.serial_conn.write(full_command.encode())
        time.sleep(0.1)
    
    def gps_reader_thread(self):
        """Background thread to read GPS data."""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('$'):
                        self.parse_nmea_sentence(line)
                        
            except Exception as e:
                self.get_logger().warn(f'GPS read error: {e}')
                time.sleep(0.1)
    
    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentence and update GPS data."""
        try:
            parts = sentence.split(',')
            sentence_type = parts[0]
            
            if sentence_type == '$GPGGA':
                self.parse_gga(parts)
            elif sentence_type == '$GPRMC':
                self.parse_rmc(parts)
            elif sentence_type == '$GPGSV':
                self.parse_gsv(parts)
                
        except Exception as e:
            self.get_logger().debug(f'NMEA parse error: {e}')
    
    def parse_gga(self, parts):
        """Parse GGA sentence (fix data)."""
        try:
            if len(parts) < 15:
                return
            
            # UTC time
            if parts[1]:
                self.gps_data['utc_time'] = parts[1]
            
            # Latitude
            if parts[2] and parts[3]:
                lat = float(parts[2])
                lat_deg = int(lat / 100)
                lat_min = lat - (lat_deg * 100)
                latitude = lat_deg + (lat_min / 60.0)
                if parts[3] == 'S':
                    latitude = -latitude
                self.gps_data['latitude'] = latitude
            
            # Longitude
            if parts[4] and parts[5]:
                lon = float(parts[4])
                lon_deg = int(lon / 100)
                lon_min = lon - (lon_deg * 100)
                longitude = lon_deg + (lon_min / 60.0)
                if parts[5] == 'W':
                    longitude = -longitude
                self.gps_data['longitude'] = longitude
            
            # Fix quality
            if parts[6]:
                self.gps_data['fix_quality'] = int(parts[6])
                self.gps_data['has_fix'] = self.gps_data['fix_quality'] > 0
            
            # Number of satellites
            if parts[7]:
                self.gps_data['satellites'] = int(parts[7])
            
            # HDOP
            if parts[8]:
                self.gps_data['hdop'] = float(parts[8])
            
            # Altitude
            if parts[9]:
                self.gps_data['altitude'] = float(parts[9])
                
        except ValueError as e:
            self.get_logger().debug(f'GGA parse error: {e}')
    
    def parse_rmc(self, parts):
        """Parse RMC sentence (recommended minimum)."""
        try:
            if len(parts) < 12:
                return
            
            # Status (A = active, V = void)
            if parts[2] == 'A':
                self.gps_data['has_fix'] = True
            else:
                self.gps_data['has_fix'] = False
                return
            
            # Date
            if parts[9]:
                self.gps_data['date'] = parts[9]
            
            # Speed over ground (knots)
            if parts[7]:
                speed_knots = float(parts[7])
                self.gps_data['speed'] = speed_knots * 0.514444  # Convert to m/s
            
            # Track angle (degrees)
            if parts[8]:
                self.gps_data['track'] = float(parts[8])
                
        except ValueError as e:
            self.get_logger().debug(f'RMC parse error: {e}')
    
    def parse_gsv(self, parts):
        """Parse GSV sentence (satellites in view)."""
        try:
            if len(parts) < 4:
                return
            
            # This is a simplified parser - could be expanded for detailed satellite info
            pass
            
        except ValueError as e:
            self.get_logger().debug(f'GSV parse error: {e}')
    
    def publish_gps_data(self):
        """Publish GPS data to ROS topics."""
        stamp = self.get_clock().now().to_msg()
        
        # Publish NavSatFix
        fix_msg = NavSatFix()
        fix_msg.header.stamp = stamp
        fix_msg.header.frame_id = 'gps'
        
        fix_msg.latitude = self.gps_data['latitude']
        fix_msg.longitude = self.gps_data['longitude']
        fix_msg.altitude = self.gps_data['altitude']
        
        # Set status
        fix_msg.status.status = NavSatStatus.STATUS_FIX if self.gps_data['has_fix'] else NavSatStatus.STATUS_NO_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set covariance based on HDOP
        hdop = self.gps_data['hdop']
        if hdop < 2.0:
            covariance = 1.0  # Good accuracy
        elif hdop < 5.0:
            covariance = 4.0  # Moderate accuracy
        else:
            covariance = 16.0  # Poor accuracy
        
        fix_msg.position_covariance[0] = covariance  # East
        fix_msg.position_covariance[4] = covariance  # North
        fix_msg.position_covariance[8] = covariance * 4  # Up (typically worse)
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        self.fix_pub.publish(fix_msg)
        
        # Publish time reference
        if self.gps_data['utc_time'] and self.gps_data['date']:
            time_msg = TimeReference()
            time_msg.header.stamp = stamp
            time_msg.header.frame_id = 'gps'
            time_msg.source = 'gps'
            
            # Convert GPS time to Unix timestamp
            try:
                date_str = self.gps_data['date']
                time_str = self.gps_data['utc_time']
                
                # Parse date (DDMMYY)
                day = int(date_str[:2])
                month = int(date_str[2:4])
                year = 2000 + int(date_str[4:6])
                
                # Parse time (HHMMSS.sss)
                hour = int(time_str[:2])
                minute = int(time_str[2:4])
                second = int(float(time_str[4:]))
                microsecond = int((float(time_str[4:]) % 1) * 1000000)
                
                gps_datetime = datetime(year, month, day, hour, minute, second, microsecond, timezone.utc)
                time_msg.time_ref = gps_datetime.timestamp()
                
                self.time_pub.publish(time_msg)
                
            except ValueError as e:
                self.get_logger().debug(f'Time conversion error: {e}')
        
        # Publish other data
        speed_msg = Float64()
        speed_msg.data = self.gps_data['speed']
        self.speed_pub.publish(speed_msg)
        
        track_msg = Float64()
        track_msg.data = self.gps_data['track']
        self.track_pub.publish(track_msg)
        
        sat_msg = Float64()
        sat_msg.data = float(self.gps_data['satellites'])
        self.satellites_pub.publish(sat_msg)
        
        hdop_msg = Float64()
        hdop_msg.data = self.gps_data['hdop']
        self.hdop_pub.publish(hdop_msg)
        
        # Status messages
        status_msg = String()
        if self.gps_data['has_fix']:
            status_msg.data = f"GPS Fix: {self.gps_data['satellites']} sats, HDOP: {self.gps_data['hdop']:.1f}"
        else:
            status_msg.data = f"No GPS Fix: {self.gps_data['satellites']} sats visible"
        self.status_pub.publish(status_msg)
        
        fix_status_msg = Bool()
        fix_status_msg.data = self.gps_data['has_fix']
        self.fix_status_pub.publish(fix_status_msg)
    
    def destroy_node(self):
        """Clean up on node shutdown."""
        self.running = False
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join(timeout=2.0)
        
        if self.serial_conn:
            self.serial_conn.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()