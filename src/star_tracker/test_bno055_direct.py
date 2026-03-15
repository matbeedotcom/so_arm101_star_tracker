#!/usr/bin/env python3

"""Direct test of BNO055 I2C communication"""

import smbus2 as smbus
import time
import struct

# BNO055 I2C address
I2C_ADDRESS = 0x28
I2C_BUS = 1

# BNO055 Registers
CHIP_ID_REG = 0x00
CHIP_ID = 0xA0
OPR_MODE_REG = 0x3D
EULER_H_LSB = 0x1A
CALIB_STAT = 0x35
SYS_TRIGGER = 0x3F

# Operation modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF = 0x0C

def test_bno055():
    """Test BNO055 communication and read euler angles."""
    try:
        # Initialize I2C
        bus = smbus.SMBus(I2C_BUS)
        print(f"Opened I2C bus {I2C_BUS}")

        # Check chip ID
        chip_id = bus.read_byte_data(I2C_ADDRESS, CHIP_ID_REG)
        print(f"Chip ID: 0x{chip_id:02x} (expected 0x{CHIP_ID:02x})")

        if chip_id != CHIP_ID:
            print("Warning: Unexpected chip ID!")

        # Read current operation mode
        op_mode = bus.read_byte_data(I2C_ADDRESS, OPR_MODE_REG)
        print(f"Current operation mode: 0x{op_mode:02x}")

        # Set to config mode
        bus.write_byte_data(I2C_ADDRESS, OPR_MODE_REG, OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        print("Set to CONFIG mode")

        # Set to NDOF mode (9-DOF fusion)
        bus.write_byte_data(I2C_ADDRESS, OPR_MODE_REG, OPERATION_MODE_NDOF)
        time.sleep(0.025)
        print("Set to NDOF mode (9-DOF fusion)")

        # Wait for sensor to stabilize
        time.sleep(0.5)

        print("\nReading euler angles (press Ctrl+C to stop):")
        print("Heading(Yaw) | Roll | Pitch")
        print("-" * 40)

        while True:
            try:
                # Read 6 bytes of euler data
                data = bus.read_i2c_block_data(I2C_ADDRESS, EULER_H_LSB, 6)

                # Convert to euler angles (in degrees)
                heading = struct.unpack('<h', bytes(data[0:2]))[0] / 16.0
                roll = struct.unpack('<h', bytes(data[2:4]))[0] / 16.0
                pitch = struct.unpack('<h', bytes(data[4:6]))[0] / 16.0

                # Read calibration status
                calib = bus.read_byte_data(I2C_ADDRESS, CALIB_STAT)
                sys_calib = (calib >> 6) & 0x03
                gyro_calib = (calib >> 4) & 0x03
                accel_calib = (calib >> 2) & 0x03
                mag_calib = calib & 0x03

                print(f"{heading:7.1f}° | {roll:7.1f}° | {pitch:7.1f}° | Calib: S{sys_calib} G{gyro_calib} A{accel_calib} M{mag_calib}", end='\r')

                time.sleep(0.1)

            except Exception as e:
                print(f"\nError reading sensor: {e}")
                break

    except Exception as e:
        print(f"Failed to initialize BNO055: {e}")
        return False

    finally:
        try:
            bus.close()
        except:
            pass

if __name__ == "__main__":
    test_bno055()