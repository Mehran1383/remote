import serial
import time
from ubx.ubx import UBX

# Configuration
PORT = '/dev/ttyACM0'  # Change this to your actual serial port
BAUDRATE = 38400       # Common baudrate for ZED-F9R

# UBX message to set RTK Fixed mode (DGNSS configuration)
RTK_FIXED_CONFIG = bytes.fromhex(
    'B5 62 06 24 24 00 00 00 03 00 00 00 00 00 00 00'
    '00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00'
    '00 00 00 00 00 00 00 00 00 00 00 00 5A 63'
)

# UBX message to save configuration to flash/battery backed RAM
SAVE_CONFIG = bytes.fromhex(
    'B5 62 06 09 0D 00 00 00 00 00 00 00 00 00 FF FF'
    '00 00 03 1B 9A'
)

def configure_rtk_fixed():
    try:
        # Open serial connection
        ser = serial.Serial(PORT, BAUDRATE, timeout=5)
        print(f"Connected to {PORT} at {BAUDRATE} baud")
        
        # Create UBX parser (optional, for verification)
        ubx_parser = UBX()
        
        # Send RTK Fixed configuration
        print("Configuring RTK Fixed mode...")
        ser.write(RTK_FIXED_CONFIG)
        time.sleep(0.1)
        
        # Verify the message was sent correctly
        if ser.out_waiting == 0:
            print("RTK Fixed configuration sent successfully")
        else:
            print("Warning: Not all bytes were sent")
        
        # Save configuration to flash/battery backed RAM
        print("Saving configuration...")
        ser.write(SAVE_CONFIG)
        time.sleep(0.1)
        
        # Verification
        if ser.out_waiting == 0:
            print("Configuration saved successfully")
        else:
            print("Warning: Not all bytes were sent")
        
        # Close connection
        ser.close()
        print("Configuration complete")
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    configure_rtk_fixed()