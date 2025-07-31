import serial
import time

def configure_rtk_fixed(port='/dev/ttyACM0', baudrate=38400):
    """
    Configure ZED-F9R receiver for RTK Fixed mode
    
    Args:
        port (str): Serial port name (e.g., '/dev/ttyACM0' or 'COM3')
        baudrate (int): Baud rate (typically 38400 or 115200)
    """
    # UBX protocol header and checksum for RTK Fixed configuration
    # This is the raw UBX message for CFG-DGNSS with mode=3 (RTK Fixed)
    rtk_fixed_config = bytes.fromhex(
        'B5 62 06 24 24 00 00 00 03 00 00 00 00 00 00 00'
        '00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00'
        '00 00 00 00 00 00 00 00 00 00 00 00 5A 63'
    )
    
    # UBX message to save configuration to non-volatile memory
    save_config = bytes.fromhex(
        'B5 62 06 09 0D 00 00 00 00 00 00 00 00 00 FF FF'
        '00 00 03 1B 9A'
    )
    
    try:
        # Open serial connection
        with serial.Serial(port, baudrate, timeout=5) as ser:
            print(f"Connected to {port} at {baudrate} baud")
            
            # Send RTK Fixed configuration
            print("Configuring RTK Fixed mode...")
            ser.write(rtk_fixed_config)
            time.sleep(0.5)  # Wait for command to process
            
            # Save configuration
            print("Saving configuration...")
            ser.write(save_config)
            time.sleep(0.5)
            
            print("Configuration complete!")
            
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Update these values to match your setup
    configure_rtk_fixed(port='/dev/ttyACM0', baudrate=38400)