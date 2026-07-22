import serial
import time
import csv
import os

# --- CONFIGURATION ---
# Change 'COM3' to your actual Arduino port (e.g., 'COM5' on Windows or '/dev/cu.usbserial-...' on Mac)
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

def record_exercise_session():
    # 1. Get the label name from the user to create a clean file structure
    exercise_name = input("Enter exercise label (e.g., bicep_curl, shoulder_press, lateral_raise): ").strip().lower()
    set_number = input("Enter set number (e.g., 1, 2, 3): ").strip()
    
    filename = f"{exercise_name}_set{set_number}.csv"
    
    print(f"\n[Connecting] Opening connection to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Give the hardware a moment to reset cleanly
        print("[Connected] Ready to log.")
    except Exception as e:
        print(f"[Error] Could not open serial port: {e}")
        return

    print("\n" + "="*50)
    print(f" READY TO LOG DATA FOR: {exercise_name.upper()} (Set {set_number})")
    print(" Press CTRL+C at any time to stop recording and save the file.")
    print("="*50 + "\n")
    
    input("Press ENTER when you are ready to start lifting...")
    print("\n>>> RECORDING STARTED! Live streaming to CSV... >>>\n")
    
    # 2. Open file and establish standard Edge Impulse headers
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Headers matching your 6 discrete features
        writer.writerow(["accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ"])
        
        row_count = 0
        try:
            ser.reset_input_buffer() # Clear old buffer data before starting
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Ensure it's a comma-separated data line, ignoring string logs
                    parts = line.split(',')
                    if len(parts) == 6:
                        try:
                            # Validate they are floating numbers
                            floats = [float(x) for x in parts]
                            writer.writerow(floats)
                            row_count += 1
                            
                            # Print a rolling update so you know it's capturing data
                            if row_count % 50 == 0:
                                print(f" Captured {row_count} data points...")
                        except ValueError:
                            # Skip lines that fail validation (e.g., raw print messages)
                            continue
                            
        except KeyboardInterrupt:
            # 3. Graceful exit on CTRL+C
            print("\n\n>>> STOPPED RECORDING! >>>")
            print(f"[Saved] Successfully wrote {row_count} rows to: {os.path.abspath(filename)}")
            
        finally:
            ser.close()

if __name__ == "__main__":
    record_exercise_session()