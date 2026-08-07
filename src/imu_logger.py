import serial
import time
import csv
import os

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

def record_exercise_session():
    exercise_name = input("Enter exercise label (e.g., bicepCurl, lateralRaise): ").strip()
    set_number = input("Enter set number (e.g., 1, 2, 3): ").strip()
    
    target_dir = os.path.join("training data", exercise_name)
    os.makedirs(target_dir, exist_ok=True)
    
    filename = os.path.join(target_dir, f"{exercise_name}_set{set_number}.csv")
    
    print(f"\n[Connecting] Opening connection to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) 
        print("[Connected] Ready to log.")
    except Exception as e:
        print(f"[Error] Could not open serial port: {e}")
        return

    print("\n" + "="*50)
    print(f" READY TO LOG DATA FOR: {exercise_name} (Set {set_number})")
    print(f" Target File: {filename}")
    print(" Press CTRL+C at any time to stop recording and save.")
    print("="*50 + "\n")
    
    input("Press ENTER when you are ready to start lifting...")
    print("\n>>> RECORDING STARTED! Live streaming to CSV... >>>\n")
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ", "pitch", "roll"])
        
        row_count = 0
        try:
            ser.reset_input_buffer()
            while True:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    parts = line.split(',')
                    if len(parts) == 8:
                        try:
                            floats = [float(x) for x in parts]
                            writer.writerow(floats)
                            row_count += 1
                            
                            if row_count % 50 == 0:
                                print(f" Captured {row_count} data points...")
                        except ValueError:
                            continue
                            
        except KeyboardInterrupt:
            print("\n\n>>> STOPPED RECORDING! >>>")
            print(f"[Saved] Successfully wrote {row_count} rows to: {os.path.abspath(filename)}")
            
        finally:
            ser.close()

if __name__ == "__main__":
    record_exercise_session()
