import serial
import csv
import time
from datetime import datetime

arduino = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2) 

csv_filename = 'drone_data.csv'
with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Timestamp', 'Roll', 'Roll_PID', 'Pitch', 'Pitch_PID', 
                     'Motor_FL', 'Motor_FR', 'Motor_RL', 'Motor_RR', 'ROLL_P', 'ROLL_I', 'ROLL_D'])
    
    try:
        while True:
            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8').strip()
                
                if line:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    # Fix the data parsing issue
                    # Replace problematic patterns where numbers are concatenated
                    line_fixed = line.replace('20.00-0.21', '20.00,-0.21')
                    line_fixed = line_fixed.replace('0.05,-25.76', '0.05,-25.76')
                    
                    data_values = line_fixed.split(',')
                    
                    # Changed from 8 to 11 to match your CSV header (11 data columns)
                    if len(data_values) == 11:
                        print(f"Received: {line} at {timestamp}")
                        
                        row = [timestamp] + data_values
                        writer.writerow(row)
                        csvfile.flush() 
                    else:
                        print(f"Incomplete data received ({len(data_values)} values): {line}")
                        print(f"Parsed values: {data_values}")
                        
    except KeyboardInterrupt:
        print("Data collection stopped")
    finally:
        arduino.close()