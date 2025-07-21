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
                    
                    data_values = line.split(',')
                    
                    if len(data_values) == 10:
                        print(f"Received: {line} at {timestamp}")
                        
                        row = [timestamp] + data_values
                        writer.writerow(row)
                        csvfile.flush() 
                    else:
                        print(f"Incomplete data received: {line}")
                        
    except KeyboardInterrupt:
        print("Data collection stopped")
    finally:
        arduino.close()