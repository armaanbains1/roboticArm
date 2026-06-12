import serial
import time

SERIAL_PORT = 'COM3'  
BAUD_RATE = 115200

try:
    esp32 = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    time.sleep(2) 
except Exception as e:
    print(f"Could not open serial port {SERIAL_PORT}: {e}")
    esp32 = None

def writeSerial(x, y, z=10.0, command=' '):

    if esp32 is None or not esp32.is_open:
        print("Serial port not open. Cannot send data.")
        return

    try:

        payload = f"{x:.2f},{y:.2f},{z:.2f} {command}\n"
        
        esp32.write(payload.encode('utf-8'))
        x = 600 - x
        if (x>300):
            x = x-300
            x = x*(-1)

        print(x/10)
        print(y/10)
        print(z)

        print(f"Sent to ESP32: {payload.strip()}")
        
    except Exception as e:
        print(f"Failed to write to serial: {e}")
