import serial

ssid = input("Enter SSID: ")
password = input("Enter Password: ")

combined = f"{ssid},{password}\n"

with serial.Serial('COM3', 115200, timeout=2) as ser:  # Adjust COM port
    ser.write(combined.encode())
    while True:
        line = ser.readline().decode().strip()
        if line:
            print(line)
