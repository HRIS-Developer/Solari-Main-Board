import serial
import struct
import time

# Set the correct serial port and baud rate
SERIAL_PORT = 'COM3'  # Change to your port (e.g., /dev/ttyUSB0 on Linux/macOS)
BAUD_RATE = 115200

def capture_image():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5) as ser:
            time.sleep(2)  # Wait for the ESP32 to reset

            # Send 'capture' command
            ser.write(b'capture\n')
            print("Sent 'capture' command")

            # Wait for 'IMG' sync header
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if line == "IMG":
                    print("IMG header received")
                    break
                else:
                    print(f"Waiting for IMG header... got: {line}")

            # Read 4 bytes as image length (little-endian unsigned int)
            length_bytes = ser.read(4)
            img_len = struct.unpack('<I', length_bytes)[0]
            print(f"Image length: {img_len} bytes")

            # Read the image data
            image_data = b''
            while len(image_data) < img_len:
                chunk = ser.read(img_len - len(image_data))
                if not chunk:
                    print("Timeout or serial error")
                    return
                image_data += chunk

            # Save image to file
            filename = f"captured.jpg"
            with open(filename, 'wb') as f:
                f.write(image_data)

            print(f"Image saved as '{filename}'")

    except Exception as e:
        print("Error:", e)

# Run it
capture_image()
