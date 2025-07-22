import serial
import wave
import time
import speech_recognition as sr
import struct

# Serial settings
ser = serial.Serial('COM3', 115200)  # Replace with your port
sample_rate = 16000
channels = 1
sampwidth = 2  # 2 bytes (16-bit)

filename = 'recorded_audio.wav'
image_filename = 'captured.jpg'

print("Recording... Press Ctrl+C to stop.")

# Record and save WAV
with wave.open(filename, 'wb') as wf:
    wf.setnchannels(channels)
    wf.setsampwidth(sampwidth)
    wf.setframerate(sample_rate)

    try:
        while True:
            data = ser.read(512)
            wf.writeframes(data)
    except KeyboardInterrupt:
        print("\nRecording finished.")
        ser.reset_input_buffer()  # Clear any junk before image
        pass

# --- IMAGE CAPTURE ---
print("Capturing image...")
ser.write(b'CAPTURE\n')

# Wait for 'IMG\n' marker
marker = b''
while marker != b'IMG\n':
    marker = ser.readline()
    if marker == b'':
        print("Timeout or no response.")
        exit()

# Read 4 bytes for image size
size_bytes = ser.read(4)
if len(size_bytes) != 4:
    print("Failed to read image size.")
    exit()

image_size = struct.unpack('<I', size_bytes)[0]
print(f"Image size: {image_size} bytes")

# Read the image data
image_data = bytearray()
while len(image_data) < image_size:
    chunk = ser.read(image_size - len(image_data))
    if not chunk:
        print("Error: Timeout or disconnection.")
        break
    image_data.extend(chunk)

# Save image
with open(image_filename, 'wb') as img_file:
    img_file.write(image_data)

print("Image saved as:", image_filename)

# --- SPEECH TO TEXT ---
print("Converting speech to text...")

recognizer = sr.Recognizer()
with sr.AudioFile(filename) as source:
    audio_data = recognizer.record(source)

try:
    text = recognizer.recognize_google(audio_data)
    print("Recognized Speech:\n", text)
except sr.UnknownValueError:
    print("Could not understand audio.")
except sr.RequestError as e:
    print("Google Speech Recognition error:", e)
