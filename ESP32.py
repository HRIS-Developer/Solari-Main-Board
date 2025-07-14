import serial
import wave
import time
import speech_recognition as sr

# Serial settings
ser = serial.Serial('COM3', 115200)  # Replace COM3 with your port

# WAV file settings
sample_rate = 16000
channels = 1
sampwidth = 2  # 2 bytes (16-bit)

filename = 'recorded_audio.wav'
print("Recording... Press Ctrl+C to stop.")

# Record from serial and save to WAV file
with wave.open(filename, 'wb') as wf:
    wf.setnchannels(channels)
    wf.setsampwidth(sampwidth)
    wf.setframerate(sample_rate)

    try:
        while True:
            data = ser.read(512)  # read 256 samples (512 bytes)
            wf.writeframes(data)
    except KeyboardInterrupt:
        print("\nRecording stopped.")
        ser.close()

# --- SPEECH-TO-TEXT ---
print("Converting speech to text...")

# Load the WAV and recognize
recognizer = sr.Recognizer()
with sr.AudioFile(filename) as source:
    audio_data = recognizer.record(source)

try:
    # Use Google Web Speech API
    text = recognizer.recognize_google(audio_data)
    print("Recognized Speech:\n", text)
except sr.UnknownValueError:
    print("Could not understand audio.")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
