import wave
import numpy as np

samplerate = 8000
# samplerate = 48000

data = []
for i in range(20000):
    data += [i % (2**16)] + [0] * ((samplerate // 1000) - 1)

# Create a saw wave and convert to (little-endian) 16 bit integers.
audio = np.array(data).astype("<h")

with wave.open("sound1.wav", "w") as f:
    # 1 Channel
    f.setnchannels(1)
    # 2 bytes per sample.
    f.setsampwidth(2)
    f.setframerate(samplerate)
    f.writeframes(audio.tobytes())