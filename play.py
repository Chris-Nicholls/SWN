import sys
import time

import sounddevice as sd
import soundfile as sf

input_file = sys.argv[1] if len(sys.argv) > 1 else 'build/main.wav'

data, samplerate = sf.read(input_file, dtype='int16', always_2d=True)

duration = len(data) / samplerate
sd.play(data, samplerate=samplerate, device='External Headphones')

bar_width = 40
start = time.monotonic()
try:
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            elapsed = duration
        frac = elapsed / duration if duration > 0 else 1.0
        filled = int(bar_width * frac)
        bar = '#' * filled + '-' * (bar_width - filled)
        remaining = max(0.0, duration - elapsed)
        sys.stdout.write(
            f'\r[{bar}] {elapsed:6.1f}s / {duration:6.1f}s  (-{remaining:5.1f}s)'
        )
        sys.stdout.flush()
        if elapsed >= duration:
            break
        time.sleep(0.05)
    sd.wait()
finally:
    sys.stdout.write('\n')
    sys.stdout.flush()
