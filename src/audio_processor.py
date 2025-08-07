#!/usr/bin/env python3

import sounddevice as sd
import whisper
from scipy.io.wavfile import write


class AudioProcessor:
    def __init__(self, model_size="small"):
        self.model = whisper.load_model(model_size)
    
    def record_audio(self, filename="output.wav", duration=5, fs=16000):
        print(f"Recording for {duration} seconds")
        recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
        sd.wait()
        write(filename, fs, recording)
        print("Recording finished")
        return filename
    
    def transcribe_audio(self, audio_file, language="hr"):
        result = self.model.transcribe(audio_file, language=language)
        print(f"Result text: {result['text']}")
        return result["text"]
    
    def record_and_transcribe(self, filename="output.wav", duration=5, fs=16000, language="hr"):
        self.record_audio(filename, duration, fs)
        return self.transcribe_audio(filename, language) 