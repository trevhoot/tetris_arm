#!usr/bin/env python 

import pyaudio
import wave

chunk = 1024

f = wave.open(r"Original_Tetris_Theme_Song_A.wav","r")
song = f
p = pyaudio.PyAudio()

stream = p.open(format = p.get_format_from_width(f.getsampwidth()),
					channels = f.getnchannels(),
					rate = f.getframerate(),
					output = True)

data = f.readframes(chunk)

while(True):
	while data!= '':
		stream.write(data)
		data = f.readframes(chunk)
	f = wave.open(r"Original_Tetris_Theme_Song_A.wav","r")
	data = f.readframes(chunk)

p.terminate()