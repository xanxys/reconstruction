#!/bin/python3
import argparse
import numpy as np
import scipy.signal
import json
import wave

to_frequency = 44100

data = json.load(open('bcj_l1.json'))

from_frequency = data['frequency']
from_samples = np.array(data['data'])

# acc
to_samples = scipy.signal.resample(
	from_samples,
	int(to_frequency / from_frequency * len(from_samples)))

def integrate(xs):
	v = 0
	dt = 1 / to_frequency
	vs = []
	for x in xs:
		v += x * dt
		vs.append(v)
	return np.array(vs)

# acc -> pos
to_samples = integrate(integrate(to_samples))


# Apply 40-Hz HPF to maximize audible loudness
b, a = scipy.signal.butter(
	5,
	40 / (to_frequency / 2),
	'highpass')

to_samples = scipy.signal.lfilter(b, a, to_samples)



wav = wave.open('output.wav', mode='w')
wav.setnchannels(1)
wav.setsampwidth(2)
wav.setframerate(to_frequency)
wav.writeframes((to_samples / np.abs(to_samples).max() * ((2**15) - 1)).astype(np.int16))

