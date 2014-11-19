#!/bin/python3
import argparse
import numpy as np
import scipy.signal
import logging
import json
import wave

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def convert_bcj_to_sound():
    """
    Read BCJ acceleration data from disk (hard coded path)
    and writes sound to yet another hard-coded path.

    TODO: needs refactoring
    TODO: sound synthesis logic must be separated from IO
    """
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


class AccelerogramDataParser86(object):
    """
    Parse text format acceleration log data.
    (column-oriented)
    Only applicable to bcj.or.jp 1986 dataset.

    TODO: There certainly should be a name for this format.

    They look like:
    FILE ....            CC00021001
    40.0001.00.400518S90W-0117 U.S.A.       ...            CC00021002
    ...
    """
    def __init__(self, fobj):
        """
        Parse given file-like object.
        Use other methods to access parsed data.

        last 8 chars: some kind of sequential integr

        Line := Content TypeTag SeqId
        TypeTag := "CC" | "IC" | "FC" | " "
        SeqId := Int(8-char)

        A, V, D are probably acceleration, velocity, displacement.
        I'm not sure which is the original measured data.

        Data are contained sequentially, like
        0 1 2 3 4 "A" SeqId
        5 6 7 8   "A" SeqId
        """
        data_a = []
        data_v = []
        data_d = []
        for line in fobj:
            line = line.rstrip()
            # A proper line has 80 chars.
            if len(line) != 80:
                continue

            content = line[:-10]
            data_type = line[-10:-8]
            if data_type == 'A ':
                data_a += map(float, content.split())
            elif data_type == 'V ':
                data_v += map(float, content.split())
            elif data_type == 'D ':
                data_d += map(float, content.split())
            else:
                pass
        self.data_acc = np.array(data_a)
        self.data_vel = np.array(data_v)
        self.data_dis = np.array(data_d)
        logger.info(
            "Loaded earthquake data #V:%d #A:%d #D:%d",
            len(self.data_acc), len(self.data_vel), len(self.data_dis))

if __name__ == '__main__':
    parser = AccelerogramDataParser86(open(
        '/data/research/2014/earthquake/1986/86-Elcentew.txt'))
