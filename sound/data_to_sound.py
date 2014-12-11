#!/bin/python3
import argparse
import numpy as np
import scipy.signal
import scipy.interpolate
import logging
import json
import wave

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def resample_linear(samples, freq_src, freq_dst):
    """
    Resample using lienar interpolation.
    You need to apply LPF before downsampling to prevent aliases.
    """
    n_src = len(samples)
    n_dst = int(n_src * freq_dst / freq_src)
    fn = scipy.interpolate.interp1d(
        np.arange(n_src) / freq_src,
        samples,
        kind='linear',
        bounds_error=False,
        fill_value=0)
    return fn(np.arange(n_dst) / freq_dst)


def convert_acc_to_sound(acc3d, s_freq, path):
    """
    Convert 1-d acceleration to sound.
    return: None

    WAVE object at 44100 Hz, 16-bit, 1-ch will be written to
    path
    """
    to_frequency = 44100

    accum_sound = None
    for axis in range(1):
        acc = acc3d[:, axis].copy()
        acc -= np.mean(acc)  # mean sub

        # acc
        to_samples = resample_linear(
            acc, s_freq, to_frequency)
        print("NS: %d" % len(to_samples))

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

        # Apply  HPF to maximize audible loudness
        b, a = scipy.signal.butter(
            5,
            40 / (to_frequency / 2),
            'highpass')

        to_samples = scipy.signal.lfilter(b, a, to_samples)
        if accum_sound is None:
            accum_sound = to_samples
        else:
            accum_sound += to_samples

    accum_sound = accum_sound[:int(len(accum_sound) * 0.95)]
    print("#smp: %d" % len(accum_sound))

    wav = wave.open(path, mode='w')
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(to_frequency)

    # normalize
    accum_sound = accum_sound / np.abs(accum_sound).max()

    wav.writeframes(
        (accum_sound * ((2**15) - 1)).astype(np.int16).tostring())
    wav.close()



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
    to_samples = resample_linear(
        from_samples, from_frequency, to_frequency)

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

    wav = wave.open('bcj.wav', mode='w')
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


def load_94Hachi():
    """
    Unit: gal = cm/s^2
    sample freq: 50 Hz
    """
    def get_acc(fobj):
        ls = []
        for l in fobj:
            ls.append(l)
        val_size = 7
        data = []
        for l in ls[5:]:
            if len(l) != 81:
                continue
            for i in range(10):
                data.append(float(l[i * val_size:(i + 1) * val_size]))
        return np.array(data)

    base = '/data/research/2014/earthquake/1994/94-Hachi-'
    accs = []
    for suffix in ['ew.txt', 'ns.txt', 'ud.txt']:
        p = base + suffix
        accs.append(get_acc(open(p)))

    acc_3d = np.array(accs).T
    print("Acc: shape=%s" % str(acc_3d.shape))
    return acc_3d


if __name__ == '__main__':
    acc3d = load_94Hachi()
    wav = convert_acc_to_sound(acc3d, 50, "Hachi.wav")

    acc_pack = {
        "freq": "50",
        "comment": "Hachi",
        "accel": [list(map(float, v)) for v in acc3d]
    }
    json.dump(acc_pack, open("Hachi.json", "w"))


    convert_bcj_to_sound()
    # parser_ew = AccelerogramDataParser86(open(
    #     '/data/research/2014/earthquake/1986/86-Elcentew.txt'))
    # parser_ns = AccelerogramDataParser86(open(
    #     '/data/research/2014/earthquake/1986/86-Elcentns.txt'))
