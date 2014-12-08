#!/bin/python3
import argparse
import numpy as np
import scipy.signal
import scipy.interpolate
import logging
import json
import wave
import os
import os.path

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    cutoff_normalized = cutoff / nyq
    b, a = scipy.signal.butter(
        order, cutoff_normalized,
        btype='lowpass', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    """
    cutoff: cutoff frequency / Hz
    fs: sampling frequency / Hz
    """
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = scipy.signal.lfilter(b, a, data)
    return y


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


def write_wave_asset(path, samples, freq=44100):
    """
    Dump 16-bit monoral WAVE to path.
    sample value range is assumed to be [-1, 1] (values outside
    this range will be clamped)
    """
    wav = wave.open(path, mode='wb')
    wav.setnchannels(1)
    wav.setframerate(freq)
    wav.setsampwidth(2)
    samples = samples * (2**15)
    samples = np.minimum(samples, 2 ** 15 - 1)
    samples = np.maximum(samples, - 2 ** 15)
    wav.writeframes(samples.astype(np.int16))


def read_wave_asset(path, freq=44100):
    """
    Returns: single-channel, [-1, 1] float samples at freq.
    """
    wav = wave.open(path, mode='rb')
    logger.info("#ch=%d freq=%d %d byte/sample" % (
        wav.getnchannels(), wav.getframerate(), wav.getsampwidth()))
    if wav.getsampwidth() <= 1:
        raise RuntimeError("Sample size must be >= 2 byte/sample")
    # Convert to sequence of int16.
    blob = wav.readframes(wav.getnframes())
    freq_src = wav.getframerate()
    samples = (
        np.frombuffer(blob, 'b')
        .reshape(-1, wav.getsampwidth())[:, wav.getsampwidth()-2:]
        .flatten()
        .view('i2'))
    # choose first channel
    samples = samples.reshape(-1, wav.getnchannels())[:, 0]
    # normalize to [-1, 1] float
    samples = samples.astype(np.float32) / (2 ** 15)
    # cut high-freq components to prevent alias, when downsampling.
    if freq < freq_src:
        samples = butter_lowpass_filter(samples, freq / 2, freq_src)
    if freq != freq_src:
        samples = resample_linear(samples, freq_src, freq)
    #return samples
    # test output
    write_wave_asset("test-" + os.path.basename(path), samples, freq)

    return samples


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="""
Simulate random collision soundscape from individual sound assets.
Each sound asset must be a mostly silent wave file, containing
one or more distinctive "hits". Hits are automatically separated.

If a sound asset is stereo, only the first channel will be used.
""",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--sound-assets', required=True, type=str,
        help='Directory path containing wav sounds.')
    parser.add_argument(
        '--simulate', type=str, default=None,
        help='Output wave file path for soundscape simulation.')

    args = parser.parse_args()

    base_freq = 44100
    assets = []
    for e in os.listdir(args.sound_assets):
        path = os.path.join(args.sound_assets, e)
        logger.info('Reading wav from %s', path)
        assets.append(read_wave_asset(path, freq=base_freq))

    if args.simulate is not None:
        logger.info("Collision soundscape generation mode")
