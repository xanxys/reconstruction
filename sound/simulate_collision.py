#!/bin/python3
import argparse
import numpy as np
import scipy.signal
import scipy.interpolate
import logging
import wave
import os
import os.path
import random
import itertools

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
    return samples


def split_collisions(sound, freq=44100, window_duration=0.01, output_figure=False):
    """
    sound: [-1, 1] float array sound
    result: array of normalized, individual collisions

    input sound must be sufficiently high quality (low noise when silent)
    for split_collisions to work.
    """
    n_window = int(window_duration * freq)
    assert(len(sound) > n_window)

    thresh_peak = 0.5
    thresh_cut_b = 0.05
    thresh_cut_e = 0.01
    margin_beginning = 0.005  # 5msec

    # find peak regions.
    sound = sound / np.max(np.abs(sound))
    peaks = np.convolve(
        np.abs(sound) > thresh_peak,
        np.ones([n_window]) / n_window) > 0
    peak_regions = []
    for (key, ixs) in itertools.groupby(enumerate(peaks), key=lambda x: x[1]):
        if not key:
            continue
        ix_begin = next(ixs)[0]
        ix_end = max(ixs)[0]
        if ix_begin < ix_end:
            peak_regions.append((ix_begin, ix_end))
    # extend regions forward and backward until sound level is too low.
    exist_b = np.convolve(
        np.abs(sound) > thresh_cut_b,
        np.ones([n_window]) / n_window) > 0
    exist_e = np.convolve(
        np.abs(sound) > thresh_cut_e,
        np.ones([n_window]) / n_window) > 0
    regions = []
    for (ix_peak_begin, ix_peak_end) in peak_regions:
        ix_begin = 0
        for ix in range(ix_peak_begin, -1, -1):
            if not exist_b[ix]:
                ix_begin = ix
                break
        # insert slight margin before beginning
        ix_begin = max(0, ix_begin - int(margin_beginning * freq))
        ix_end = len(sound) - 1
        for ix in range(ix_peak_end, len(sound)):
            if not exist_e[ix]:
                ix_end = ix
                break
        regions.append((ix_begin, ix_end))
    # merge overlapping regions
    if len(regions) == 0:
        logger.warn("No collision region found")
        return []
    regions_merged = []
    prev_region = regions[0]
    for (i0, i1) in regions[1:]:
        if i0 <= prev_region[1]:
            # overlapping -> merge
            prev_region = (
                min(i0, prev_region[0]),
                max(i1, prev_region[1]))
        else:
            # non-overlapping -> commit previous
            regions_merged.append(prev_region)
            prev_region = (i0, i1)
    regions_merged.append(prev_region)
    logger.info("%d regions found", len(regions_merged))

    # visualize regions
    if output_figure:
        import random
        import matplotlib.pyplot as plt
        reg_v = sound.copy() * 0
        for (i0, i1) in regions_merged:
            reg_v[i0:i1] = 0.5

        ts = np.arange(len(sound)) / freq
        fig, pl = plt.subplots(1, 1)
        fig.set_size_inches(6, 4)
        pl.plot(ts, sound)
        pl.plot(ts, reg_v)
        pl.set_xlabel("Time [s]")
        pl.set_ylabel("Amplitude")
        pl.grid()
        fig.tight_layout()
        fig.savefig("fig-collision-%02d.png" % random.randint(0, 99))

    # clip and normalize each region
    collisions = []
    for (i0, i1) in regions_merged:
        coll = sound[i0:i1]
        collisions.append(coll / np.max(np.abs(coll)))
    return collisions


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
    parser.add_argument(
        '--figure', action='store_true',
        help='Output figures of sounds and detected regions.')
    parser.add_argument(
        '--gen', type=str, default=None,
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
        coll_freq = 50.0  # avg. collision/sec
        duration = 10.0
        events = []
        # generate random events
        for i in range(int(coll_freq * duration)):
            t = random.random() * duration
            events.append(
                (t, 1 / coll_freq, assets[random.randint(0, len(assets) - 1)]))

        # mix events into single sound stream
        n_samples = int(base_freq * duration)
        samples = np.zeros([n_samples], dtype=np.float32)
        for (offset, scale, data) in events:
            i_offset = int(offset * base_freq)
            n_copy = min(len(data), n_samples - i_offset)
            samples[i_offset:i_offset+n_copy] += data[:n_copy] * scale

        logger.info("writing soundscape to %s", args.simulate)
        write_wave_asset(args.simulate, samples)
    elif args.gen is not None:
        sounds = []
        for e in os.listdir(args.sound_assets):
            path = os.path.join(args.sound_assets, e)
            logger.info('Reading wav from %s', path)
            asset = read_wave_asset(path, freq=base_freq)
            sounds += split_collisions(asset, output_figure=args.figure)

        logger.info("%d total collisions found", len(sounds))
        for (i, sound) in enumerate(sounds):
            write_wave_asset(
                "%s/collision-%d.wav" % (args.gen, i + 1),
                sound)
