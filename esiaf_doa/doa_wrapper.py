import pyroomacoustics as pra
from pyroomacoustics.doa import circ_dist
import numpy as np

# algorithms parameters
SNR = 0.    # signal-to-noise ratio
c = 343.    # speed of sound
fs = 16000  # sampling frequency
nfft = 256  # FFT size
freq_bins = np.arange(5, 60)  # FFT bins to use for estimation


def create_mic_config(config):
    mic_array = config['mic_array']
    xes = []
    ys = []
    for mic in mic_array:
        xes.append(mic_array[mic]['x'])
        ys.append(mic_array[mic]['y'])

    complete = np.array([xes, ys])
    return complete

class DOA:

    def __init__(self, config):
        algo_name = config['algorithm']
        self.mic_config = create_mic_config(config)
        self.doa = pra.doa.algorithms[algo_name](self.mic_config, fs, nfft, c=c, max_four=4)

    def _prepare_audio(self, audio):
        return audio

    def process_audio(self, audio):
        # prepare signal
        signal = self._prepare_audio(audio)

        # calculate doa
        self.doa.locate_sources(signal, freq_bins=freq_bins)

        # calculate doa in degrees
        doa = self.doa.azimuth_recon / np.pi * 180.

        return doa