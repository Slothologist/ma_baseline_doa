import pyroomacoustics as pra
import numpy as np

# algorithms parameters
SNR = 3.    # signal-to-noise ratio
fs = 16000  # sampling frequency
nfft = 256  # FFT size
freq_range = [300, 3500]  # frequency range over which to perform doa


def create_mic_config(config):
    mic_array = config['mic_array']
    xes = []
    ys = []
    for channel in ['channel_' + str(x) for x in range(1, 5)]:
        xes.append(mic_array[channel]['x'])
        ys.append(mic_array[channel]['y'])

    complete = np.array([xes, ys])
    return complete


class DOA:

    def __init__(self, config):
        self.algo_name = config['algorithm']
        self.mic_config = create_mic_config(config)
        self.doa = pra.doa.algorithms[self.algo_name](self.mic_config, fs, nfft, num_src=1, max_four=4)

    @staticmethod
    def _preprocessing(audio):
        X = np.array([pra.stft(signal, nfft, nfft // 2, transform=np.fft.rfft).T for signal in audio])
        return X

    def process_audio(self, audio):
        # bring audio to frequency domain
        # freq = self._calc_freq_domain(audio)
        X = self._preprocessing(audio)

        # calculate doa
        self.doa.locate_sources(X, freq_range=freq_range)

        # calculate doa in degrees
        doa = self.doa.azimuth_recon / np.pi * 180.

        return doa[0]