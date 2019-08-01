import pyroomacoustics as pra
import numpy as np
import matplotlib.pyplot as plt
import threading
import time

# algorithms parameters
SNR = 3.    # signal-to-noise ratio
fs = 16000  # sampling frequency
nfft = 256  # FFT size
freq_range = [10, 10000]  # frequency range over which to perform doa


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

        spatial_resp = [0.0 for _ in range(360)]
        self.c_dirty_img = np.r_[spatial_resp, spatial_resp[0]]

        self.lock = threading.Lock()

        def plot_thread():
            plt.ion()
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='polar')
            phi_plt = self.doa.grid.azimuth
            c_phi_plt = np.r_[phi_plt, phi_plt[0]]
            line1, = ax.plot(c_phi_plt, 1. + 10. * self.c_dirty_img, linewidth=3,
                    alpha=0.55, linestyle='-',
                    label="spatial spectrum")

            while True:
                with self.lock:
                    line1.set_ydata(self.c_dirty_img)
                fig.canvas.draw()
                fig.canvas.flush_events()
                time.sleep(0.1)

        t = threading.Thread(target=plot_thread)
        t.start()



    @staticmethod
    def _preprocessing(audio):
        X = np.array([pra.stft(signal, nfft, nfft // 2, transform=np.fft.rfft).T for signal in audio])
        return X

    def process_audio(self, audio):
        # bring audio to frequency domain
        X = self._preprocessing(audio)

        # calculate doa
        self.doa.locate_sources(X, freq_range=freq_range)

        # calculate doa in degrees
        doa = self.doa.azimuth_recon / np.pi * 180.
        spatial_resp = self.doa.grid.values
        min_val = spatial_resp.min()
        max_val = spatial_resp.max()
        spatial_resp = (spatial_resp - min_val) / (max_val - min_val)

        with self.lock:
            self.c_dirty_img = np.r_[spatial_resp, spatial_resp[0]]

        return doa[0]