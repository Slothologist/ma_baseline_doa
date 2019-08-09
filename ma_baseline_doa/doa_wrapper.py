import pyroomacoustics as pra
import numpy as np
import math
import_successful = None
try:
    import matplotlib.pyplot as plt
    import threading
    import time
except:
    import_successful = False
else:
    import_successful = True


# algorithms parameters
SNR = 25.    # signal-to-noise ratio
fs = 16000  # sampling frequency
nfft = 256  # FFT size
freq_range = [4000, 8000]  # frequency range over which to perform doa


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

    def __init__(self, config, plot=False):
        self.algo_name = config['algorithm']
        self.mic_config = create_mic_config(config)
        self.doa = pra.doa.algorithms[self.algo_name](self.mic_config, fs, nfft, num_src=1, max_four=4)
        self.plot = plot
        self.noise_matrix = np.array([1.0 for _ in range(360)])

        if self.plot and import_successful:
            spatial_resp = [0.0 for _ in range(360)]
            self.c_dirty_img = np.r_[spatial_resp, spatial_resp[0]]
            self.magnitude = [0. for _ in range(nfft//2)]

            self.lock = threading.Lock()

            def plot_thread():
                plt.ion()
                fig = plt.figure()
                ax = fig.add_subplot(211, projection='polar')
                phi_plt = self.doa.grid.azimuth
                c_phi_plt = np.r_[phi_plt, phi_plt[0]]
                line1, = ax.plot(c_phi_plt, 1. + 10. * self.c_dirty_img, linewidth=3,
                        alpha=0.55, linestyle='-',
                        label="spatial spectrum")
                ax.set_title("spatial spectrum")

                fre = [i * fs / 256 for i in range(nfft//2)]
                bx = fig.add_subplot(212)
                line2,  = bx.plot(fre, [0. for _ in fre])
                bx.set_ylim(0, 1)
                bx.set_title('frequency graph')
                bx.set_ylabel('Intensity (normalized)')
                bx.set_xlabel('Frequency in Hz')

                while True:
                    with self.lock:
                        line1.set_ydata(self.c_dirty_img)
                        line2.set_ydata(self.magnitude)
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    time.sleep(0.1)

            t = threading.Thread(target=plot_thread)
            t.start()
        else:
            self.plot = False



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

        if self.plot:
            spatial_resp = self.doa.grid.values
            min_val = spatial_resp.min()
            max_val = spatial_resp.max()
            spatial_resp = (spatial_resp - min_val) / (max_val - min_val)
            #spatial_resp = (spatial_resp - noise_min) /noise_min
            #self.noise_matrix = [self.noise_matrix[i] if self.noise_matrix[i] < spatial_resp[i] else spatial_resp[i] for i in range(0,360)]

            if self.algo_name == 'FRIDA':
                spatial_resp = np.abs(self.doa._gen_dirty_img())

            def window_func(window):
                def hamming(n, N):
                    a = 0.54
                    b = 1 - a
                    ret = a - b * math.cos(2 * math.pi * n / (N - 1))
                    return ret
                return [hamming(index, len(window)) * wert for index, wert in enumerate(window)]

            freq = np.fft.fft(np.array(window_func(audio[0][:nfft])), nfft)
            freq = freq[:nfft//2]
            mag = np.array([(x.real*x.real + x.imag*x.imag) for x in freq])
            min_val = mag.min()
            max_val = mag.max()
            mag = (mag - min_val) / (max_val - min_val)
            with self.lock:
                self.magnitude = mag
                self.c_dirty_img = np.r_[spatial_resp, spatial_resp[0]]
        return doa[0]
