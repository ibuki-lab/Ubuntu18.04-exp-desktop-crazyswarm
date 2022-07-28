import numpy as np
from numpy.fft import fft, ifft, fftshift, fftfreq
from scipy.signal import max_len_seq


def gen_mls(n):
    mls = max_len_seq(n)[0]
    print(mls)
    print(sum(mls), sum(mls == 0))
    return mls

def main():
    seq = gen_mls(21)
    spec = fft(seq)
    acorr = np.correlate(seq, seq, 'full')

if __name__ == '__main__':
    main()