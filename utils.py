from datetime import datetime
from warnings import warn
from scipy import signal
import numpy as np


def matlab_style_gauss2D(shape=(3,3),sigma=0.5):
    """
    2D gaussian mask - should give the same result as MATLAB's
    fspecial('gaussian',[shape],[sigma])
    """
    m,n = [(ss-1.)/2. for ss in shape]
    y,x = np.ogrid[-m:m+1,-n:n+1]
    h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
    h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
    sumh = h.sum()
    if sumh != 0:
        h /= sumh
    return h


def process_cwssim_subband(im1_subband, im2_subband, window, w, mode='valid'):
    corr = im1_subband * np.conj(im2_subband)
    varr = np.square(np.abs(im1_subband)) + np.square(np.abs(im2_subband))
    corr_band = signal.convolve2d(window, corr, mode=mode)
    varr_band = signal.convolve2d(window, varr, mode=mode)
    # todo - add K in? what is a good value for this? cssim_map = (2 * np.abs(corr_band) + K). / (varr_band + K)
    cssim_map = (2 * np.abs(corr_band)) / (varr_band)
    return np.sum( np.sum( cssim_map * w) )


def cwssim_index(im1_band1, im1_band2, im2_band1, im2_band2, window, w):
    return np.mean((process_cwssim_subband(im1_band1, im2_band1, window=window, w=w),
                    process_cwssim_subband(im1_band2, im2_band2, window=window, w=w)))


def generate_window(band_h, band_w, winsize=7):

    window = np.ones((winsize, winsize))
    window = window / np.sum(np.sum(window))
    s = np.array((band_h, band_w))
    if np.any((s - winsize + 1) <= 0):
        warn('Gaussian window size is too large resulting in a projected window size of {}'.format(s))
    w = matlab_style_gauss2D(s - winsize + 1, band_h / 4.0)
    return window, w


def date_string():
    ''' get date string in a format handy for saving unique log file/folder names'''
    return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)