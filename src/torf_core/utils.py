# global imports
from datetime import datetime
from warnings import warn
from scipy import signal
import numpy as np
import cv2
import os

# local imports
from definitions_cwssim import IM_SEQUENCES

def matlab_style_gauss2D(shape=(3,3),sigma=0.5):
    """
    2D gaussian mask - should give the same result as MATLAB's fspecial('gaussian',[shape],[sigma])

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
    """
    convert from polar coordinates to cartesian coordinates
    :param rho: length
    :param phi: angle
    :return:
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return (x, y)


def get_fwd_drone_ims(resize=False, im_w=235, im_h=150, resize_method=cv2.INTER_CUBIC):

    files_dir = os.path.join(IM_SEQUENCES, "fwd_drone")
    im_list = []
    for idx in range(40):
        im_name = files_dir + '/fwd_drone_' + str(idx + 1) + '.jpg'

        this_im = cv2.imread(im_name, cv2.IMREAD_GRAYSCALE)
        if resize:
            # kernel_size = 5
            # this_im = cv2.GaussianBlur(this_im, (kernel_size, kernel_size), 0)
            this_im = cv2.resize(this_im, (im_w, im_h), resize_method)
        im_list.append(this_im)

    if len(im_list) == 0:
        raise 'No images found in {}'
    ims = np.asarray(im_list)

    return ims
