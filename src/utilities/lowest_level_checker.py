"""
A script that calculates the dimensions of the lowest level of a CWSSIM pyramid

Note - we use the actual pyramid decompositions rather than logarithms in case the "ceil" operations effect the results

"""


# global imports
from scipy import signal

# local imports
from torf_core.utils import generate_window
from torf_core.base import *

levels = 5
nbands = 2

im_ws = np.arange(255, 100, -1).astype(np.uint8)

# Previous Results:
# pixel width   | lowest dimension | highest dimension
# ------------------------------------------------------
# 5 - 72 - 80
# 6 - 81 - 96
# 7 - 97 - 112
# 8 - 113 - 128
# 9 - 129 - 144
# 10 - 145

for im_w in im_ws:
    pyramid_constructor = Steerable_complex_wavelet_pyramid(im_h=im_w, im_w=im_w, levels=levels, nbands=nbands)
    band_h, band_w = pyramid_constructor.lowest_level_dim
    if band_h <= 7:
        winsize = band_h
    else:
        winsize = 7
    [window, w] = generate_window(band_h, band_h, winsize=winsize)
    dummy_pic = np.ones((band_h, band_w))
    kernel_res = signal.convolve2d(window, dummy_pic, mode='same', boundary='fill')
    kernel_size = np.shape(kernel_res)
    print(('input: {} output: {} kernel size {}'.format(im_w, band_w, kernel_size)))