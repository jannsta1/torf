
from __future__ import division

from scipy import signal

from utils import generate_window

from torf.base import *


levels = 5
nbands = 2

im_ws = [235, 230, 220]
im_ws = np.arange(255, 100, -1).astype(np.uint8)


# 145x226
#103x160
#99x155

# 5 - 72 - 80
# 6 - 81 - 96
# 7 - 97 - 112
# 8 - 113 - 128
# 9 - 129 - 144
# 10 - 145

# 103x160 output: 7x10
for im_w in im_ws:

    # im_h = np.ceil(im_w / (235/150.0)).astype(np.uint8)
    pyramid_constructor = Steerable_complex_wavelet_pyramid(im_h=im_w, im_w=im_w, levels=levels, nbands=nbands)
    band_h, band_w = pyramid_constructor.lowest_level_dim

    if band_h <= 7:
        winsize = band_h
    else:
        winsize = 7


    [window, w] = generate_window(band_h, band_h, winsize=winsize)

    dummy_pic = np.ones((band_h, band_w))
    # print(np.shape(dummy_pic))
    # print(np.shape(window))
    # print((window))
    # kernel_res = signal.convolve2d(window, dummy_pic, mode='valid', boundary='fill')
    kernel_res = signal.convolve2d(window, dummy_pic, mode='same', boundary='fill')

    kernel_size = np.shape(kernel_res)

    print ('input: {} output: {} kernel size {}'.format(im_w, band_w, kernel_size))