"""
This code is based on the principles outlined in:

J Portilla and E P Simoncelli A Parametric Texture Model based on Joint Statistics of Complex Wavelet Coefficients Int'l
Journal of Computer Vision. October, 2000.

or a more concise background is available:

background information is available: https://www.cns.nyu.edu/~eero/steerpyr/

The code in this file is an adaptation of the code in the following repo:

https://github.com/LabForComputationalVision/matlabPyrTools

Where possible, variable names have been preserved for easy comparison with this toolbox

"""



from scipy.interpolate import interp1d
from scipy.special import factorial
import matplotlib.pyplot as plt
from copy import copy
import numpy as np
import cv2

LOW_MASK = 1
HI_MASK = 2
BAND1 = 1
BAND2 = 2

LEVEL_1_IDX = 0


class Scy_pyr_builder_base(object):

    def __init__(self,
                    im_h,
                    im_w,
                    levels=5,
                    nbands=2,
                    twidth=1,
                    real_data_dtypes=np.float32,
                    complex_data_dtype=np.csingle,
                    ):

        self.im_h = im_h
        self.im_w = im_w
        self.dims_pyr_top_level = np.array((self.im_h, self.im_w))
        self.twidth = twidth
        self.complex_data_dtype = complex_data_dtype
        self.real_data_dtypes = real_data_dtypes

        self.levels = levels
        self.nbands = nbands
        self.order = self.nbands - 1

        self.lev_h = np.zeros(self.levels + 2, dtype=np.int)
        self.lev_w = np.zeros_like(self.lev_h, dtype=np.int)

        self.lev_h[0] = self.im_h
        self.lev_w[0] = self.im_w

        ###############################################################################
        # Preallocate memory for filter masks

        # recursively calculate the dimensions of each level
        for level in range(self.levels + 1):
            self.lev_h[level + 1] = np.int(np.ceil(self.lev_h[level] / 2.0))
            self.lev_w[level + 1] = np.int(np.ceil(self.lev_w[level] / 2.0))

        # print('lev_h', self.lev_h)
        # print('lev_w', self.lev_w)

        self.lostart = np.zeros((self.levels, 2))
        self.loend = np.zeros((self.levels, 2))

        # high_pass_mask - is a list of length "level" with a
        self.high_mask = [None] * self.levels
        self.lo_mask = [None] * self.levels
        self.angle_mask = [[None for i in range(self.nbands)] for j in range(self.levels)]
        # print self.angle_mask
        # print len(self.angle_mask)
        self.lodft = [None] * self.levels
        self.coefficients = [None] * self.nbands

        for level in range(self.levels):
            # print level
            self.high_mask[level] = np.zeros((self.lev_h[level], self.lev_w[level]), dtype=self.real_data_dtypes)

        # lo mask on the first level
        self.lo0mask = np.zeros((self.lev_h[LEVEL_1_IDX], self.lev_w[LEVEL_1_IDX]), dtype=self.real_data_dtypes)

        # lo mask on subsequent levels
        for level in range(self.levels):
            self.lo_mask[level] = np.zeros((self.lev_h[level + 1], self.lev_w[level + 1]),
                                           dtype=self.real_data_dtypes)

        # bandpass filter array
        for level in range(self.levels):
            for band in range(self.nbands):
                self.angle_mask[level][band] = np.zeros((self.lev_h[level], self.lev_w[level]),
                                                        dtype=self.real_data_dtypes)

        # lo dft array
        for level in range(self.levels):
            self.lodft[level] = np.zeros((self.lev_h[level], self.lev_w[level]), dtype=self.complex_data_dtype)
        for band in range(self.nbands):
            self.coefficients[band] = np.zeros((self.lev_h[self.levels], self.lev_w[self.levels]),
                                               dtype=self.real_data_dtypes)

    @property
    def i(self):
        """ returns unary imaginary number """
        return np.complex(0, 1)

    @property
    def lowest_level_dim(self):
        return np.array((self.lev_h[self.levels-1], self.lev_w[self.levels-1]))

    def build_scf(self, im):
        """
        Decompose an image using the preallocated filter masks

        :param im:
        :param export2matlab:
        :return:
        """
        # process the top layer filters
        imdft = np.fft.fftshift(np.fft.fft2(im).astype(self.complex_data_dtype))
        self.lodft[0][:, :] = (imdft * self.lo0mask).astype(self.complex_data_dtype)

        for level in range(self.levels):
            # if in lowest level

            if level == self.levels-1:
                this_band_dft = [None] * self.nbands
                for band in range(self.nbands):
                    this_band_dft[band] = (np.power(self.i, (self.nbands-1))) * self.lodft[level] * self.angle_mask[level][band] * self.high_mask[level]
                    self.coefficients[band] = np.fft.ifft2(np.fft.ifftshift(this_band_dft[band]).astype(self.complex_data_dtype))
                # print '####### Debug pyramid decomposition ########'
                # print level
                # print np.shape(this_band_dft[band])
                # Exit when the lowest level is reached and return these bands
                return self.coefficients

            lo_temp = self.lodft[level][self.lostart[level, 0]:self.loend[level, 0]+1,
                      self.lostart[level, 1]:self.loend[level, 1]+1]
            # print '####### Debug pyramid decomposition ########'
            # print level
            # print np.shape(self.lodft[level + 1])
            # print np.shape(lo_temp)
            # print np.shape(self.lo_mask[level])
            self.lodft[level+1][:, :] = lo_temp * self.lo_mask[level]

    def plot_lo_masks(self):
        """
        plots the low pass filter masks that are used in polar Fourier space
        :return:
        """

        qty = len(self.lo_mask)
        ax = [None] * qty

        fig, axs = plt.subplots(qty)

        for idx in range(qty):
            ax[idx] = axs[idx].matshow(self.lo_mask[idx])

        plt.show()

    def plot_high_masks(self):
        """
        plots the high pass filter masks that are used in polar Fourier space
        :return:
        """
        qty = len(self.high_mask)
        ax = [None] * qty

        fig, axs = plt.subplots(qty)

        for idx in range(qty):
            ax[idx] = axs[idx].matshow(self.high_mask[idx])

        plt.show()

    def plot_angle_masks_b1(self):
        """
        plots the first orientation of band pass filter masks that are used in polar Fourier space
        :return:
        """
        ax = [None] * self.levels

        fig, axs = plt.subplots(self.levels)

        for idx in range(self.levels):
            ax[idx] = axs[idx].matshow(self.angle_mask[idx][0])

        plt.show()


class Steerable_complex_wavelet_pyramid(Scy_pyr_builder_base):

    def __init__(self,
                        im_h,
                        im_w,
                        levels=5,
                        nbands=2,
                        ):

        super(Steerable_complex_wavelet_pyramid, self).__init__(im_h=im_h, im_w=im_w, levels=levels, nbands=nbands)

        self.generate_radial_transition_function()
        self.preprocess_masks()

    def preprocess_masks(self):
        """
        Preallocates filter masks once at initialisation time to save time for each image decomposition.
        This is a recursive process.
        :return:
        """
        # collect object vals that will be used recursively in this function
        this_log_rad = copy(self.log_rad)
        this_angle = copy(self.angle)

        for level in range(self.levels):

            self.high_mask[level] = self.point_op(this_log_rad, self.Yrcos, self.Xrcos_low[level], self.delta_Xrcos)
            for band in range(self.nbands):
                self.angle_mask[level][band] = self.point_op(this_angle, self.Ycosn,
                                                        self.Xcosn[0] + np.pi * (band) / self.nbands,
                                                        self.Xcosn[1] - self.Xcosn[0])

            this_im_dims = np.array((self.lev_h[level], self.lev_w[level]))

            ctr = np.ceil((this_im_dims + 0.5) / 2.0)
            lodims = np.ceil((this_im_dims - 0.5) / 2.0)
            loctr = np.ceil((lodims + 0.5) / 2.0)
            self.lostart[level, :] = ctr - loctr # + 1 - 1
            self.loend[level, :] = self.lostart[level, :] + lodims - 1

            self.lostart = self.lostart.astype(np.int)
            self.loend = self.loend.astype(np.int)

            low_slice = slice(self.lostart[level, 0], self.loend[level, 0]+1, 1)
            hi_slice = slice(self.lostart[level, 1], self.loend[level, 1]+1, 1)

            this_log_rad = this_log_rad[low_slice, hi_slice]
            this_angle = this_angle[low_slice, hi_slice]
            self.lo_mask[level] = self.point_op(this_log_rad, self.YIrcos, self.Xrcos_low[level], self.delta_Xrcos)

    def point_op(self, data, Y, x, x_delta):
        """
        Performs the "Point operation function" (PointOp) from Simoncelli's toolbox. Note, we don't use the optimised
        C-code because we only perform this once at initialisation time
        :param data: image
        :param Y: look up table
        :param x: origin
        :param x_delta: increment
        :return: image data linearly interpolated to new range
        """
        X = x + x_delta * np.arange(np.shape(Y)[0])
        f = interp1d(X, Y, fill_value="extrapolate")
        return np.reshape(f(data.ravel()), np.shape(data))

    def generate_radial_transition_function(self, lutsize=1024):
        """
        :param lutsize:
        :param plot_filters:
        :return:
        """
        ctr_x = np.int(np.floor((self.im_h) / 2.0))
        ctr_y = np.int(np.floor((self.im_w) / 2.0))
        xramp, yramp = np.meshgrid((np.arange(self.im_w)-ctr_y) / (self.im_w/2.0), (np.arange(self.im_h) - ctr_x) / (self.im_h / 2.0))
        self.angle = np.arctan2(yramp, xramp)
        self.log_rad = np.linalg.norm( np.dstack((xramp, yramp)), axis=2)    # np.linalg.norm((xramp, yramp))
        self.log_rad[ctr_x, ctr_y] = self.log_rad[ctr_x, ctr_y -1]
        self.log_rad = np.log2(self.log_rad)

        # Radial transition function(a raised cosine in log-frequency):
        [self.Xrcos, self.Yrcos] = self.rcosFn(self.twidth, (-self.twidth / 2.0), (0, 1))
        self.delta_Xrcos = self.Xrcos[1] - self.Xrcos[0]

        self.Xrcos_low = np.zeros(self.levels)
        self.Xrcos_hi = np.zeros_like(self.Xrcos_low)

        decrement = np.log2(2)
        self.Xrcos_low[0] = self.Xrcos[0] - decrement
        self.Xrcos_hi[0] = self.Xrcos[1] - decrement
        for idx in range(1, self.levels):
            self.Xrcos_low[idx] = self.Xrcos_low[idx - 1] - decrement
            self.Xrcos_hi[idx] = self.Xrcos_hi[idx - 1] - decrement

        self.Yrcos = np.sqrt(self.Yrcos)
        self.YIrcos = np.abs(np.sqrt(1.0 - np.square(self.Yrcos)))

        self.lo0mask = self.point_op(self.log_rad, self.YIrcos, self.Xrcos[0], self.delta_Xrcos)

        self.Xcosn = np.pi * ( np.arange(-2*lutsize-1, lutsize+2, 1) ) / lutsize        # [-2 * pi:pi]
        self.alfa = ( (np.pi + self.Xcosn) % (2 * np.pi) ) - np.pi
        self.const = (np.power((2*self.order),2)) * ( np.square(factorial(self.order, exact=True)) ) / (self.nbands * factorial(2 * self.order, exact=True))

        self.Ycosn = 2 * np.sqrt(self.const) * ( np.power(np.cos(self.Xcosn), self.order)) * (np.abs(self.alfa) < np.pi / 2)

    def rcosFn(self, width=1, position=1.0, values=np.array((0,1))):
        '''Return a lookup table containing a "raised cosine" soft threshold function
        Y =  VALUES(1) + (VALUES(2)-VALUES(1)) * cos^2( PI/2 * (X - POSITION + WIDTH)/WIDTH )
        this lookup table is suitable for use by `pointOp`
        Arguments
        ---------
        width : `float`
            the width of the region over which the transition occurs
        position : `float`
            the location of the center of the threshold
        values : `tuple`
            2-tuple specifying the values to the left and right of the transition.
        Returns
        -------
        X : `np.array`
            the x values of this raised cosine
        Y : `np.array`
            the y values of this raised cosine
        '''
        sz = 256  # arbitrary!
        X = np.pi * np.arange(-sz - 1, 2) / (2 * sz)
        Y = values[0] + (values[1] - values[0]) * np.cos(X) ** 2
        # make sure end values are repeated, for extrapolation...
        Y[0] = Y[1]
        Y[sz + 2] = Y[sz + 1]
        X = position + (2 * width / np.pi) * (X + np.pi / 4)
        return X, Y


def speed_test(repeats=1000):

    from time import time
    from .definitions_cwssim import IM_SEQUENCES
    from os import path

    p_build = Steerable_complex_wavelet_pyramid(im_h=150, im_w=235)
    im = cv2.imread(path.join(IM_SEQUENCES, 'fwd_drone', 'fwd_drone_1.jpg'), 0)

    t_start = time()
    print(('performing speed test with {} repeats'.format(repeats)))
    for idx in range(repeats):
        coefficients = p_build.build_scf(im)
    t_end = time()
    t_mean = (t_end - t_start) / repeats
    print(('mean processing time was {}s'.format(t_mean)))


def plot_masks(im_h=150, im_w=235):
    """
    Plots the preallocated masks of the steerable pyramid
    :param im_h:
    :param im_w:
    :return:
    """
    py = Steerable_complex_wavelet_pyramid(im_h=im_h, im_w=im_w)

    py.plot_lo_masks()
    py.plot_high_masks()
    py.plot_angle_masks_b1()


if __name__ == '__main__':

    # run some diagnostic tests
    speed_test()
    plot_masks(im_h=150, im_w=235)
