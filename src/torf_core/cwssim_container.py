

# global imports
from multiprocessing import RawArray, Pool
from functools import partial
from ctypes import c_double
from time import time

# local imports
from utils import *
from base import *


############################################################################################
# this block of code is here because we can't use instance methods in pool so using global vars instead
var_dict = {}
def init_worker(band_1_bank, band_1_bank_shape, band_2_bank, band_2_bank_shape, window, w):
    """
    Initialises data structures for applying CWSSIM to the lowest level of a CWSSIM pyramid with 2 orientation bands

    # todo - create an arbitrary orientation version
    """
    # Use a dictionary to show the data mapping to our worker
    var_dict['band_1_bank'] = band_1_bank
    var_dict['band_1_bank_shape'] = band_1_bank_shape
    var_dict['band_2_bank'] = band_2_bank
    var_dict['band_2_bank_shape'] = band_2_bank_shape
    var_dict['window'] = window
    var_dict['w'] = w

def worker_func(this_band_1, this_band_2, im_idx):
    """
    Pool worker function for applying CWSSIM to the lowest level of a CWSSIM pyramid with 2 orientation bands

    # todo - create an arbitrary orientation version

    :param this_band_1: lowest level CWSSIM mask orientation 1
    :param this_band_2: lowest level CWSSIM mask orientation 2
    :param im_idx: index of image in memory bank to compare with
    :return:
    """
    # Simply computes the sum of the i-th row of the input matrix X
    band_1_bank = np.frombuffer(var_dict['band_1_bank'], dtype=np.csingle).reshape(var_dict['band_1_bank_shape'])
    band_2_bank = np.frombuffer(var_dict['band_2_bank'], dtype=np.csingle).reshape(var_dict['band_2_bank_shape'])

    return cwssim_index(im1_band1=this_band_1,
                        im1_band2=this_band_2,
                        im2_band1=band_1_bank[im_idx, :, :],
                        im2_band2=band_2_bank[im_idx, :, :],
                        window=var_dict['window'],
                        w=var_dict['w'],
                        )

####################################################################################


class Cwsim_container(object):
    """
    A class that stores a bank of images and allows for a query image to be compared via torf with all the images in
    the bank. For efficiency we just store the banks of the lowest level of the torf pyramid

    """

    def __init__(self, im_h, im_w, max_depth, levels=5, nbands=2, winsize=7, real_data_dtype=np.float32,
                 complex_data_dtype=np.csingle):

        self.real_data_dtype = real_data_dtype
        self.complex_data_dtype = complex_data_dtype

        self.im_h = im_h
        self.im_w = im_w
        self.levels = levels
        self.nbands = nbands

        self.pyramid_constructor = Steerable_complex_wavelet_pyramid(im_h=im_h, im_w=im_w, levels=levels, nbands=nbands)

        band_h, band_w = self.pyramid_constructor.lowest_level_dim
        print(('lowest level dims: {}x{}'.format(band_h, band_w)))

        self.band_list = [None] * self.nbands

        # 4D tensor containing all learned coefficient memory
        self.coefficient_tensor = np.zeros((max_depth, self.nbands, band_h, band_w), dtype=complex_data_dtype)

        self.__current_image_cnt = 0
        self.max_depth = max_depth

        self.window, self.w = generate_window(winsize=winsize, band_h=band_h, band_w=band_w)

        self.results = []

    @property
    def qty_images_stored(self):
        """
        How many images have been added to the memory bank
        """
        return self.__current_image_cnt

    @property
    def memory_full(self):
        """
        Is the memory bank full?
        """
        return self.__current_image_cnt >= self.max_depth

    def add_image(self, im):
        """
        Add image to the memory bank
        :param im: Image to add - expected datatype uint8 numpy array
        :return:
        """
        if not self.memory_full:
            # todo - avoid this memory copy?
            coefficients = self.pyramid_constructor.build_scf(im=im)
            for band in range(self.nbands):
                self.coefficient_tensor[self.__current_image_cnt, band, :, :] = coefficients[band]

            self.__current_image_cnt += 1
            return True
        else:
            # we return false so that an error can be published by the calling module
            return False

    def query_image(self, im):
        """
        compares the query image to all images in the query bank
        :param im:
        :return:
        """
        coefficients = self.pyramid_constructor.build_scf(im=im)
        results = np.zeros(self.qty_images_stored)

        for im_idx in range(self.qty_images_stored):
            results[im_idx] = self.cwssim_index(coefficients_1=coefficients,
                                                coefficients_2=self.coefficient_tensor[im_idx, :, :, :])
        return results

    def max_query_image(self, im):
        """
        Returns maximum value when an image is querried against all images in the memory bank
        :param im:
        :return:
        """
        return np.max(self.query_image(im))

    def max_query_image_with_index(self, im):
        """
        Returns the best score in addition to the index of this image
        :param im:
        :return:
        """
        result = self.query_image(im)
        return np.max(result), np.argmax(result)

    # todo - harmonise this version with multiproces version (currently in utils)
    def cwssim_index(self, coefficients_1, coefficients_2):
        """
        calculate similarity between 2 sets of coefficients
        :param coefficients_1:
        :param coefficients_2:
        :return:
        """
        mean_list = []
        for band in range(self.nbands):
            mean_list.append(self.process_cwssim_subband(coefficients_1[band], coefficients_2[band]))
        return np.mean(mean_list)

    def self_im_query(self, im1_idx, im2_idx):
        """
        compare the coefficients of 2 images that are already in the memory bank
        :param im1_idx:
        :param im2_idx:
        :return:
        """
        return self.cwssim_index(coefficients_1=self.coefficient_tensor[im1_idx, :, :, :],
                                 coefficients_2=self.coefficient_tensor[im2_idx, :, :, :])

    def resize_memory_bank(self, discard_last=False):
        """
        Used if the blank images in bank_1 and bank_2 should be culled (e.g. before inverting images)

        Added discard_last as there somtimes seems to be an unpopulated last image - should be further investigated.
        Update. Think this was due to self.new_image_evt in torf_ros.py not being cleared at the end of an outbound
        mission

        :return:
        """

        if discard_last:
            self.coefficient_tensor = self.coefficient_tensor[0:self.qty_images_stored-1, :, :, :]
            self.__current_image_cnt = self.__current_image_cnt - 1
        else:
            self.coefficient_tensor = self.coefficient_tensor[0:self.qty_images_stored, :, :, :]

    def reverse_image_ids(self):
        """
        reverse the order of images stored in the memory bank

        sometimes useful when the order images are stored relates to the location they were stored
        :return:
        """
        self.resize_memory_bank()
        self.coefficient_tensor = self.coefficient_tensor[::-1, :, :, :]

    def self_im_query_all(self, query_idx, plot_output=False):
        """
        compare an image already stored in the memory bank with all other images in the memory bank

        :param query_idx:
        :param plot_output:
        :return:
        """
        t1 = time()
        results = np.zeros(self.qty_images_stored)
        for idx in range(self.qty_images_stored):
            results[idx] = self.self_im_query(idx, query_idx)
        print((time() -t1))

        if plot_output:
            plt.plot(results, marker='D')
            plt.show()
        return results

    # todo - harmonise with the version in utils?
    def process_cwssim_subband(self, im1_subband, im2_subband, mode='valid'):
        """
        find the correlation of 2 CWSSIM image subbands using the covariance of the local window defined by self.window
        and the weights, self.w

        :param im1_subband:
        :param im2_subband:
        :param mode:
        :return:
        """
        corr = im1_subband * np.conj(im2_subband)
        varr = np.square(np.abs(im1_subband)) + np.square(np.abs(im2_subband))
        corr_band = signal.convolve2d(self.window, corr, mode=mode)
        varr_band = signal.convolve2d(self.window, varr, mode=mode)
        # todo - add K in? what is a good value for this? cssim_map = (2 * np.abs(corr_band) + K). / (varr_band + K)
        cssim_map = (2 * np.abs(corr_band)) / (varr_band)

        return np.sum(np.sum(cssim_map * self.w))

    #############################################################################
    # multiprocessing enabled functions

    def max_query_image_mp(self, im):
        """
        Returns the best score
        :param im:
        :return:
        """
        return np.max(self.query_image_mp(im))


    def max_query_image_with_index_mp(self, im):
        """
        Returns the best score in addition to the index of this image
        :param im:
        :return:
        """
        result = self.query_image_mp(im)
        return np.max(result), np.argmax(result)


    def query_image_mp(self, im):
        """
        compares the query image to all images in the query bank

        :param im:
        :return:
        """
        coefficients = self.pyramid_constructor.build_scf(im=im)

        # this_band_1, this_band_2

        results = self.pool.map(partial(worker_func, coefficients[0], coefficients[1]), list(range(self.qty_images_stored)))

        return results

    def self_im_query_all_mp(self, query_idx, plot_output=False):

        t1 = time()
        results = np.zeros(self.qty_images_stored)
        for idx in range(self.qty_images_stored):
            results[idx] = self.self_im_query(idx, query_idx)
        print((time() -t1))

        if plot_output:
            plt.plot(results, marker='D')
            plt.show()
        return results

    def gen_random_complex_coefficients(self):
        im_h, im_w = self.coefficient_tensor[0, 0, :, :].shape
        A = np.random.rand(im_h, im_w, 2)
        return np.apply_along_axis(lambda args: [complex(*args)], 2, A).squeeze()


    def data2raw_array(self, data):

        data_shape = np.shape(data)
        data_size = np.size(data) # np.prod(np.asarray(data_shape, dtype=int))   # i.e how many elements

        # to get the size we make a 1d array of the datatype and then get the number of bytes for this
        # no_bytes = np.array(1.0, dtype=data.dtype).nbytes
        if self.complex_data_dtype == np.complex128:
            raw_array = RawArray(c_double, data_size*2)
        elif self.complex_data_dtype == np.csingle:
            raw_array = RawArray(c_double, data_size)
        else:
            raise AttributeError ("haven't yet introduced c_types data for type {}. Either change the data type to one "
                                  "of the available options or update the code base".format(self.complex_data_dtype))

        # copy the data into numpy
        raw_array_np = np.frombuffer(raw_array, dtype=data.dtype).reshape(data_shape)
        np.copyto(raw_array_np, data)

        return raw_array_np, data_shape

    def prepare_memory_bank_outside(self, processes=4):
        """
        Prepares memory for future calls of query_image_mp
        this should be run once all data has been collected but ahead of the time to intensively process images
        :return:
        """

        # todo - do we need to clean up child processes in the pool? have tried self.pool.daemon = True maybe this does the job?

        # put memory data into RawArray. Nb. this operation is only suitable (process safe) for data that is read only.
        # :. Once this is done we can't change the images in the array
        # todo - how to have a variable number of bands?
        if self.nbands != 2:
            raise NotImplementedError('Multiprocessing only currently supports 2 subbands')

        self.band_1_bank = copy(self.coefficient_tensor[:, 0, :, :])
        self.band_2_bank = copy(self.coefficient_tensor[:, 1, :, :])
        b1_raw_array, b1_raw_array_shape = self.data2raw_array(self.band_1_bank)
        b2_raw_array, b2_raw_array_shape = self.data2raw_array(self.band_2_bank)

        # b1_raw_array, b1_raw_array_shape = self.data2raw_array(self.coefficient_tensor[:, 0, :, :])  # 0 = first band
        # b2_raw_array, b2_raw_array_shape = self.data2raw_array(self.coefficient_tensor[:, 1, :, :])  # 1 = second band

        # initialise our pool
        self.pool = Pool(processes=processes, initializer=init_worker,
                         initargs=(b1_raw_array, b1_raw_array_shape, b2_raw_array, b2_raw_array_shape, self.window, self.w))
        self.pool.daemon = True

        # generate some random data for an initialisation run
        this_band_1 = copy(self.gen_random_complex_coefficients()).astype(self.complex_data_dtype)
        this_band_2 = copy(self.gen_random_complex_coefficients()).astype(self.complex_data_dtype)

        # we run the algorithm on data as doing this seems to speed up subsequent operations
        # print ('***********8')
        # print (np.shape(this_band_1), type(this_band_1[0,0]))
        # print (np.shape(this_band_2))
        # print (self.coefficient_tensor[0, 0, :, :].shape)
        self.results = self.pool.map(partial(worker_func, this_band_1, this_band_2), list(range(self.qty_images_stored)))

    def print_info(self):

        depth, height, width = self.band_1_bank.shape
        print(('Memory bank shape is {} {} {}'.format(depth, height, width)))

    def tidy_up(self):

        if hasattr(self, 'pool'):
            print ('Attempting to close our multiprocessing pool down now')
            self.pool.close()
            print ('Pool closed')
            # self.pool.join()
            print ('Pool joined')


class Cwsim_container_from_ims(Cwsim_container):

    def __init__(self,
                 ims,
                 real_data_dtype=np.float32,
                 complex_data_dtype=np.csingle,
                 levels=5,
                 nbands=2,
                 winsize=7,
                 ):

        max_depth, im_h, im_w = np.shape(ims)

        super(Cwsim_container_from_ims, self).__init__(im_h=im_h, im_w=im_w, max_depth=max_depth,
                                                       levels=levels, nbands=nbands, winsize=winsize,
                                                       real_data_dtype=real_data_dtype,
                                                       complex_data_dtype=complex_data_dtype,
                                                       )

        # we add all images into our image bank
        for im in ims:
            self.add_image(im)


def response_across_im_series(ims=None, multiprocess=True, test_idx=20, levels=5, im_w=235, im_h=150,
                              plot_output=False):
    """
    perform torf against every image in dataset - the highest response should be at the test_idx

    :param test_idx:
    :return:
    """

    if not ims:
        ims = get_fwd_drone_ims(im_w=im_w, im_h=im_h)

    cc = Cwsim_container_from_ims(ims=ims, levels=levels)
    if multiprocess:
        cc.prepare_memory_bank_outside()
        results = cc.self_im_query_all_mp(test_idx, plot_output=plot_output)
    else:
        results = cc.self_im_query_all(test_idx, plot_output=plot_output)

    return results


if __name__ == '__main__':

    response_across_im_series(plot_output=True)
    response_across_im_series(plot_output=True, multiprocess=False)



