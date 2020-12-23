from __future__ import division

# global imports
import pytest

# local imports
from src.definitions_cwssim import DATA
from src.cwssim_container import *


@pytest.fixture
def sidesweep_image_sequence(file_prefix='side_drone'):
    files_dir = os.path.join(DATA, 'im_sequences/' + file_prefix)
    im_qty = len([name for name in os.listdir(files_dir) if name.endswith('.jpg')])
    # print im_qty
    im_list = []
    for idx in range(im_qty):
        im_name = files_dir + '/' + file_prefix + '_' + str(idx + 1) + '.jpg'
        im_list.append(cv2.imread(im_name, cv2.IMREAD_GRAYSCALE))
    return im_list


def test_compare_serial_with_multiprocess(sidesweep_image_sequence):
    """
    To check if we are getting a speed gain from multiprocessing. NB. the size of image bank will be a factor on whether
    this is worthwhile or not
    
    :param sidesweep_image_sequence:
    :return:
    """

    cc = Cwsim_container_from_ims(ims=sidesweep_image_sequence)

    serial_times = []
    for idx, im in enumerate(sidesweep_image_sequence):
        t1 = time()
        cc.query_image(im)
        t2 = time() - t1
        serial_times.append(t2)
    serial_mean = np.mean(serial_times)

    # prepare for multiprocess stuff
    cc.prepare_memory_bank_outside()
    test_im = sidesweep_image_sequence[1]
    cc.query_image_mp(test_im)
    multip_times = []
    for idx, im in enumerate(sidesweep_image_sequence):
        t1 = time()
        cc.query_image_mp(im)
        t2 = time() - t1
        multip_times.append(t2)
    multip_mean = np.mean(multip_times)
    print('Serial mean: {}, multip mean: {} - speedup = {}'.format(serial_mean, multip_mean,serial_mean / multip_mean))


def test_compare_single_and_multiprocess_results(sidesweep_image_sequence):
    """
    Compares the self test results of single and multiprocessing output
    :param sidesweep_image_sequence:
    :return:
    """
    cc = Cwsim_container_from_ims(ims=sidesweep_image_sequence)
    test_idx = 20
    results_single_proc = cc.self_im_query_all(test_idx, plot_output=False)

    test_im = sidesweep_image_sequence[test_idx]
    cc.prepare_memory_bank_outside()
    results_multi_proc = cc.query_image_mp(test_im)

    plt.plot(results_single_proc, marker='D', label='single_proc')
    plt.plot(results_multi_proc, marker='D', label='multi_proc')
    plt.legend()
    plt.draw()
    plt.show(block=False)
    plt.pause(3)

    assert np.allclose(results_single_proc, results_multi_proc)


@pytest.mark.parametrize("dtypes", [(np.float32, np.csingle), (np.float64, np.complex)])
def test_dtype(sidesweep_image_sequence, dtypes):
    """
    running this test on the hp z600 (with mkl support) seems to show no difference between double and single variables
    in terms of speed. If anything the doubles are quicker??

    :param sidesweep_image_sequence:
    :param dtypes:
    :return:
    """
    t1 = time()
    print ('testing dtypes: {}'.format(dtypes))
    cc = Cwsim_container_from_ims(ims=sidesweep_image_sequence, real_data_dtype=dtypes[0], complex_data_dtype=dtypes[1])
    t2 = time()


    for im in sidesweep_image_sequence:
        cc.query_image(im)
    t3 = time()
    img_qty = len(sidesweep_image_sequence)
    process_time_per_image = (t3-t2)/img_qty
    print ('preperation took {} processing {} images took {}'.format(t2 - t1, img_qty, t3-t2))
    print ('processing time per image = {}'.format(process_time_per_image))
