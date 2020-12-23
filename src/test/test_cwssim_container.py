import pytest
from src.cwssim_container import Cwsim_container_from_ims, response_across_im_series
from src.utils import get_fwd_drone_ims


# @pytest.fixture
# def ims(im_w=235, im_h=150):
#     return get_fwd_drone_ims(im_w=im_w, im_h=im_h)


def test_response_across_im_series_multi_process():
    response_across_im_series()


def test_response_across_im_series_single_process():
    response_across_im_series(multiprocess=False)
