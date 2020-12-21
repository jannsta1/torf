
import cv2
import glob


def get_test_images(image_dir, image_extension='.jpg'):
    '''
    returns a list of images that are read from data contained in this repo
    :return:
    '''
    return [cv2.imread(file, cv2.IMREAD_GRAYSCALE) for file in glob.glob(image_dir + "/*" + image_extension)]



# get our test images
files_dir = "/disk2/Dropbox/MATLAB/ssim_test/data/im_sequences/fwd_drone"
ims = get_test_images(files_dir)


# now create


