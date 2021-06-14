# global imports
from matplotlib import pyplot as plt
import numpy as np
import cv2
import os

# local imports
from cwssim_container import Cwsim_container
from definitions_cwssim import DATA


# helper function to load
def load_image_sequence(im_sequence_dir):
    files_dir = os.path.join(DATA, 'im_sequences', im_sequence_dir)
    im_qty = len([name for name in os.listdir(files_dir) if name.endswith('.png')])
    print(im_qty)
    im_list = []
    for idx in range(im_qty):
        im_name = os.path.join(files_dir, str(idx + 1) + '.png')
        im_list.append(cv2.imread(im_name, cv2.IMREAD_GRAYSCALE))
    return im_list


# load inbound and outbound image sets
outbound_img_list = load_image_sequence(im_sequence_dir='torf_example_outbound')
inbound_img_list = load_image_sequence(im_sequence_dir='torf_example_inbound')

# calculate image properties
im_h, im_w = np.shape(outbound_img_list[0])
qty_images_out = len(outbound_img_list) - 1
qty_images_in = len(inbound_img_list) - 1

# create the CWSSIM container class
cc = Cwsim_container(im_h=im_h, im_w=im_w, max_depth=qty_images_out)

# load training images to the container class
for idx in range(qty_images_out):
    rotated_im = np.rot90(outbound_img_list[idx], 2)
    cc.add_image(rotated_im)

# prepare container for CWSSIM evaluation of test images with multiple processes
cc.print_info()
cc.reverse_image_ids()
cc.prepare_memory_bank_outside()

# initialise arrays to store the results
scores = np.zeros(qty_images_in)
indexes = np.zeros(qty_images_in)
result_matrix = np.zeros((qty_images_out, qty_images_in))

# compute the results
for idx in range(qty_images_in):
    # print(idx)
    scores[idx], indexes[idx] = cc.max_query_image_with_index_mp(inbound_img_list[idx])
    these_scores = cc.query_image_mp(inbound_img_list[idx])
    result_matrix[:, idx] = these_scores

# plot the results
plt.matshow(result_matrix)
plt.colorbar()
plt.xlabel('Inbound index')
plt.ylabel('Outbound index')
plt.show()

plt.plot(scores)
plt.xlabel('Inbound index')
plt.ylabel('Familiarity Score')
plt.show()

