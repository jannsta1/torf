#!/usr/bin/env python2
from generate_singlecam_sdf import generate_singlecam_sdf_function
from autorun_base import *
from torf.utils import date_string

args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'),
                                      lower_default=1.0, range_default=10.0, delta_default=1.0)

# Generate the test arangement
lower = args_dict['tester'].lower
upper = lower + args_dict['tester'].range + 0.01   # small offset added so that the last value is included in np.arange
delta = args_dict['tester'].delta
fovs_deg = np.arange(lower, upper, delta)
print ('testing outbound camera field of view values: {}'.format(fovs_deg))


cwssim_image_width = 15            # width of coefficient space at level of comparison

# iterate through tests
passed = []
list_of_summaries = []
test_id = date_string()

pixel_widths = [161, 145, 129, 113, 97, 81]
lowest_dim   = [ 11, 10,   9,   8,  7,  6]

for idx, pixel_width in enumerate(pixel_widths):

    fov_rad = np.deg2rad(42)
    save_name = 'cwssim_cam_lowest_width_' + str(lowest_dim[idx])

    generate_singlecam_sdf_function(fov_horizontal_rad=fov_rad, cam_w=pixel_width, cam_h=pixel_width)

    args_dict['sm'].outbound_images = 170
    args_dict['sm'].duration_out = 15.0
    args_dict['sm'].im_w = pixel_width
    args_dict['sm'].im_h = pixel_width

    if lowest_dim[idx] < 7:
        args_dict['sm'].winsize = lowest_dim[idx]

    these_results = do_autorun(args_dict['tester'], args_dict['sm'], save_name=save_name,
                               autosort=True, test_id=test_id, test_variable=pixel_width)
    list_of_summaries.append(these_results)

# print mission status (gives an indication if any of the tests have failed due to gazebo crashing (but they may fail
# for other reasons))
for idx, fov_deg in enumerate(fovs_deg):
    print ('for height {} the summary is: \n'.format(fov_deg))
    print (list_of_summaries[idx])

