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
for fov_deg in fovs_deg:
    fov_rad = np.deg2rad(fov_deg * cwssim_image_width)
    save_name = 'cwssim_cam_fov_' + str(np.round(fov_deg, 1))
    generate_singlecam_sdf_function(fov_horizontal_rad=fov_rad)

    args_dict['sm'].outbound_images = 170
    args_dict['sm'].duration_out = 15.0
    args_dict['sm'].multiplier = 5.0
    # args_dict['sm'].inbound_configuration = 'timed'

    these_results = do_autorun(args_dict['tester'], args_dict['sm'], save_name=save_name,
                               autosort=True, test_id=test_id, test_variable=fov_deg)
    list_of_summaries.append(these_results)

# print mission status (gives an indication if any of the tests have failed due to gazebo crashing (but they may fail
# for other reasons))
for idx, fov_deg in enumerate(fovs_deg):
    print ('for height {} the summary is: \n'.format(fov_deg))
    print (list_of_summaries[idx])

