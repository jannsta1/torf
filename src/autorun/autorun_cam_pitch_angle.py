#!/usr/bin/env python2
from autorun_base import *
from generate_singlecam_sdf import generate_singlecam_sdf_function

args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'))

# Generate the test arangement
lower = args_dict['tester'].lower
upper = lower + args_dict['tester'].range + 0.01   # small offset added so that the last value is included in np.arange
delta = args_dict['tester'].delta
pitches = np.arange(lower, upper, delta)
print ('testing outbound height values: {} with inbound value always at {}'.format(pitches, args_dict['sm'].mission_hgt))

# iterate through tests
passed = []
list_of_summaries = []
for pitch in pitches:
    save_name = 'cwssim_cam_pitch_' + str(np.round(pitch, 1))
    generate_singlecam_sdf_function(pitch_angle_deg=pitch)
    these_results = do_autorun(args_dict['tester'], args_dict['sm'], save_name=save_name)
    list_of_summaries.append(these_results)

# print mission status (gives an indication if any of the tests have failed due to gazebo crashing (but they may fail
# for other reasons))
for idx, pitch in enumerate(pitches):
    print ('for height {} the summary is: \n'.format(pitch))
    print (list_of_summaries[idx])
