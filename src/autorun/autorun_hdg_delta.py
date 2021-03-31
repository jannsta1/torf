#!/usr/bin/env python3
from .autorun_base import *

args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'),
                                      lower_default=0.0, range_default=7.0, delta_default=1.0)

## Generate the test arangement
lower = args_dict['tester'].lower
upper = lower + args_dict['tester'].range + 0.01   # small offset added so that the last value is included in np.arange
delta = args_dict['tester'].delta
heading_offsets_deg = np.arange(lower, upper, delta)
print(('testing outbound height values: {} with inbound value always at {}'.format(heading_offsets_deg, args_dict['sm'].mission_hgt)))

passed = []
list_of_summaries = []
for heading_offset in heading_offsets_deg:

    heading_offset_rad = np.deg2rad(heading_offset)

    save_name = 'hdg_offset_' + str(np.round(heading_offset, 2))
    args_dict['sm'].hdg_in_offset = heading_offset_rad
    args_dict['sm'].outbound_images = 170
    args_dict['sm'].duration_out = 15.0
    # args_dict['sm'].inbound_configuration = 'timed'
    these_results = do_autorun(args_dict['tester'], args_dict['sm'], save_name=save_name)
    list_of_summaries.append(these_results)

# print mission status (gives an indication if any of the tests have failed due to gazebo crashing (but they may fail
# for other reasons))
for idx, heading_offset in enumerate(heading_offsets_deg):
    print(('for heading {} the summary is: \n'.format(heading_offset)))
    print((list_of_summaries[idx]))
