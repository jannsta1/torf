#!/usr/bin/env python3
from .autorun_base import *

args_dict = parse_repeat_cwssim_model(sys.argv[1:], required_parsers=('tester', 'sm'),
                                      lower_default=5.0, range_default=1.0, delta_default=0.1)

## Generate the test arangement
lower = args_dict['tester'].lower
upper = lower + args_dict['tester'].range + 0.01   # small offset added so that the last value is included in np.arange
delta = args_dict['tester'].delta
heights = np.arange(lower, upper, delta)
print(('testing outbound height values: {} with inbound value always at {}'.format(heights, args_dict['sm'].mission_hgt)))

passed = []
list_of_summaries = []
for height in heights:
    save_name = 'hgt_exp_ob_' + str(np.round(height, 1)) + '_ib_' + str(np.round(args_dict['sm'].mission_hgt, 1))
    args_dict['sm'].outbound_hgt = height
    these_results = do_autorun(args_dict['tester'], args_dict['sm'], save_name=save_name)
    list_of_summaries.append(these_results)

# print mission status (gives an indication if any of the tests have failed due to gazebo crashing (but they may fail
# for other reasons))
for idx, height in enumerate(heights):
    print(('for height {} the summary is: \n'.format(height)))
    print((list_of_summaries[idx]))
