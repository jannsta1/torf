#!/usr/bin/env python

import argparse

from src.torf_ros import TorfCwssimHomingMission
from src.torf_ros import *
import rospy

if __name__ == '__main__':
    """
    This launches a node that can be used to test whether the host machine can process the torf node fast enough given 
    the specified test parameters (see arg options below)
    
    NB - there must be an image node (specified by topic 'camera_topic_name') being pulished for this test to run
    """
    node_namespace='cwssim_speedtest_node'
    rospy.init_node(name=node_namespace, anonymous=True, log_level=rospy.DEBUG)

    parser = argparse.ArgumentParser(description="This parses arguments for a sweep mission.")
    parser.add_argument('-d', '--duration', type=float, default=60.0)
    parser.add_argument('-i', '--image_bank_size', type=int, default=100)
    parser.add_argument('-t', '--camera_topic_name', type=str, default='/resize_img/image')
    # parser.add_argument('-s', '--state_estimation_type', help="which sensing modality is used for state estimation", type=parse_sem, default='UNKNOWN')
    parser.add_argument('-p', '--multi_process', type=str2bool, default=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    instructions = {}
    instructions[0] = Idle_state(
        state_label='time_to_init',  # allow a bit of time for image bank to acrew
        timeout=10,
    )
    instructions[1] = Idle_state(
        state_label='Sweep_state',    # this means we will be evaluating our image bank
        timeout=args.duration,
    )
    instructions[2] = Post_run_state(
        state_label='Sweep_state',  # this means we will be evaluating our image bank
        timeout=10,
    )

    cx_class = TorfCwssimHomingMission(
                            flight_instructions=instructions,
                            node_namespace=node_namespace,
                            camera_topic_name=args.camera_topic_name,
                            max_outbound_images=args.image_bank_size,
                            state_estimation_mode=State_estimation_method.UNKNOWN,
                            use_multi_processing=args.multi_process,
                        )

    # generate random images and add to image bank
    for idx in np.arange(args.image_bank_size):
        random_image = np.random.randint(0, 255, (cx_class.cwssim.im_h, cx_class.cwssim.im_w), dtype=np.uint8)
        cx_class.cwssim.add_image(im=random_image)

    # simulate memory bank
    cx_class.initialise_memory_bank_for_multiprocessing()

    cx_class.run()
    cx_class.tidy_up()