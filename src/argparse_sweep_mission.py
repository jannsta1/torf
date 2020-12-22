import numpy as np
import argparse
import rospy

def check_angle_of_attack(value):
    """ ensure that the angle of attack argument is within the permissable range """

    fval = float(value)
    if fval <= 0:
        raise argparse.ArgumentTypeError("Angle of attack should be greater than zero, is {}".format(fval))
    elif fval >= 90:
        raise argparse.ArgumentTypeError("Angle of attack should be less than 90, is {}".format(fval))
    return fval

def str2bool(v):
    """ flexible parsing of boolean arguments """
    if v[0] == " ":
        v = v[1::]
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def sm_argparse(sys_argv_in, started_by_ros=True, return_args=True, subparsers=None):
    """
    Main parsing method for sidesweep missions

    :param sys_argv_in:
    :param started_by_ros:
    :param return_args:
    :param subparsers:
    :return:
    """

    if subparsers:
        parser = subparsers.add_parser('sm', description="This parses arguments for a sweep mission.")
    else:
        parser = argparse.ArgumentParser(description="This parses arguments for a sweep mission.")

    # mission flight parameters
    parser.add_argument('-d', '--duration_out', type=float, default=20.0)
    parser.add_argument('-a', '--mission_hgt', type=float, default=5.0)
    parser.add_argument('-u', '--outbound_hgt', type=float, default=np.nan)
    parser.add_argument('-b', '--hdg_out', type=float, default=0.0, help="Outbound mission heading in radians")
    parser.add_argument('--hdg_in_offset', type=float, default=0.0, help="Fixed inbound mission heading noise offset in radians")
    parser.add_argument('-z', '--angle_of_attack', help='angle of attack of sweep, (0 < -c < 90) where a higher attaack'
                        'angle will result in more turns', type=check_angle_of_attack, default=70.0)
    parser.add_argument('--search_angle_of_attack', help='search angle of attack of sweep, (0 < -c < 90) where a higher attaack'
                        'angle will result in more turns', type=check_angle_of_attack, default=70.0)
    parser.add_argument('-c', '--outbound_configuration', help="Type (configuration) of outbound travel (e.g. fixed"
                        " heading)", type=str, default='fixed_heading')
    parser.add_argument('--outbound_sin_freq', help=" todo ", type=float, default=0.1)
    parser.add_argument('--t_familiar', help=" todo ", type=float, default=1.0)
    parser.add_argument('-v', '--vel_tgt', type=float, default=1.0)
    parser.add_argument('-m', '--multiplier', type=float, default=4.0,
                            help='inbound route timeout in multiples of the outbound route')
    parser.add_argument('-s', '--amplitude', type=float, default=10.0)
    parser.add_argument('--mission_type', default='active', choices=['active', 'passive'],
                            help='Whether we make use of the familiarity signal')
    parser.add_argument('-x', '--x_offset', type=float, default=0.0)

    # TORF parameters
    parser.add_argument('-r', '--termination_slowdown', type=bool, default=True, help='do we slow down as we get close to the end of the run')
    parser.add_argument('-e', '--close_to_end_thresh_high', type=int, default=25, help='the index where we consider to be getting close to the nest')
    parser.add_argument('-g', '--close_to_end_thresh_low', type=int, default=8, help='the index where we consider to be at or near the nest')
    parser.add_argument('-k', '--slowdown_factor', type=float, default=0.7, help='A factor for increasing the slowdown rate should be in range [0 1] - with a lower number being a greater slowdown')
    parser.add_argument('-l', '--t_turnaround', type=float, default=10, help="How long we wait at the turn around point for - increase if the camera feed is still blurry at this point")

    # arguments for visual homing algorithm
    parser.add_argument('-q', '--camera_topic', type=str, default='/resize_img/image', help='name of the camera topic')
    parser.add_argument('-p', '--multi_process', type=bool, default=True)
    parser.add_argument('-i', '--outbound_images', type=int, default=180, help='how many outbound images will be stored')
    parser.add_argument('-t', '--cwssim_thresh', type=float, default=0.82, help='minimum score before we consider a match to be on the route')

    # torf and camera parameters
    parser.add_argument('--flip_images', type=str2bool, default=True)
    parser.add_argument('--winsize', type=int, default=7)
    parser.add_argument('--im_w', type=int, default=235, help="Camera resolution (horizontal)")
    parser.add_argument('--im_h', type=int, default=150, help="Camera resolution (vertical)")
    parser.add_argument('--resize_w', type=int, default=None, help="Resize camera image width (internal to torf_ros module)")
    parser.add_argument('--resize_h', type=int, default=None, help="Resize camera image height (internal to torf_ros module)")
    parser.add_argument('--levels', type=int, default=5, help="Camera resolution (vertical)")
    parser.add_argument('--nbands', type=int, default=2, help="Camera resolution (vertical)")

    if not return_args:
        return

    if started_by_ros:
        args = parser.parse_args(rospy.myargv(argv=sys_argv_in)[1:])
    else:
        args = parser.parse_args(rospy.myargv(argv=sys_argv_in))

    # by default the outbound height is the same as the inbound height
    if np.isnan(args.outbound_hgt):
        args.outbound_hgt = args.mission_hgt
        print ('Using the same mission heights outbound:{} inbound:{}'.format(args.outbound_hgt, args.mission_hgt))
    else:
        print ('Using a different mission heights outbound:{} inbound:{}'.format(args.outbound_hgt, args.mission_hgt))

    return args
