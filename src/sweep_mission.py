#!/usr/bin/env python

from __future__ import division

from flight_states import Sweep_wave, Sweep_wave_familiarity
from pyx4_base.pyx4_base import Pyx4_base

from cx_model_px4.flight_behaviours import Sin_wave_2  # todo - this should be moved to pyx4
from pyx4_base.mission_states import *
from mavros_msgs.msg import PositionTarget

from argparse_sweep_mission import *
from utils import pol2cart


def generate_sweep_mission(
                args_in,
                 ):

    """
    generates a parametised mission that flies out with a target heading, then flies back to the tack off location in a
    zig-zag fashion but with a heading that is opposite to the outbound direction (i.e. plus pi), this means that on the
    return path the heading direction non-aligned with the craft's velocity velocity

    purpose is to test whether images on the outbound path are more familiar than those on the inbound path

    :param args_in:
    :return:
    """

    # initialise instruction vars
    instructions = {}
    instruction_cnt = 0

    ################# new instruction
    instructions[instruction_cnt] = Arming_state(
                                        # instruction_type='take_off',
                                        # instruction_args={'to_altitude_tgt':args_in.height},
                                        timeout=90.0
                                        )
    instruction_cnt += 1

    ################# new instruction
    instructions[instruction_cnt] = Take_off_state(
                                         # instruction_type='hold',
                                         to_altitude_tgt=args_in.outbound_hgt,
                                         yaw_type='pos',
                                         heading_tgt_rad=args_in.hdg_out,
                                         )
    instruction_cnt += 1


    # if we have an offset
    if abs(args_in.x_offset) > 1e-6:
        # calculate the offset in local coordinates based on the target heading
        x_offset_setpoint = np.cos(args_in.hdg_out) * args_in.x_offset
        y_offset_setpoint = np.sin(args_in.hdg_out) * args_in.x_offset
        extra_travel_time = 5 + (args_in.x_offset * args_in.vel_tgt) + 2   # extra time to settle at the starting point
    else:
        x_offset_setpoint = 0
        y_offset_setpoint = 0
        extra_travel_time = 5

    ################# new instruction, hover at departure point
    instructions[instruction_cnt] = Waypoint_state(
        state_label='Hovering at Offset',
        waypoint_type='hold',
        timeout=5 + extra_travel_time,
        xy_type='pos',
        x_setpoint=x_offset_setpoint,
        y_setpoint=y_offset_setpoint,
        z_type='pos',
        z_setpoint=args_in.outbound_hgt,
        yaw_type='pos',
        yaw_setpoint=args_in.hdg_out,
        coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
    )
    instruction_cnt += 1

    ################# new instruction, Head out
    x_tgt, y_tgt = pol2cart(args_in.vel_tgt, args_in.hdg_out)
    if args_in.outbound_configuration == 'fixed_heading':
        instructions[instruction_cnt] = Waypoint_state(
                                            state_label='Head_out',   # do not chnage this label - used for analysis
                                            waypoint_type='vel',
                                            timeout=args_in.duration_out,
                                            xy_type='vel',
                                            x_setpoint=x_tgt,
                                            y_setpoint=y_tgt,
                                            z_type='pos',
                                            z_setpoint=args_in.outbound_hgt,
                                            yaw_type='pos',
                                            yaw_setpoint=args_in.hdg_out,
                                            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                     )
    elif args_in.outbound_configuration == 'sin_wave_2':
        instructions[instruction_cnt] = Sin_wave_2(
            state_label='Head_out',
            mission_z_tgt=args_in.outbound_hgt,
            mission_vel_tgt=args_in.vel_tgt,
            heading_tgt_rad=args_in.hdg_out,
            timeout=args_in.duration_out,
            # inhibit_turning=INHIBIT_TURNING_MASK(args_in.inhibit_turning),
            # max_nest_dist=args_in.max_nest_distance,
            sin_freq=args_in.outbound_sin_freq,
        )
    else:
        raise AttributeError ('unknown mission config {}'.format(args.outbound_configuration))

    instruction_cnt += 1

    if args_in.flip_images:
        ################# new instruction, hover at return point
        instructions[instruction_cnt] = Hold_pos_state(
            state_label='Turn around time',   # do not chnage this label - used for analysis
            xy_type='vel',
            x_setpoint=0.0,
            y_setpoint=0.0,
            z_setpoint=args_in.mission_hgt,
            yaw_setpoint=args_in.hdg_out + np.pi,
            timeout=args_in.t_turnaround,
        )
        instruction_cnt += 1
    else:
        ################# new instruction, return to departure point
        instructions[instruction_cnt] = Waypoint_state(
            state_label='Return to start',
            waypoint_type='pos',
            timeout=args_in.duration_out + 5,
            xy_type='pos',
            x_setpoint=x_offset_setpoint,
            y_setpoint=y_offset_setpoint,
            z_type='pos',
            z_setpoint=args_in.outbound_hgt,
            yaw_type='pos',
            yaw_setpoint=args_in.hdg_out,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt += 1
        instructions[instruction_cnt] = Waypoint_state(
            state_label='Hovering at Offset',
            waypoint_type='hold',
            timeout=5,
            xy_type='pos',
            x_setpoint=x_offset_setpoint,
            y_setpoint=y_offset_setpoint,
            z_type='pos',
            z_setpoint=args_in.outbound_hgt,
            yaw_type='pos',
            yaw_setpoint=args_in.hdg_out,
            coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
        )
        instruction_cnt += 1
        args_in.hdg_out = args_in.hdg_out + np.pi  # flip by 180 to counteract the presumed flip in the sidesweep part

    ################# new ins   truction, depart at fixed heading
    if args_in.mission_type == 'active':
        instructions[instruction_cnt] = Sweep_wave_familiarity(
            state_label='Sweep_state',      # do not chnage this label - used for analysis
            timeout=args_in.duration_out * args_in.multiplier,
            mission_z_tgt=args_in.mission_hgt,
            heading_tgt_rad=args_in.hdg_out + np.pi,
            mission_vel_tgt=args_in.vel_tgt,
            amplitude=args_in.amplitude,
            cwssim_thresh=args_in.cwssim_thresh,
            close_to_end_thresh_high=args_in.close_to_end_thresh_high,
            close_to_end_thresh_low=args_in.close_to_end_thresh_low,
            slow_down_at_end=args_in.termination_slowdown,
            angle_of_attack_degs=args_in.angle_of_attack,
            search_angle_of_attack_degs=args_in.search_angle_of_attack,
            slowdown_factor=args_in.slowdown_factor,
            t_familiar=args_in.t_familiar,
            hdg_offset=args_in.hdg_in_offset,
        )
    elif args_in.mission_type == 'passive':
        instructions[instruction_cnt] = Sweep_wave(
            state_label='Sweep_state',
            timeout=args_in.duration_out * args_in.multiplier,
            mission_z_tgt=args_in.mission_hgt,
            heading_tgt_rad=args_in.hdg_out + np.pi,
            mission_vel_tgt=args_in.vel_tgt,
            amplitude=args_in.amplitude,
        )
    else:
        raise ValueError('unkown mission type: {}'.format(args_in.mission_type))
    instruction_cnt += 1

    ################# new instruction, hover at final state
    instructions[instruction_cnt] = Hold_pos_state(
        state_label='Hovering at completion',
        yaw_setpoint=args_in.hdg_out + np.pi,
        timeout=20,
    )
    instruction_cnt += 1

    ################# new instruction, hover at departure point
    instructions[instruction_cnt] = Waypoint_state(
        state_label='Return_home',
        waypoint_type='pos',
        timeout=60,
        xy_type='pos',
        x_setpoint=0,
        y_setpoint=0,
        z_type='pos',
        z_setpoint=args_in.mission_hgt,
        yaw_type='pos',
        yaw_setpoint=args_in.hdg_out,
        coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
    )
    instruction_cnt += 1

    ################# landing instructions
    instructions[instruction_cnt] = Landing_state(
        instruction_type='land',
        timeout=30,
    )
    instruction_cnt += 1

    ################# landing instructions
    instructions[instruction_cnt] = Post_run_state(
        instruction_type='post_run',
        timeout=30,
    )
    instruction_cnt += 1

    # todo - check that instruction keys do not have nay numerical gaps (perhaps do this in pyx4)
    return instructions


if __name__ == '__main__':

    args = sm_argparse(sys_argv_in=sys.argv)

    flight_instructions = generate_sweep_mission(args_in=args)

    rospy.init_node('pyx4_parametised_node', anonymous=True, log_level=rospy.DEBUG)

    pyx4 = Pyx4_base(flight_instructions=flight_instructions)
    pyx4.run()

