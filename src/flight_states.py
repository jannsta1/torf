from copy import copy
import rospy

from pyx4_base.mission_states import *
from pyx4_base.setpoint_bitmasks import MASK_XY_VEL__Z_POS__YAW_POS
from utils import pol2cart


class Sweep_wave(Generic_mission_state):
    """
    Flies in a zig-zag fashion but with a fixed heading resulting in a potentially non-aligned heading and velocity

    purpose is to test whether images on the outbound path are more familiar than those on the inbound path

    :param args_in:
    :return:
    """

    def __init__(self,
                 state_label='sweep wave',
                 flight_instruction_type='Sweep_wave',
                 timeout=40.0,
                 mavros_message_node=None,
                 mission_z_tgt=2.0,
                 mission_vel_tgt=1.0,
                 heading_tgt_rad=0.0,
                 hdg_offset=0.0,                # offset between heading (of direction of travel) and drone angle
                 parent=None,
                 angle_of_attack_degs=70.0,
                 search_angle_of_attack_degs=70.0,
                 amplitude=10,
                 **kwargs
                 ):

        super(Sweep_wave, self).__init__(
                                            flight_instruction_type=flight_instruction_type,
                                            state_label=state_label,
                                            timeout=timeout,
                                            mavros_message_node=mavros_message_node,
                                            mission_z_tgt=mission_z_tgt,
                                            mission_vel_tgt=mission_vel_tgt,
                                            parent_ref=parent,
                                            **kwargs
                                          )  # sub and super class args

        self.heading_tgt_rad = heading_tgt_rad
        self.mission_z_tgt = mission_z_tgt
        self.mission_vel_tgt = mission_vel_tgt

        self.new_sample = False
        self.amplitude = amplitude

        self.angle_of_attack_degs = angle_of_attack_degs
        self.search_angle_of_attack_degs = search_angle_of_attack_degs   # we can set a different angle of attack if we are off route
        self.sweep_direction = 1                               # [1 or -1] to set the direction of the sweep wave
        self.hdg_offset = hdg_offset
        self.this_hdg_tgt = 0.0

        self.set_heading(custom_angle_of_attack=self.search_angle_of_attack_degs)

    def set_heading(self, custom_angle_of_attack=None):

        if custom_angle_of_attack:
            print('custom')
            self.this_hdg_tgt = self.heading_tgt_rad + (self.sweep_direction * np.deg2rad(custom_angle_of_attack))
        else:
            print('not custon')
            self.this_hdg_tgt = self.heading_tgt_rad + (self.sweep_direction * np.deg2rad(self.angle_of_attack_degs))

        self.sweep_direction *= -1
        self.x_tgt, self.y_tgt = pol2cart(self.mission_vel_tgt, self.this_hdg_tgt)

    def precondition_check(self):

        # todo: log this event in cx model
        self.t_start_s = rospy.get_time()
        self.set_new_heading_timer(self.amplitude/2.0)
        self.preconditions_satisfied = True

    def set_new_heading_timer(self, duration):
        self.new_heading_timer = rospy.Timer(rospy.Duration(duration), self.new_heading_callback, oneshot=True)

    def new_heading_callback(self, event):
        """
        callback from new heading timer creates and event to initialise a new sweep direction
        :param event:
        :return:
        """
        self.new_sample = True

    def step(self):

        if self.new_sample:
            rospy.loginfo('new heading sample')
            self.new_sample = False
            self.new_heading_timer.shutdown()
            self.set_new_heading_timer(self.amplitude)
            self.set_heading()

        # publish new setpoint
        self.x_vel = self.x_tgt
        self.y_vel = self.y_tgt

        self.z = self.mission_z_tgt # height_interp(t_journey_s)
        self.yaw = self.heading_tgt_rad          # could also use self.bearing_to_nest(), but angle could be more different to way out
        self.coordinate_frame = 1
        self.type_mask = MASK_XY_VEL__Z_POS__YAW_POS # 1027


class Sweep_wave_familiarity(Sweep_wave):
    """
    This modulated version of the sweep wave is as per the Sweep wave but the reverse in direction of the sweep wave
    can be instigated based on the state of the visual processing function. Here torf is used to evaluate the view
    familiarity and the duration of the sweepback is calculated based on the location of the previous best score


        :param state_label:
        :param timeout:
        :param mavros_message_node:
        :param mission_z_tgt:
        :param mission_vel_tgt:
        :param heading_tgt_rad:
        :param parent:
        :param max_nest_dist:
        :param angle_of_attack_degs:
        :param amplitude:
        :param kwargs:
    """

    def __init__(self,
                 flight_instruction_type='Sweep_wave_modulated',
                 state_label='sweep_wave_modulated',
                 timeout=100.0,
                 mavros_message_node=None,
                 mission_z_tgt=2.0,
                 mission_vel_tgt=1.0,
                 heading_tgt_rad=0.0,
                 hdg_offset=0.0,
                 parent=None,
                 slow_down_at_end=True,
                 angle_of_attack_degs=70,
                 search_angle_of_attack_degs=70,
                 amplitude=10,
                 cwssim_thresh=0.82,
                 t_familiar=1.0,              # minimum time after peak familiarity before switchback
                 close_to_end_thresh_high=40,
                 close_to_end_thresh_low=10,
                 lat_slow_down_threshold_pc=0.9,  # the threshold of percentage of lateral motion (expressed in time) before lateral slowdown is initialised
                 slowdown_factor=0.5,
                 **kwargs
                 ):

        super(Sweep_wave_familiarity, self).__init__(
                                            flight_instruction_type=flight_instruction_type,
                                            state_label=state_label,
                                            timeout=timeout,
                                            mavros_message_node=mavros_message_node,
                                            mission_z_tgt=mission_z_tgt,
                                            mission_vel_tgt=mission_vel_tgt,
                                            heading_tgt_rad=heading_tgt_rad,
                                            parent=parent,
                                            angle_of_attack_degs=angle_of_attack_degs,
                                            search_angle_of_attack_degs=search_angle_of_attack_degs,
                                            amplitude=amplitude,
                                            hdg_offset=hdg_offset,
                                            **kwargs
                                          )  # sub and super class args


        # parse variables
        self.cwssim_thresh = cwssim_thresh
        self.slow_down_at_end = slow_down_at_end                        # logic flag
        self.close_to_end_thresh_high = close_to_end_thresh_high        # logic flag
        self.close_to_end_thresh_low = close_to_end_thresh_low          # logic flag
        self._tgt_event_duration = amplitude
        self._lat_slow_down_threshold_pc = lat_slow_down_threshold_pc
        self.t_familiar = t_familiar

        # initialise variables
        self.reset_time_since_best_score()
        self.reset_time_since_familiar()
        self.this_best_cwssim = 0.0                      # the best torf score for the latest image from our camera
        self.last_best_cwssim = 0.0                      # the best torf score for all images in this sweep
        self.this_best_cwssim_idx = 100                  # todo, as below
        self.last_best_cwssim_idx = 100                  # self._parent_ref.qty_images_stored # todo why doesn't this work? I.e. want to set this to the number of images stored in the image bank
        self.best_cwssim_idx_previous_sweep = 88888      # todo, as above

        self.on_route_counter = 0                        # how many consecutive turns have we been on the route for?
        self._ros_time_of_last_evt = rospy.get_time()
        self._longitudinal_slowdown_factor = 1.0         # initially there should be no longitudinal slowdown
        self.on_last_sweep = False

        self.cwssim_exceeded_this_sweep = False
        self.sweep_counter = 0                           # counts how many sweeps have been completed
        self.new_sample_if_cwssim_exceeded = False

        self.slowdown_factor = slowdown_factor

    def precondition_check(self):

        # todo: log this event in cx model
        self.t_start_s = rospy.get_time()
        rospy.logdebug('starting first switch back which is half the usual length')
        self.new_sample = False
        self.set_new_heading_timer(self.amplitude/2.0)
        # self.set_new_truncated_heading_timer(self.amplitude)   # we don't want
        self.preconditions_satisfied = True

    @property
    def ros_time_since_best_score(self):
        return rospy.get_time() - self.ros_time_at_best_score

    @property
    def ros_time_since_familiar(self):
        return rospy.get_time() - self.ros_time_at_familiar

    @property
    def on_route(self):
        """
        Used to determine if we are on a route by counting how many contiguous sweeps have passed the on route threshold
        :return:
        """
        return self.on_route_counter > 2


    def reset_time_since_best_score(self):
        """
        uses the ros time to specify when the best score was acquired
        :return:
        """
        self.ros_time_at_best_score = rospy.get_time()

    def reset_time_since_familiar(self):
        """
        uses the ros time to specify when the best score was acquired
        :return:
        """
        self.ros_time_at_familiar = rospy.get_time()


    def set_new_heading_timer(self, duration):
        """
        set timer for this sweep
        :param duration:
        :return:
        """
        self._ros_time_of_last_evt = rospy.get_time()
        self._tgt_event_duration = duration
        self.new_heading_timer = rospy.Timer(rospy.Duration(duration), self.new_heading_callback, oneshot=True)


    def begin_new_sidesweep(self, evt_type='unspecified'):

        rospy.loginfo('new heading sample due to {} - best torf score was {} with {} idx occurd {}s ago'
                      .format(evt_type, self.this_best_cwssim, self.this_best_cwssim_idx, self.ros_time_since_best_score))

        rospy.loginfo('t familiar {}'.format(self.t_familiar))
        rospy.loginfo('cwssim_exceeded_this_sweep {}'.format(self.cwssim_exceeded_this_sweep))

        self.new_heading_timer.shutdown()
        # self.new_truncated_heading_timer.shutdown()
        self.set_new_heading_timer(self.amplitude)

        if self.cwssim_exceeded_this_sweep:
            self.set_heading()
        else:
            self.set_heading(custom_angle_of_attack=self.search_angle_of_attack_degs)

        ## update parent ref so that data can be sent in the ros update message
        self._parent_ref.sweep_index = self.sweep_counter
        self._parent_ref.best_cwssim_score_previous_run = self.this_best_cwssim

        # reset our flags for the next sweep
        self.last_best_cwssim = self.this_best_cwssim
        self.best_cwssim_idx_previous_sweep = self.this_best_cwssim_idx
        self.this_best_cwssim = 0.0
        self.reset_time_since_best_score()
        self.reset_time_since_familiar()
        self.sweep_counter += 1
        self.new_sample = False
        self.new_sample_if_cwssim_exceeded = False
        self.cwssim_exceeded_this_sweep = False

        if self.last_best_cwssim_idx < self.close_to_end_thresh_low:
            self.on_last_sweep = True

    def quit_state_final_sweep_completed(self, message=""):

        # stop aircraft before next state starts
        self.x_tgt = 0.0
        self.y_tgt = 0.0
        self.stay_alive = False
        rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!! Sweep_wave_familiarity - Mission success !!!!!!!!!!!!!!!!!!!!!!!!')
        rospy.loginfo('Visual navigation - at home - {}'.format(message))
        rospy.loginfo('Setting XY velocity to 0')
        rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!! Mission success !!!!!!!!!!!!!!!!!!!!!!!!')

    def step(self):
        """
        main per iteration behaviour - the following informal state machine provides logic for initilising switchbacks
        based on the discovery of familiar patches in the previous sidesweep
        :return:
        """

        if self.new_sample:
            if not self.on_last_sweep:
                self.begin_new_sidesweep(evt_type='full timeout')
            else:
                self.quit_state_final_sweep_completed(message="timed out - full timer")

        if self.cwssim_exceeded_this_sweep:
            if not self.on_last_sweep:
                if self.ros_time_since_familiar > self.t_familiar:
                    self.begin_new_sidesweep(evt_type='short timeout')
            else:
                self.quit_state_final_sweep_completed(message="timed out - truncated sweep")

        # a method for terminating at the last sweep - strategy is to stop once the torf threshold is exceeded
        if self.on_last_sweep:
            if self.cwssim_exceeded_this_sweep:
                self.quit_state_final_sweep_completed(
                    message="found a high torf {} idx {}".format(self.this_best_cwssim, self.this_best_cwssim_idx))

        ## longitudinal slow down calculations
        # if our index is below a threshold (close_to_end_thresh_high) ramp the longitudinal speed down. If we are on
        # the last sweep we stop longitudinal speed altogether
        if self.slow_down_at_end:
            if self.best_cwssim_idx_previous_sweep < self.close_to_end_thresh_high:
                if self.on_last_sweep:
                    self._longitudinal_slowdown_factor = 0.0
                else:
                    # self._longitudinal_slowdown_factor = self.last_best_cwssim_idx / self.close_to_end_thresh_high
                    self._longitudinal_slowdown_factor = (self.best_cwssim_idx_previous_sweep / self.close_to_end_thresh_high) * self.slowdown_factor
                    self._longitudinal_slowdown_factor = np.clip(self._longitudinal_slowdown_factor, 0.2, self.mission_vel_tgt)
            else:
                # print ('this best torf: {} close tro end thresh {}'.format(self.this_best_cwssim_idx, self.close_to_end_thresh_high))
                self._longitudinal_slowdown_factor = 1.0
        else:
            self._longitudinal_slowdown_factor = 1.0

        # we only need to do this section when a new image comes in but there was no trivial way to detect this at the time of writing
        self.this_data_copy = copy(self._parent_ref.cwssim_score)   # (probably overkill but just to make sure the data doesn't change between checking and setting the best score - would be better to set a lock)
        self.this_data_idx_copy = copy(self._parent_ref.cwssim_score_idx)   # (probably overkill but just to make sure the data doesn't change between checking and setting the best score)
        if self.this_data_copy > self.this_best_cwssim:
            self.this_best_cwssim = self.this_data_copy
            self.this_best_cwssim_idx = self.this_data_idx_copy
            self.last_best_cwssim_idx = self.this_best_cwssim_idx
            self.reset_time_since_best_score()
        if self.this_best_cwssim > self.cwssim_thresh:
            self.cwssim_exceeded_this_sweep = True

        if self.this_data_copy> self.cwssim_thresh:
            self.reset_time_since_familiar()
        # print ('last best idx: {} slowdown factor {}'.format(self.best_cwssim_idx_previous_sweep, self._longitudinal_slowdown_factor))

        # publish new setpoint
        # varying setpoint data:
        self.x_vel = self.x_tgt * self._longitudinal_slowdown_factor
        self.y_vel = self.y_tgt
        # static setpoint data:
        self.z = self.mission_z_tgt
        self.yaw = self.heading_tgt_rad + self.hdg_offset
        self.coordinate_frame = 1
        self.type_mask = MASK_XY_VEL__Z_POS__YAW_POS