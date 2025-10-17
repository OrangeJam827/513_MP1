"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple


# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold
        self.k_cruise = 0.5
        self.k_follow = 0.3

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        """This is the main run step of the controller.

        Here, you will have to read in the observations `obs`, process it, and output an
        acceleration value and the operation mode.
        
        The acceleration value must be some value between -10.0 and 10.0.

        For the operation mode output True when "following" and False when "cruising".

        Note that the acceleration value is really some control input that is used
        internally to compute the throttle an brake values of the car.

        Below is some example code where the car just outputs the control value 10.0
        """

        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead

        # set defaut cruising
        mode = Mode.CRUISING
        acceleration = 0.0

        if dist_to_lead > self.distance_threshold:
            # cruising mode
            speed_error = desired_speed - ego_velocity
            acceleration = self.k_cruise * speed_error
            mode = Mode.CRUISING
        else:
            # following mode, keep the safety distance
            distance_error = dist_to_lead - self.distance_threshold
            acceleration = self.k_follow * distance_error
            mode = Mode.FOLLOWING

        # limit the acceleration
        acceleration = max(min(acceleration, 10.0), -10.0)

        return acceleration, mode