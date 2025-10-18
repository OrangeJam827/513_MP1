"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple


# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, distance_threshold: float, dt: float = 0.1):
        self.distance_threshold = distance_threshold
        self.prev_distance_to_lead = None
        self.dt = dt  # simulation timestep

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead

        mode = Mode.CRUISING
        acceleration = 0.0

        # detect if ego is closing gap
        closing_gap = (
            self.prev_distance_to_lead is not None
            and dist_to_lead < self.prev_distance_to_lead
        )

        # --- CASE 1: Within following threshold ---
        if dist_to_lead <= self.distance_threshold:
            mode = Mode.FOLLOWING

            if ego_velocity > 0.10:
                # decelerate if too close
                acceleration = -(ego_velocity - 0.10)*0.6
            else:
                if closing_gap:
                    # decelerate if closing gap
                    acceleration = -0.5
                else:
                    # maintain speed to increase gap
                    acceleration = 0.0 

        # --- CASE 2: Far from lead (cruising mode) ---
        else:
            mode = Mode.CRUISING
            speed_error = desired_speed - ego_velocity
            if ego_velocity != desired_speed:
                acceleration = min(speed_error* 0.2, dist_to_lead - self.distance_threshold)
            else:
                acceleration = 0.0

        # clip acceleration to physical limits
        acceleration = max(min(acceleration, 10.0), -10.0)

        # update memory
        self.prev_distance_to_lead = dist_to_lead

        return acceleration, mode
