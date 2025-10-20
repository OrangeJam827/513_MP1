"""This file is the main controller file.
Here, you will design the controller for your for the adaptive cruise control system.
"""
from mp1_simulator.simulator import Observation, Mode
from typing import Tuple

class Controller:
    def __init__(self, distance_threshold: float, dt: float = 0.1):
        # distance_threshold (4m) is used as minimum safe distance
        self.min_safe_distance = distance_threshold
        self.dt = dt
        # Mode switching threshold must be 40 meters
        self.mode_switch_threshold = 40.0
        
        # Control parameters
        self.time_gap = 1.2  # Time gap in seconds
        self.kp_speed = 2.5  # Speed control gain (increased for faster target speed reaching)
        self.kp_distance = 1.2  # Distance control gain
        self.kd_distance = 0.6  # Derivative control gain
        
        # Memory variables
        self.prev_distance_to_lead = None
        self.prev_ego_velocity = None
        
    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead
        
        # === Estimate relative velocity and lead vehicle speed ===
        relative_velocity = 0.0
        lead_velocity_est = ego_velocity
        
        if self.prev_distance_to_lead is not None:
            # Rate of distance change = relative velocity
            relative_velocity = (dist_to_lead - self.prev_distance_to_lead) / self.dt
            # Lead vehicle speed = ego velocity + relative velocity
            lead_velocity_est = ego_velocity + relative_velocity
            # Limit estimation range
            lead_velocity_est = max(0.0, min(lead_velocity_est, desired_speed * 1.5))
        
        # === Dynamic safe distance ===
        dynamic_safe_distance = self.min_safe_distance + self.time_gap * ego_velocity
        
        # === Mode decision
        if dist_to_lead > self.mode_switch_threshold:
            mode = Mode.CRUISING
        else:
            mode = Mode.FOLLOWING
        
        # === Control logic ===
        if mode == Mode.CRUISING:
            # Cruising mode: quickly reach target speed
            speed_error = desired_speed - ego_velocity
            acceleration = self.kp_speed * speed_error
            
        else:  # FOLLOWING mode
            distance_error = dist_to_lead - dynamic_safe_distance
            
            # Decide behavior based on lead vehicle speed and distance error
            if lead_velocity_est > 0.5:  # Lead vehicle is moving
                
                if distance_error > 3.0:
                    # Sufficient distance, can match or approach target speed
                    target_velocity = min(lead_velocity_est * 1.05, desired_speed)
                    speed_error = target_velocity - ego_velocity
                    
                    # Combine distance control and speed control
                    distance_component = self.kp_distance * distance_error + self.kd_distance * relative_velocity
                    speed_component = self.kp_speed * speed_error * 0.6
                    
                    acceleration = min(distance_component, speed_component)
                    
                elif distance_error > 0:
                    # Moderate distance, try to match lead vehicle speed
                    speed_error = lead_velocity_est - ego_velocity
                    acceleration = (
                        self.kp_speed * speed_error * 0.5 
                        + self.kd_distance * relative_velocity
                    )
                    
                else:
                    # Too close but lead is moving, gentle deceleration
                    acceleration = (
                        self.kp_distance * distance_error 
                        + self.kd_distance * relative_velocity
                    )
                    
            else:  # Lead vehicle has stopped or is very slow
                if distance_error < -2.0:
                    # Too close, need to decelerate
                    acceleration = self.kp_distance * distance_error * 1.5
                elif ego_velocity > 0.3:
                    # Distance is okay but still moving, slowly come to stop
                    acceleration = -ego_velocity * 1.2
                else:
                    # Velocity is very low, maintain stopped state
                    acceleration = 0.0
        
        # === Limit acceleration range ===
        acceleration = max(-10.0, min(10.0, acceleration))
        
        # === Anti-rollback protection ===
        if ego_velocity < 0.2 and acceleration < 0:
            max_decel = -ego_velocity / self.dt
            acceleration = max(acceleration, max_decel)
        
        # === Update memory ===
        self.prev_distance_to_lead = dist_to_lead
        self.prev_ego_velocity = ego_velocity
        
        return acceleration, mode