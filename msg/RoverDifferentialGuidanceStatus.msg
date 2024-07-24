uint64 timestamp # time since system start (microseconds)

float32 desired_speed 	      # [m/s] Desired forward speed for the rover
float32 lookahead_distance    # [m] Lookahead distance of pure the pursuit controller
float32 heading_error 	      # [deg] Heading error of the pure pursuit controller
float32 pid_heading_integral  # Integral of the PID for the desired yaw rate during missions
float32 pid_throttle_integral # Integral of the PID for the throttle during missions

# TOPICS rover_differential_guidance_status
