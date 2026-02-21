# Argus Gate - argus_interfaces/msg/TeleopCOmmand.msg
# Copyright (c) 2026, Luca G49
# All rights reserved. Licensed under MIT License.

float32 pitch_speed
float32 yaw_speed
float32 target_pitch
float32 target_yaw
int16 requested_mode
bool send_target
bool execute_action
bool reset_error