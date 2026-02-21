# Argus Gate - argus_interfaces/msg/PlcCommand.msg
# Copyright (c) 2026, Luca-G49
# All rights reserved. Licensed under MIT License.

uint16 life_word
bool ack
bool exec
bool fire
bool pitch_jog_p
bool pitch_jog_n
bool yaw_jog_p
bool yaw_jog_n
int16 mode
float32  pitch_override
float32  yaw_override
float32  target_pitch
float32  target_yaw
uint16 checksum
