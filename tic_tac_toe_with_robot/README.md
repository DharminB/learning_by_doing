# Tic Tac Toe with robot

# Instructions

Changes before launch `mir_bringup` and `mir_moveit_youbot_brsu_1`
- Remove gripper controller block in
  `mir_manipulation/mir_moveit_youbot_brsu_1/config/controllers.yaml`
- Change param "youBotHasBase" to "false" in
  `mir_robots/mir_bringup/components/youbot_oodl_driver.launch`
- Change param "is_camera_required" to "false" in
  `mir_robots/mir_bringup/robot.launch`
- [optional] Change from `/dev/input/js0` to `/dev/input/js1` in
  `mir_hardware_config/youbot-brsu-1/config/joy.yaml`

```
roslaunch mir_bringup robot.launch
roslaunch mir_moveit_youbot_brsu_1 move_group.launch
roslaunch mir_audio_receiver tts_request_executor.launch
roslaunch tic_tac_toe_with_robot play_tic_tac_toe.launch
```
