# Bring up

## Run SITL

### External RUN

```bash
/home/user/git/ardupilot/build/sitl/bin/ardurover -S \
--model JSON \
--speedup 1 \
--slave 0 \
--defaults /home/user/workspaces/pov_ws/src/pov_bringup/config/rover.param,/home/user/workspaces/pov_ws/src/pov_bringup/config/rover-skid.param \
--sim-address=127.0.0.1 -I0
```

## Bridge
```
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspaces/pov_ws/src/pov_bringup/config/bridge.yaml
```