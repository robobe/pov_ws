#/bin/sh

src/pov_bringup/bin/ardurover.4.5.7 -S \
--model JSON \
--speedup 1 \
--slave 0 \
--defaults `pwd`/src/pov_bringup/config/rover.param,`pwd`/src/pov_bringup/config/rover-skid.param  \
-I0 \
-sim-address=127.0.0.1

#/home/user/git/ardupilot/build/sitl/bin/ardurover -S --model JSON --speedup 1 --slave 0 --defaults /home/user/workspaces/pov_ws/src/pov_bringup/config/rover.param,/home/user/workspaces/pov_ws/src/pov_bringup/config/rover-skid.param --sim-address=127.0.0.1 -I0


# src/pov_bringup/bin/ardurover" "-S" \
# "--model" "JSON" \
# "--speedup" "1" \
# "--slave" "0" \
# "--defaults" "Tools/autotest/default_params/rover.parm,Tools/autotest/default_params/rover-skid.parm,/home/user/Downloads/my.param" \
# "--sim-address=127.0.0.1" "-I0"


##

