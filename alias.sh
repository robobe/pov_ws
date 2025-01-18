alias dev="tmuxp load src/pov_bringup/tmux/dev.yaml"
alias aruco="tmuxp load src/pov_bringup/tmux/aruco.yaml"
alias diff="tmuxp load src/pov_bringup/tmux/diff.yaml"

alias kill_gz="kill -9 \`ps -e -o pid,command | grep '[g]z sim' | awk '{print $1}'\`"