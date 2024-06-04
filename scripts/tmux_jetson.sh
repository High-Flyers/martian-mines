#!/bin/bash
session="martian-mines"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'docker-main'
tmux send-keys -t $session:$window 'cd /home/hf/Documents/martian-mines/docker && sudo docker compose up -d && sudo docker compose exec martian-mines /bin/bash' C-m

window=1
tmux new-window -t $session:$window -n 'docker-extra'
tmux send-keys -t $session:$window 'sleep 10 && cd /home/hf/Documents/martian-mines/docker && sudo docker compose exec martian-mines /bin/bash' C-m

window=2
tmux new-window -t $session:$window -n 'mavlink'
tmux send-keys -t $session:$window 'sudo mavlink-routerd /dev/ttyTHS0:921600 -e 127.0.0.1:14550 -e 10.42.1.1:14550' C-m

tmux attach-session -t $session
