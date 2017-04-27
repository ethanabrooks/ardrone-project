#! /usr/bin/env bash

set -e
logdir=ardrone
spec_path=spec.json
session=a3c

while [[ $# -gt 1 ]]; do
  key="$1"

  case $key in
      -l|--log-dir)
        logdir="$2"
        shift # past argument
      ;;
      -s|--spec-path)
        spec_path="$2"
        shift # past argument
      ;;
      -t|--target-session)
        session="$2"
        shift # past argument
      ;;
      *)
      ;;
  esac
  shift # past argument or value
done

workers=$(cat $spec_path | jq -cr "{worker}[]" | tr -d '[]"')
ps=$(cat $spec_path | jq -cr "{ps}[]" | tr -d "[]\"'")
num_workers=$(cat $spec_path | jq "{worker}[]|length")

source catkin/devel/setup.bash
roscd a3c
rm -rf $logdir && true
docker build ~/ardrone-project/ -t ardrone

kill $( lsof -i:12345 -t ) > /dev/null 2>&1 && true
kill $( lsof -i:12222-12223 -t ) > /dev/null 2>&1 && true 
for i in $(seq 0 $(($num_workers - 1))); do
  docker kill w-$i && true
done

echo Creating tmux session and windows...
tmux kill-session -t $session && true
tmux new-session -s $session -n ps -d bash
tmux new-window -t $session -n tb bash
tmux new-window -t $session -n htop bash
for i in $(seq 0 $(($num_workers - 1))); do
  tmux new-window -t $session -n w-$i bash
done
sleep 1

echo Executing commands in TMUX
tmux send-keys -t a3c:ps\
 "CUDA_VISIBLE_DEVICES= $(pwd)/job.py\
 --log-dir $logdir\
 --env-id gazebo\
 --num-workers $num_workers\
 --job-name ps\
" Enter

for i in $(seq 0 $(($num_workers - 1))); do
  tmux send-keys -t a3c:w-$i\
 "docker run -it --rm --name=w-$i --net=host\
 ardrone /start.sh false \
'--log-dir $logdir\
 --env-id gazebo\
 --num-workers $num_workers\
 --task $i\
 --remote 1\
 --workers $workers\
 --ps $ps\
'" Enter
done

tmux send-keys -t a3c:tb 'tensorboard --logdir ardrone --port 12345' Enter
tmux send-keys -t a3c:htop 'htop' Enter

echo 'Use `tmux attach -t a3c` to watch process output
Use `tmux kill-session -t a3c` to kill the job
Point your browser to http://localhost:12345 to see Tensorboard'
