#! /usr/bin/env bash

set -e
session=a3c
num_workers=2
net=a3cnet
env_id=gazebo

while [[ $# -gt 1 ]]; do
  key="$1"

  case $key in
      -e|--env-id)
        env_id="$2"
        shift # past argument
      ;;
      -l|--log-dir)
        logdir="$2"
        shift # past argument
      ;;
      -w|--num-workers)
        num_workers="$2"
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

logdir=$(pwd)/logs/$env_id
docker_log=/logs
start_ip=2
ps=ps:1222$start_ip
workers=$(awk -vORS=, "BEGIN {
   for (i = 0; i < $num_workers; ++i) { 
     print \"w-\"i\":1222\"(i + 1 + $start_ip)
   } 
 }" | sed 's/,$//')
workers=${workers},172.17.0.1:12222

kill $( lsof -i:12345 -t ) > /dev/null 2>&1 && true
kill $( lsof -i:12222-12223 -t ) > /dev/null 2>&1 && true 
docker kill ps && true
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

docker network create $net && true

if [[ "$env_id" = gazebo ]]; then
  image=ardrone
  start_script=ardrone.sh
else
  image=ardrone
  start_script=job.sh
fi

#source catkin/devel/setup.bash
docker run --rm -it -v $(dirname $logdir):/del $image\
  rm -rf del/$(basename $logdir) && true
docker run --rm -it -v $(dirname $logdir):/mk $image\
  mkdir mk/$(basename $logdir) && true
exit
mkdir -p $logdir && true
docker build . -t $image

echo Executing commands in TMUX
tmux send-keys -t a3c:ps\
 "docker run -it --rm --name=ps --net=$net $image /job.sh \
 '\
 --log-dir $docker_log\
 --env-id $env_id\
 --num-workers $num_workers\
 --job-name ps\
 --workers $workers\
 --ps $ps\
'" Enter

for i in $(seq 0 $(($num_workers - 1))); do
  tmux send-keys -t a3c:w-$i\
 "docker run -it\
 --volume=$logdir:$docker_log\
 --rm\
 --name=w-$i\
 --net=$net $image\
 /$start_script\
 '--log-dir $docker_log\
 --env-id $env_id\
 --num-workers $num_workers\
 --task $i\
 --remote 1\
 --workers $workers\
 --ps $ps\
' false" Enter
done

tmux send-keys -t a3c:tb "tensorboard --logdir $logdir --port 12345" Enter
tmux send-keys -t a3c:htop 'htop' Enter

echo 'Use `tmux attach -t a3c` to watch process output
Use `tmux kill-session -t a3c` to kill the job
Point your browser to http://localhost:12345 to see Tensorboard'
