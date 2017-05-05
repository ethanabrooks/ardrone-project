#! /usr/bin/env bash

set -e
session=a3c
num_workers=4
net=a3cnet
env_id=CartPole-v0
learning_rate=0.00001
policy=MLPpolicy
visualise=false
delete_logdir=true

while [[ $# -gt 0 ]]; do
  key="$1"

  case $key in
      -h|--help)
        echo \
"
usage: train.py [-h] [-w NUM_WORKERS] [-r REMOTES] [-e ENV_ID] [-l LOG_DIR]
                [-s SPEC_PATH] [-n] [-m MODE] [--visualise]

Run commands

optional arguments:
  -h, --help            show this help message and exit
  -w NUM_WORKERS, --num-workers NUM_WORKERS
                        Number of workers
  -e ENV_ID, --env-id ENV_ID
                        Environment id
  -l LOG_DIR, --log-dir LOG_DIR
                        Log directory path
  -r LEARN_RATE, --learning-rate LEARN_RATE
                        Learning rate (scientific notation is acceptable)
  -p POLICY_NAME, --policy POLICY_NAME
                        MLPpolicy|LSTMpolicy
  --visualise           Render environment as agent interacts with it (doesn't
                        slow down gazebo, but drastically slows gym environments)
"
        exit
      ;;
      --visualise)
        visualise=true
      ;;
      -d|--delete-logdir)
        delete_logdir=true
      ;;
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
      -r|--learning-rate)
        learning_rate="$2"
        shift # past argument
      ;;
      -p|--policy)
        policy="$2"
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

bash kill.sh $session && true

# create new session and windows
tmux new-session -s $session -n ps -d bash
tmux new-window -t $session -n tb bash
tmux new-window -t $session -n htop bash
for i in $(seq 0 $(($num_workers - 1))); do
  tmux new-window -t $session -n w-$i bash
done

if [[ -z "$(docker network ls | grep $net)" ]]; then
  echo "Creating docker '$net' network"
  docker network create $net
fi

if [[ "$env_id" = gazebo ]]; then
  image=ardrone
  start_script=ardrone.sh
else
  if $visualise; then
    echo '
    Visualization is only configured for the gazebo environment. For all other
    environments, use

    $ python train.py --visualise
    '
    exit
  fi

  image=ardrone
  start_script=job.sh
fi

if $delete_logdir; then
  echo Deleting $logdir
  docker run --rm -it -v $(dirname $logdir):/del $image\
    rm -rf del/$(basename $logdir) && true
  docker run --rm -it -v $(dirname $logdir):/mk $image\
    mkdir mk/$(basename $logdir) && true
fi

docker build . -t $image

# args common to ps and workers
job_args="\
 --log-dir $docker_log\
 --env-id $env_id\
 --num-workers $num_workers\
 --policy $policy\
 --learning-rate $learning_rate
 --workers $workers\
 --ps $ps\
"

# execute commands in TMUX
tmux send-keys -t a3c:ps\
 "docker run -it --rm --name=ps --net=$net $image\
 /job.sh '--job-name ps $job_args'"\
 Enter
for i in $(seq 0 $(($num_workers - 1))); do
 tmux send-keys -t a3c:w-$i\
 "docker run -it --volume=$logdir:$docker_log --rm --name=w-$i --net=$net $image\
 /$start_script '--job-name worker --task $i --remote 1 $job_args' false"\
 Enter
done
tmux send-keys -t a3c:tb "tensorboard --logdir $logdir --port 12345" Enter
tmux send-keys -t a3c:htop 'htop' Enter

echo "

Arguments:
session:     $session
num-workers: $num_workers
net:         $net
env-id:      $env_id
learn-rate:  $learning_rate
policy:      $policy

"'
Use `tmux attach -t a3c` to watch process output
Use `bash kill.sh` to kill the job
Point your browser to http://localhost:12345 to see Tensorboard
'

if $visualise; then
  bash gazebo_gui.sh
fi
