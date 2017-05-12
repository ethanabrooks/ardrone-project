#! /usr/bin/env bash

session=${1:-a3c}

# kill processes using ports
kill $( lsof -i:12345 -t ) > /dev/null 2>&1
kill $( lsof -i:12222-12223 -t ) > /dev/null 2>&1

echo Killing any docker instances names 'ps' or 'w-[i]'
workers=$(docker ps -q --filter 'name=w-*')
ps=$(docker ps -q --filter 'name=ps')
if [[ ! -z "$workers" ]]; then
  docker kill $workers
fi

if [[ ! -z "$ps" ]]; then
  docker kill $ps
fi

if [[ ! -z $(tmux list-session | grep $session) ]]; then
  echo Killing previous $session session
  tmux kill-session -t $session
fi
