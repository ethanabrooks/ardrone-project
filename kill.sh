#! /usr/bin/env bash

if [[ $# != 1 ]]; then
  echo 'usage: kill.sh [SESSION_NAME]'
  exit
fi

# kill processes using ports
kill $( lsof -i:12345 -t ) > /dev/null 2>&1
kill $( lsof -i:12222-12223 -t ) > /dev/null 2>&1

echo Killing any docker instances names 'ps' or 'w-[i]'
workers=$(docker ps -q --filter 'name=w-*')
ps=$(docker ps -q --filter 'name=ps')
if [[ ! -z "$workers" ]]; then
  docker kill $workers $ps
fi

if [[ ! -z $(tmux list-session | grep $1) ]]; then
  echo Killing previous $1 session
  tmux kill-session -t $1 && true
fi
