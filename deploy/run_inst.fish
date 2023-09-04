#!/usr/bin/env fish

function docker_inst
  echo docker container $argv
  exec docker container $argv
end

function docker_launch -a arg_image
  set -l inst_name inst.$arg_image
  set -l inst_args $argv[2..-1]
  set -l inst_status (docker container inspect -f '{{.State.Status}}' $inst_name 2>/dev/null)

  if test $status -ne 0
    echo create new instance of image $arg_image
    docker_inst run --name $inst_name $inst_args -it $arg_image
    return
  end

  echo reuse container instance $inst_name

  switch $inst_status
    case created exited
      docker_inst start $inst_name -ia

    case running
      docker_inst exec $inst_name -it

    case paused
      docker_inst unpause $inst_name
      docker_inst exec $inst_name -it

    case dead
      if docker_inst rm $inst_name
        docker_inst run --name $inst_name $inst_args -it $arg_image
      else
        echo fail to remove dead container instance of $inst_name
        echo please try to remove it manually
      end

    case '*'
      echo the container $inst_name is under $inst_status status
      echo please wait for it to finish
  end
end

argparse --name run-inst --ignore-unknown 'n/name=!test -n "$_flag_value"' -- $argv

if not set -q _flag_name
  echo require flag \'-n/--name\' to be set
  exit 1
end

set -l inst_vols                       \
  /Users/ru/.ssh:/home/drinker/.ssh:ro \
  /Users/ru/Workspace/MCPT:/home/drinker/codebase:ro

set -l arg_image $_flag_name
set -l inst_args --platform linux/amd64 -v$inst_vols $argv

docker_launch $arg_image $inst_args
