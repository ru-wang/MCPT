#!/usr/bin/env fish

function docker_builder
  echo docker buildx $argv
  exec docker buildx $argv
end

argparse --name build-image --ignore-unknown \
  'n/name=!test -n "$_flag_value"'           \
  'w/workdir=!test -d "$_flag_value"'        \
  -- $argv

if not set -q _flag_name
  echo require flag \'-n/--name\' to be set
  exit 1
end

set -l arg_image $_flag_name
set -l arg_workdir (dirname (status current-filename))
if set -q _flag_workdir
  set arg_workdir $_flag_workdir
end

set -l build_args --platform linux/amd64 -t $arg_image $argv
docker_builder build $build_args $arg_workdir
