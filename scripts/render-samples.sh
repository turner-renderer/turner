#!/bin/sh
set -e

if [ $(uname) = "Linux" ]; then
  apk add --no-cache py2-pip imagemagick
  pip install b2
fi

scene=$1
output=$2
commit_hash=$3
fullname=$(basename $scene)
filename=${fullname%.*}

# Render images
mkdir -p "$output"
build/raycaster $scene \
    | convert - $output/${commit_hash}-${filename}-raycast.png
build/raytracer $scene \
    | convert - $output/${commit_hash}-${filename}-raytrace.png
build/pathtracer -p1 -m1 $scene \
    | convert - $output/${commit_hash}-${filename}-pathtrace.png
build/radiosity exact -e10 $scene \
    | convert - $output/${commit_hash}-${filename}-rad-exact.png
build/radiosity hierarchical --gouraud --max-subdivisions=1 -e10 $scene \
	| convert - $output/${commit_hash}-${filename}-rad-hierarchical.png

# Upload images and comment on PR
if [ -n "$B2_APP_KEY" ] && [ -n "$B2_ACCOUNT_ID" ]; then
  if [ $TRAVIS_PULL_REQUEST -ne "false" ]; then
    scripts/upload-and-comment.py -c $commit_hash -pr $TRAVIS_PULL_REQUEST $output
  else
    scripts/upload-and-comment.py -c $commit_hash $output
  fi
fi
