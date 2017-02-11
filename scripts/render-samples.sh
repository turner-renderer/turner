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

# Upload images
if [ -n "$B2_APP_KEY" ] && [ -n "$B2_ACCOUNT_ID" ]; then
  b2 authorize-account $B2_ACCOUNT_ID $B2_APP_KEY
  for file in $(ls "$output" | grep "${commit_hash}.*png"); do
      b2 upload-file turner $output/$file $file
  done
fi
