#!/bin/sh
set -ex

if [ $(uname) = "Linux" ]; then
  apk add --no-cache imagemagick
fi

scene=$1
output=$2
fullname=$(basename $scene)
filename=${fullname%.*}

mkdir -p "$output"
build/raycaster $scene | convert - $output/${filename}_raycast.png
build/raytracer $scene | convert - $output/${filename}_raytrace.png
build/pathtracer -p1 -m1 $scene | convert - $output/${filename}_pathtrace.png
build/radiosity exact -e10 $scene | convert - $output/${filename}_rad_exact.png
build/radiosity hierarchical --gouraud --max-subdivisions=1 -e10 $scene \
	| convert - $output/${filename}_rad_hierarchical.png
