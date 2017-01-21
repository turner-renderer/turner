#!/bin/sh
set -ex

scene=$1
output=$2
fullname=$(basename $scene)
filename=${fullname%.*}

./raycaster $scene | convert - $output/${filename}_raycast.png
./raytracer $scene | convert - $output/${filename}_raytrace.png
./pathtracer -p1 -m1 $scene | convert - $output/${filename}_pathtrace.png
./radiosity direct -e10 $scene | convert - $output/${filename}_raddir.png
./radiosity hierarchical --gouraud --max-subdivisions=1 -e10 $scene \
	| convert - $output/${filename}_radhier.png
