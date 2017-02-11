#!/bin/sh
set -ex

if [ $(uname) = "Linux" ]; then
  apk add --no-cache py2-pip imagemagick
  pip install b2
fi

scene=$1
output=$2
commit_hash=$3
pr=$4
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
  body=""
  for file in $(ls "$output" | grep "${commit_hash}.*png"); do
      image_url=$(b2 upload-file turner $output/$file $file | grep "URL by file name" | cut -d: -f2-)
      image_name=$(echo $file | cut -d- -f2-)
      body="${body} <img src=\\\"${image_url:1}\\\" width=\\\"100\\\">"
  done
  if [ $pr -ne "false" ]; then
    curl -u "$GITHUB_USER:$GITHUB_TOKEN" \
         -H "Accept: application/vnd.github.black-cat-preview+json" \
         -d "{ \"body\":\"${body}\", \"event\":\"COMMENT\" }" \
         https://api.github.com/repos/turner-renderer/turner/pulls/${pr}/reviews
  fi
fi
