#!/bin/sh
set -e

if [ "$#" -ne 2 ]; then
	cat<<EOF
USAGE: ./sync.sh <commit> <output>

Synchronize rendered images in image storage for commit <commit> with images in
folder <output>.
EOF
	exit 1
fi

b2 sync \
	--excludeRegex ".*" \
	--includeRegex "$1.*" \
	--skipNewer \
	"b2://turner" "$2"
