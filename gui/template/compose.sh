#!/bin/sh

[ ! -f "$1" ] && echo "Error: invalid file $1" && exit

cat header.html $1 footer.html > ../$1
