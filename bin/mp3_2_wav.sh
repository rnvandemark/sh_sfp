#!/bin/bash

[[ $# -ne 1 ]] && exit -1

TMP_FILE_NAME="$(mktemp --suffix=.wav)"
ffmpeg -i "$1" -y "$TMP_FILE_NAME"
[[ $? -ne 0 ]] && exit -2

echo "$TMP_FILE_NAME"
exit 0
