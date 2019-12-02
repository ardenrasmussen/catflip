#!/bin/bash

name=$1
ext=$2

ffmpeg -i $name/%d.png $name.$ext
