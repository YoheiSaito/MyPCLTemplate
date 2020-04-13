#!/bin/bash
PROJECT_PATH=$(pwd)
dataset=/home/pcd_data/
cmake() {
    docker run -it --rm \
        -v $PROJECT_PATH:/project -v$dataset:/dataset \
        yohei31to/pcl_build:latest /cmake.sh
}

make() {
    docker run -it --rm \
        -v $PROJECT_PATH:/project -v$dataset:/dataset \
        yohei31to/pcl_build:latest /make.sh
}

run() {
    if [ "$@" ]; then
        cmd=/project/$@
    else
        cmd=/bin/bash
    fi
    docker run -it --rm \
        -v /tmp/.X11-unix/:/tmp/.X11-unix/ -e DISPLAY=$DISPLAY \
        --device=/dev/dri:/dev/dri \
        -v $PROJECT_PATH:/project -v$dataset:$dataset -v$dataset:/dataset \
        yohei31to/pcl_build:latest $cmd
}

$@
