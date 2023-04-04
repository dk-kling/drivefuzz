#!/bin/sh
docker build --no-cache --rm -t carla-autoware:improved -f Dockerfile . "$@"

