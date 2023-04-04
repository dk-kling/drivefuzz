#!/bin/bash

docker run --name="carla-$USER" \
  -d --rm \
  -p 2000-2002:2000-2002 \
  --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 --gpus 'device=0' \
  carlasim/carla:0.9.10.1 \
  /bin/bash -c  \
  'SDL_VIDEODRIVER=offscreen CUDA_DEVICE_ORDER=PCI_BUS_ID \
  CUDA_VISIBLE_DEVICES=1 ./CarlaUE4.sh -ResX=640 -ResY=360 \
  -nosound -windowed -opengl \
  -carla-rpc-port=2000 \
  -quality-level=Epic'
