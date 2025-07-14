### Build container image:
docker build -t vio-docker:latest .


### Run docker container:
docker run -it --rm   --name vio   --gpus all   --runtime=nvidia   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1   -e XDG_RUNTIME_DIR=/tmp/runtime-root   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v /mnt/D/code/python/vio:/workspace   -w /workspace   vio-docker


### Run gazebo simulator:
gz sim


### Listen fot raspberry pi
ping raspberrypi.local

### Access raspberry pi using ssh
ssh dark-neonus@192.168.0.56

### Access raspberry pi using ssh better way
ssh -L 5900:localhost:5900 dark-neonus@192.168.0.56
