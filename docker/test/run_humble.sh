docker run -it --network=host \
    --gpus all --runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  \
    --name=test_humble test_humble:latest 
