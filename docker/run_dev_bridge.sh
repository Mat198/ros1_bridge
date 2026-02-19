docker run -it --network=host  \
    -v $HOME/dev/ros1_bridge:/bridge_ws/src/ros1_bridge \
    -v $HOME/dev/bags:/bags \
    --name=ros1_bridge ros1_bridge:latest bash