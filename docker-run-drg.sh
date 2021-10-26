#!/bin/bash
docker run --detach \
  --env PASSWORD="PASS" \
  --env PORT=8080 \
  --publish 8080:8080 \
  --volume $PWD/code_server_config/start_code_server.sh:/home/docker/catkin_ws/start_code_server.sh \
  --volume $PWD/code_server_config/pp_env_model:/home/docker/catkin_ws/.vscode \
  --volume $PWD/code_server_config/bashrc:/home/docker/.bashrc \
  ghcr.io/watonomous/dynamic-relation-graph:latest


#     - ROS_MASTER_URI=http://rosmaster:11311/
#   user: ${FIXUID:?}:${FIXGID:?}
#   volumes:
#     - ./code_server_config/start_code_server.sh:/home/docker/catkin_ws/start_code_server.sh
#     - ./code_server_config/pp_env_model:/home/docker/catkin_ws/.vscode
#     - ./code_server_config/bashrc:/home/docker/.bashrc
