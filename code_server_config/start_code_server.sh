#!/bin/bash

sudo chown -R docker /home/docker/.local/
mkdir -p ~/catkin_ws/.vscode/extensions
cd ~/catkin_ws/.vscode/extensions
while read extension_link; do
  if [[ $extension_link != /# ]]; then
    wget --no-clobber $extension_link;
    /usr/bin/code-server --force --install-extension ${extension_link##*/}
  fi
done <"$(find ~/catkin_ws/.vscode -name 'extensions.txt')"

/usr/bin/code-server --bind-addr 0.0.0.0:8080 /home/docker/catkin_ws
