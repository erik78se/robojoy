#!/usr/bin/env bash
set -o pipefail

USER=robot
GROUP=robot

USERID="$(stat -c '%u' "$ROBOTREPO")"
GROUPID="$(stat -c '%g' "$ROBOTREPO")"

groupadd -g "$GROUPID" "$GROUP" 2> /dev/null
useradd -o -m --uid "$USERID" --gid "$GROUPID" "$USER"
echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user
# FIXME: Should be variable
echo "source /opt/ros/kinetic/setup.bash" >> /home/robot/.bashrc

# shellcheck disable=SC2068
exec su "$USER" $@

