FROM ros:kinetic
ARG rosversion=kinetic
RUN apt update; apt upgrade -y
RUN apt install -y sudo tmux less
# Dependencies for ROS
RUN apt install -y libusb-dev libx11-dev libspnav-dev libcwiid-dev
RUN apt install -y arduino-mk \         
                   ros-$rosversion-joy \
		   ros-$rosversion-roslint
