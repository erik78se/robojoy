FROM ros:kinetic
ARG rosversion=kinetic
RUN apt update; apt upgrade -y
RUN apt install -y sudo ros-$rosversion-joy-teleop \
                ros-$rosversion-teleop-twist-joy \
		ros-$rosversion-joy \
		ros-$rosversion-rosserial-arduino
