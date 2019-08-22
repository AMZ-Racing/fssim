FROM ros:melodic-ros-base-xenial
MAINTAINER britss@ethz.ch

RUN apt update && apt install sudo linux-headers-$(uname -r) software-properties-common python-catkin-tools pylint cppcheck build-essential rsync checkinstall -y
RUN echo "%jenkins ALL=NOPASSWD: ALL" >> /etc/sudoers
RUN mkdir /home/jenkins
RUN useradd -u 1001 -d /home/jenkins jenkins
RUN chown -R jenkins:jenkins /home/jenkins
RUN chmod 755 /home/jenkins
RUN usermod -a -G sudo jenkins
