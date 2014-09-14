FROM fedora:20

# Basic stuff
RUN yum update -y; yum install -y scons git clang nano; yum clean all

# Project-dependent stuff
RUN yum install -y \
	protobuf protobuf-compiler \
	pcl pcl-devel eigen3-devel opencv-devel; \
	yum clean all
# pcl: 1.7.1
# eigen: 3.2.2
# opencv: 2.4.7
# cgal: 4.3
# boost: 1.53.0

# Setup SSH for git clone
ENV HOME /root
ADD ssh/ /root/.ssh
RUN chmod 600 /root/.ssh/*; ssh-keyscan bitbucket.org > /root/.ssh/known_hosts

# Get Project and compile
# TODO: Squash this after compilation success
RUN yum install -y \
	jsoncpp-devel CGAL CGAL-devel \
	glew-devel glfw glfw-devel \
	gtest gtest-devel protobuf-devel \
	boost-filesystem boost-program-options boost-system boost-thread
RUN git clone git@bitbucket.org:xanxys/reconstruction.git; \
	cd reconstruction; \
	git checkout docker-based; \
	scons -j 8
