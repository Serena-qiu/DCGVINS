FROM ros:kinetic-perception
ENV EIGEN_VERSION="3.3.3"
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

# Set up thread number for building
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
    else export USE_PROC=$(($(nproc)/2)) ; fi && \
    apt-get update && apt-get install -y \
    git \
    cmake \
    libatlas-base-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    python-catkin-tools \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf && \
    rm -rf /var/lib/apt/lists/* && \
    # Install eigen
    git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    git checkout tags/${EIGEN_VERSION} && \
    mkdir build && cd build && \
    cmake .. && make install && \
    cd ../.. && rm -rf eigen && \
    # Build and install Ceres
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../.. && rm -rf ceres-solver && \
    # Create workspace
    mkdir -p $CATKIN_WS/src && \
    cd $CATKIN_WS/src && \
    # Clone gnss_comm repo
    git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

# Copy only necessary source directories
COPY estimator/ $CATKIN_WS/src/GVINS/estimator/
COPY camera_model/ $CATKIN_WS/src/GVINS/camera_model/
COPY feature_tracker/ $CATKIN_WS/src/GVINS/feature_tracker/
COPY config/ $CATKIN_WS/src/GVINS/config/

# Apply C++14 compatibility fixes
RUN cd /root/catkin_ws/src/GVINS && \
    echo "Applying C++14 compatibility fixes..." && \
    find . \( -name "*.hpp" -o -name "*.cpp" -o -name "*.h" \) -exec sed -i 's/std::shared_mutex/std::mutex/g' {} + && \
    find . \( -name "*.hpp" -o -name "*.cpp" -o -name "*.h" \) -exec sed -i 's/std::shared_lock/std::lock_guard/g' {} + && \
    echo "Applying GVINS build fix for D2R redefinition..." && \
    sed -i 's|double D2R = M_PI / 180.0;|// double D2R = M_PI / 180.0;|g' /root/catkin_ws/src/GVINS/estimator/src/estimator_node.cpp
    
# Build workspace
WORKDIR $CATKIN_WS
ENV TERM=xterm
ENV PYTHONIOENCODING=UTF-8

# Configure catkin
RUN catkin config \
    --extend /opt/ros/$ROS_DISTRO \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_CXX_FLAGS="-std=c++14"

# First build gnss_comm
RUN catkin build gnss_comm

# Then build remaining packages
RUN catkin build && \
    sed -i '/exec "$@"/i \
    source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

WORKDIR $CATKIN_WS
