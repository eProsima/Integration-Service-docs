FROM ros:noetic AS noetic_overlay

# Install the examples
RUN apt-get update
RUN apt install ros-noetic-roscpp-tutorials -y

# Fix the script
RUN sed -i -e "s?\$ROS_DISTRO?$ROS_DISTRO?g" -e "s?setup?local_setup?g" /ros_entrypoint.sh

FROM ros:foxy AS foxy_overlay

# Install the examples
RUN apt-get update;
RUN apt-get install ros-foxy-demo-nodes-cpp ros-foxy-example-interfaces -y

# Fix the script
RUN sed -i -e "s?\$ROS_DISTRO?$ROS_DISTRO?g" -e "s?setup?local_setup?g" /ros_entrypoint.sh

# merge both ros images
FROM ubuntu:focal as is_build

# Copy bins-libs
COPY --from=noetic_overlay /usr /usr
COPY --from=foxy_overlay /usr /usr

# Copy ROS overlays
COPY --from=noetic_overlay /opt/ros/noetic /opt/ros/noetic
COPY --from=foxy_overlay /opt/ros/foxy /opt/ros/foxy

# Copy extra python dependencies
COPY --from=foxy_overlay /etc /etc

# Copy initialization scripts
COPY --from=noetic_overlay /ros_entrypoint.sh /ros1_entrypoint.sh
COPY --from=foxy_overlay /ros_entrypoint.sh /ros2_entrypoint.sh

# Install Integration Service dependencies
RUN apt-get update && apt-get install -f -y git \
    libyaml-cpp-dev \
    libboost-program-options-dev libboost-system-dev \
    libwebsocketpp-dev \
    libcurlpp-dev libasio-dev libcurl4-openssl-dev

# Prepare Integration Service workspace
RUN mkdir -p /is_ws/src \
    && cd /is_ws/src \
    && git clone https://github.com/eProsima/Integration-Service.git is \
    && git clone https://github.com/eProsima/xtypes.git -b feature/cross_platform --recursive xtypes \
    && git clone https://github.com/eProsima/WebSocket-SH.git \
    && git clone https://github.com/eProsima/ROS1-SH.git \ 
    && git clone https://github.com/eProsima/ROS2-SH.git \
    && git clone https://github.com/eProsima/FastDDS-SH.git \
    && git clone https://github.com/eProsima/FIWARE-SH.git

# Compile the Integration Service
WORKDIR /is_ws

RUN /ros1_entrypoint.sh \
    colcon build --event-handler=console_direct+ \
    --packages-up-to-regex is-examples is-ros1 \
    --cmake-args -DBUILD_ROS1_EXAMPLES=ON -DMIX_ROS_PACKAGES="example_interfaces" --install-base /opt/is

RUN /ros2_entrypoint.sh \
    colcon build --event-handler=console_direct+ \
    --packages-skip-regex is-ros1 \
    --cmake-args -DBUILD_FASTDDS_EXAMPLES=ON -DBUILD_ROS1_EXAMPLES=OFF \
    -DBUILD_WEBSOCKET_EXAMPLES=ON \
    -DMIX_ROS_PACKAGES="example_interfaces" --install-base /opt/is

RUN sed -e 's?ros/noetic?is?g' /ros1_entrypoint.sh > /is_entrypoint.sh && \
    chmod +x /is_entrypoint.sh

# Just keep the binaries to save space
FROM ubuntu:focal

# Copy bins-libs
COPY --from=is_build /usr /usr

# Copy ROS overlays
COPY --from=is_build /opt /opt

# Copy initialization scripts
COPY --from=is_build /*_entrypoint.sh /

# Copy extra python dependencies
COPY --from=is_build /etc /etc

# Gather the examples
COPY --from=is_build /is_ws/build/is-examples/dds/DDSHelloWorld/DDSHelloWorld /home
COPY --from=is_build /is_ws/build/is-examples/dds/DDSAddTwoInts/DDSAddTwoInts /home
COPY --from=is_build /is_ws/build/is-examples/websocket/WebSocketAddTwoInts/WebSocketAddTwoInts /home
COPY --from=is_build /is_ws/src/is/examples /home

# Add the html client examples from the context
ADD ws_client_*.html /home 
