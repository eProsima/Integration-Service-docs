.. _docker_samples:

Running samples on docker
=========================

A dockerfile is provided in `eProsima website <www.eprosima.com>`_ to simplify running the examples.
The docker image is loaded decompressing the provided `is_samples.tar.gz` and
importing it using the docker cli.

 .. code-block:: bash

    $ unzip -p is_samples.tar.gz | docker import - is:samples 

ROS 2 talker to DDS subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros_2_talker_to_dds_subscriber>`.
In one terminal launch the **ROS2** publisher:

 .. code-block:: bash

    $ docker run -t --name ros2_dds is:samples /ros2_entrypoint.sh ros2 run demo_nodes_cpp talker

in another terminal launch the **DDS** subscriber:

 .. code-block:: bash

    $ docker exec -t ros2_dds /home/DDSHelloWorld -m subscriber

finally launch the *Integration-Service* to provide a bridge:

 .. code-block:: bash

    $ docker exec -t ros2_dds /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/fastdds_ros2__helloworld.yaml

DDS publisher to ROS 2 listener
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <dds-publisher_to_ros_2_listener>`.

In one terminal launch the **DDS** publisher:

 .. code-block:: bash

    $ docker run -t --name dds_ros2 is:samples /home/DDSHelloWorld -m publisher

in another terminal launch the **ROS2** listener:

 .. code-block:: bash

    $ docker exec -t dds_ros2 /ros2_entrypoint.sh ros2 run demo_nodes_cpp listener

finally launch the integration service to provide a bridge:

 .. code-block:: bash

    $ docker exec -t dds_ros2 /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/fastdds_ros2__helloworld.yaml


ROS 1 pub to ROS 2 echo
^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros_1_pub_to_ros_2_echo>`.

In one terminal launch the **ROS1** master node:

 .. code-block:: bash

    $ docker run -t --name ros1_ros2 is:samples /ros1_entrypoint.sh roscore

in another terminal launch the **ROS1** publisher:

 .. code-block:: bash

    $ docker exec -t ros1_ros2 /ros1_entrypoint.sh rostopic pub /hello_ros2 std_msgs/String "Hello, ros2"

in another terminal launch the **ROS2** listener:

 .. code-block:: bash

    $ docker exec -t ros1_ros2 /ros2_entrypoint.sh ros2 topic echo hello_ros2 std_msgs/String

finally launch the integration service to provide a bridge:

 .. code-block:: bash

    $ docker exec -t ros1_ros2 /is_entrypoint.sh /ros2_entrypoint.sh /ros1_entrypoint.sh integration-service /home/basic/ros1_ros2__helloworld.yaml

ROS 2 pub to ROS 1 echo
^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros_2_pub_to_ros_1_echo>`.

In one terminal launch the **ROS1** master node:

 .. code-block:: bash

    $ docker run -t --name ros2_ros1 is:samples /ros1_entrypoint.sh roscore

in another terminal launch the **ROS1** subscriber:

 .. code-block:: bash

    $ docker exec -t ros2_ros1 /ros1_entrypoint.sh rostopic echo /hello_ros1

in another terminal launch the **ROS2** publisher:

 .. code-block:: bash

    $ docker exec -t ros2_ros1 /ros2_entrypoint.sh ros2 topic pub -r 1 /hello_ros1 std_msgs/String "{data: 'Hello, ros1'}"

finally launch the integration service to provide a bridge:

 .. code-block:: bash

    $ docker exec -t ros2_ros1 /is_entrypoint.sh /ros2_entrypoint.sh /ros1_entrypoint.sh integration-service /home/basic/ros1_ros2__helloworld.yaml

ROS 2 pub to WebSocket client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <_ros_2_pub_to_websocket_client>`.

In one terminal launch the **ROS2** publisher:

 .. code-block:: bash

    $ docker run -t -p 80:80 --name ros2_ws is:samples /ros2_entrypoint.sh ros2 topic pub -r 1 hello_websocket std_msgs/String "{data: 'Hello WebSocket'}"

in another terminal launch *Integration-Service* to bridge **ROS2** to *Websocket*:

 .. code-block:: bash

    $ docker exec -t ros2_ws /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/ros2_websocket__helloworld.yaml

launch the *Websocket client* subscriber in the browser `clicking here <../../ws_client_sub.html>`_.

WebSocket client to ROS 2 echo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <websocket_client_to_ros_2_echo>`.

In one terminal launch the **ROS2** subscriber:

 .. code-block:: bash

    $ docker run -t -p 80:80 --name ws_ros2 is:samples /ros2_entrypoint.sh ros2 topic echo hello_ros2 std_msgs/String

in another terminal launch *Integration-Service* to bridge **ROS2** to *Websocket*:

 .. code-block:: bash

    $ docker exec -t ws_ros2 /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/ros2_websocket__helloworld.yaml

launch the *Websocket client* publisher in the browser `clicking here <../../ws_client_pub.html>`_.
