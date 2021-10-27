.. _docker_samples:

Running samples on docker
=========================

The docker image required to run the examples is provided in `eProsima website <www.eprosima.com/index.php/downloads-all>`_.
The image is loaded decompressing the provided `is_samples.tar.gz` and
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

The example explanation is available :ref:`here <ros_2_pub_to_websocket_client>`.

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

DDS Domain ID change
^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <dds_change_of_domain>`.

In a terminal launch a **DDS** subscriber on domain 3:

 .. code-block:: bash

    $ docker run -t --name domain_bridge is:samples /home/DDSHelloWorld -m subscriber -n hello_domain_3 -d 3

In another terminal launch a **DDS** publisher on domain 5:

 .. code-block:: bash

    $ docker exec -t domain_bridge /home/DDSHelloWorld -m publisher -n hello_domain_3 -d 5

Finally launch *Integration-Service* in a another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t domain_bridge /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/fastdds__domain_id_change.yaml


ROS 2 Domain ID change
^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros2_change_of_domain>`.

In a terminal launch a **ROS2** publisher under domain 5:

 .. code-block:: bash

    $ docker run -t --name ros2_domain_bridge -e "ROS_DOMAIN_ID=5" is:samples /ros2_entrypoint.sh ros2 topic pub -r 1 /string_topic std_msgs/String "{data: 'Hello, ros1'}"

In another terminal launch a **ROS2** subscriber under domain 10:

 .. code-block:: bash

    $ docker exec -t -e "ROS_DOMAIN_ID=10" ros2_domain_bridge /ros2_entrypoint.sh ros2 topic echo /string_topic std_msgs/String

Finally launch *Integration-Service* in a another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t ros2_domain_bridge /is_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/ros2__domain_id_change.yaml

DDS Service Server
^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <dds_server_bridge>`.

In a terminal launch the **DDSAddTwoInts** server example:

 .. code-block:: bash

    $ docker run -ti -p 80:80 --name dds_server is:samples /home/DDSAddTwoInts -m server

Launch the **ROS1** master node:

 .. code-block:: bash

    $ docker exec -d dds_server /ros1_entrypoint.sh roscore

Launch the *Integration-Service* in another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t dds_server /is_entrypoint.sh /ros2_entrypoint.sh /ros1_entrypoint.sh integration-service /home/basic/fastdds_server__addtwoints.yaml

In another terminal call the server from **ROS1**:

 .. code-block:: bash

    $ docker exec -t dds_server /ros1_entrypoint.sh rosservice call /add_two_ints 3 4

In order to call the server from **ROS2** do:

 .. code-block:: bash

    $ docker exec -t dds_server /ros2_entrypoint.sh ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"

In order to call the server using *WebSockets* from a browser `click here <../../ws_client_svr.html>`_.

ROS 1 Service Server
^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros1_server_bridge>`.

In a terminal launch the **ROS1** server example:

 .. code-block:: bash

    $ docker run -d -p 80:80 --name ros1_server is:samples /ros1_entrypoint.sh roscore
    $ docker exec -t ros1_server /ros1_entrypoint.sh rosrun add_two_ints_server add_two_ints_server_node

Launch the *Integration-Service* in another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t ros1_server /is_entrypoint.sh /ros1_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/ros1_server__addtwoints.yaml

In order to call the server from **DDS**:

 .. code-block:: bash

    $ docker exec -t ros1_server /home/DDSAddTwoInts -m client -c 5

In order to call the server from **ROS2** do:

 .. code-block:: bash

    $ docker exec -t ros1_server /ros2_entrypoint.sh ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"

In order to call the server using *WebSockets* from a browser `click here <../../ws_client_svr.html>`_.

ROS 1 Service Server
^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <ros2_server_bridge>`.

In a terminal launch the **ROS2** server example:

 .. code-block:: bash

    $ docker run -t -p 80:80 --name ros2_server is:samples /ros2_entrypoint.sh ros2 run demo_nodes_cpp add_two_ints_server

Launch the **ROS1** master node:

 .. code-block:: bash

    $ docker exec -d ros2_server /ros1_entrypoint.sh roscore

Launch the *Integration-Service* in another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t ros2_server /is_entrypoint.sh /ros1_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/ros2_server__addtwoints.yaml

In order to call the server from **DDS**:

 .. code-block:: bash

    $ docker exec -t ros2_server /home/DDSAddTwoInts -m client -c 5

In order to call the server from **ROS1**:

 .. code-block:: bash

    $ docker exec -t ros2_server /ros1_entrypoint.sh rosservice call /add_two_ints 3 4

In order to call the server using *WebSockets* from a browser `click here <../../ws_client_svr.html>`_.

WebSocket Service Server
^^^^^^^^^^^^^^^^^^^^^^^^

The example explanation is available :ref:`here <websocket_server_bridge>`.

In a terminal launch the *WebSocket* server example:

 .. code-block:: bash

    $ docker run -t --name ws_server is:samples /home/WebSocketAddTwoInts

Launch the **ROS1** master node:

 .. code-block:: bash

    $ docker exec -d ws_server /ros1_entrypoint.sh roscore

Launch the *Integration-Service* in another terminal as bridge:

 .. code-block:: bash

    $ docker exec -t ws_server /is_entrypoint.sh /ros1_entrypoint.sh /ros2_entrypoint.sh integration-service /home/basic/websocket_server__addtwoints.yaml

In order to call the server from **DDS**:

 .. code-block:: bash

    $ docker exec -t ws_server /home/DDSAddTwoInts -m client -c 5

In order to call the server from **ROS1**:

 .. code-block:: bash

    $ docker exec -t ws_server /ros1_entrypoint.sh rosservice call /add_two_ints 3 4

In order to call the server from **ROS2** do:

 .. code-block:: bash

    $ docker exec -t ws_server /ros2_entrypoint.sh ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"
