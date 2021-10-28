.. _ros2_websocket_bridge_pubsub:

ROS 2 - WebSocket bridge
========================

Another relevant use-case for *Integration Service* is that of connecting a *WebSocket* and a *ROS 2* application

The examples detailed below addresses the situation of a *ROS 2* :code:`talker-listener` example
communicating with a *WebSocket* :code:`client`.

.. image:: images/ros2-websocket.png


.. _ros2-websocket_requirements:

Requirements
^^^^^^^^^^^^

To prepare the deployment and setup the environment, you need to have *Integration Service* correctly
installed in your system.
To do so, please follow the steps delineated in the :ref:`installation` section.

Also, to get this example working, the following requirements must be met:

* Having **ROS 2** (*Foxy* or superior) installed, with the :code:`talker-listener` example working.

* Having the **ROS 2 System Handle** installed. You can download it from the
  `ROS2-SH dedicated repository <https://github.com/eProsima/ROS2-SH>`_ into the :code:`is-workspace` where you have *Integration Service* installed:

 .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/ROS2-SH.git src/ROS2-SH

* Having `OpenSSL <https://www.openssl.org/>`_ and `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_ installed:

 .. code-block:: bash

      apt install libssl-dev libwebsocketpp-dev

* Having the **WebSocket System Handle** installed. You can download it from the `WebSocket-SH dedicated repository <https://github.com/eProsima/WebSocket-SH>`_ into the :code:`is-workspace` where you have *Integration Service* installed:

 .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/WebSocket-SH.git src/WebSocket-SH


After you have everything correctly installed in your :code:`is-workspace`, build the packages by running:

 .. code-block:: bash

    colcon build


Deployment
^^^^^^^^^^

Below we explain how to deploy an example of this communication in both directions allowed.

.. _ros_2_pub_to_websocket_client:

ROS 2 pub to WebSocket client
-----------------------------

To enable communication from *ROS 2* to a *WebSocket client*, open two terminals:

* In the first terminal, source your *ROS 2* installation and execute a *ROS 2* :code:`pub`:

 .. code-block:: bash

      source /opt/ros/$<ROS2_DISTRO>/setup.bash
      ros2 topic pub /hello_websocket std_msgs/msg/String “{data: Hello WebSocket}”

* In the second terminal, go to the :code:`is-workspace` folder, source the *ROS 2* and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the
  `ros2_websocket__helloworld.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/ros2_websocket__helloworld.yaml>`_
  configuration file located in the :code:`src/Integration-Service/basic` folder:

 .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$<ROS2_DISTRO>/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/ros2_websocket__helloworld.yaml

Up to this point, the *Integration Service* should have created a *WebSocket server* application
within the *WebSocket System Handle*, to listen and handle petitions coming from a *WebSocket client*.

In order to test the intercommunication between a **ROS 2** publisher and a demo *WebSocket client* subscriber application
`click here <../../../ws_client_sub.html>`__.
The hyperlink leads to a webpage that creates a *WebSocket* connection to
:code:`ws://localhost:80` where the *Integration-Service** has created the
*WebSocket Server* according with the **yaml** file.

The *WebSocket System Handle* uses this `handshake protocol <https://github.com/RobotWebTools/rosbridge_suite>`_.
Basically once the connection is established the server will send messages to advertise which topics and types are
available:

 .. code-block:: JavaScript

    {"op": "advertise", "topic": "hello_websocket”, "type": "std_msgs/String"}

The webpage answers by requesting a subscription to the advertised topic:

 .. code-block:: JavaScript

    {"op": "subscriber", "topic": "hello_websocket", "msg": {"data": "Hello WebSocket"}}

The server will proceed to relay all messages available on the requested topic
in the format:

 .. code-block:: JavaScript

    {"msg":{"data":"Hello WebSocket"},"op":"publish","topic":"hello_websocket"}

The webpage will add a new line for each **ROS2** message received.

.. _websocket_client_to_ros_2_echo:

WebSocket client to ROS 2 echo
------------------------------

To enable communication from a *WebSocket client* to *ROS 2*, open two terminals:

* In the first terminal, source your *ROS 2* installation and execute a *ROS 2* :code:`echo`:

 .. code-block:: bash

      source /opt/ros/$<ROS2_DISTRO>/setup.bash
      ros2 topic echo /hello_ros2 std_msgs/msg/String

* In the second terminal, go to the :code:`is-workspace` folder, source the *ROS 2* and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the
  `ros2_websocket__helloworld.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/ros2_websocket__helloworld.yaml>`_
  configuration file located in the :code:`src/Integration-Service/basic` folder:

 .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$<ROS2_DISTRO>/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/ros2_websocket__helloworld.yaml

Up to this point, the *Integration Service* should have created a *WebSocket server* application
within the *WebSocket System Handle*, to listen and handle petitions coming from a *WebSocket client*.

In order to test the intercommunication between *WebSocket client* publisher
application and a **ROS 2** subscriber `click here <../../../ws_client_pub.html>`__.
The hyperlink leads to a webpage that creates a *WebSocket* connection to
:code:`ws://localhost:80` where the *Integration-Service** has created the
*WebSocket Server* according with the **yaml** file.

The *WebSocket System Handle* uses this `handshake protocol <https://github.com/RobotWebTools/rosbridge_suite>`_.
Basically once the connection is established the client must advertise the new
topic available by sending the following message to the server:

 .. code-block:: JavaScript

    {"op": "advertise", "topic": "hello_ros2”, "type": "std_msgs/String"}

After this, we can send individual messages from the *WebSocket client*, using the *publish* operation:

 .. code-block:: JavaScript

    {"op": "publish", "topic": "hello_ros2", "msg": {"data": "Hello ROS 2"}}

The messages should be shown in the *ROS 2* echo terminal.
