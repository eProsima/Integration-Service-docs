.. _websocket_server_bridge:

WebSocket Service Server
========================

This example tackles the task of bridging a *WebSocket* server with one or more client applications,
implemented using a wide variety of protocols.

Specifically, we discuss how to forward petitions coming from *Fast DDS*, *ROS 1* and *ROS 2*
service client applications to a *WebSocket* :code:`WebSocketAddTwoInts` server application,
so that it can process them and fulfill each request with a proper answer message.

.. image:: images/websocket-server.png

.. note::

    If you are looking for an example on how to perform a service request from a *WebSocket* client
    to another protocol, please refer to any of the remaining examples in the
    :ref:`server/client examples <examples_different_protocols_services>` section.


.. _websocket-server_requirements:

Requirements
^^^^^^^^^^^^

To prepare the deployment and setup the environment, you need to have *Integration Service*
correctly installed in your system.
To do so, please follow the steps delineated in the :ref:`installation` section.

Also, to get this example working, the following requirements must be met:

* Having **Fast DDS** (*v.2.0.0* or superior) installed and the *Integration Service*
  :code:`DDSAddTwoInts` example working.
  This example can be found in the main *Integration Service* repository, under the
  `examples/utils/dds/DDSAddTwoInts <https://github.com/eProsima/Integration-Service/tree/main/examples/utils/dds/DDSAddTwoInts>`_ folder;
  to compile it, you can either compile the whole *Integration Service* project using :code:`colcon` with the CMake flag
  :code:`BUILD_EXAMPLES` enabled; or execute the following steps:

  .. code-block:: bash

    cd ~/is-workspace/src/Integration-Service/examples/utils/dds/DDSAddTwoInts
    mkdir build && cd build
    cmake .. -DBUILD_EXAMPLES=ON && make

* Having the **Fast DDS System Handle** installed. You can download it from the
  `FastDDS-SH dedicated repository <https://github.com/eProsima/FastDDS-SH>`_
  into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/FastDDS-SH.git src/FastDDS-SH

* Having **ROS 1** (*Melodic* or superior) installed and the *Integration Service*
  :code:`example_interfaces` *ROS 1* package compiled.
  This package can be found in the main *Integration Service* repository, under the
  `examples/utils/ros1/src/example_interfaces <https://github.com/eProsima/Integration-Service/tree/main/examples/utils/ros1/src/example_interfaces>`_ folder.
  To compile and install it:

  .. code-block:: bash

      source /opt/ros/$<ROS1_DISTRO>/setup.bash
      cd ~/is-workspace/src/Integration-Service/example/utils/ros1/catkin_ws
      catkin_make -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/$<ROS1_DISTRO> install

* Having the **ROS 1 System Handle** installed. You can download it from the
  `ROS1-SH dedicated repository <https://github.com/eProsima/ROS1-SH>`_ into the
  :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/ROS1-SH.git src/ROS1-SH

* Having **ROS 2** (*Foxy* or superior) installed, along with the :code:`example_interfaces` types package.
  To install it:

  .. code-block:: bash

      apt install ros-$<ROS2_DISTRO>-example-interfaces

* Having the **ROS 2 System Handle** installed. You can download it from the
  `ROS2-SH dedicated repository <https://github.com/eProsima/ROS2-SH>`_ into the :code:`is-workspace`
  where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/ROS2-SH.git src/ROS2-SH src/ros2-sh

* Having `OpenSSL <https://www.openssl.org/>`_ and `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_ installed:

  .. code-block:: bash

      apt install libssl-dev libwebsocketpp-dev

  Also, the *Integration Service* :code:`WebSocketAddTwoInts` example will be needed for the tutorial.
  This example application can be found in the main *Integration Service* repository, under the
  `examples/utils/websocket/WebSocketAddTwoInts <https://github.com/eProsima/Integration-Service/tree/main/examples/utils/websocket/WebSocketAddTwoInts>`_ folder.
  To compile it, you can either compile the whole *Integration Service* project using :code:`colcon` with the CMake flag
  :code:`BUILD_EXAMPLES` enabled; or execute the following steps:

  .. code-block:: bash

    cd ~/is-workspace/src/Integration-Service/examples/utils/websocket/WebSocketAddTwoInts
    mkdir build && cd build
    cmake .. -DBUILD_EXAMPLES=ON && make

* Having the **WebSocket System Handle** installed.
  You can download it from the `WebSocket-SH dedicated repository <https://github.com/eProsima/WebSocket-SH>`_ into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/WebSocket-SH.git src/WebSocket-SH

After you have everything correctly installed in your :code:`is-workspace`, build the packages by running:

.. code-block:: bash

    source /opt/ros/$<ROS2_DISTRO>/setup.bash
    colcon build --packages-skip-regex is-ros1 -DMIX_ROS_PACKAGES="example_interfaces"
    source /opt/ros/$<ROS1_DISTRO>/setup.bash
    colcon build --cmake-args -DBUILD_EXAMPLES=ON -DMIX_ROS_PACKAGES="example_interfaces"

Deployment
^^^^^^^^^^

Below we explain how to deploy a full example of this communication, calling the *WebSocket* service from
each of the available clients.

Launch the WebSocket AddTwoInts server
--------------------------------------

To do so, open a terminal, go to the :code:`is-workspace` folder and execute the following command:

.. code-block:: bash

    cd ~/is-workspace
    ./build/is-examples/websocket/WebSocketAddTwoInts/WebSocketAddTwoInts

The *WebSocket* server will start running, listening for incoming client connection petitions;
after that, it will we able to dispatch service request petitions with a proper answer message.

Execute Integration Service
---------------------------

Open two terminals:

* In the first terminal, source the *ROS 1* installation and run the :code:`roscore`:

  .. code-block:: bash

      source /opt/ros/$<ROS1_DISTRO>/setup.bash
      roscore

* In the second terminal, go to the :code:`is-workspace` folder, source the *ROS 1*, *ROS 2* and local installations, and execute
  *Integration Service* with the :code:`integration-service` command followed by the
  `websocket_server__addtwoints.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/websocket_server__addtwoints.yaml>`_
  configuration file located in the :code:`src/Integration-Service/examples/basic` folder.

  .. code-block:: bash

      source /opt/ros/$<ROS1_DISTRO>/setup.bash
      source /opt/ros/$<ROS2_DISTRO>/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/websocket_server__addtwoints.yaml

Call the service from Fast DDS
------------------------------

In a new terminal, go to the :code:`is-workspace` folder and execute the following command:

.. code-block:: bash

    ./build/is-examples/dds/DDSAddTwoInts/DDSAddTwoInts -m client -c <number_of_requests>

The *DDSAddTwoInts* example application will request to add two numbers an specific amount of times,
specified with the :code:`-c` flag; if not present, ten requests will be performed by default.

For instance, if :code:`-c 4`, should see something like this in your screen,
indicating that the *WebSocket* server is processing the requests:

.. code-block:: bash

    AddTwoIntsService client running under DDS Domain ID: 0
    AddTwoIntsService client performing 4 requests.
    AddTwoIntsService client:
            - Request 1 + 3
            - Received response: 4
    AddTwoIntsService client:
            - Request 2 + 4
            - Received response: 6
    AddTwoIntsService client:
            - Request 3 + 5
            - Received response: 8
    AddTwoIntsService client:
            - Request 4 + 6
            - Received response: 10

Call the service from ROS 1
---------------------------

In a new terminal, source your *ROS 1* installation and invoke the service by executing the following
instructions:

.. code-block:: bash

    source /opt/ros/$<ROS1_DISTRO>/setup.bash
    rosservice call /add_two_ints 3 4

You should receive the following output from the *WebSocket* server processing the petition:

.. code-block:: bash

    sum: 7

Call the service from ROS 2
---------------------------

In a new terminal, source your *ROS 2* installation and invoke the service by executing the following
instruction:

.. code-block:: bash

    source /opt/ros/$<ROS2_DISTRO>/setup.bash
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 17}"

You should receive the following output from the *WebSocket* server processing the petition:

.. code-block:: bash

    waiting for service to become available...
    requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=17)

    response:
    example_interfaces.srv.AddTwoInts_Response(sum=22)
