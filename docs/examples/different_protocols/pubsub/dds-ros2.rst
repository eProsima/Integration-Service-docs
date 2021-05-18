.. _dds_ros2_bridge_pubsub:

DDS - ROS 2 bridge
==================

In this example we address a very common situation faced in the robotics world:
that of bridging *DDS* and *ROS 2*. Specifically, we discuss how to do so with the
*Fast DDS* implementation.

A user with knowledge of both systems may be aware that *ROS 2* uses *DDS* as a middleware but hides some of
*DDS*' configuration details, thus making a direct communication between the two problematic.
By using *Integration Service*, this task can be eased, and achieved with minimal effort from the
user's side.

The steps described below address such a situation, by putting into
communication a *ROS 2* :code:`talker-listener` example with a *Fast DDS* :code:`DDSHelloWorld` example.

.. image:: images/dds-ros2.png


.. _dds-ros2_requirements:

Requirements
^^^^^^^^^^^^

To prepare the deployment and setup the environment, you need to have *Integration Service*
correctly installed in your system.
To do so, please follow the steps delineated in the :ref:`installation` section.

Also, to get this example working, the following requirements must be met:

* Having **ROS 2** (*Foxy* or superior) installed, with the :code:`talker-listener` example working.

* Having the **ROS 2 System Handle** installed. You can download it from the
  `ROS2-SH dedicated repository <https://github.com/eProsima/ROS2-SH>`_ into the :code:`is-workspace`
  where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/ROS2-SH.git src/ROS2-SH src/ros2-sh

* Having **Fast DDS** (*v.2.0.0* or superior) installed and the *Integration Service*
  :code:`DDSHelloWorld` example working.
  This example can be found in the main *Integration Service* repository, under the
  `examples/utils/dds/DDSHelloWorld <https://github.com/eProsima/Integration-Service/tree/main/examples/utils/dds/DDSHelloWorld>`_ folder;
  to compile it, you can either compile the whole *Integration Service* project using :code:`colcon` with the CMake flag
  :code:`BUILD_EXAMPLES` enabled; or execute the following steps:

  .. code-block:: bash

    cd ~/is-workspace/src/IS/examples/utils/dds/DDSHelloWorld
    mkdir build && cd build
    cmake .. && make

* Having the **Fast DDS System Handle** installed. You can download it from the
  `FastDDS-SH dedicated repository <https://github.com/eProsima/FastDDS-SH>`_
  into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/FastDDS-SH.git src/FastDDS-SH


After you have everything correctly installed in your :code:`is-workspace`, build the packages by running:

.. code-block:: bash

    colcon build --cmake-args -DBUILD_EXAMPLES=ON


Deployment
^^^^^^^^^^

Below we explain how to deploy an example of this communication in both directions allowed.


ROS 2 talker to DDS subscriber
------------------------------

To enable communication from *ROS 2* to *Fast DDS*, open three terminals:

* In the first terminal, source your *ROS 2* installation and execute a *ROS 2* :code:`talker`:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp talker

* In the second terminal, execute a *Fast DDS* HelloWorld :code:`subscriber`
  from within the :code:`is-workspace`:

  .. code-block:: bash

      cd ~/is-workspace
      source install/setup.bash
      ./build/DDSHelloWorld/DDSHelloWorld -m subscriber

At this point, the two applications cannot communicate due to the incompatibility of their *topics* and *types*.
This is where *Integration Service* comes into play to make the communication possible.

* In the third terminal, go to the :code:`is-workspace` folder, source the *ROS 2* and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the
  `fastdds_ros2__helloworld.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/fastdds_ros2__helloworld.yaml>`_
  configuration file located in the :code:`src/Integration-Service/examples/basic` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/fastdds_ros2__helloworld.yaml

Once the last command is executed, the two applications will start communicating.

DDS publisher to ROS 2 listener
-------------------------------

To enable communication from *Fast DDS* to *ROS 2*, open three terminals:

* In the first terminal, execute a *Fast DDS* HelloWorld :code:`publisher`
  from within the :code:`is-workspace`:

  .. code-block:: bash

      cd ~/is-workspace
      source install/setup.bash
      ./build/DDSHelloWorld/DDSHelloWorld -m publisher

* In the second terminal, source your *ROS 2* installation and execute a *ROS 2* :code:`listener`:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp listener

At this point, the two applications cannot communicate due to the incompatibility of their *topics* and *types*.
This is where *Integration Service* comes into play to make the communication possible.

* In the third terminal, go to the :code:`is-workspace` folder, source the *ROS 2* and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the
  `fastdds_ros2__helloworld.yaml <https://github.com/eProsima/Integration-Service/blob/main/examples/basic/fastdds_ros2__helloworld.yaml>`_
  configuration file located in the :code:`src/Integration-Service/examples/basic` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/basic/fastdds_ros2__helloworld.yaml

Once the last command is executed, the two applications will start communicating.
