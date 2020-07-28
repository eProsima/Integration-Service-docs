.. _dds_bridge:

DDS bridge
==========

A typical scenario faced when bridging different *DDS*-based systems is that these systems use incompatible
configurations.
This happens for example in the communication between *DDS* and *ROS2*.

.. image:: DDS_NO_COMS.png

A user with knowledge of both systems may be aware that *ROS2* uses *DDS* as a middleware but hides some of
*DDS*' configuration details, thus making a direct communication between the two difficult, if not impossible.
By using *eProsima Integration-Service*, this communication can be eased and achieved with minimal effort from the
user's side.

This section is intended to illustrate the *DDS*-*ROS2* communication as an example of this
type of *eProsima Integration-Service*-mediated bridge, by putting into communication a *ROS2* *talker-listener*
example with a *Fast-RTPS* HelloWorld example.

.. image:: DDS_WITH_IS.png

.. _ros2_comm:

Example: ROS2 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _ros2_requirements:

Requirements
------------

To prepare the deployment and setup the environment, you need to have *eProsima Integration-Service* correctly
installed in your system. To do so, please follow the steps delined in :ref:`Getting Started <getting_started>` and
read carefully the :ref:`Important remarks <important_remarks>` section.

To get this example working, the following requirements must be met:

- Having *ROS2* (Crystal or superior) installed, with the *talker-listener* example working.
- Having the `HelloWorldExample <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/common/HelloWorldExample>`_
  compiled.
  To do so, go to the :code:`~/is-workspace/src/soss-dds/examples/common/HelloWorldExample` folder and type:

  .. code-block:: bash

      mkdir build
      cd build
      cmake ..
      make

- Having the `ros2_std_msgs <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/common/ros2_std_msgs>`_
  compiled.
  To do so, go to the :code:`~/is-workspace/src/soss-dds/examples/common/ros2_std_msgs` folder and type:

  .. code-block:: bash

      mkdir build
      cd build
      cmake ..
      make

- Having the **SOSS-ROS2 System-Handle** installed. Unless configured otherwise, this package is built automatically
  when *eProsima Integration-Service* is installed.

ROS2 talker to DDS subscriber
-----------------------------

To enable communication from ROS2 to DDS, open three terminals:

- In the first terminal, execute a *ROS2* :code:`talker`

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp talker

- In the second terminal, execute the *Fast-DDS* HelloWorld :code:`subscriber`

  .. code-block:: bash

      cd ~/is-workspace
      source install/setup.bash
      ./src/soss-dds/examples/common/HelloWorldExample/build/HelloWorldExample subscriber

At this point, the two applications cannot communicate due to the incompatibility of their **topic** and **type** in
their *DDS* configuration. This is where *eProsima Integration-Service* comes into play to make the communication
possible.

- In the third terminal, go to the :code:`is-workspace` folder where you have *eProsima Integration-Service* installed,
  and execute it using the :code:`soss` command followed by the
  `dds_ros2_string.yaml <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/udp/ros2_dds_string.yaml>`__
  configuration file located in the :code:`src/soss-dds/examples/udp/` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      soss src/soss-dds/examples/udp/ros2_dds_string.yaml

Once the last command is executed, the two *DDS* applications will start communicating.

DDS publisher to ROS2 listener
------------------------------

To test the same communication the other way around, launch the *ROS2* :code:`listener`, the  HelloWorld
:code:`publisher` and execute *eProsima Integration-Service* using the :code:`soss` command followed by the
`ros2_dds_string.yaml <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/udp/dds_ros2_string.yaml>`__
configuration file located in the :code:`src/soss-dds/examples/udp/` folder:

.. code-block:: bash

    cd ~/is-workspace
    source /opt/ros/$ROS2_DISTRO/setup.bash
    source install/setup.bash
    soss src/soss-dds/examples/udp/dds_ros2_string.yaml
