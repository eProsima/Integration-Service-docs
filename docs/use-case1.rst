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

This section is intented to illustrate the *DDS*-*ROS2* communication as an example of this
type of *eProsima Integration-Service*-mediated bridge, by putting into communication a *ROS2* *talker-listener*
example with a *Fast-RTPS* HelloWorld example.

.. image:: DDS_WITH_IS.png

Example: ROS2 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the deployment and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important remarks <important remarks>`
section.

Also, to get this example working, the following additional requirements must be met:

- Having *ROS2* (Crystal or superior) installed, with the *talker-listener* example working.
- Having *Fast-RTPS* installed (at least v1.9.2), with the HelloWorld example working.

Open three terminals:

- In the first terminal, execute a *ROS2* :code:`talker`

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- In the second terminal, execute the *Fast-RTPS* HelloWorld :code:`subscriber`

.. code-block:: bash

    ./HelloWorldExample subscriber

At this point, the two applications cannot communicate due to the incompatibility of
their **topic** and **type** in their *DDS* configuration. This is where *eProsima Integration-Service* comes
into play to make the communication possible.

- In the third terminal, execute *eProsima Integration-Service* using the :code:`soss` command and with the
  `dds_ros2_string.yaml <https://github.com/eProsima/SOSS-DDS/blob/feature/xtypes-dds/examples/udp/dds_ros2_string.yaml>`__
  configuration file located in the :code:`soss-dds/examples/udp/` folder.

.. code-block:: bash

    soss soss-dds/examples/udp/dds_ros2_string.yaml

Once the last command is executed, the two *DDS* applications will start communicating.

To test the same communication the other way around,
launch the *ROS2* :code:`listener`, the  HelloWorld :code:`publisher` and the same :code:`soss`
command.

**Note**: Each time you execute *eProsima Integration-Service* with the :code:`soss` command in a new shell,
please make sure to have done the sourcing of the colcon overlay with the command

.. code-block:: bash

    source install/setup.bash

Also, remember to source the *ROS2* insallation in the first and third shells with the command

.. code-block:: bash

    source /opt/ros/$ROS2_DISTRO/setup.bash

As an alternative, you can add the opportune source commands to the :code:`.bashrc` file.

.. _comment_1: Currently, soss-ros2-test is failing to compile, so `std_msgs/String` isn't being generated.
.. _comment_2: Maybe some changes must be done to allow the conversion between the struct types.
