DDS bridge
==========

A typical scenario faced when bridging different :code:`DDS`-based systems is that these systems use incompatible
configurations.
This happens for example in the communication between :code:`DDS` and :code:`ROS2`.

.. image:: DDS_NO_COMS.png

A user with knowledge of both systems may be aware that :code:`ROS2` uses :code:`DDS` as a middleware but hides some of 
:code:`DDS`' configuration details, thus making a direct communication between the two difficult, if not impossible.
By using :code:`integration-service`, this communication can be eased and achieved with minimal effort from the
user's side.

This section is intented to illustrate the :code:`DDS`-:code:`ROS2` communication as an example of this
type of :code:`integration-service`-mediated bridge, by putting into communication a :code:`ROS2` *talker-listener*
example with a :code:`Fast-RTPS` *HelloWorld* example.

.. image:: DDS_WITH_IS.png

Example: ROS2 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the system and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important reminders <important reminders>`
section.

Also, to get this example working, the following additional requirements must be met:

- Having :code:`ROS2` (Crystal or superior) installed, with the *talker-listener* example working.
- Having :code:`Fast-RTPS` installed (at least v1.9.2), with the *HelloWorld* example working.

**Note**: If you built the :code:`integration-service` and/or :code:`ROS2` packages with colcon, please make sure
to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open three terminals:

- In the first terminal, execute a :code:`ROS2` *talker*

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- In the second terminal, execute the :code:`Fast-RTPS` HelloWorld *subscriber*

.. code-block:: bash

    ./HelloWorldExample subscriber

At this point, the two applications cannot communicate due to the incompatibility of
their **topic** and **type** in their :code:`DDS` configuration. This is where :code:`integration-service` comes
into play to make this communication possible.

- In the third terminal, execute :code:`integration-service` using the :code:`dds_ros2_string.yaml` configuration file
  located in the :code:`soss-dds/examples/udp/` folder.

.. code-block:: bash

    soss soss-dds/examples/udp/dds_ros2_string.yaml

Once the last command is executed, the two :code:`DDS` applications will start communicating.

To test the same communication the other way around,
launch the :code:`ROS2` *listener*, the  HelloWorld *publisher* and the same :code:`soss`
command.

.. _comment_1: Currently, soss-ros2-test is failing to compile, so `std_msgs/String` isn't being generated.
.. _comment_2: Maybe some changes must be done to allow the conversion between the struct types.