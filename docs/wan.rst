.. _wan_comm:

WAN communication
=================

One of the most critical and powerful use-cases is that of two systems located in different geographical regions
which need to communicate through the Internet, using a *WAN* connection.

Using a pair of *eProsima Integration-Service* instances, or **System-Handles**, one for each system,
this scenario can be addressed with a **secure TCP tunnel** thanks to the **SSL TCP** capabilities of *Fast-RTPS*.

.. image:: WAN.png

In this case, we can see *eProsima Integration-Service* as a gateway to translate each system to *DDS* over
*SSL-TCP*. A proper configuration of the destination router and firewalls will allow the communication.

The example below illustrates how to configure *eProsima Integration-Service* to achieve WAN communication.

.. _wan_tcp_tunneling:

Example: WAN TCP tunneling
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _wan_requirements:

Requirements
------------

To prepare the deployment and setup the environment, you need to have *eProsima Integration-Service* correctly
installed in your system. To do so, please follow the steps delined in :ref:`Getting Started <getting_started>` and
read carefully the :ref:`Important remarks <important_remarks>` section.

To test this example properly, you need two separated subnets that are not connected but both with internet
access, or a testing environment simulating this scenario (for example, two routers, with one of them acting as
ISP of the second).

Note that route tables and NAT must be configured so as to ensure proper port redirection before starting the test.

.. figure:: WAN_example.png

    The IP addresses shown only serve the purpose of illustrating the example, but the important information is the
    **real** public IP of the *server* machine. Also, its router must enable NAT to forward the listening port to
    the *server*.

Also, to get this example working, the following requirements must be met in both machines:

- Having *ROS2* (Crystal or superior) installed, with the *talker-listener* example working.
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

Once the environment is prepared and tested (for example, using a port-scanner), modify the file :code:`wan_config.xml`
to match the IP address and port of with the WAN IP address and forwarded port of your environment.

This test will launch a *ROS2* :code:`talker` in the *server* machine, and a *ROS2* :code:`listener` in the *client*
machine. An *eProsima Integration-Service* instance will communicate each application with the one in the other machine
using the WAN-TCP communication capabilities of *Fast-DDS*.

Executing the WAN communication
-------------------------------

Open 2 terminals in each machine:

On the *server* side:

- In the first terminal, launch the *ROS2* :code:`talker` example:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp talker

- In the second terminal, go to the :code:`is-workspace` folder where you have *eProsima Integration-Service* installed
  and execute it using the :code:`soss` command followed by the
  `server YAML <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/wan/wan_server_talker.yaml>`__
  configuration file located in the :code:`src/soss-dds/examples/wan` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      soss src/soss-dds/examples/wan/wan_server_talker.yaml

On the *client* side:

- In the first terminal, launch the *ROS2* :code:`listener` example:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp listener

- In the second terminal, go to the :code:`is-workspace` folder where you have *eProsima Integration-Service* installed
  and sourced, and execute it using the :code:`soss` command followed by the
  `client YAML <https://github.com/eProsima/SOSS-DDS/tree/doc/examples/examples/wan/wan_client_listener.yaml>`__
  configuration file located in the :code:`src/soss-dds/examples/wan` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      soss src/soss-dds/examples/wan/wan_client_listener.yaml

Once the two *eProsima Integration-Service* instances match, the *talker-listener* example will start to communicate.
If the test doesn't work, review carefully your NAT configuration.
