.. _wan_tcp_tunneling:

WAN-TCP tunneling
=================

One of the most critical and powerful use-cases of *Integration Service*
is that of two systems located in different geographical regions
which need to communicate through the Internet, using a *WAN* connection.

Using a pair of *Integration Service* instances, one for each system,
this scenario can be addressed with a secure TCP tunnel thanks to the *SSL-TCP* capabilities of *Fast DDS*.

*Integration Service* acts as a gateway to translate each system to *DDS*, which then makes the tunneling over
*SSL-TCP* possible. A proper configuration of the destination router and firewalls allows the communication.

The example discussed here illustrates, specifically, how to configure *Integration Service* to achieve WAN communication between two separated *ROS 2* instances.
Notice, however, that any other applications from systems integrated in the *Integration Service* ecosystem could be bridged across the *WAN*, thanks to the *Fast DDS System Handle* TCP tunneling capabilities.

.. image:: images/WAN.png


.. _wan-tcp_requirements:

Requirements
^^^^^^^^^^^^

To prepare the deployment and setup the environment, you need to have *Integration Service* correctly
installed in your system.
To do so, please follow the steps delineated in the :ref:`installation` section.

Also, to test this example properly, you need two separate subnets that are not connected but both with internet
access, or a testing environment simulating this scenario (for example, two routers, with one of them acting as
an ISP for the second).

Notice that both the route tables and the NAT must be configured so as to ensure proper port redirection
before starting the test.

.. note::

    The IP addresses shown here only serve the purpose of illustrating the example. The important information is the
    **real** public IP of the *server* machine. Also, its router must enable the NAT to forward the listening port to
    the *server*.

Also, to get this example working, the following requirements must be met in both machines:

* Having **ROS 2** (*Foxy* or superior) installed, with the :code:`talker-listener` example working.

* Having the **ROS 2 System Handle** installed. You can download it from the
  `ROS2-SH dedicated repository <https://github.com/eProsima/ROS2-SH>`_ into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/ROS2-SH.git src/ROS2-SH

* Having **Fast DDS** (v.2.0.0 or superior) installed, with the
  `:code:`DDSHelloWorld` example` <https://fast-dds.docs.eprosima.com/en/latest/fastdds/getting_started/simple_app/simple_app.html>_ working.

* Having the **Fast DDS System Handle** installed. You can download it from the
  `FastDDS-SH dedicated repository <https://github.com/eProsima/FastDDS-SH>`_ into the :code:`is-workspace` where you have *Integration Service* installed:

  .. code-block:: bash

      cd ~/is-workspace
      git clone https://github.com/eProsima/FastDDS-SH.git src/FastDDS-SH

After you have everything correctly installed, build the packages by running:

.. code-block:: bash

    colcon build

Once the environment is prepared and tested (for example, using a port-scanner), modify the file :code:`wan_config.xml` inside the folder
:code:`src/FastDDS-SH/examples/wan/` to match the IP address and port of with the WAN IP address and forwarded port of your environment.

Deployment
^^^^^^^^^^

This examples launches a *ROS 2* :code:`talker` in the *server* machine, and a *ROS 2* :code:`listener` in the *client* machine.
An *Integration Service* instance will communicate these two applications by translating the *types* and *topics* of *ROS 2*
to those of *Fast DDS*, and then use the WAN-TCP communication capabilities of the latter to operate the tunneling.

To test it, open two terminals in each machine.

**On the server side:**

* In the first terminal, source the *ROS 2* installation and launch the *ROS 2* :code:`talker` example:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp talker

* In the second terminal, go to the :code:`is-workspace` folder, source the *ROS 2*, *Fast DDS*, and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the the `server YAML <https://github.com/eProsima/Integration-Service/blob/main/examples/wan_tunneling/ros2__wan_helloworld/wan_server_talker.yaml>`_ configuration file located in the :code:`src/Integration-Service/examples/wan_tunneling/ros2__wan_helloworld` folder:

  .. code-block:: bash

      cd ~/is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/wan_tunneling/ros2__wan_helloworld/wan_server_talker.yaml

**On the client side:**

* In the first terminal, launch the *ROS 2* :code:`listener` example:

  .. code-block:: bash

      source /opt/ros/$ROS2_DISTRO/setup.bash
      ros2 run demo_nodes_cpp listener

* In the second terminal, go to the :code:`is-workspace` folder, source the *ROS 2*, *Fast DDS*, and local installations,
  and execute *Integration Service* with the :code:`integration-service` command followed by the the `client YAML <https://github.com/eProsima/Integration-Service/blob/main/examples/wan_tunneling/ros2__wan_helloworld/wan_client_listener.yaml>`_ configuration file located in the :code:`src/Integration-Service/examples/wan_tunneling/ros2__wan_helloworld` folder:


  .. code-block:: bash

      cd ~/dds-is-workspace
      source /opt/ros/$ROS2_DISTRO/setup.bash
      source install/setup.bash
      integration-service src/Integration-Service/examples/wan_tunneling/ros2__wan_helloworld/wan_client_listener.yaml

Once the two *Integration Service* instances match, the *ROS 2* :code:`talker-listener` example will start to communicate.

.. warning::

    If the test doesn't work, review carefully your NAT configuration.
