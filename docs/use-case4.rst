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


Example: WAN TCP tunneling
^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the deployment and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important remarks <important remarks>`
section.

Also, to test this example properly, you need two separated subnets that are not connected but both with internet access,
or a testing environment simulating this scenario (for example, two routers, with one of them acting as
ISP of the second).

Notice that route tables and NAT must be configured so as to ensure proper port redirection before starting the test.

.. figure:: WAN_example.png

    The IP addresses shown only serve the purpose of illustrating the example, but the important information is the
    **real** public IP of the *server* machine. Also, its router must enable NAT to forward the listening port to
    the *server*.

Once the environment is prepared and tested (for example, using a port-scanner), modify the file :code:`wan_config.xml`
to match the IP address and port of with the WAN IP address and forwarded port of your environment.


This test will launch a *ROS2* :code:`talker` in the *server* machine, and a *ROS2* :code:`listener` in the *client*
machine. In both machines, an *eProsima Integration-Service* instance will communicate with the other using WAN-TCP
communication capabilities of *Fast-RTPS*.

So, the requirement in both machines is to have *ROS2* (Crystal or superior) installed
with a *talker-listener* example working.

Open 2 terminals in each machine:

On the *server* side:

- Launch *ROS2* :code:`talker` example:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- Launch *eProsima Integration-Service* using the :code:`soss` command and with the *server* YAML:

.. code-block:: bash

    soss example/wan/server.yaml

On the *client* side:

- Launch *ROS2* :code:`listener` example:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener

- Launch *eProsima Integration-Service* using the :code:`soss` command and with the *client* YAML:

.. code-block:: bash

    soss example/wan/client.yaml

Once the two *eProsima Integration-Service* instances match, the *talker-listener* example will start to communicate.
If the test doesn't work, review carefully your NAT configuration.

**Note**: Each time you execute *eProsima Integration-Service* with the :code:`soss` command in a new shell,
please make sure to have done the sourcing of the colcon overlay with the command

.. code-block:: bash

    source install/setup.bash

Also, remember to source the *ROS2* insallation in all shells

.. code-block:: bash

    source /opt/ros/$ROS2_DISTRO/setup.bash

As an alternative, you can add the opportune source commands to the :code:`.bashrc` file.

.. _comment_4: wan_config.xml
.. _comment_5: create server.yaml and client.yaml both loading wan_config.xml, but different profiles