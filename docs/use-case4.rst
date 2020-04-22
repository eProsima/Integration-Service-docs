WAN communication
=================

One of the most critical and powerful use-cases is that of two systems located in different geographical regions
which need to communicate through the Internet, using a *WAN* connection.

Using a pair of :code:`integration-service` instances, or **System-Handles**, one for each system,
this scenario can be addressed with a **secure TCP tunnel** thanks to the **SSL TCP** capabilities of `Fast-RTPS`.

.. image:: WAN.png

In this case, we can see :code:`integration-service` as a gateway to translate each system to :code:`DDS`over
:code:`SSL-TCP`. A proper configuration of the destination router and firewalls will allow the communication.

The example below illustrates how to configure :code:`integration-service` to achieve WAN communication.


Example: WAN TCP tunneling
^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the system and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important reminders <important reminders>`
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


This test will launch a :code:`ROS2` talker in the *server* machine, and a :code:`ROS2` listener in the *client*
machine. In both machines, an :code:`integration-service` instance will communicate with the other using WAN-TCP
communication capabilities of `Fast-RTPS`.

So, the requirement in both machines is to have :code:`ROS2` (Crystal or superior) installed
with a *talker-listener* example working.

**Note**: If you built the :code:`integration-service` and/or :code:`ROS2` packages with colcon, please make
sure to have done all the required sourcing of the colcon overlays or, in alternative, to have added the opportune
source commands to the .bashrc file, as explained in the :ref:`Getting Started <getting started>` section.

Open 2 terminals in each machine:

In the *server* side:

- Launch :code:`ROS2` talker example:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

- Launch :code:`soss` with the *server* YAML:

.. code-block:: bash

    soss example/wan/server.yaml

In the *client* side:

- Launch :code:`ROS2` listener example:

.. code-block:: bash

    ros2 run demo_nodes_cpp listener

- Launch :code:`soss` with the *client* YAML:

.. code-block:: bash

    soss example/wan/client.yaml

Once the two :code:`integration-service` instances match, the talker-listener example will start to communicate.
If the test doesn't work, review carefully your NAT configuration.

.. _comment_4: wan_config.xml
.. _comment_5: create server.yaml and client.yaml both loading wan_config.xml, but different profiles