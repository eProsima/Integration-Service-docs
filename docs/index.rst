.. eProsima Integration-Services documentation master file.

eProsima Integration-Services Documentation
===========================================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

*eProsima Integration-Services* is a tool based on `SOSS <https://github.com/eProsima/soss_v2>`__ and its
**System-Handle** `SOSS-DDS <https://github.com/eProsima/SOSS-DDS>`__ to allow intercommunicating any
*DDS* based system with any other protocol, including other *DDS* systems, integrate them into a larger system,
even more complex system.

Here is the minimal example, which translates a single topic from ROS 2 to DDS:

.. code-block:: yaml

    systems:
        ros2: { type: ros2 }
        dds: { types-from: ros2 }
    topics:
        chatter: { type: std_msgs/String, route: {from: ros2, to: dds} }

The intent is that different translations are possible by only changing the configuration file. For example, by changing
the specified middlewares, we can obtain an instance which translates between WebSocket+JSON (as produced and consumed
by a standard Web browser) and DDS:

.. code-block:: yaml

    types:
        idls:
            ->
                module std_msgs
                {
                    struct String
                    {
                        string data;
                    };
                };
    systems:
        web: { type: websocket_client, types-from: robot, host: localhost, port: 12345 }
        robot: { type: dds }
    routes:
        web2robot: {from: web, to: robot}
    topics:
        chatter: { type: "std_msgs/String", route: web2robot }

This documentation is organized into the following sections:

* :ref:`relatedlinks`
* :ref:`user`

.. _relatedlinks:

.. toctree::
    :caption: Related Links

    getting_started

.. _user:

.. toctree::
    :caption: User Manual

    use_cases
    examples

