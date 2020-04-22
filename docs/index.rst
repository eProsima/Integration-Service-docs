.. eProsima Integration-Services documentation master file.

eProsima Integration-Services Documentation
===========================================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

*eProsima Integration-Service* is a tool based on `SOSS <https://github.com/eProsima/soss_v2>`__ and its
**System-Handle** `SOSS-DDS <https://github.com/eProsima/SOSS-DDS>`__ that allows intercommunicating any
:code:`DDS`-based system with any other protocol, including other :code:`DDS` protocols, integrating them into a larger,
more complex system.

*eProsima Integration-Service* can be configured with a YAML text file, through which the user can provide a mapping
between the topics and services on the :code:`DDS`-based middleware and those on the system(s) to which the user
wants to bridge it.


Below you can find a minimal example of the information that the YAML configuration file should contain.
In this example, a single topic is translated from :code:`ROS2` to :code:`DDS`:

.. code-block:: yaml

    systems:
        ros2: { type: ros2 }
        dds: { types-from: ros2 }
    topics:
        chatter: { type: std_msgs/String, route: {from: ros2, to: dds} }

The versatility of *eProsima Integration-Services* is that it offers the possibilty to operate different translations
by only changing the configuration file.
For example, by changing
the specified middlewares, one can obtain an instance which translates between :code:`WebSocket+JSON`
(as produced and consumed by a standard Web browser) and :code:`DDS`:

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
    :caption: User manual

    usecases_content
    use-case1
    use-case2
    use-case3
    use-case4

