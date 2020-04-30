.. eProsima Integration-Services documentation master file.

eProsima Integration-Service
============================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

*eProsima Integration-Service* is a tool based on `SOSS <https://github.com/eProsima/soss_v2>`__ and its
**System-Handle** `SOSS-DDS <https://github.com/eProsima/SOSS-DDS>`__ that allows intercommunicating any
*DDS*-based system with any other protocol, including other *DDS* systems, integrating them into a larger,
more complex system.

*eProsima Integration-Service* can be configured with a YAML text file, through which the user can provide a mapping
between the topics and services on the *DDS*-based middleware and those on the system(s) to which the user
wants to bridge it.

.. image:: CONCEPT.png

Main Features
^^^^^^^^^^^^^

*eProsima Integration-Service* provides a plugin-based platform that is easily and intuitively configurable.
This section explains these key features.

System Handles
--------------

An *eProsima Integration-Service* instance can connect *N* middlewares through dedicated plugins that speak the same
language as the core.
This common language is `eProsima xtypes <https://github.com/eProsima/xtypes>`__; a fast and lightweight
`OMG DDS-XTYPES standard <https://www.omg.org/spec/DDS-XTypes>`__ C++11 header-only implementation.
These plugins, or **System-Handles**, are discovered by *eProsima Integration-Service* at runtime
after they have been installed.

Built-in **System-Handles** are provided for connecting *Orion ContextBroker*, *ROS*, *ROS2*, and *WebSocket* to the
*DDS* world.
New **System-Handles** for additional protocols can be easily created, automatically allowing communication of the
new protocol with *DDS* and with the middlewares that are already supported
(detailed information on how to create a **System-Handle** can be found here [TODO: link to soss documentation]).
Thanks to this, downstream users can extend *eProsima Integration-Service* to communicate *DDS*-based systems
with any middleware.

The plugin-based framework is especially advantageous when it comes to integrating a new *DDS* component into a complex
system where the rest of sub-systems use incompatible protocols, or viceversa.
Indeed, once all protocols of interest are communicated with *eProsima Integration-Service*, each via a dedicated
**System-Handle**, the integration is mediated by the core and relies on centralization rather than on the creation
of dedicated bridges for each pair of components.
For a system made of *N* components, this means that the number of new software parts to add grows as *N*
rather than *N²*.

.. image:: SH_2.png

YAML configuration files
------------------------

*eProsima Integration-Service* is configured by means of a YAML file that specifies a set of compulsory fields,
plus some optional ones.
The most common fields required to configure a **System-Handle** are:

* :code:`types`: specifies the IDL types used by *eProsima Integration-Service* to transmit messages.

* :code:`systems`: specifies the middlewares involved in the communication.

  * :code:`types-from`: allows the middleware to inherit the type from another system.

* :code:`routes`: specifies which bridges SOSS needs to create.

* :code:`topics`/:code:`services`: specify the topics exchanged over the above bridges in either publisher/subscriber
  or client/server type communications.

This configuration approach is profitable when *DDS* is integrated into complex systems, since a single YAML file
is needed no matter how many protocols are being communicated.

Below you can find a minimal example of the information that the YAML configuration file should contain.
In this example, a single topic is translated from *ROS2* to *DDS*:

.. code-block:: yaml

    systems:
        ros2: { type: ros2 }
        dds: { types-from: ros2 }
    topics:
        chatter: { type: std_msgs/String, route: {from: ros2, to: dds} }

The versatility of *eProsima Integration-Service* is that it offers the possibilty to operate different translations
by only changing the configuration file.
For example, by changing
the specified middlewares, one can obtain an instance which translates between *WebSocket+JSON*
(as produced and consumed by a standard Web browser) and *DDS*:

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

Additional features
-------------------

**Free and Open Source.**

The *eProsima Integration-Service* core, and all **System-Handles** available to date are free and open source.

**Easily configurable.**

As detailed above, an *eProsima Integration-Service* instance is easily configurable by means of a YAML file.
For more information on how to do so, please consult the link: YAML configuration
[TODO: link].

**Easy to extend to new platforms.**

New platforms can easily enter the *eProsima Integration-Service* world by generating the plugin, or **System-Handle**
needed by the core to integrate them.
For more information on **System-Handles**, please consult the link: System Handle Creation
[TODO: link].

**Easy to use.**

Installing and running *eProsima Integration-Service* is intuitive and straightforward. Please refer to the
:ref:`Getting Started <getting started>` section to be guided through the installation process.

**Commercial support.**

Available at support@eprosima.com

Structure of the Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This documentation is organized into the following sections.

Installation Manual
-------------------

This section is meant to provide the user with an easy-to-use installation guide and is organized as follows:

.. toctree::
    :caption: Installation Manual

    external_dep
    getting_started

User Manual
-----------

In this section we discuss the most representative use-cases demonstrating
*eProsima Integration-Service*'s functionalities.
For each use-case, a related example is presented and the user is guided step-by-step through the
installation protocol and environment preparation necessary to have the examples set up and working.
It is organized as follows:

.. _user_man:

.. toctree::
    :caption: User Manual

    usecases_content
    use-case1
    use-case2
    use-case3
    use-case4


