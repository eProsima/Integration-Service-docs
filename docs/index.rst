.. eProsima Integration-Services documentation master file.

.. _intro:

eProsima Integration Service
============================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

*eProsima Integration Service* is a tool that enables intercommunicating an arbitrary number of protocols that
speak different languages.

If one has a number of complex systems and wills to combine them to create a larger, even more
complex system, *Integration Service* can act as an
intermediate message-passing tool that, by speaking a common language, centralizes and mediates the integration.

The communication between the different protocols is made possible by system-specific plugins, or
*System Handles*.
These provide the necessary conversion between the target protocols and the common representation
language spoken by *Integration Service*, based on an implementation of the
`xTypes <https://www.omg.org/spec/DDS-XTypes/About-DDS-XTypes/>`_.
Once a system is communicated with the core, it enters the *Integration Service* world and can
straightforwardly reach out to any other system that already exists in this world.

*Integration Service* is configured by means of a **YAML** text file, through which the user can provide a
mapping between the topics and services handled by the middlewares of the systems involved.

.. image:: images/general.png

*Integration Service* comprises the following elements:

#. The :ref:`core` engine.
#. The :ref:`shs` or plugins, for each supported protocol.
#. A :ref:`yaml_files`, which follows a specific syntax.

.. _core:

Integration Service Core
^^^^^^^^^^^^^^^^^^^^^^^^

*Integration Service* provides a plugin-based platform that is easily and intuitively configurable.
An *Integration Service* instance can connect *N* middlewares through dedicated plugins that speak the same
language as the core.
This common language is `eProsima xtypes <https://github.com/eProsima/xtypes>`_; a fast and lightweight
`OMG DDS-XTYPES standard <https://www.omg.org/spec/DDS-XTypes>`_ C++17 header-only implementation.
Find more information on the core and on the *XTypes* representation language in the :ref:`is_core` section
of this documentation.

.. _shs:

System Handles
^^^^^^^^^^^^^^

The plugins, or **System Handles**, are discovered by *Integration Service* at runtime
after they have been installed.

Available *System Handles* up-to-date are listed below:

.. list-table::
    :name: available_shs
    :header-rows: 1
    :align: left

    * - *System Handle*
      - Repository
    * - **Fast DDS System Handle**
      - https://github.com/eProsima/FastDDS-SH
    * - **ROS 2 System Handle**
      - https://github.com/eProsima/ROS2-SH
    * - **ROS 1 System Handle**
      - https://github.com/eProsima/ROS1-SH
    * - **WEBSOCKET System Handle**
      - https://github.com/eProsima/WebSocket-SH
    * - **FIWARE System Handle**
      - https://github.com/eProsima/FIWARE-SH

New *System Handles* for additional protocols can be easily created, automatically allowing communication of the
new protocol with the middlewares that are already supported.
Detailed information on how to create a *System Handle* can be found in the
:ref:`sh` section of this documentation.

The plugin-based framework is specially advantageous when it comes to integrating a new component into a complex
system where the rest of sub-systems use incompatible protocols.
Indeed, once all protocols of interest are communicated with the core, each via a dedicated
*System Handle*, the integration happens straightforwardly.
The great advantage of using *Integration Service* is that it relies on centralization rather than on the creation
of dedicated bridges for each pair of components.
For a system made of *N* components, this means that the number of new software parts to add grows as *N*
rather than *N²*.

For further information, please refer to the :ref:`System Handle specific section <sh>` of the documentation.

.. _yaml_files:

YAML configuration files
^^^^^^^^^^^^^^^^^^^^^^^^

*Integration Service* is configured by means of a **YAML** file that specifies a set of compulsory fields,
plus some optional ones.

This configuration approach is especially profitable when it comes to integrating large systems, since a single YAML file
is needed no matter how many protocols are being communicated.

The strength of this approach is that different translations are possible by only changing the configuration file.
This means that no compilation steps are required between each *Integration Service* instantiation, as
it is configured at runtime.

Detailed information on how to configure an *Integration Service*-mediated communication via a YAML file
can be found in the :ref:`yaml_config` section of this documentation.

.. _main_features:

Main features
^^^^^^^^^^^^^

#. **Free and Open Source:** The `Integration Service Core <https://github.com/eProsima/Integration-Service>`_, and all :ref:`System Handles available to date <available_shs>` are free and open source.
   Consult the :ref:`useful_links` section of the documentation to be redirected to the relevant repositories.
#. **Easily configurable:** As detailed above, an *Integration Service* instance is easily configurable by means of a YAML file.
   For more information on how to do so, please consult the :ref:`yaml_config` section of this documentation.
#. **Easy to extend to new platforms:** New platforms can easily enter the *Integration Service* world by generating the plugin, or *System Handle*
   needed by the core to integrate them.
   For more information on **System-Handles**, please consult the :ref:`sh` section of this documentation.
#. **Easy to use:** Installing and running *Integration Service* is intuitive and straightforward. Please refer to the
   :ref:`Getting Started <getting_started>` section to be guided through the installation process.

Typical use-cases
^^^^^^^^^^^^^^^^^

*Integration Service* comes in handy for a varied set of application scenarios, such as:

* **Communication among systems** using different protocols which handle incompatible types, topics, and services.
  A specific example that depicts this functionality would be the :ref:`ROS 1 to ROS 2 intercommunication example <ros1-ros2_bridge>`.
* **Integration of systems under the same protocol** which are isolated per specific protocol features. The :ref:`DDS change of Domain ID <dds_change_of_domain>`
  example falls into this category.
* **Communication through the Internet** between systems hosted by logically separated LANs located in different geographical regions.
  A specific example using *Fast DDS* WAN capabilities can be found :ref:`here <wan_tcp_tunneling>`.

.. _structure_of_the_doc:

Structure of the Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This documentation is organized into the sections listed below:

*  :ref:`Installation Manual <installation_manual>`
*  :ref:`User Manual <user_manual>`
*  :ref:`Use-case and Examples <use_case_and_examples>`

.. toctree::
   :caption: Installation Manual
   :maxdepth: 2
   :numbered:
   :hidden:

   external_dep
   getting_started

.. toctree::
    :caption: User Manual
    :maxdepth: 2
    :numbered:
    :hidden:

    is_core
    sh
    yaml_config
    existing_shs

.. toctree::
    :caption: Use-cases and Examples
    :maxdepth: 2
    :numbered:
    :hidden:

    usecases_content
    dds-ros2
    dds_change_domain
    ros1-ros2
    ros2_change_domain
    ros2-websocket
    fiware-ros2
    wan

Contact and commercial support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Find more about us at `eProsima's webpage <https://eprosima.com/>`_.

Support available at:

* Email: support@eprosima.com
* Phone: +34 91 804 34 48
