.. _external_dependencies:

Useful Material
===============

In this page, we provide a compendium of relevant material useful for building and instantiating an *Integration Service* project.

First of all we list the dependencies required for an *Integration Service* instance to function. To do so, we distinguish between the dependencies of the core and those of the *System Handles*.
We also provide a list of links that may come in handy for developing and deploying an *eProsima Integration  Service* project.

- :ref:`core_deps`
- :ref:`sh_deps`
- :ref:`useful_links`


.. _core_deps:

Core Dependencies
^^^^^^^^^^^^^^^^^

The core needs the following to be installed:

**CMake**

*CMake 3.5* is required to build the project files.

**C++**

*eProsima Integration-Service* uses standard C++14.

**colcon**

If installed using colcon, `colcon <https://colcon.readthedocs.io/en/released/user/installation.html>`_ becomes
a dependency.


.. _sh_deps:

System Handles Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Beyond the dependencies of the core, each **System-Handle** has its own specific dependencies.
Find them listed in the table below:

.. list-table::
    :header-rows: 1
    :align: left

    * - *System Handle*
      - External Dependencies
    * - **Fast DDS SH**
      - `Fast DDS installation <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ (v2.0.0 or superior)
    * - **ROS 2 SH**
      - ROS 2 installation (`Foxy <https://docs.ros.org/en/foxy/Installation.html>`_ or superior)
    * - **ROS 1 SH**
      - `ROS 1 installation <http://wiki.ros.org/ROS/Installation>`_ (`Melodic <http://wiki.ros.org/melodic/Installation>`_ or `Noetic <http://wiki.ros.org/noetic/Installation>`_)
    * - **WEBSOCKET SH**
      - `OpenSSL <https://www.openssl.org/>`_, `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_
    * - **FIWARE SH**
      - `Asio C++ Library <https://think-async.com/Asio/>`_, `cURLpp library <http://www.curlpp.org/>`_, `cURL library <https://curl.se/>`_

.. note::

    The *Fast DDS System Handle* requires an installation of *Fast DDS* to work. The *System Handle* first looks into the system for a previous installation of *Fast DDS* v2.0.0 or superior. If it doesn't find one, it downloads and installs its own version.

.. _useful_links:

Useful Links
^^^^^^^^^^^^

* `Integration Service core <https://github.com/eProsima/is-core>`_
* `Colcon Manual <https://colcon.readthedocs.io/en/released/user/installation.html>`_
* *System Handles* repositories

  .. list-table::
     :header-rows: 1
     :align: left

     * - *System Handle*
       - Repository
     * - **Fast DDS SH**
       - https://github.com/eProsima/FastDDS-SH
     * - **ROS 2 SH**
       - https://github.com/eProsima/ROS2-SH
     * - **ROS 1 SH**
       - https://github.com/eProsima/ROS1-SH
     * - **WEBSOCKET SH**
       - https://github.com/eProsima/WebSocket-SH
     * - **FIWARE SH**
       - https://github.com/eProsima/FIWARE-SH
* `eProsima Fast-DDS <https://github.com/eProsima/Fast-DDS/>`_
* `ROS 2 <https://index.ros.org/doc/ros2/>`_
* `ROS 1 <https://www.ros.org/>`_
* `FIWARE Orion Context Broker <https://fiware-orion.readthedocs.io/en/master/>`_
