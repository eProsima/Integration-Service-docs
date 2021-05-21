.. _external_dependencies:

Dependencies
============

In this page, we provide a list of the dependencies required for an *Integration Service* instance to function.
To do so, we distinguish between the dependencies of the *Integration Service Core* and those of the *System Handles*.

.. _core_deps:

Core
^^^^

The core needs the following to be installed:

* `CMake <https://cmake.org/>`_: At least version *3.5* is required to build the project files.
* `C++ <https://isocpp.org/>`_: *eProsima Integration-Service* uses standard C++14.
* `colcon <https://colcon.readthedocs.io/en/released/user/installation.html>`_: If installed using *colcon*, it becomes
  a dependency.
* `YAML-cpp <https://packages.ubuntu.com/search?keywords=libyaml-cpp-dev>`_: *YAML* parser and emitter in C++.
* `Boost program options <https://packages.ubuntu.com/search?keywords=libboost-program-options-dev>`_: Library that allows
  obtaining name-value pairs from the config file.

.. _sh_deps:

System Handles
^^^^^^^^^^^^^^

Beyond the dependencies of the core, each **System-Handle** has its own specific dependencies.
Find them listed in the table below:

.. list-table::
    :header-rows: 1
    :width: 100%

    * - *System Handle*
      - External Dependencies
    * - **Fast DDS SH**
      - `Fast DDS installation <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ (v2.0.0 or superior)
    * - **ROS 2 SH**
      - `ROS 2 installation <https://docs.ros.org/en/foxy/Releases.html#list-of-distributions>`_ (`Foxy <https://docs.ros.org/en/foxy/Installation.html>`_ or superior)
    * - **ROS 1 SH**
      - `ROS 1 installation <http://wiki.ros.org/ROS/Installation>`_ (`Melodic <http://wiki.ros.org/melodic/Installation>`_ or `Noetic <http://wiki.ros.org/noetic/Installation>`_)
    * - **WebSocket SH**
      - `OpenSSL <https://www.openssl.org/>`_, `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_
    * - **FIWARE SH**
      - `Asio C++ Library <https://think-async.com/Asio/>`_, `cURLpp library <http://www.curlpp.org/>`_, `cURL library <https://curl.se/>`_
