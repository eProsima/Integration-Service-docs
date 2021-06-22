.. role:: raw-html(raw)
    :format: html

.. _external_dependencies:

Dependencies
============

On this page, we provide a list of the dependencies required for an *Integration Service* instance to function.
To do so, we distinguish between those requirements that are common to all the repositories, the ones regarding
the *Integration Service Core* and those of each *System Handle*.

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `CMake <https://cmake.org/>`_
    - At least version *3.5* is required to build the project files.
    - :code:`apt install cmake`
  * - `C++ <https://isocpp.org/>`_
    - *eProsima Integration-Service* uses standard C++14.
    - :code:`apt install build-essential`
  * - `colcon <https://colcon.readthedocs.io/en/released/user/installation.html>`_
    - Command line tool to build and test multiple software packages.
    - `Colcon installation guide <https://colcon.readthedocs.io/en/released/user/installation.html>`_

.. _core_deps:

Core
^^^^

The *Integration Service Core* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `YAML-cpp <https://github.com/jbeder/yaml-cpp>`_
    - *YAML* parser and emitter in C++.
    - :code:`apt install libyaml-cpp-dev`
  * - `Boost program options <https://github.com/boostorg/program_options>`_
    - Allows obtaining name-value pairs from the config file.
    - :code:`apt install libboost-program-options-dev`

.. note::
  `eProsima xTypes <https://github.com/eProsima/xtypes>`_ is an additional dependency
  but it is not necessary to install it, since if the `Integration Service Core <https://github.com/eProsima/Integration-Service>`_
  repository is cloned using the :code:`--recursive` option, it is downloaded automatically.

.. _sh_deps:

System Handles
^^^^^^^^^^^^^^

Beyond the dependencies of the core, each **System Handle** has its own specific dependencies.

:raw-html:`<h4>FastDDS System Handle</h4>`

The *FastDDS System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `FastDDS (v2.0.0 or superior) <https://github.com/eProsima/Fast-DDS>`_
    - eProsima C++ implementation for *DDS*.
    - `Binaries installation guide <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ :raw-html:`<br />`
      `Sources installation guide <https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html>`_

:raw-html:`<h4>FIWARE System Handle</h4>`

The *FIWARE System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `Asio C++ Library <https://think-async.com/Asio/>`_
    - C++ library for network and low-level I/O programming.
    - :code:`apt install libasio-dev`
  * - `cURLpp library <http://www.curlpp.org/>`_
    - C++ wrapper for *libcURL*.
    - :code:`apt install libcurlpp-dev`
  * - `cURL library <https://curl.se/>`_
    - Command-line tool for getting or sending data using *URL* syntax.
    - :code:`apt install libcurl4-openssl-dev`

:raw-html:`<h4>ROS 1 System Handle</h4>`

The *ROS 1 System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `ROS 1 <http://wiki.ros.org/Distributions>`_
    - *Melodic/Noetic ROS 1* distribution.
    - `Melodic installation guide <http://wiki.ros.org/melodic/Installation>`_ :raw-html:`<br />`
      `Noetic installation guide <http://wiki.ros.org/noetic/Installation>`_

:raw-html:`<h4>ROS 2 System Handle</h4>`

The *Static ROS 2 System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `ROS 2 <https://docs.ros.org/en/foxy/Releases.html#list-of-distributions>`_
    - *Foxy/Galactic ROS 2* distribution.
    - `Foxy installation guide <https://docs.ros.org/en/foxy/Installation.html>`_ :raw-html:`<br />`
      `Galactic installation guide <https://docs.ros.org/en/galactic/Installation.html>`_

The *Dynamic ROS 2 System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `FastDDS (v2.0.0 or superior) <https://github.com/eProsima/Fast-DDS>`_
    - eProsima C++ implementation for *DDS*.
    - `Binaries installation guide <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ :raw-html:`<br />`
      `Sources installation guide <https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html>`_

:raw-html:`<h4>WebSocket System Handle</h4>`

The *WebSocket System Handle* has the following requirements:

.. list-table::
  :header-rows: 1
  :width: 100%

  * - Dependency
    - Description
    - Installation
  * - `OpenSSL <https://www.openssl.org/>`_
    - Toolkit for *TLS* and *SSL* protocols.
    - :code:`apt install libssl-dev`
  * - `WebSocket++ <https://github.com/zaphoyd/websocketpp>`_
    - *WebSocket* Protocol C++ library implementation.
    - :code:`apt install libwebsocketpp-dev`
