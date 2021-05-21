.. _installation:

Installation
============

This section provides the user with an easy-to-use installation guide of both
the *Integration Service* and of the *System Handles*,
and an explanation of how to launch and deploy an *Integration Service* project.

.. _workspace_setup:

Workspace Setup
^^^^^^^^^^^^^^^

This section explains step by step the workspace configuration required to use *Integration Service*.
It is divided into two subsections, which describe the configuration of the *Integration Service Core*
and the *System Handles* respectively.

.. _core_installation:

Core
----

The *Integration Service* core consist of many CMake packages which can be configured and built manually, but we recommend to use `colcon <https://colcon.readthedocs.io/en/released/index.html>`_,
as it makes the job much smoother.

The starting point is to create a `colcon workspace` and clone the
`Integration-Service <https://github.com/eProsima/Integration-Service>`_ repository, containing the core. To do so, follow the next instructions:

.. code-block:: bash

    mkdir ~/is-workspace
    cd ~/is-workspace
    git clone https://github.com/eProsima/Integration-Service.git src/Integration-Service --recursive

At this point, you have the *Integration Service* library correctly cloned into your
:code:`is-workspace/src/Integration-Service` folder.

.. note::

    The :code:`--recursive` flag is needed to correctly initialize the *xTypes* library as a submodule.

.. TODO: When tool for automatically cloning the repos of the desired System Handles is ready,
   add a description of how to do so direclty from the core repo.

.. _adding_shs:

System Handles
--------------

As discussed in the :ref:`Introduction <intro>` section, *Integration Service* allows to bring
an arbitrary number of middlewares into communication, each integrated into the core with
a dedicated *System Handle*.

The workflow is thus dependent on the middlewares involved in the desired communication.
The up-to-date list of the available *System Handles* and the repositories hosting them is provided
in the :ref:`built_in_sh` section.

Depending on the use-case, you might need to have either one, two, or more *System Handles* installed.
In the :ref:`examples` section, you can find a collection of relevant examples clarifying how to use
these plugins according to your needs.

You will have to clone the repositories of the desired *System Handles*
into the previously created :code:`is-workspace`:

.. code-block:: bash

    cd ~/is-workspace
    git clone https://github.com/eProsima/<middleware_1-SH>.git src/middleware_1-SH
    ...
    git clone https://github.com/eProsima/<middleware_N_SH>.git src/middleware_2-SH

Where :code:`<middleware_i-SH>`, with *i = 1, .., N* refers to the *i*-th *System Handle* needed
for carrying out the integration, chosen among the ones listed in the :ref:`built_in_sh` section.
Each such *System Handle* will be cloned in a dedicated :code:`src/middleware_i-SH` folder
inside your :code:`is-workspace`.

.. note:: If using a custom *System Handle* which is not present in the *eProsima* GitHub organization, clone the dedicated repository into the :code:`is-workspace`.

.. _build:

Build
^^^^^

Once all the necessary packages have been cloned, they need to be built.
To do so, execute from the :code:`is-workspace`:

.. code-block:: bash

    colcon build <COMPILATION_FLAGS>

.. note:: :code:`<COMPILATION_FLAGS>` refers to the optional flags used to configure *Integration Service*. For further details refers to the :ref:`global_compilation_flags` section.

Once that's finished building and before launching your *Integration Service* project,
you need to source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash


.. _global_compilation_flags:

Global compilation flags
------------------------

*Integration Service* uses CMake for building and packaging the project.
There are several CMake flags, which can be tuned during the configuration step:

* :code:`BUILD_LIBRARY`: This compilation flag can be used to completely disable the compilation of
  the *Integration Service* set of libraries, that is, the *Integration Service Core* and all the
  existing *System Handles* existing in the `colcon` workspace. It is enabled by default.

  This flag is useful, for example, to speed up the documentation generation process, when building the
  :ref:`api_reference` from the *Doxygen* source code comments.

  .. code-block:: bash

    ~/is_ws$ colcon build --cmake-args -DBUILD_LIBRARY=OFF

* :code:`BUILD_API_REFERENCE`: It is used to generate all the necessary files for building the
  :ref:`api_reference` section of this documentation, starting from the source code comments written
  using a *Doxygen*-like format. It is disabled by default; to use it:

  .. code-block:: bash

    ~/is_ws$ colcon build --cmake-args -DBUILD_API_REFERENCE=ON

* :code:`BUILD_TESTS`: When compiling *Integration Service*, use the :code:`-DBUILD_TESTS=ON` CMake option
  to compile both the unitary tests for the Integration Service Core and the unitary
  and integration tests for all the *System Handles* present in the `colcon` workspace:

  .. code-block:: bash

    ~/is_ws$ colcon build --cmake-args -DBUILD_TESTS=ON

* :code:`BUILD_EXAMPLES`: Allows to compile utilities that can be used for the several provided
  usage examples for *Integration Service*, located under the `examples/utils <https://github.com/eProsima/Integration-Service/tree/main/examples/utils>`_ folder of the core repository.
  These applications can be used to test the *Integration Service* with some of the provided YAML configuration
  files, which are located under the `examples/basic <https://github.com/eProsima/Integration-Service/tree/main/examples/basic>`_ directory of the core repository:

  .. code-block:: bash

    ~/is_ws$ colcon build --cmake-args -DBUILD_EXAMPLES=ON

  To date, the following user application examples and utility packages are available:

  **DDS**

  * :code:`DDSHelloWorld`: A simple publisher/subscriber C++ application, running under *Fast DDS*.
    It publishes or subscribes to a simple string topic, named *HelloWorldTopic*.
    As an alternative to `colcon`, in order to compile the `DDSHelloWorld` example, the following commands can be executed:

    .. code-block:: bash

        ~/is_ws$ cd examples/utils/dds/DDSHelloWorld
        ~/is_ws/examples/utils/dds/DDSHelloWorld$ mkdir build
        ~/is_ws/examples/utils/dds/DDSHelloWorld$ cd build
        ~/is_ws/examples/utils/dds/DDSHelloWorld/build$ cmake .. -DBUILD_EXAMPLES=ON
        ~/is_ws/examples/utils/dds/DDSHelloWorld/build$ make

    The resulting executable will be located inside the :code:`build` folder, and named :code:`DDSHelloWorld`.
    Please execute :code:`DDSHelloWorld -h` to see a full list of supported input parameters.

  * :code:`DDSAddTwoInts`: A simple server/client C++ application, running under *Fast DDS*.
    It allows performing service requests and replies to a service named *AddTwoIntsService*,
    which consists of two integer numbers as request type and answers with a single value,
    indicating the sum of them.
    As an alternative to `colcon`, in order to compile the `DDSAddTwoInts` example, the following commands can be executed:

    .. code-block:: bash

        ~/is_ws$ cd examples/utils/dds/DDSAddTwoInts
        ~/is_ws/examples/utils/dds/DDSAddTwoInts$ mkdir build
        ~/is_ws/examples/utils/dds/DDSAddTwoInts$ cd build
        ~/is_ws/examples/utils/dds/DDSAddTwoInts/build$ cmake .. -DBUILD_EXAMPLES=ON
        ~/is_ws/examples/utils/dds/DDSAddTwoInts/build$ make

    The resulting executable will be located inside the :code:`build` folder, and named :code:`DDSAddTwoInts`.
    Please execute :code:`DDSAddTwoInts -h` to see a full list of supported input parameters.

    .. note::
      In order to compile these examples you need to have *FastDDS* (v.2.0.0 or superior) and its dependencies installed.

  **ROS 1**

  * :code:`add_two_ints_server`: A simple C++ server application, running under *ROS 1*.
    It listens to requests coming from *ROS 1* clients and produces an appropriate answer for them;
    specifically, it is capable of listening to a *ROS 1* service called :code:`add_two_ints`,
    which consists of two integer numbers as request type and answers with a single value,
    indicating the sum of them.
    As an alternative to `colcon`, in order to compile the `add_two_ints_server` example, the following commands can be executed:

    .. code-block:: bash

        ~/is_ws$ cd examples/utils/ros1/add_two_ints_server
        ~/is_ws/examples/utils/ros1/add_two_ints_server$ mkdir build
        ~/is_ws/examples/utils/ros1/add_two_ints_server$ cd build
        ~/is_ws/examples/utils/ros1/add_two_ints_server/build$ cmake .. -DBUILD_EXAMPLES=ON
        ~/is_ws/examples/utils/ros1/add_two_ints_server/build$ make

    The resulting executable will be located inside the :code:`build/devel/lib/add_two_ints_server`
    folder, and named :code:`add_two_ints_server_node`.



  * :code:`example_interfaces`: *ROS 1* package containing the service type definitions for the
    `AddTwoInts` services examples, for which the *ROS 1* type support files will be automatically generated.
    As specified in the :ref:`services examples tutorials <examples_different_protocols_services>`,
    it must be compiled and installed in the system, using `catkin`:

    .. code-block:: bash

        ~/is_ws$ cd examples/utils/ros1/
        ~/is_ws/examples/utils/ros1$ catkin_make -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS1_DISTRO install

    .. note::
      In order to compile this example you need to have *ROS 1* (Melodic or superior) installed and sourced,
      and the *Integration Service* :code:`example_interfaces` ROS 1 package compiled.

  **WebSocket**

  * :code:`WebSocketAddTwoInts`: A simple server/client C++ application, running under *WebSocket++*.
    It allows performing service requests and replies to a service named *add_two_ints*,
    which consists of two integer numbers as request type and answers with a single value,
    indicating the sum of them.
    As an alternative to `colcon`, in order to compile the `WebSocketAddTwoInts` example, the following commands can be executed:

    .. code-block:: bash

        ~/is_ws$ cd examples/utils/websocket/WebSocketAddTwoInts
        ~/is_ws/examples/utils/websocket/WebSocketAddTwoInts$ mkdir build
        ~/is_ws/examples/utils/websocket/WebSocketAddTwoInts$ cd build
        ~/is_ws/examples/utils/websocket/WebSocketAddTwoInts/build$ cmake .. -DBUILD_EXAMPLES=ON
        ~/is_ws/examples/utils/websocket/WebSocketAddTwoInts/build$ make

    The resulting executable will be located inside the :code:`build` folder, and named :code:`DDSAddTwoInts`.
    Please execute :code:`WebSocketAddTwoInts -h` to see a full list of supported input parameters.

    .. note::
      In order to compile this example you need to have *OpenSSL* and *WebSocket++* installed.


.. _deployment:

Deployment
^^^^^^^^^^

The :code:`is-workspace` is now prepared for running an *Integration Service* instance.

The communication can be configured using a YAML file as explained in section :ref:`yaml_config`.
Once created, it is passed to *Integration Service* with the following instruction:

.. code-block:: bash

    integration-service <config.yaml>

As soon as *Integration Service* is initiated, the desired protocols can be communicated
by launching them in independent terminal windows.
To get a better taste of how to do so, refer to the :ref:`examples` section,
which provides several examples of how to connect instances of systems that are already integrated
into the *Integration Service* ecosystem.

.. note::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment. As an alternative, you can copy the source command with the full path of your local installation to
    your :code:`.bashrc` file as:

    .. code-block:: bash

        source /PATH-TO-YOUR-IS-WORKSPACE/is-workspace/install/setup.bash
