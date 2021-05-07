.. _getting_started:

Getting Started
===============

**Table of Contents**

* :ref:`core_installation`
* :ref:`adding_shs`
* :ref:`config_and_deploy`
* :ref:`getting_help`

This section provides the user with an easy-to-use installation guide of both the *Integration Service* and of the *System Handles*, and an explication of how to launch and deploy an *Integration Service* project.


.. _core_installation:

Core installation
^^^^^^^^^^^^^^^^^

The *Integration Service* core and the associated *System Handles* repositories consist of many CMake packages which can be configured and built manually, but we recommend to use `colcon <https://colcon.readthedocs.io/en/released/index.html>`_,
as it makes the job much smoother.

The starting point of any *Integration Service*-mediated integration is to create a
`colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`_ and clone the
`Integration-Service <https://github.com/eProsima/Integration-Service>`_ repository, containing the core.

.. code-block:: bash

    mkdir ~/is-workspace
    cd ~/is-workspace
    git clone https://github.com/eProsima/Integration-Service.git src/Integration-Service --recursive

At this point, you have the *Integration Service* library correcly cloned into your :code:`is-workspace/src/Integration-Service` folder.

.. note::

    The :code:`--recursive` flag is needed for correclty installing the *XTypes* library as a submodule.

.. TODO: Check sentence above.
.. TODO: When tool for automatically cloning the repos of the desired System Handles is ready, add a description of how to do so direclty from the core repo.


.. _global_compilation_flags:

Global compilation flags
------------------------

*Integration Service* uses CMake for building and packaging the project.
There are several CMake flags, which can be tuned during the configuration step:

* :code:`BUILD_TESTS`: When compiling *Integration Service*, use the :code:`-DBUILD_TESTS=ON` CMake option
    to compile both the unitary tests for the [Integration Service Core](core/) and the unitary
    and integration tests for all the *System Handles* present in the :code:`colcon` workspace:

    .. code-block:: bash
    
        ~/is_ws$ colcon build --cmake-args -DBUILD_TESTS=ON

* :code:`BUILD_EXAMPLES`: Allows to compile utilities that can be used for the several provided
    usage examples for *Integration Service*, located under the `examples/utils <https://github.com/eProsima/Integration-Service/tree/main/examples/utils>`_ folder of the core repository.
    These applications can be used to test the *Integration Service* with some of the provided YAML configuration
    files, which are located under the `examples/basic <https://github.com/eProsima/Integration-Service/tree/main/examples/basic>`_ directory of the core repository:

    .. code-block:: bash

        ~/is_ws$ colcon build --cmake-args -DBUILD_EXAMPLES=ON

To date, the following user application examples are available:

* :code:`DDSHelloWorld`: A simple publisher/subscriber application, running under `Fast DDS <https://fast-dds.docs.eprosima.com/>`_.
  It publishes or subscribes to a simple string topic, named *HelloWorldTopic*.
  As an alternative to `colcon`, in order to compile the `DDSHelloWorld` example, the following commands can be executed:

  .. code-block:: bash

      ~/is_ws$ cd examples/utils/DDSHelloWorld
      ~/is_ws/examples/utils/DDSHelloWorld$ mkdir build
      ~/is_ws/examples/utils/DDSHelloWorld$ cd build
      ~/is_ws/examples/utils/DDSHelloWorld/build$ cmake ..
      ~/is_ws/examples/utils/DDSHelloWorld$ make

The resulting executable will be located inside the :code:`build` folder, and named :code:`DDSHelloWorld`.

.. _adding_shs:

Adding System Handles
^^^^^^^^^^^^^^^^^^^^^

As discussed in the :ref:`introductory section <intro>`, *Integration Service* allows
to bring an arbitrary number of middlewares into communication, each integrated into the core with
a dedicated *System Handle*.

The workflow is thus dependent on the middlewares involved in the desired communication.
The up-to-date list of the available *System Handles*
and the repositories hosting them is provided in the :ref:`<external_dependencies>` page.

Depending on the use-case, you might need to have either one, two, or more *System Handles* installed. In the :ref:`use-cases_and_examples` section, you can find a collection of relevant examples clarifying how to use these plugins according to your needs.

You will have to clone the repositories of the desired *System Handles* into the previously created :code:`is-workspace`:

.. code-block:: bash

    cd ~/is-workspace
    git clone https://github.com/eProsima/<middleware_1-SH>.git src/middleware_1-SH
    ...
    git clone https://github.com/eProsima/<middleware_N_SH>.git src/middleware_2-SH

Where :code:`<middleware_i-SH>`, with *i = 1, .., N* refers to the *i*-th *System Handle* needed for carrying out the integration, chosen among the ones listed in the :ref:`<useful_links>` section. Each such *System Handle* will be cloned in a dedicated :code:`src/middleware_i-SH` folder inside your :code:`is-workspace`.

If using a custom *System Handle* which is not present in the
*eProsima* GitHub organization, clone the dedicated repository into the :code:`is-workspace`.

Once all the necessary packages have been cloned, they need to be built. To do so, execute from within the :code:`is-workspace`:

.. code-block:: bash

    colcon build

Once that's finished building and before launching your *Integration Service* project, you need to source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

.. important::

    The *Integration Service System Handles* use CMake for building and packaging the project.
    There are several CMake flags for each specific *System Handle*, which can be tuned during the configuration step. Find detailed information regarding the flags that can be used for each in the :ref:`existing_shs` page.


.. _config_and_deploy:

Configuration and Deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :code:`is-workspace` is now prepared for running an *Integration Service* instance.

However, before deploying an *Integration Service* project, the integration must be configured. This is done by means of a
YAML file that describes how messages should be passed among the middlewares involved in the integration.
To this aim, it needs to be filled out and prepared according to the systems, topics, services, routes
and types that are needed by all the middlewares involved. To learn more on how to do so, please
refer to the :ref:`yaml_config` section.

Once the communication has been properly configured, *Integration Service* can be run.
From the fully overlaid shell, execute the :code:`integration-service` command, followed by the name of the
YAML configuration file:

.. code-block:: bash

    integration-service <config.yaml>

Once *Integration Service* is initiated, the desired protocols can be communicated by launching them
in independent terminal windows. To get a better taste of how to do so,
refer to the :ref:`use-cases-and-examples` section, which provides several examples on how to connect
instances of systems which are already integrated into the *Integration Service* ecosystem.

.. note::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment. As an alternative, you can copy the source command with the full path of your local installation to
    your :code:`.bashrc` file as:

    .. code-block:: bash

        source /PATH-TO-YOUR-IS-WORKSPACE/is-workspace/install/setup.bash

In this way, your local installation will be automatically sourced and made available everywhere in your system.

.. _getting_help:

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`_ or by phone at `+34 91 804 34 48 <tel:+34918043448>`_.

