Getting Started
===============

**Table of Contents**

* :ref:`Installation`

* :ref:`Deployment`

* :ref:`Getting Help`

This section is meant to provide the user with an easy-to-use installation guide, and an explication of how to launch
an *eProsima Integration-Service* instance.

Installation
^^^^^^^^^^^^

The *eProsima Integration-Service* repository consists of many cmake packages which can be configured and built
manually, but we recommend to use `colcon <https://colcon.readthedocs.io/en/released/index.html>`__,
as it makes the job much smoother.

Create a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__ and clone the
`SOSS <https://github.com/eProsima/soss_v2/tree/feature/xtypes-dds>`__
and the
`SOSS-DDS <https://github.com/eProsima/SOSS-DDS/tree/feature/xtypes-dds>`__ repositories:

.. code-block:: bash

    mkdir -p is-workspace
    cd is-workspace
    git clone ssh://git@github.com/eProsima/soss_v2 src/soss --recursive -b feature/xtypes-dds
    git clone ssh://git@github.com/eProsima/SOSS-DDS src/soss-dds --recursive -b feature/xtypes-dds

**Note**: The :code:`--recursive` flag in the first :code:`git clone` command is mandatory to download some
required third-parties. The :code:`--recursive` flag in the second command can be omitted in the case you have
`Fast-RTPS <https://github.com/eProsima/Fast-RTPS/>`__ already installed in your system.

Once *eProsima Integration-Service* is in the :code:`src` directory of your colcon workspace, you can build the packages
by running:

.. code-block:: bash

    colcon build

If any package is missing dependencies **causing the compilation to fail**, you can add the flag
:code:`--packages-up-to soss-dds-test` to make sure that you at least build :code:`soss-dds-test`:

.. code-block:: bash

    colcon build --packages-up-to soss-dds-test

.. note::

    :code:`colcon build` will build the package :code:`soss-core` and all the built-in **System-Handles**.
    If you don't want to build the built-in **System-Handles** you can execute
    :code:`colcon build --packages-up-to soss-core`.
    If you only want a to build a sub-set of built-in **System-Handles** you can use the same directive
    with the name of the packages, for example:

    .. code-block:: bash

        colcon build --packages-up-to soss-ros2 soss-fiware

    The built-in **System-Handles** packages are:

    * :code:`soss-ros2`: ROS2 **System-Handle**.

    * :code:`soss-websocket`: WebSocket **System-Handle**.

    * :code:`soss-mock`: Mock **System-Handle** for testing purposes.

    * :code:`soss-echo`: Echo **System-Handle** for example purposes.

    Additional **System-Handles** in their own repositories:

    * :code:`soss-fiware`: `Fiware Orion ContextBroker System-Handle <https://github.com/eProsima/SOSS-FIWARE>`__.

    * :code:`soss-ros1`: `ROS System-Handle <https://github.com/eProsima/soss-ros1>`__.

    * :code:`soss-dds`: `DDS System-Handle <https://github.com/eProsima/SOSS-DDS>`__.

    Most of the **System-Handle** packages include a :code:`-test` package for testing purposes.

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash


Deployment
^^^^^^^^^^

You can now run an *eProsima Integration-Service* instance it in order to bring an arbitrary number of middlewares
into the *DDS* world.

The workflow is dependent on the specific systems involved, given that each is communicated with
*eProsima Integration-Service* via a dedicated **System-Handle**.

First of all, you will have to clone the repositories of the **System-Handles** that your use-case requires
into your :code:`is-workspace`.
To find the **System-Handles** supported to date, refer to the :ref:`Related Links <related links>` section of this
documentation.

Once all the necessary packages have been cloned, you need to build them. To do so, run:

.. code-block:: bash

    colcon build

with the possible addition of flags depending on the specific use-case. Once that's finished building, you can source
the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

The workspace is now prepared for running an *eProsima Integration-Service* instance. From the fully overlaid shell,
you will have to execute the :code:`soss` command, followed by the name of the YAML configuration file that describes 
how messages should be passed among *DDS* and the middlewares involved:

.. code-block:: bash
    
    soss <config.yaml>

Once *eProsima Integration-Service* is initiated, the user will be able to communicate the desired protocols.

**Note**: The sourcing of the local colcon overlay is required every time the colcon workspace is opened in
a new shell environment.
As an alternative, you can copy the source command with the full path of your local installation to your
:code:`.bashrc` file as:

.. code-block:: bash

    source PATH_TO_WORKSPACE/is-workspace/install/setup.bash

Where :code:`PATH_TO_WORKSPACE` is the path to the local *eProsima Integration-Service* worskspace.

..
 From now, :code:`soss` should be able to locate *eProsima Integration-Service* (:code:`SOSS-DDS`) **System-Handle**.

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`__ or by phone at `+34 91 804 34 48 <tel:+34918043448>`__.

