Getting Started
===============

This section is meant to provide the user with an easy-to-use installation guide.

Installation
^^^^^^^^^^^^

The *eProsima Integration-Service* repository consists of many cmake packages which can be configured and built
manually, but we recommend to use `colcon <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__,
as it makes the job much smoother.

Create a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__:

.. code-block:: bash

    cd
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

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

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
