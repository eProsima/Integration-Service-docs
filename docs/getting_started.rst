Getting Started
===============

**Table of Contents**

* :ref:`Installation`

* :ref:`Getting Help`

Installation
^^^^^^^^^^^^

To build `integration-services` it is recommended use a
`colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__.
The `integration-services` repo consists of many cmake packages which can be configured and built manually,
but colcon makes the job much smoother.

We will assume that you have installed
`Fast-RTPS <https://github.com/eProsima/Fast-RTPS/>`__ using any system-wide method, and
`soss <https://github.com/eProsima/soss_v2>`__ following its installation manual, and source it,
os system-wide.

Create a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__ and clone
`integration-service` into it:

.. code-block:: bash

    cd
    mkdir -p workspaces/is
    cd workspaces/is
    git clone ssh://git@github.com/eProsima/SOSS-DDS src/is -b feature/xtypes-dds

Once `integration-service` is in the `src` directory of your colcon workspace and you have sourced `soss` you can run
`colcon build`:

.. code-block:: bash

    colcon build

If any packages are missing dependencies **causing the compilation to fail**, you can add the flag
`--packages-up-to soss-dds-test` to make sure that you at least build `soss-dds-test`:

.. code-block:: bash

    colcon build --packages-up-to soss-dds-test

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

From now, `soss` should be able to locate `integration-service` (`SOSS-DDS`) **System-Handle**.

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`__ or by phone at `+34 91 804 34 48 <tel:+34918043448>`__.
