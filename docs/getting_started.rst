Getting Started
===============

**Table of Contents**

* :ref:`Installation`

* :ref:`Getting Help`

Installation
^^^^^^^^^^^^

The :code:`integration-service` repository consists of many cmake packages which can be configured and built manually,
but we recommend to use a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__,
as it makes the job much smoother.

For guiding you through this tutorial, we will assume that you have installed:

- `Fast-RTPS <https://github.com/eProsima/Fast-RTPS/>`__ using any system-wide method;
- `soss <https://github.com/eProsima/soss_v2>`__, by either following its installation manual (which you can find here) and then sourcing the colcon overlay, or system-wide.

Create a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__ and clone
:code:`integration-service` into it:

.. code-block:: bash

    cd
    mkdir -p workspaces/is
    cd workspaces/is
    git clone ssh://git@github.com/eProsima/SOSS-DDS src/is -b feature/xtypes-dds

**Note**: if you don't have :code:`Fast-RTPS` installed, you can add the :code:`--recursive` flag to the
:code:`git clone` command as an alternative to download the required :code:`Fast-RTPS` third-parties.

Once :code:`integration-service` is in the `src` directory of your colcon workspace and you have sourced
:code:`soss` as explained here [TODO: add link to Getting Started section of soss documentation],
you can run :code:`colcon build`:

.. code-block:: bash

    colcon build

If any package is missing dependencies **causing the compilation to fail**, you can add the flag
:code:`--packages-up-to soss-dds-test` to make sure that you at least build :code:`soss-dds-test`:

.. code-block:: bash

    colcon build --packages-up-to soss-dds-test

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

**Note**: the sourcing of the local colcon overlay is required every time the colcon workspace is opened in
a new shell environment.
As an alternative, you can copy the source command with the full path of your local installation to your 
:code:`.bashrc` file as:

.. code-block:: bash

    source ~/PATH_TO_WORKSPACE/workspaces/is/install/setup.bash

Where :code:`PATH_TO_WORKSPACE` is the path to you local :code:`integration-service` worskspace.

..
 From now, :code:`soss` should be able to locate :code:`integration-service` (:code:`SOSS-DDS`) **System-Handle**.

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`__ or by phone at `+34 91 804 34 48 <tel:+34918043448>`__.
