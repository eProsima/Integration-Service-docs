.. _fiware_sh:

FIWARE System Handle
====================

This repository contains the source code of the *Integration Service System Handle*
for the `FIWARE <https://www.fiware.org/>`_ middleware protocol, widely used in the robotics field.

The main purpose of the *FIWARE System Handle* is that of establishing a connection between a *FIWARE's Context Broker*
and an application running over a different middleware implementation.
This is the classic use-case for *Integration Service*.

Dependencies
^^^^^^^^^^^^

The dependencies of the *FIWARE System Handle* are:

* `Asio C++ Library <https://think-async.com/Asio/>`_
* `cURLpp library <http://www.curlpp.org/>`_
* `cURL library <https://curl.se/>`_

Configuration
^^^^^^^^^^^^^

Regarding the *FIWARE System Handle*, there are several specific parameters which must be configured
for the *FIWARE* middleware. These parameters are mandatory, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`fiware`. In addition to the
  :code:`type` and :code:`types-from` fields, the *FIWARE System Handle* accepts some specific
  configuration fields:

  .. code-block:: yaml

     systems:
        fiware:
            type: fiware
            host: localhost
            port: 1026


* :code:`port`: The specific port where the *FIWARE's Context Broker* will listen for incoming connections.
  This field is required.
* :code:`host`: The IP address of the *FIWARE's Context Broker*. This field is required.

Examples
^^^^^^^^

There is one example that you can find in this documentation in which the *FIWARE System Handle*
is employed in the communication:

* :ref:`fiware_ros2_bridge_pubsub`

Compilation flags
^^^^^^^^^^^^^^^^^

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*FIWARE System Handle*. They are listed below:

* :code:`BUILD_FIWARE_TESTS`: Allows to specifically compile the *FIWARE System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag,
  with the objective of minimizing building time. To use it, after making sure that the *FIWARE System Handle*
  is present in your :code:`colcon` workspace, execute the following command:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_FIWARE_TESTS=ON


.. TODO: complete when it is uploaded to read the docs

.. API Reference
.. ^^^^^^^^^^^^^
