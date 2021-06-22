.. _ros2_sh:

ROS 2 System Handles
====================

The *ROS 2 System Handles* can be used for two main purposes:

* Connection between a *ROS 2* application and an application running over a different middleware implementation.
  This is the classic use-case for *Integration Service*.

* Connecting two *ROS 2* applications running under different Domain IDs.

Two implementations can be distinguished: *static* one based on *ROS 2* :code:`rclcpp` and *dynamic* one based on *Fast DDS*
(by the moment it is only available for the Publisher-Subscriber paradigm).

The main advantages of the *Dynamic ROS 2 System Handle* over the *Static ROS 2 System Handle* are the following ones:

* It allows using types defined by *IDL* without previously generating and installing the *ROS 2 Type Support* for that type,
  which gives it a greater versatility with respect to the *Static ROS 2 System Handle*.

* It is not necessary to decide which *ROS 2 Built-in Types* you are going to use during the compilation phase,
  as it allows you to use any of them.


Dependencies
^^^^^^^^^^^^

Depending on the *ROS 2 System Handle* mode that wants to be used the list of dependencies varies.

**Static ROS 2 System Handle**

  The only dependency is to have a ROS 2 installation (`Foxy <https://docs.ros.org/en/foxy/Installation.html>`_ or superior) in your system.

**Dynamic ROS 2 System Handle**

  The only dependency is to have a `Fast DDS installation <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ (v2.0.0 or superior) in your system.

Configuration
^^^^^^^^^^^^^

Regarding the *ROS 2 System Handles*, there are several specific parameters which can be configured
for the *ROS 2* middleware. All of these parameters are optional, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`ros2`. In addition to the
  :code:`type` and :code:`types-from` fields,
  the *ROS 2 System Handles* accept the following specific configuration fields:

  .. code-block:: yaml

      systems:
        ros2:
          type: ros2
          namespace: "/"
          node_name: "my_ros2_node"
          domain: 4

  * :code:`namespace`: The *namespace* of the ROS 2 node created by the *ROS 2 System Handle*.

  * :code:`node_name`: The *ROS 2 System Handle* node name.

  * :code:`domain`: Provides with an easy way to change the *Domain ID* of the ROS 2 entities created
    by the *ROS 2 System Handle*.

  The *Dynamic ROS 2 System Handle* has an additional parameter:

  .. code-block:: yaml

    systems:
      ros2:
        type: ros2
        namespace: "/"
        node_name: "my_ros2_node"
        domain: 4
        using: [std_msgs/String, geometry_msgs]

  * :code:`using`: List of the *ROS 2* Built-in types or packages that want to be used in the communication.

Examples
^^^^^^^^

There are several examples that you can find in this documentation in which the
*Static ROS 2 System Handle* is employed in the communication process. Some of them are presented here:

* :ref:`ros1_ros2_bridge_pubsub`
* :ref:`dds_ros2_bridge_pubsub`
* :ref:`ros2_websocket_bridge_pubsub`
* :ref:`fiware_ros2_bridge_pubsub`
* :ref:`ros2_server_bridge`
* :ref:`ros2_change_of_domain`

.. _ros2_compilation_flags:

Compilation flags
^^^^^^^^^^^^^^^^^


Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*ROS 2 System Handles*; they are listed below:

* :code:`BUILD_ROS2_TESTS`: Allows to specifically compile the *ROS 2 System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag,
  with the objective of minimizing building time. To use it, after making sure that the *ROS 2 System Handle*
  is present in your :code:`colcon` workspace, execute the following command:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_ROS2_TESTS=ON

* :code:`IS_ROS2_DISTRO`: This flag is intended to select the *ROS 2* distro that should be used to compile
  the *ROS 2 System Handle*. If not set, the version will be retrieved from the last *ROS distro* sourced in
  the compilation environment; this means that if the last *ROS* environment sourced corresponds to *ROS 1*,
  the compilation process will stop and warn the user about it.

* :code:`IS_ROS2_SH_MODE`: This flag is to decide which *ROS 2 System Handle* mode will be compiled, as the
  static and dynamic modes are exclusive which means that they cannot be compiled at the same time.
  It accepts two different values: :code:`static` or :code:`dynamic`.

The following flags are only applicable for the *Static ROS 2 System Handle*:

* :code:`MIX_ROS_PACKAGES`: It accepts as an argument a list of `ROS packages <https://index.ros.org/packages/>`_,
  such as :code:`std_msgs`, :code:`geometry_msgs`, :code:`sensor_msgs`, :code:`nav_msgs`...
  for which the required transformation library to convert the specific *ROS 2* type definitions into *xTypes*,
  and the other way around, will be built. This list is shared with the `ROS 1 System Handle <https://github.com/eProsima/ROS1-SH#compilation-flags>`_,
  meaning that the ROS packages specified in the `MIX_ROS_PACKAGES` variable will also be built for *ROS 1*
  if the corresponding *System Handle* is present within the *Integration Service* workspace.
  To avoid possible errors, if a certain package is only present in *ROS 2*,
  the `MIX_ROS2_PACKAGES` flag must be used instead.

  These transformation libraries are also known within the *Integration Service*
  context as :code:`Middleware Interface Extension` or :code:`mix` libraries.

  By default, only the :code:`std_msgs_mix` library is compiled, unless the :code:`BUILD_TESTS`
  or :code:`BUILD_ROS2_TESTS` is used, case in which some additional ROS 2 packages :code:`mix` files
  required for testing will be built.

  If the user wants to compile some additional packages to use them with *Integration Service*,
  the following command must be launched to compile it, adding as much packages to the list as desired:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DMIX_ROS_PACKAGES="std_msgs geometry_msgs sensor_msgs nav_msgs"

* :code:`MIX_ROS2_PACKAGES`: It is used just as the `MIX_ROS_PACKAGES` flag, but will only affect *ROS 2*;
  this means that the `mix` generation engine will not search within the *ROS 1* packages,
  allowing to compile specific *ROS 2* packages independently.

  For example, if a user wants to compile a certain package `dummy_msgs` independently from *ROS 2*,
  but compiling `std_msgs` and `geometry_msgs` for both the *ROS 1* and *Static ROS 2 System Handle*,
  the following command should be executed:

  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DMIX_ROS_PACKAGES="std_msgs geometry_msgs" -DMIX_ROS2_PACKAGES="dummy_msgs"


API Reference
^^^^^^^^^^^^^

The *Integration Service API Reference* constitutes an independent section within this documentation.
To access the *ROS 2 System Handle* subsection, use this :ref:`link <api_is_ros2_sh>`.
