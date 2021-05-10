.. _dds_sh:

Fast DDS System Handle
======================

The *Fast DDS System Handle* can be used for three main purposes:

* Connection between a *DDS* application and an application running over a different middleware implementation.
  This is the classic use-case for *Integration Service*.

* Connecting two *DDS* applications running under different Domain IDs.

* Creating a *TCP tunnel*, by running an *Integration Service* instance on each of the
  machines you want to establish a communication between.

Dependencies
^^^^^^^^^^^^

The only dependency of this *System Handle* is to have a `Fast DDS installation <https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html>`_ (v2.0.0 or superior) in your system.

.. note::

    The *Fast DDS System Handle* requires an installation of *Fast DDS* to work. The *System Handle* first looks into the system for a previous installation of *Fast DDS* v2.0.0 or superior. If it doesn't find one, 
    it downloads and installs its own version.


Configuration
^^^^^^^^^^^^^

Regarding the *Fast DDS System Handle*, there are several specific parameters which can be configured
for the *DDS* middleware. All of these parameters are optional, and are suboptions of the main
five sections:

* :code:`systems`: The system :code:`type` must be :code:`fastdds`. In addition to the :code:`type` and :code:`types-from` fields,
  the *Fast DDS System Handle*.

  .. code-block:: yaml
  
      systems:
          dds:
          type: fastdds
          participant:
              domain_id: 3
              file_path: <path_to_xml_profiles_file>.xml
              profile_name: fastdds-sh-participant-profile


   This section accepts the following specific configuration fields for the *Fast DDS System Handle*:

  * :code:`participant`: Allows to add a specific configuration for the `Fast DDS DomainParticipant <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html>`_:

    * :code:`domain_id`: Provides an easy way to change the *Domain ID* of the DDS entities created
      by the *Fast DDS System Handle*.

    * :code:`file_path`: Path to an XML file, containing a configuration profile for the System Handle
      participant. More information about Fast DDS XML profiles and how to fully customize the
      properties of DDS entities through them is available `here <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_.

    * :code:`profile_name`: Within the provided XML file, the name of the XML profile associated to the
      *Integration Service Fast DDS System Handle* participant.

Examples
^^^^^^^^

There are three examples that you can find in this documentation in which the *Fast DDS System Handle* is employed in the communication:

* :ref:`dds-ros2_bridge`
* :ref:`dds_change_of_domain`
* :ref:`wan_tcp_tunneling`

Compilation flags
^^^^^^^^^^^^^^^^^

Besides the :ref:`global_compilation_flags` available for the
whole *Integration Service* product suite, there are some specific flags which apply only to the
*Fast DDS System Handle*. They are listed below:

* :code:`BUILD_FASTDDS_TESTS`: Allows to specifically compile the *Fast DDS System Handle* unitary and
  integration tests. It is useful to avoid compiling each *System Handle*'section test suite present
  in the :code:`colcon` workspace, which is what would happen if using the :code:`BUILD_TESTS` flag, with the objective of minimizing building time. To use it, after making sure that the *Fast DDS System Handle*
  is present in your :code:`colcon` workspace, execute the following command:
  
  .. code-block:: bash

      ~/is_ws$ colcon build --cmake-args -DBUILD_FASTDDS_TESTS=ON

.. TODO: complete when it is uploaded to read the docs

.. API Reference
.. ^^^^^^^^^^^^^