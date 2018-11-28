.. eProsima Integration Service documentation master file.

eProsima Integration Service Documentation
===========================================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

| *eProsima Integration Service* is a library based on *Fast RTPS* for creating parameterized communication bridges between different systems, services, and protocols.
| It is also able to perform transformations over the messages such as customized routing and mapping between input and output attributes or data modification.

|
| The main features of *Integration Service* are:

* Connects two different domains.
* Mapping between different data types.
* User-defined operations over the received messages (:ref:`transformation library`).
* Communication with other environments, like *ROS2*.

This documentation is organized into the following sections:

* :ref:`relatedlinks`
* :ref:`user`
* :ref:`notes`

.. _relatedlinks:

.. toctree::
    :caption: Related Links

    Getting started <https://github.com/eProsima/Integration-Service/blob/master/README.md>

.. _user:

.. toctree::
    :caption: User Manual

    concepts_terms
    configuration
    technical

.. toctree::
    :caption: Use cases

    communicatedds
    transformdata
    dynamicdata
    tcptunnel
    newprotocol

.. _notes:

.. toctree::
    :caption: Release Notes

    notes
