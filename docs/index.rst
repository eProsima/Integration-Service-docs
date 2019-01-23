.. eProsima Integration Service documentation master file.

eProsima Integration Service Documentation
===========================================

.. image:: logo.png
    :height: 80px
    :width: 80px
    :align: left
    :alt: eProsima
    :target: http://www.eprosima.com/

*eProsima Integration Service* is a library based on *Fast RTPS* which allows
integrating distant and incompatible systems.
It allows RTPS applications to communicate across domains, and Local and Wide Area Networks,
passing through firewall and NAT.
*eProsima Integration Service* permits to apply customized data transformations and
to filter between the endpoints.
This enables the communication between legacy, incompatible and/or newly developed systems
without modifying any line of code of those.

*Integration Service* provides an easy-to-use API that allows adding support to any protocol.
*Integration Service* is able to create routing tables allowing to create filtering and
complex communication diagrams, like a writer communicating with several readers,
and one of those readers receiving information from several writers at the same time.

The main features of *Integration Service* are:

- Multi-platform: Supports Linux and Windows Operating Systems.

- Integrates and expands real-time systems across WANs without effort and without
  stopping or modifying existing applications.

- Allows to build modular systems, including over already deployed systems.

- Provides precise control over the data flow allowing to split networks and
  protecting them with firewalls and NATs.

- Transforms data by changing topic names or data types, applies different quality of services.

- Allows to define customized operations and filtering over the data values allowing
  to integrate different versions of topic definitions.

- Exposes selected topics and accepts data from remote locations, allows testing the connectivity,
  topic compatibility, and use-cases, easing development, integration, and testing.

- Allows remote connection to already deployed systems, for data analysis and verification.

- Provides an easy-to-use API allowing to add support to non-RTPS systems and legacy protocols.

- Based on Fast-RTPS, but not attached to it, eProsima Integration Service can communicate
  any RTPS based application out-of-the-box.

.. image:: IS_INTRO.png
    :align: center

This documentation is organized into the following sections:

* :ref:`relatedlinks`
* :ref:`user`
* :ref:`notes`

.. _relatedlinks:

.. toctree::
    :caption: Related Links

    getting_started

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
