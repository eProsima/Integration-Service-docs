.. eProsima Integration Services documentation master file.

eProsima Integration Services Documentation
===========================================

.. image:: logo.png
   :height: 80px
   :width: 80px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eProsima Integration Services* is a library and an utility based on *Fast RTPS* for making communication bridges between different systems, services and protocols. With *Integration Services* the user can create parametric communication bridges between applications. At the same time, it is able to perform some transformations over the messages such as customized routing, mapping between input and output attributes or data modification.

Some of the main features of *Integration Services* are:

* Connections for jumping from topics which are running on different domains.
* Adapters for mapping the attributes from types with different IDL definitions.
* User-defined operations over the circulating messages.
* Communication with others environments, as *ROS2*.


This documentation is organized into the following sections:

* :ref:`installation`
* :ref:`user`
* :ref:`notes`

.. _installation:

.. toctree::
   :caption: Installation manual

   requirements
   binaries
   sources


.. _user:

.. toctree::
   :caption: User Manual

   introduction
   overview
   xml-config-file
   libraries
   examples


.. _notes:

.. toctree::
   :caption: Release Notes

   notes
