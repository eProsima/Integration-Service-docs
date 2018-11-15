.. eProsima Integration Service documentation master file.

eProsima Integration Service Documentation
===========================================

.. image:: logo.png
   :height: 80px
   :width: 80px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eProsima Integration Service* is a library based on *Fast RTPS* for creating parameterized communication bridges between different systems, services and protocols. 
Also it is able to perform transformations over the messages such as customized routing and mapping between input and output attributes or data modification.

The main features of *Integration Service* are:

* Connects two different domains.
* Mapping between different data types.
* User-defined operations over the received messages (:ref:`transformation libraries`).
* Communication with other environments, as *ROS2*.


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
