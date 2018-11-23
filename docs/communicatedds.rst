Communicate two DDS applications
================================

Let's imagine we have two already deployed DDS applications, that uses compatible types and have some minimal
configuration differences. At any moment, it may become necessary to put into communication both DDS worlds, but their
current configuration and types prevent us to make it in a direct way.
We could modify the configuration and types of one of the worlds to make them fully compatible, but that normally isn't
desired.

.. image:: DDS_NO_COMS.png
    :align: center

With *Integration Service* we can configure two *endpoints* with a simple *transformation function* and our DDS world
will communicate, directly and without change any of its configuration.

.. image:: DDS_WITH_IS.png
    :align: center

Routing usage and configuration
-------------------------------

To solve our hypothetical problem we must configure *Integration Service* through its XML :ref:`configuration` file with
one endpoint able to write and read to each *DDS world*. Depending of the *Topic Data Type* of each *DDS world* we
also need a :ref:`Transformation Library` to provide *transformation functions* that will transform the data from
*DDS World A* to *DDS World B* and vice versa.

For example:

The *endpoints* must be configured in the :ref:`Fast-RTPS profiles` section.

.. literalinclude:: config_comms_dds.xml
    :language: XML
    :start-after: <!-- fast-rtps profiles -->
    :end-before: <!-- end fast-rtps profiles -->

An our needed :ref:`Connectors` are declared below:

.. literalinclude:: config_comms_dds.xml
    :language: XML
    :start-after: <!-- connectors -->
    :end-before: <!-- end connectors -->

You only must be careful to relate the correct *participant* with the correct *publisher* or *subscriber*, in this case:
:class:`publisher A` and :class:`subscriber A` *endpoints* belong to :class:`DDS World A` *participant*, and
:class:`publisher B` and :class:`subscriber B` *endpoints* belong to :class:`DDS World B` *participant*.

To transform the data between the *DDS World A* data and the *DDS World B* data,
we will use a *Transformation Library* named :class:`libtransformationDDS.so` that have two
*transformation functions* implemented: :class:`transformA_to_B`, to perform the
tranformation in one way, and  :class:`transformB_to_A` to perform the transformation in the other way.


Routing with Integration Service
--------------------------------

As both *DDS Worlds* use the same protocol, and *Integration Service* supports it out-of-the-box,
we have nothing more to do to allow the communication at protocol level.
But, we still have to convert from one *Topic Data Type* to another.
We should add our own :ref:`transformation library` implementing two functions: :class:`transformA_to_B` to transform
the data from *DDS World A* to *DDS World B*, and :class:`transformB_to_A` to transform the data from *DDS World B*
to *DDS World A*. We will name this library :class:`libtransformationDDS.so`.

We are going to create a source file named :class:`transformationDDS.cpp` with the implementation of both functions.

The first we must do is to include the *TopicDataTypes* involved. For our example, imagine we have an *IDL* file for
each *DDS World* data type.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: /* Type A
    :end-before: // End Type A */

For *DDS World A* data type. After generate the *C++* code of this *IDL* file, *FastRTPSGen* should have generated
a file named :class:`DDSTypeAPubSubTypes.h`. If you want to know more about *FastRTPSGen*, please go to
`Fast RTPS documentation <http://docs.eprosima.com/en/latest/>`__.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: /* Type B
    :end-before: // End Type B */

In the same way for *DDS World B* data type, after generate the *C++* code of this *IDL* file,
*FastRTPSGen* should have generated a file named :class:`DDSTypeBPubSubTypes.h`.

So with these types in mind, we should include both *PubSubTypes* headers in our *transformation library*.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // Include
    :end-before: // End include

The next part isn't mandatory, but we usually add it because it help us making the library portable between different
operating systems and keeps the source code clear to read.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // Definitions
    :end-before: // End Definitions

If you decide to include this part as well, keep it in mind when we will create the *CMakeLists.txt* file.

Now, we are in condition to start with the implementation of each function. We will start with ``transformA_to_B``:

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // A to B
    :end-before: // End A to B

sdfsdfsdf

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // B to A
    :end-before: // End B to A



Creating new routes
-------------------

blah blah