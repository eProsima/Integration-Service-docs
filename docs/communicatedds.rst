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

For example, we are going to create a file named :class:`config.xml`.

The *endpoints* must be configured in the :ref:`Fast-RTPS profiles` section.

.. literalinclude:: config_comms_dds.xml
    :language: XML
    :start-after: <!-- fast-rtps profiles -->
    :end-before: <!-- end fast-rtps profiles -->

And our needed :ref:`Connectors` are declared below:

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

Remember that the root tag of our :class:`config.xml` file must be ``<is>`` as describred in the :ref:`configuration`.


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
operating systems and keeps the source code clear to read. It adds some definitions for the compiler to build the
library as shared and to work properly both when importing or exporting its symbols.

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

This function will receive the data from the *DDS World A* and will convert it in data to be published
to the *DDS World B*.
A *standarized* way to do that is to *deserialize* the input data into our ``DDSTypeA`` using its
``DDSTypeAPubSubType``.

Then we can apply our custom data transformation.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // Data transformation from A to B
    :lines: 1,2

Finally, we must serialize the ``DDSTypeB`` data using its ``DDSTypeBPubSubType`` into ``serialized_output``.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // Data transformation from A to B
    :lines: 5,6

The function ``transformB_to_A`` follows the same schema, but does its own data conversion.

.. literalinclude:: transformationDDS.cpp
    :language: cpp
    :start-after: // B to A
    :end-before: // End B to A

After that, we have our *transformation library* implemented, but we still need to build it.
Of course, you could use any build system at your wish, but *IS* provides a *CMakeLists.txt* template that we will use
here as example.

First, we are going to rename the cmake project to *transformationDDS*.

.. literalinclude:: comms_dds_CMake.txt
    :language: cmake
    :lines: 1

We keep all *C++11* and *CMake* version as it is. If you create your *CMakeLists.txt* from scratch remember that
*FastRTPSGen* generates files that depend on *Fast CDR* and *Fast RTPS*, so you must include both dependencies to your
*CMakeLists.txt*.

.. literalinclude:: comms_dds_CMake.txt
    :language: cmake
    :start-after: # packages
    :lines: 1,2

Do you remember the *definitions* section of our *transformation library* that could help us to make the library
more portable? This is where we set the values of these preprocesor definitions to build our library exporting symbols.

.. literalinclude:: comms_dds_CMake.txt
    :language: cmake
    :start-after: # definitions
    :lines: 1-4

Finally we indicate to *CMake* our source code and the library we want to build, along with its dependencies.

.. literalinclude:: comms_dds_CMake.txt
    :language: cmake
    :start-after: # transformationDDS library
    :lines: 1-3

After that, we can just generate our library using *CMake*.

.. code-block:: bash

    $ cmake .
    $ make

It should generate our *libtransformationDDS.so* in the current directory that is the library that
*IS* expects when loads our :class:`config.xml` file.

At this point, we have our configuration file :class:`config.xml` created, and our *transformation library*
*libtransformationDDS.so* built. We are able to launch *IS* with our :class:`config.xml` and enjoy how both *DDS Worlds*
start to communicate, applying our *transformation functions*.

.. code-block:: bash

    $ integration_service config.xml

Creating new routes
-------------------

With the knowledge adquired after study and solve this scenario, you should be able to add new connectors between both
*DDS Worlds* or other *DDS World* (like a new *DDS World C* for example) following this steps:

- Create and configure the needed :ref:`Fast-RTPS profiles` in your XML configuration file.
- Create the needed :ref:`Connectors` in your XML configuration file.
- Implementing your custom :ref:`transformation functions <Transformation Library>`, if needed.
- Generating your library binary.
- Executing *IS* with your XML configuration file.

.. image:: DDS_ROUTES.png
    :align: center


Domain Change Example
---------------------

This example shows how *IS* can communicate two *participants* that belong to different *domains*.

To execute the example properly, we must first compile the example itself, from the `domain_change example location <https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/domain_change>`_.

Linux:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Windows:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake -G "Visual Studio 14 2015 Win64" ..
    $ cmake --build .

The compilation will generate an example application named *DomainChange* in the build directory.
When we execute *DomainChange* as a publisher, it will create its *participant* in the *domain* **0**.
If we launch *DomainChange* as a subscriber, it will create its *participant* in the *domain* **5** instead.

Now, we must launch *DomainChange* in both setups:

.. code-block:: bash

    $ ./DomainChange publisher

And in another terminal:

.. code-block:: bash

    $ ./DomainChange subscriber

As both instances are bound to different *domains*, the applications will not communicate.
But once we launch IS with the `config.xml <https://github.com/eProsima/Integration-Service/blob/feature/TCP_DynTypes/examples/domain_change/config.xml>`__ that comes with the example, both *DomainChange* instances will begin to communicate.

In another terminal:

.. code-block:: bash

    $ cd <path_to_is_source>/examples/domain_change
    $ integration_service config.xml

Here, we can see a schema that represents the internal flow in this example.

.. image:: DomainChange.png
    :align: center
