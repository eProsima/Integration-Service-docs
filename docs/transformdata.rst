Data transformation
===================

There are situations where two independent RTPS systems need to share information.
Maybe those RTPS systems use different data types so there is not possible a direct communication.
In this scene is where *Integration Service*, and its ability transforming data, is useful and necessary.
We can define a function to make possible the understanding between the RTPS systems.

Follow image shows the data flow throw transformation function.

.. image:: TRANF_CASE.png
    :align: center

Transformation usage and configuration
--------------------------------------

*Integration Service* must be configured through its XML :ref:`configuration` file.
In this case is needed a :ref:`Transformation Library` to provide *transformation functions* that will transform the data from *DDS World A* to *DDS World B* and vice versa.

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
we will use a *Transformation Library* named :class:`libtransformationData.so` that have
*transformation functions* implemented: :class:`transformA_to_B`, to perform the
tranformation in one way.


Data transformation with Integration Service
--------------------------------------------

We should add our own :ref:`transformation library` implementing two functions: :class:`transformA_to_B` to transform
the data from *DDS World A* to *DDS World B*, and :class:`transformB_to_A` to transform the data from *DDS World B*
to *DDS World A*. We will name this library :class:`libtransformationData.so`.

We are going to create a source file named :class:`transformationDDS.cpp` with the implementation of both functions.

The first we must do is to include the *TopicDataTypes* involved. For our example, imagine we have an *IDL* file for
each *DDS World* data type.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: /* Type A
    :end-before: // End Type A */

For *DDS World A* data type. After generate the *C++* code of this *IDL* file, *FastRTPSGen* should have generated
a file named :class:`DDSTypeAPubSubTypes.h`. If you want to know more about *FastRTPSGen*, please go to
`Fast RTPS documentation <http://docs.eprosima.com/en/latest/>`__.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: /* Type B
    :end-before: // End Type B */

In the same way for *DDS World B* data type, after generate the *C++* code of this *IDL* file,
*FastRTPSGen* should have generated a file named :class:`DDSTypeBPubSubTypes.h`.

So with these types in mind, we should include both *PubSubTypes* headers in our *transformation library*.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // Include
    :end-before: // End include

The next part isn't mandatory, but we usually add it because it help us making the library portable between different
operating systems and keeps the source code clear to read.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // Definitions
    :end-before: // End Definitions

If you decide to include this part as well, keep it in mind when we will create the *CMakeLists.txt* file.

Now, we are in condition to start with the implementation of each function. We will start with ``transformA_to_B``:

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // A to B
    :end-before: // End A to B

After that, we have our *transformation library* implemented, but we still need to build it.
Of course, you could use any build system at your wish, but *IS* provides a *CMakeLists.txt* template that we will use
here as example.

First, we are going to rename the cmake project to *transformationData*.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :lines: 1

We keep all *C++11* and *CMake* version as it is. If you create your *CMakeLists.txt* from scratch remember that
*FastRTPSGen* generates files that depend on *Fast CDR* and *Fast RTPS*, so you must include both dependencies to your
*CMakeLists.txt*.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # packages
    :lines: 1,2

Do you remember the *definitions* section of our *transformation library* that could help us to make the library
more portable.
This is where we set the values of these preprocesor definitions to build our library exporting symbols.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # definitions
    :lines: 1-4

Finally we indicate to *CMake* our source code and the library we want to build, along with its dependencies.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # transformationData library
    :lines: 1-3

After that, we can just generate our library using *CMake*.

.. code-block:: bash

    $ cmake .
    $ make

It should generate our *libtransformationData.so* in the current directory that is the library that
*IS* expects when loads our :class:`config.xml` file.

At this point, we have our configuration file :class:`config.xml` created, and our *transformation library*
*libtransformationData.so* built. We are able to launch *IS* with our :class:`config.xml` and enjoy how both *DDS Worlds*
start to communicate, applying our *transformation functions*.

.. code-block:: bash

    $ integration_service config.xml

Creating new transformations
----------------------------

Now, we are able to define transformation functions to adapt data types between two different domains.
The steps needed to do it are:

- Create and configure the needed :ref:`Fast-RTPS profiles` in your XML configuration file.
- Create the needed :ref:`Connectors` in your XML configuration file.
- Implementing your custom :ref:`transformation functions <Transformation Library>`.
- Generating your library binary.
- Executing *IS* with your XML configuration file.

Transformation Data example
---------------------------

There is an example implemented in
`dynamic_types example <https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/dynamic_types>`_
where you can see the use of a transformation function.

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

This example allow the communication between
`HelloWorld <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C++/HelloWorldExample>`_ and
`Keys <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C++/Keys>`_ examples from FastRTPS.
The HelloWorld example must be started as a publisher and the Keys example as a subscriber.

.. code-block:: bash

    $ ./HelloWorld publisher

And in another terminal:

.. code-block:: bash

    $ ./Keys subscriber

You will notice that there is no communication between both applications.
Run the *Integration Service* with one of the provided configuration files,
and both applications will start to communicate.

.. code-block:: bash

    $ ./integration_service static_static_config_win.xml
