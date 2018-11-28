Data transformation
===================

One of the most common issues during the integration of new systems is making them compatible with
the ones already implemented in the current environment. Even using RTPS systems there could
be situations where two different data types cause direct communication impossible.
In this scene is where *Integration Service*, and its ability to transform data, is useful to adapt different data types to be compatible.

The following image shows the data flow throw transformation function.

.. image:: TRANF_CASE.png
    :align: center

Transformation usage and configuration
--------------------------------------

*Integration Service* must be configured through its XML :ref:`configuration` file.
In this case, a :ref:`Transformation Library` is needed to provide *transformation functions* that will transform the data from *DDS World A* to *DDS World B* and vice versa.

In this example, a file named :class:`config.xml` needs to be created.
The *endpoints* must be configured in the :ref:`Fast-RTPS profiles` section.

.. literalinclude:: config_comms_dds.xml
    :language: XML
    :start-after: <!-- fast-rtps profiles -->
    :end-before: <!-- end fast-rtps profiles -->
    :dedent: 4

And the needed :ref:`Connectors` are declared below:

.. literalinclude:: config_comms_dds.xml
    :language: XML
    :start-after: <!-- connectors -->
    :end-before: <!-- end connectors -->
    :dedent: 4

It's important to associate the correct *participant* with the correct *publisher* or *subscriber*, in this case:
:class:`publisher A` and :class:`subscriber A` *endpoints* belong to :class:`DDS World A` *participant*, and
:class:`publisher B` and :class:`subscriber B` *endpoints* belong to :class:`DDS World B` *participant*.

To transform the data between the *DDS World A* data and the *DDS World B* data,
Integration Service will use a *Transformation Library* named :class:`libtransformationData.so` that have
*transformation functions* implemented: :class:`transformA_to_B`, to perform the
transformation in one way, and :class:`transformB_to_A` to perform the other way.


Data transformation with Integration Service
--------------------------------------------

This example needs a custom :ref:`transformation library` implementing two functions: :class:`transformA_to_B` to transform
the data from *DDS World A* to *DDS World B*, and :class:`transformB_to_A` to transform the data from *DDS World B*
to *DDS World A*. The name of this library will be :class:`libtransformationData.so`.

The implementation of both functions will be stored in a source file named :class:`transformationDDS.cpp`.

The first step of the implementation is including the *TopicDataTypes* involved.
In the example, each *DDS World* data type is related to an *IDL* file.

For *DDS World A* data type:

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: /* Type A
    :end-before: // End Type A */

After generating the *C++* code of this *IDL* file, *FastRTPSGen* will generate
a file named :class:`DDSTypeAPubSubTypes.h`. If you want to know more about *FastRTPSGen*, please go to
`Fast RTPS documentation <http://docs.eprosima.com/en/latest/>`__.

In the same way for *DDS World B* data type:

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: /* Type B
    :end-before: // End Type B */

After generating the *C++* code of this *IDL* file, *FastRTPSGen* will generate a file named :class:`DDSTypeBPubSubTypes.h`.

So with these types in mind, it's mandatory to include both *PubSubTypes* headers in the *transformation library*.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // Include
    :end-before: // End include

.. _cmake_definitions_code_transformdata:

The next part is optional, but it helps to make the library portable between different
operating systems and keeps the source code clear to read.

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // Definitions
    :end-before: // End Definitions

This optional section should be taken in mind during the creation of the *CMakeLists.txt* file to configure the project.
After these steps to set the workspace, this will be the code of ``transformA_to_B``:

.. literalinclude:: transform_data.cpp
    :language: cpp
    :start-after: // A to B
    :end-before: // End A to B

After writing this function and ``transformB_to_A`` with the opposite conversion, the *transformation library* has been implemented, but it needs to be built.
*Integration Service* provides a *CMakeLists.txt* template that can be used as in this example.

The first step is renaming the cmake project to *transformationData*.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :lines: 1

It's recommendable to keep all *C++11* and *CMake* version as it is, but to create the *CMakeLists.txt* from scratch
it's important to keep in mind that *FastRTPSGen* generates files that depend on *Fast CDR* and *Fast RTPS*,
so both libraries must be included as dependencies to the *CMakeLists.txt*.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # packages
    :lines: 1,2

To make the library more portable the cmake file needs to add the
:ref:`preprocessor definitions <cmake_definitions_code_transformdata>` to build the library exporting symbols.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # definitions
    :lines: 1-4

The final step is indicating to *CMake* the source code files and the library to build, along with its dependencies.

.. literalinclude:: transform_data_CMake.txt
    :language: cmake
    :start-after: # transformationData library
    :lines: 1-3

After that, *CMake* will generate the library running these commands:

.. code-block:: bash

    $ cmake .
    $ make

It should generate the file *libtransformationData.so* in the current directory, and that's the library that
*IS* expects when loads the :class:`config.xml` file.

At this point, there is a configuration file :class:`config.xml` created, and a *transformation library*
*libtransformationData.so* built.
*Integration Service* now allows connecting both *DDS Worlds* adapting their data types to be compatible.

.. code-block:: bash

    $ integration_service config.xml

Creating new transformations
----------------------------

The steps needed to define *transformation functions* adapting data types between two different domains are:

- Create and configure the needed :ref:`Fast-RTPS profiles` in an XML configuration file.
- Create the needed :ref:`Connectors` in the XML configuration file.
- Implementing custom :ref:`transformation functions <Transformation Library>`.
- Generating the binary of the library.
- Executing *Integration Service* with the XML configuration file.

Transformation Data example
---------------------------

There is an example implemented in
`dynamic_types example <https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/dynamic_types>`_
that shows the use of a transformation function.

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

This example creates the communication bridge between the
`HelloWorld <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C++/HelloWorldExample>`_ and the
`Keys <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C++/Keys>`_ examples from FastRTPS.
HelloWorldExample must be started as a publisher and the Keys example as a subscriber.

.. code-block:: bash

    $ ./HelloWorldExample publisher

And in another terminal:

.. code-block:: bash

    $ ./keys subscriber

In this step, the applications don’t communicate between them,
but after starting the *Integration Service* with the provided configuration file,
the communication starts.

.. code-block:: bash

    $ ./integration_service static_static_config_win.xml
