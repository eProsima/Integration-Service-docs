Dynamic Data Integration
========================

*Fast-RTPS 1.7.0* includes *Dynamic Data Types* and *XML Types* as new features.
*Integration Service* benefits from these features allowing us to integrate *Dynamic Data Types*.

In this *use case* we are going to illustrate a scenario using both types of *Dynamic Data* declararions
(*Dynamic Data Types* and *XML Types*) and a *transformation function*.
For a full *use case* explaining *transformation functions* you can see :ref:`Data transformation`.

The following image shows the data flow throw *transformation function*.

.. image:: DYNAMIC_CASE.png
    :align: center

Dynamic Data usage and configuration
------------------------------------

We have two ways to declare *Dynamic Data Types* in *Integration Service*.
We can make use of a :ref:`IS Types Libraries <Types Library>` or we can declare them directly as
:ref:`Fast-RTPS XML Types <IS Types configuration>` in the :ref:`XML configuration file <configuration>`.

.. literalinclude:: configuration.xml
    :language: xml
    :start-after: <!-- IS Types Start -->
    :end-before: <!-- IS Types End -->
    :dedent: 4

As example, we are going to define two *Dynamic Data Types*: :class:`TypeA` and :class:`TypeB`.
They will be used to communicate our *DDS World A* with *DDS World B*, applying a *transformation function*, that
will be the typical scenario.

As first step, we are going to define our library through the XML configuration file to indicate to *IS*
which library implements each data type. In our case, both types :class:`TypeA` and :class:`TypeB` will be
implemented in the same library.

.. literalinclude:: dynamic_case.xml
    :language: xml
    :start-after: <!-- IS Types Start -->
    :end-before: <!-- IS Types End -->

Then, in the :ref:`Fast-RTPS profiles` section the *publisher* and *subscriber* can refer our data types by their name.

.. literalinclude:: dynamic_case.xml
    :language: xml
    :start-after: <!-- Profiles Start -->
    :end-before: <!-- Profiles End -->

An our needed :ref:`Connector <Connectors>` are declared below:

.. literalinclude:: dynamic_case.xml
    :language: xml
    :start-after: <!-- Connector Start -->
    :end-before: <!-- Connector End -->

As this scenario focus on *Dynamic Topic Data*, we are not going to explain how to create the
:ref:`transformation library`, but you have a complete *use case* about it in :ref:`Data transformation` section.

Dynamic Data with Integration Service
-------------------------------------

As we said in the previous section, there are two ways to declare *Dynamic Data Types* in *IS*.
Is we prefer to use :ref:`Fast-RTPS XML Types <IS Types configuration>`, there is no need to perform any additional
step, and our *Data Types* will be available to use directly. But, if we decide to use a :ref:`Types Library` we
need to implement it and build it.

We are going to call this library :class:`libtypeatypeb.so`.
Remember we must implement the function ``GetTopicType``.
There is a template file *typeslib.cpp* in resources folder in the *IS* repository.

We are going to start from scratch a new *Types Library* to show the full process.
We will create a new file name *typeatypeb.cpp*.

We start including all needed headers to use *Fast-RTPS Dynamic Types*.

.. literalinclude:: dynamic_case.cpp
    :language: cpp
    :start-after: // Include Start
    :end-before: // Include End

The next part isn't mandatory, but we usually add it because it help us making the library portable between different
operating systems and keeps the source code clear to read. Additionally, in this case, may be useful to add some
``using namespaces``.

.. literalinclude:: dynamic_case.cpp
    :language: cpp
    :start-after: // Define Start
    :end-before: // Define End

If you decide to include this part as well, keep it in mind when we will create the *CMakeLists.txt* file.

Usually it is useful to create auxiliar functions to create each type of data.

To create DataTypeA:

.. literalinclude:: dynamic_case.cpp
    :language: cpp
    :start-after: // DataTypeA Start
    :end-before: // DataTypeA End

To create DataTypeB:

.. literalinclude:: dynamic_case.cpp
    :language: cpp
    :start-after: // DataTypeB Start
    :end-before: // DataTypeB End

Now, we can use these auxiliar functions to simplify the ``GetTopicType`` implementation:

.. literalinclude:: dynamic_case.cpp
    :language: cpp
    :start-after: // GetTopicType Start
    :end-before: // GetTopicType End

After that, we have our *types library* implemented, but we still need to build it.
Of course, you could use any build system at your wish, but *IS* provides a *CMakeLists.txt* template that we will use
here as example.

First, we are going to rename the cmake project to *typeatypeb*.

.. literalinclude:: dynamic_case_CMake.txt
    :language: cmake
    :lines: 1

We keep all *C++11* and *CMake* version as it is. If you create your *CMakeLists.txt* from scratch remember that
*FastRTPSGen* generates files that depend on *Fast CDR* and *Fast RTPS*, so you must include both dependencies to your
*CMakeLists.txt*.

.. literalinclude:: dynamic_case_CMake.txt
    :language: cmake
    :start-after: # packages
    :lines: 1,2

Do you remember the *definitions* section of our *types library* that could help us to make the library
more portable.
This is where we set the values of these preprocesor definitions to build our library exporting symbols.

.. literalinclude:: dynamic_case_CMake.txt
    :language: cmake
    :start-after: # definitions
    :lines: 1-4

Finally we indicate to *CMake* our source code and the library we want to build, along with its dependencies.

.. literalinclude:: dynamic_case_CMake.txt
    :language: cmake
    :start-after: # typeatypeb library
    :lines: 1-3

After that, we can just generate our library using *CMake*.

.. code-block:: bash

    $ cmake .
    $ make

It should generate our *libtypeatypeb.so* in the current directory that is the library that
*IS* expects when loads our :class:`config.xml` file.

We should have created the :ref:`transformation library` too.
You have a complete *use case* about *tranformation libraries* in :ref:`Data transformation` section.

At this point, we have our configuration file :class:`config.xml` created, and our *types library*
*libtypeatypeb.so* built. We are able to launch *IS* with our :class:`config.xml` and enjoy how both types are being
generated.

.. code-block:: bash

    $ integration_service config.xml

Creating new Dynamic Data
-------------------------

Now, we are able to define *types libraries* to create *Dynamic Data Types* as well as define them through *XML Types*.
The steps needed to do it are:

- Create and configure the needed :ref:`IS Types configuration` in your XML configuration file.
- Create and configure the needed :ref:`Fast-RTPS profiles` in your XML configuration file.
- Create the needed :ref:`Connectors` in your XML configuration file.
- Implementing your custom :ref:`Types Library`, if needed.
- Generating your *types library* binary, if needed.
- Executing *IS* with your XML configuration file.

Dynamic Types example
---------------------

There is an example implemented in
`dynamic_types example  <https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/dynamic_types>`_
where you can see the use of dynamic types.

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
and both applications will start to communicate. *dyn_dyn_config[_win].xml* uses *Dynamic Types* in both flavors.

.. code-block:: bash

    $ ./integration_service dyn_dyn_config_win.xml
