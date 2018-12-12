Getting Started
===============

**Table of Contents**

* :ref:`Installation Manual`

* :ref:`Usage`

* :ref:`Example application`

* :ref:`Getting Help`

Installation Manual
^^^^^^^^^^^^^^^^^^^

Before compiling *eProsima Integration Service* you need to have installed *Fast RTPS* as described in its
`documentation <http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html>`__.
You can use *Fast RTPS* as third party too, adding `-DTHIRDPARTY=ON` in the cmake command.

To clone this project, just execute:

.. code-block:: bash

    $ git clone --recursive https://github.com/eProsima/integration-service

Now, for compiling, if you are on Linux you must execute:

.. code-block:: bash

    $ mkdir build && cd build
    $ cmake ..
    $ make
    $ sudo make install

If you are on Windows you must choose a version of Visual Studio:

.. code-block:: bash

    > mkdir build && cd build
    > cmake ..  -G "Visual Studio 14 2015 Win64"
    > cmake --build . --target install

If you want to compile *eProsima Integration Service* without an installed version of *Fast RTPS* you can add
`-DTHIRDPARTY=ON` parameter that downloads it as a third party library. If you prefer to use an already installed
*Fast RTPS*, remove that parameter.

If you are on Linux you must execute:

.. code-block:: bash

    $ mkdir build && cd build
    $ cmake -DTHIRDPARTY=ON ..
    $ make
    $ sudo make install

If you are on Windows you must choose your version of Visual Studio:

.. code-block:: bash

    > mkdir build && cd build
    > cmake ..  -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON
    > cmake --build . --target install


Usage
^^^^^

Once installed you can execute *Integration Service* from your terminal. It receives an unique *XML configuration
file* as parameter.

.. code-block:: bash

    $ integration_service config.xml

That *XML configuration file* is needed by *Integration Service* to create all the needed components to do its job.
You should read the `documentation <https://integration-services.readthedocs.io/en/latest/configuration.html>`__
to know more about how to configure *Integration Service*.

Example application
^^^^^^^^^^^^^^^^^^^

We will use
`domain_change <https://github.com/eProsima/Integration-Service/tree/master/examples/domain_change>`__ as example.
In this example, we have generated a *Fast RTPS* example application using
`FastRTPSGen <https://eprosima-fast-rtps.readthedocs.io/en/latest/geninfo.html>`__ and then configured
the publisher participant in domain **0**, and the subscriber participant in domain **5**.
This means that if a subscriber and a publisher are executed, there will be no communication between them,
even having the same topic and type.

Executing the *Integration Service* will create a bridge between the publisher (as a writer) and the subscriber
(as a reader), and communicate both applications.
`config.xml <https://github.com/eProsima/Integration-Service/tree/master/examples/domain_change/config.xml>`__
The config file, provided in this example, defines a connector like this.

.. code-block:: xml

    <connector name="domain_change">
        <reader participant_profile="domain0" subscriber_profile="is_subscriber"/>
        <writer participant_profile="domain5" publisher_profile="is_publisher"/>
    </connector>

To execute the example properly, we must first compile the example itself, from the
`domain_change example location <https://github.com/eProsima/Integration-Service/tree/master/examples/domain_change>`__.

Linux:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

Windows:

.. code-block:: bash

    > mkdir build
    > cd build
    > cmake -G "Visual Studio 14 2015 Win64" ..
    > cmake --build .

The compilation will generate an example application named DomainChange in the build directory.
When we execute DomainChange as a publisher, it will create its participant in domain 0.
If we launch DomainChange as a subscriber, it will create its participant in domain 5 instead.

Now, we must launch DomainChange in both setups:

.. code-block:: bash

    $ ./DomainChange publisher

And in another terminal:

.. code-block:: bash

    $ ./DomainChange subscriber

As both instances are bound to different domains, the applications will not communicate.
But once we launch IS with the config.xml that comes with the example,
both DomainChange instances will begin to communicate.

In another terminal:

.. code-block:: bash

    $ cd <path_to_is_source>/examples/domain_change
    $ integration_service config.xml

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`__ or by phone at `+34 91 804 34 48 <tel:+34918043448>`__.
