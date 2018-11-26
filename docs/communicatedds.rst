Communicate two DDS applications
================================

Let's imagine we have two already deployed DDS applications, that uses compatible types and have some minimal
configuration differences. At any moment, it may become necessary to put into communication both DDS worlds, but their
current configuration prevents us to make it in a direct way.
We could modify the configuration of one of the worlds to make them fully compatible, but that normally isn't
desired.

.. image:: DDS_NO_COMS.png
    :align: center

With *Integration Service* we can configure two *endpoints* and our DDS world
will communicate, directly and without changing its configuration.

.. image:: DDS_WITH_IS.png
    :align: center

Routing usage and configuration
-------------------------------

To solve our hypothetical problem we must configure *Integration Service* through its XML :ref:`configuration` file with
one endpoint able to write and read to each *DDS world*. Depending on the *Topic Data Type* of each *DDS world* we
also need a :ref:`Transformation Library` to provide *transformation functions* that will transform the data from
*DDS World A* to *DDS World B* and vice versa. See :ref:`Data Transformation` to see a scenario with
*transformation functions*.

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

Remember that the root tag of our :class:`config.xml` file must be ``<is>`` as described in the :ref:`configuration`.


Routing with Integration Service
--------------------------------

As both *DDS Worlds* use the same protocol, and *Integration Service* supports it out-of-the-box,
we have nothing more to do to allow the communication at the protocol level.

Once we have our configuration file :class:`config.xml` created, we are able to launch *IS* with our
:class:`config.xml` and enjoy how both *DDS Worlds* start to communicate.

.. code-block:: bash

    $ integration_service config.xml

Creating new routes
-------------------

With the knowledge acquired after study and solve this scenario, you should be able to add new connectors between both
*DDS Worlds* or other *DDS World* (like a new *DDS World C* for example) following this steps:

- Create and configure the needed :ref:`Fast-RTPS profiles` in your XML configuration file.
- Create the needed :ref:`Connectors` in your XML configuration file.
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
When we execute *DomainChange* as a publisher, it will create its *participant* in *domain* **0**.
If we launch *DomainChange* as a subscriber, it will create its *participant* in *domain* **5** instead.

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
