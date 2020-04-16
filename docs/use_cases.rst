Use cases
=========

In this section the most representative use-cases are shown:


DDS Bridge
^^^^^^^^^^

A typical scenario when communicating different `DDS` systems is to face that these systems use incompatible
configuration, for example, `DDS`'s HelloWorld and `ROS 2`'s Talker-Listener.

.. image:: DDS_NO_COMS.png

A user with knowledge of both systems may know that `ROS 2` uses `DDS` as middleware but hides some
configuration details making difficult, or even impossible a direct communication.

By using `integration-service` this communication can be achieved with minimal effort from the user.

.. image:: DDS_WITH_IS.png

See :ref:`ROS2` for an example of this use-case.

Integrate a large system
^^^^^^^^^^^^^^^^^^^^^^^^

Most systems evolve with the time, adding new functionality or new components.
When these new components are software or hardware, that doesn't use a protocol compatible with the rest
of the system, we are forced to create an additional component usually known as *bridge*.

If the system contains several subsystems, and each component use a different protocol, a *bridge* must be
created for each existing component pair that must communicate.

.. image:: LARGER_SYSTEM_BAD.png

`Integration-service` eases this integration allowing to integrate any `DDS` system into an already existing
system or viceversa, by just creating a **System-Handle** to communicate with the non-DDS protocol.
Also, it allows to centralise all the possible bridging by giving a common language for all **System-Handle**,
so only a **System-Handle** is needed for each component, instead of for each pair of communicating components.
Additionally, `soss`, the core of `integration-service`, already provides some of the most common protocols
**System-Handle**.
Once a proper **System-Handle** exists, the communication between different protocols is as easy as
create a near-human-readable YAML configuration file.

.. image:: LARGER_SYSTEM.png

See :ref:`Orion Context-Broker` for an example of this use-case.

Add compatibility to an old/unsupported protocol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Another typical scenario when communicating different systems is to face that these systems use different protocols,
for example, `DDS` and `Some/IP`.

.. image:: DDS_NO_SOMEIP.png

In this case, the user should create a custom `DDS` to `Some/IP` bridge that will not be reusable to communicate with
other protocols.

By using `integration-service` this communication can be achieved with minimal effort from the user.
For this example, a `Some/IP` **System-Handle** already exists, but if it doesn't, the user can create his own
**System-Handle** and then, will be able to communicate with any other protocol already supported by
`integration-service`, not only `DDS`.

.. image:: DDS_IS_SOMEIP.png

See :ref:`Some/IP` for an example of this use-case.

WAN communication
^^^^^^^^^^^^^^^^^

One of the most critical and powerful cases is when two systems located in different geographical regions
need to communicate through the Internet, using a *WAN* connection.

Using a pair of `integration-service` instances, one for each system, this scenario can be solved with
a **secure TCP tunnel** thanks to the **SSL TCP** capabilities of `Fast-RTPS`.

.. image:: WAN.png

In this case, we can see `integration-service` as a gateway to translate each system to `DDS over SSL-TCP`.
A proper configuration of the destination router and firewalls, will allow the communication.

See :ref:`WAN TCP Tunneling` for an example of this use-case.
