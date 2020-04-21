Use cases
=========

In this section the most representative use-cases of :code:`integration-service` are shown.
Namely, we will go through the following:

- :code:`DDS`-bridge
- Integrate a large system
- Add compatibility to an old/unsupported protocol
- WAN communication


DDS Bridge
^^^^^^^^^^

A typical scenario faced when bridging different :code:`DDS`-based systems is that these systems use incompatible
configurations.
This happens for example in the communication between :code:`DDS`' *HelloWorld* and :code:`ROS2`'s *talker-listener*.

.. image:: DDS_NO_COMS.png

A user with knowledge of both systems may be aware that :code:`ROS2` uses :code:`DDS` as a middleware but hides some of 
:code:`DDS`' configuration details, thus making it difficult, if not impossible, a direct communication.

By using :code:`integration-service`, this communication can be achieved with minimal effort from the user.

.. image:: DDS_WITH_IS.png

See :ref:`ROS2` for an example of this use-case.

Integrate a large system
^^^^^^^^^^^^^^^^^^^^^^^^

Most systems evolve with time, undergoing the addition of new functionalities or new components.
When these new components are based on software or hardware that don't use a protocol compatible with the rest
of the system, an additional component must be created, usually known as *bridge*.

If the system contains several subsystems, and each component uses a different protocol, a *bridge* must be
created for each existing components pair that need to be communicated, making the integration of the new
component quite unhandy.

.. image:: LARGER_SYSTEM_BAD.png

:code:`Integration-service` eases this process, allowing to integrate any :code:`DDS` system into an already
existing system or viceversa, by providing an out-of-the-box bridge that straightforwardly allows to communicate the 
:code:`DDS` and the non-:code:`DDS` protocols.

Also, the core of :code:`integration-service` allows to centralise all the possible bridges among
the subsystems by means of system-specific plugins, or **System-Handles**, that speak the same language as the core.

Once all the protocols of interest are communicated with :code:`integration-service`,
each via a dedicated **System-Handle**, the inter-components communication
can be easily implemented by means of a YAML configuration file, rather than by creating a dedicated 
bridge for each pair of communicating components.
For a system made of *N* components, this means that the number of new software parts to add grows as *N*
rather than *N²*.

Notice that :code:`integration-service` already provides the **System-Handle** for most of the more common
protocols.

.. image:: LARGER_SYSTEM.png

See :ref:`Orion Context-Broker` as an example of this use-case.

Add compatibility to an old/unsupported protocol
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Another typical scenario encountered when communicating different systems is that they use different protocols,
for example, :code:`DDS` and :code:`SOME/IP`.

.. image:: DDS_NO_SOMEIP.png

In this case, in the absence of :code:`soss` and :code:`integration service`
the user should create a custom :code:`DDS` to :code:`SOME/IP` bridge that will not be reusable to
communicate neither of the two with other protocols.

By using :code:`integration-service` instead, this communication can be achieved with minimum user's effort.
In this specific case, a :code:`SOME/IP` **System-Handle** already exists, so the communication with :code:`DDS` is 
essentially direct.
However, the communication is straightforward enough even if a dedicated **System-Handle** doesn't exist yet, as
the user can create his own **System-Handle**, thus becoming able to communicate with :code:`DDS` and
any other protocol already supported by :code:`integration-service`.
For more information regarding how to generate a **System-Handle** from scratch, please consult REF.

.. image:: DDS_IS_SOMEIP.png

See :ref:`Some/IP` for an example of this use-case.

WAN communication
^^^^^^^^^^^^^^^^^

One of the most critical and powerful use cases is that of two systems located in different geographical regions
which need to communicate through the Internet, using a *WAN* connection.

Using a pair of :code:`integration-service` instances, or **System-Handles**, one for each system,
this scenario can be addressed with a **secure TCP tunnel** thanks to the **SSL TCP** capabilities of `Fast-RTPS`.

.. image:: WAN.png

In this case, we can see :code:`integration-service` as a gateway to translate each system to :code:`DDS`over
:code:`SSL-TCP`. A proper configuration of the destination router and firewalls will allow the communication.

See :ref:`WAN TCP Tunneling` for an example of this use-case.
