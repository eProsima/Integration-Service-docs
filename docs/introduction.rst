Getting Started
===============


Brief introduction to Integration Service
------------------------------------------

Integration Service allows intercommunicating different systems, services, and protocols using a common interface.

.. image:: IS-RTPS-Other.png
   :align: center

Integration Service defines *connectors* which are a pair of *reader* and *writer*,
and optionally a *transformation function*.
Each connector will communicate the data received from its *reader* with its *writer*.
If a *transformation function* was defined, the data will travel from the *reader* to the *transformation function*
which will apply its transformations, and the result will go to the *writer*.

.. image:: fullconnector.png
   :align: center

Executing your first Integration Service
-----------------------------------------

Integration Service uses an XML configuration file to create its connectors.
The complete format of this file is explained in :ref:`Integration Service XML Configuration`.

To create our first application we will just create a new one with the followed content:

.. code-block:: xml

    <is>
        <profiles>
            <participant profile_name="domain0">
                <rtps>
                    <builtin>
                        <domainId>0</domainId>
                    </builtin>
                </rtps>
            </participant>

            <subscriber profile_name="is_subscriber">
                <topic>
                    <name>TextPubSubTopic</name>
                    <dataType>Text</dataType>
                </topic>
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </subscriber>

            <participant profile_name="domain5">
                <rtps>
                    <builtin>
                        <domainId>5</domainId>
                    </builtin>
                </rtps>
            </participant>

            <publisher profile_name="is_publisher">
                <topic>
                    <name>TextPubSubTopic</name>
                    <dataType>Text</dataType>
                </topic>
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </publisher>
        </profiles>

        <connector name="domain_change">
            <reader participant_profile="domain0" subscriber_profile="is_subscriber"/>
            <writer participant_profile="domain5" publisher_profile="is_publisher"/>
        </connector>
    </is>

Let's name this new file as **config.xml**. In the *examples* folder of Integration Service, there is an example named
*domain_change*. Building this example will generate an application called DomainExample.

If you didn't build the example yet execute the following commands in the example folder.

::

    $ mkdir build && cd build
    $ cmake ..
    $ make

To test the example, you must execute the DomainExample application as a publisher and it will create a FastRTPS publisher at domain 0.

::

    $ ./DomainExample publisher

And in the same way, execute DomainExample as a subscriber will create a FastRTPS subscriber at domain 5.

::

    $ ./DomainExample subscriber

This behavior is very similar to the FastRTPS HelloWorldExample, but this publisher and subscriber don't communicate
because they belong to different domains. Keep both DomainExample instances running in different terminals.

We can use Integration Service to allow communication between both DomainExample participants.
We just need to open the folder where we saved our **config.xml** file and execute Integration Service with the file
as an argument in another terminal.

::

    $ integration-service config.xml

Once Integration Service parses the configuration file and both participants matches, they will start to communicate.

But this is only a very simple example of what Integration Service can do.
With Integration Service we can communicate different protocols and services just implementing a few methods
from an interface in our own :ref:`bridge libraries`.
Or we can define custom data transformation making use of :ref:`transformation libraries`.
Integration Service allow us to define and use our own **TopicDataTypes**
through :ref:`types libraries` or  Fast-RTPS **XML Types**.
Another interesting functionality is to replicate data from one reader to many writers or listen from many
readers while writing to one writer, or just define an N to M relationship between writers and readers.
Finally, Integration Service is able to communicate two applications that belong to different subnetworks,
or through the Internet and behind Firewalls and NAT using Fast-RTPS **TCP Transport**.
And of course, we can use all of these features at the same time.