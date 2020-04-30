Integrate a large system
========================

Most systems evolve with time, undergoing the addition of new functionalities or new components.
When these new components are based on software or hardware that don't use a protocol compatible with the rest
of the system, an additional component must be created, usually known as *bridge*.

If the system contains several subsystems, and each component uses a different protocol, a *bridge* must be
created for each existing components pair that need to be communicated, making the integration of the new
component quite unhandy.

.. image:: LARGER_SYSTEM_BAD.png

*eProsima Integration-Service* eases this process, allowing to integrate any *DDS* system into an already
existing system or viceversa, by providing an out-of-the-box bridge that straightforwardly allows to communicate the
*DDS* and the non-*DDS* protocols.
Also, thanks, to its **System-Handle**-based structure, the core of *eProsima Integration-Service* allows to centralise
all the possible bridges.

Once all protocols are communicated with *eProsima Integration-Service*, the inter-components
communication can be easily implemented by means of an individual YAML configuration file.

As explained in the :ref:`introductory section <main features>`, *eProsima Integration-Service* already provides the
**System-Handle** for some common protocols.

.. image:: LARGER_SYSTEM.png

This section is intented to illustrate an example of how *eProsima Integration-Service* integrates a *DDS*
application into a *Orion Context-Broker* system.


Example: Orion Context-Broker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To prepare the deployment and setup the environment correctly, please follow the introductory steps delined in
:ref:`Getting Started <getting started>` and read carefully the :ref:`Important remarks <important remarks>`
section.

To make this example work, you will require to install the **SOSS-FIWARE** **System-Handle**, that you can
download from the dedicated
`SOSS-FIWARE repository <https://github.com/eProsima/SOSS-FIWARE/tree/feature/xtypes-support>`__. Clone it into the
workspace where you have *eProsima Integration-Service* installed:

.. code-block:: bash

    cd is-workspace
    git clone ssh://git@github.com/eProsima/SOSS-FIWARE src/soss-fiware -b feature/xtypes-support

And then build the packages by running:

.. code-block:: bash

    colcon build

Finally, source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

Additionally, you will require:

- An accesible *contextBroker* service.
- An installation of *Fast-RTPS* (at least v1.9.2) with the HelloWorld example working. Indeed, in order to feed
  the *contextBroker*, the example will use a *Fast-RTPS* HelloWorld :code:`publisher` or :code:`subscriber`.

Before proceeding, note that the file :code:`soss-dds/examples/udp/dds_fiware.yaml` in the :code:`src` folder of your
workspace must be edited to match the IP address and port used by the *contextBroker* configuration in the
testing environment.

Open three terminals (replace <url> with the location of the *contextBroker*,
following the format :code:`http://<ip>:<port>`):

- In the first terminal, execute the Helloworld :code:`publisher`:

.. code-block:: bash

    ./HelloWorldExample publisher

- In the second terminal, create/check the value of the :code:`data-binary` field in the *contextBroker*:

  - When testing for the first time, the structure for this test must be created in the *contextBroker*:

  .. code-block:: bash

      curl --include \
          --request POST \
          --header "Content-Type: application/json" \
          --data-binary "{ \"type\": \"String\", \"id\": \"String\", \"data\": { \"value\": \"\" } }" \
          '<url>/v2/entities?options='

  - Check the value of the attribute, if it already exists:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value?type=String

  - If the result isn't empty, set the value to empty:

  .. code-block:: bash

      curl <url>/v2/entities/String/attrs/data/value -X PUT -s -S --header 'Content-Type: text/plain' --data-binary \"\"

- Execute *eProsima Integration-Service* using the :code:`soss` command in the third terminal and with the
  `YAML <https://github.com/eProsima/SOSS-DDS/blob/feature/xtypes-dds/examples/fiware/dds_ros2_fiware_string.yaml>`__
  example file edited previously:

.. _TODO_YAML_LINK_1: Create and link properly the above YAML file.

.. code-block:: bash

    soss soss-dds/examples/udp/dds_fiware.yaml

- Check again the value of the data in the `contextBroker`:

.. code-block:: bash

    curl <url>/v2/entities/String/attrs/data/value?type=String

Now, the value must contain information (normally, :code:`HelloWorld`).

If you want to test the communication the other way around, launch Helloworld as  :code:`subscriber` and force an update
in the *contextBroker* data while *eProsima Integration-Service* is executing with the same YAML file.

**Note**: Each time you execute *eProsima Integration-Service* with the :code:`soss` command in a new shell,
please make sure to have done the sourcing of the colcon overlay with the command

.. code-block:: bash

    source install/setup.bash

or, in alternative, to have added it to the :code:`.bashrc` file.

.. _comment_3: Maybe some changes must be done to allow the conversion between the struct types.
