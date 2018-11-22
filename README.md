# eProsima Integration Service
![http://www.eprosima.com](https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ)

eProsima Integration Services is a library and an utility based on Fast RTPS for making communication bridges between different systems, services and protocols. With the Integration Services the user can create parametric communication bridges between applications. At the same time, it is able to perform some transformations over the messages such as customized routing, mapping between input and output attributes or data modification.

Some of the possibilities offered by the Integration Services are:

-    Connections for jumping from topics which are running on different domains.
-    Adapters for mapping the attributes from types with different IDL definitions.
-    User-defined operations over the circulating messages.
-    Communication with others environments, as ROS2.


<p align="center"> <img src="docs/IS-main.png" alt="Default behaviour"/> </p>

<hr></hr>

#### Table Of Contents

[Installation](#installation)

[Features](#features)

[Documentation](#examples)

[Getting Help](#getting-help)

<hr></hr>

#### Installation

Before compiling *eProsima Integration Service* you need to have installed *Fast RTPS* as described in its [features](http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html). 
To clone this project, just execute:

```bash
    $ git clone --recursive https://github.com/eProsima/integration-service
```

IMPORTANT: *eProsima Integration Service* uses new features that aren't released 
on the master branch of FastRTPS yet. 
To compile it you must switch to the *Develop* branch.

Now, for compiling, if you are on Linux you must execute:

```bash
    $ mkdir build && cd build
    $ cmake ..
    $ make
```

If you are on Windows you must choose a version of Visual Studio:

```bash
    > mkdir build && cd build
    > cmake ..  -G "Visual Studio 14 2015 Win64"
    > cmake --build .
```

If you want to compile *eProsima Integration Service* without an installed version of *Fast RTPS* you can add 
an additional parameter that downloads it as a third party library.

If you are on Linux you must execute:

```bash
    $ mkdir build && cd build
    $ cmake -DTHIRDPARTY=ON ..
    $ make
```

If you are on Windows you must choose your version of Visual Studio:

```bash
    > mkdir build && cd build
    > cmake ..  -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON
    > cmake --build .
```

<hr></hr>

## Domain Change example

In this example, we have generated a *Fast RTPS* example application using *fastrtpsgen* and then configured the publisher participant in the domain **0**, and the subscriber participant in the domain **5**.

This means that if a subscriber and a publisher are executed, there will be no communication between them, even having the same topic and type. 

Executing the *Integration Service* will create a bridge between the publisher (as a writer) and the subscriber (as a reader), and communicate both applications. The config file [config.xml](<https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/domain_change/config.xml>), provided in this example, defines a connector like this. 

```xml
    <connector name="domain_change">
        <reader participant_profile="domain0" subscriber_profile="is_subscriber"/>
        <writer participant_profile="domain5" publisher_profile="is_publisher"/>
    </connector>
```

To execute the example properly, we must first compile the example itself, from the [domain_change example location](<https://github.com/eProsima/Integration-Service/tree/feature/TCP_DynTypes/examples/domain_change>)

Linux:

```bash
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
```

Windows:

```bash
    > mkdir build
    > cd build
    > cmake -G "Visual Studio 14 2015 Win64" ..
    > cmake --build .
```

The compilation will generate an example application named DomainChange in the build directory. When we execute DomainChange as a publisher, it will create its participant in the domain 0. If we launch DomainChange as a subscriber, it will create its participant in the domain 5 instead.

Now, we must launch DomainChange in both setups:

```bash
    $ ./DomainChange publisher
```

And in another terminal:

```bash
    $ ./DomainChange subscriber
```

As both instances are bound to different domains, the applications will not communicate. But once we launch IS with the config.xml that comes with the example, both DomainChange instances will begin to communicate.

In another terminal:

```bash
    $ cd <path_to_is_source>/examples/domain_change
    $ integration_service config.xml
```

Here, we can see a schema that represents the internal flow in this example.

<hr></hr>

## Documentation

You can access the documentation online, which is hosted on [Read the Docs](https://integration-services.readthedocs.io).

<hr></hr>

## Getting Help

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.
