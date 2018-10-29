.. _installation-from-sources:

Installation from Sources
=========================

Before compiling eProsima Integration Services you need to have installed Fast RTPS as described in its `documentation <http://eprosima-fast-rtps.readthedocs.io/en/latest/binaries.html>`_. For cloning this project execute:

Clone the project from Github: ::

    $ git clone --recursive https://github.com/eProsima/integration-services
    $ mkdir integration-services/build && cd integration-services/build

If you are on Linux, execute: ::

    $ cmake ..
    $ make
    $ sudo make install

If you are on Windows, choose your version of Visual Studio using CMake option *-G*: ::

    > cmake -G "Visual Studio 14 2015 Win64" ..
	> cmake --build . --target install --config Release

