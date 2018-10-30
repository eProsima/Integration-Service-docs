Installation from Binaries
==========================

You can always download the latest binary release of *eProsima Integration Services* from the `company website <http://www.eprosima.com/>`_.

Installation for Windows 7 32-bit and 64-bit
--------------------------------------------

Execute the installer and follow the instructions, choosing your preferred Visual Studio version and architecture when prompted.

Environment Variables
^^^^^^^^^^^^^^^^^^^^^

*eProsima Integration Services* requires the following environment variables setup in order to function properly

* FASTRTPSHOME: Root folder where *eProsima Integration Services* is installed.
* Additions to the PATH: the /bin folder and the subfolder for your Visual Studio version of choice should be appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

Installation for Linux
----------------------

Extract the contents of the package. Fast-RTPS must be already installed.

Configure the compilation:

::

        $ ./configure --libdir=/usr/lib

If you didn't installed Fast-RTPS in the default path, you must provide to autotools the path of fastrtps and fastcdr libraries, for example:

::

        $ ./configure LDFLAGS='-L/home/user/Fast-RTPS/lib/libfastrtps.so -L/home/user/Fast-RTPS/lib/libfastcdr.so' --libdir=/usr/lib

If you want to compile with debug symbols (which also enables verbose mode):

::

        $ ./configure CXXFLAGS="-g -D__DEBUG" --libdir=/usr/lib

After configuring the project compile and install the library:

::

        $ sudo make install

Once installed if you want to use Integration Services as a public shared library:

::

        $ sudo ldconfig