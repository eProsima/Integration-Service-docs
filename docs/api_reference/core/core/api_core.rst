.. _api_core:

Core
----

This section of the API reference corresponds to the `include/is/core <https://github.com/eProsima/Integration-Service/tree/main/core/include/is/core>`_ 
folder of the *Integration Service* main repository.

This folder contains several files that can be divided into two different types:

* Those located on the :code:`include/is/core` folder, which are intended for parsing, configuring and 
  executing the *Integration Service* instance.

   .. toctree::

        config
        instance

* Those located on the :code:`include/is/core/runtime` folder, which corresponds with the runtime 
  necessary tools. To date, these include the search tool to load the .mix associated with the 
  system handles that will be used during execution and the tool that allows accessing the fields 
  specified in the YAML configuration file.

   .. toctree::

        runtime/fieldtostring
        runtime/middlewareinterfaceextension
        runtime/search
        runtime/stringtemplate

