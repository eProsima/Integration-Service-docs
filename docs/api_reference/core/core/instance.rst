.. _api_core_instance:

.. rst-class:: api-ref

Instance
--------

.. doxygenclass:: eprosima::is::core::Instance
    :project: IntegrationService
    :members:
    :no-link:

.. doxygentypedef:: eprosima::is::core::MiddlewarePrefixPathMap
    :project: IntegrationService
    :no-link:
    
InstanceHandle
--------------

.. doxygenclass:: eprosima::is::core::InstanceHandle
    :project: IntegrationService
    :members:
    :no-link:
    
.. doxygenfunction:: eprosima::is::run_instance(int argc, char *argv[])
    :project: IntegrationService
    :no-link:

.. doxygenfunction:: eprosima::is::run_instance(const std::string &config_file_path, const std::vector<std::string> &is_prefixes = {}, const core::MiddlewarePrefixPathMap &middleware_prefixes = {})
    :project: IntegrationService
    :no-link:

.. doxygenfunction:: eprosima::is::run_instance(const YAML::Node &config_node, const std::vector<std::string> &is_prefixes = {}, const core::MiddlewarePrefixPathMap &middleware_prefixes = {})
    :project: IntegrationService
    :no-link: