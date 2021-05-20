# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.12)

# Get version from sphinx's conf.py so that the cmake and the sphinx versions match
file(STRINGS
    ${CMAKE_SOURCE_DIR}/docs/conf.py
    LIB_VERSION_TMP
    REGEX "^version = u'[0-9]+.[0-9]+.[0-9]+'"
    )

string(REGEX REPLACE "^version = u'([0-9]+.[0-9]+.[0-9]+)'"
    "\\1"
    LIB_VERSION_STR
    ${LIB_VERSION_TMP}
    )

project(integration-service-docs VERSION "${LIB_VERSION_STR}" LANGUAGES C CXX)


####################################################################################################
# Build Doxygen documentation
####################################################################################################

if(BUILD_API_REFERENCE)
    find_package(Doxygen REQUIRED)

    # Integration Service and all the SHs are required for:
    #    * Generating the doxygen XML files that are then used for the API reference

    find_path(CORE_API_REFERENCE index.xml PATHS ${CMAKE_INSTALL_PREFIX}/../is-core/doxygen/xml REQUIRED)
    find_path(FASTDDS_API_REFERENCE index.xml PATHS ${CMAKE_INSTALL_PREFIX}/../is-fastdds/doxygen/xml REQUIRED)
    find_path(ROS1_API_REFERENCE index.xml PATHS ${CMAKE_INSTALL_PREFIX}/../is-ros1/doxygen/xml REQUIRED)
    find_path(ROS2_API_REFERENCE index.xml PATHS ${CMAKE_INSTALL_PREFIX}/../is-ros2/doxygen/xml REQUIRED)
    find_path(WEBSOCKET_API_REFERENCE index.xml PATHS ${CMAKE_INSTALL_PREFIX}/../is-websocket/doxygen/xml REQUIRED)

endif()

####################################################################################################
# Build Sphinx documentation
####################################################################################################

# Find sphinx
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Sphinx REQUIRED)

set(SPHINX_SOURCE "${CMAKE_SOURCE_DIR}/docs")

# CMake project always build the documentaion using HTML builder. Users can always build with
# other builders using sphinx directly
set(INTEGRATION_SERVICE_DOCS_BUILDER html)

# Generate the sphinx documentation
add_custom_target(Sphinx ALL
    COMMAND
    ${SPHINX_EXECUTABLE} -b ${INTEGRATION_SERVICE_DOCS_BUILDER}
    # Tell Breathe where to find the Doxygen output
    -d "${PROJECT_BINARY_DIR}/doctrees"
    ${SPHINX_SOURCE}
    ${PROJECT_BINARY_DIR}/${INTEGRATION_SERVICE_DOCS_BUILDER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating documentation with Sphinx")

# Install the generated docs
install(DIRECTORY ${PROJECT_BINARY_DIR}/${INTEGRATION_SERVICE_DOCS_BUILDER}
    DESTINATION .
    COMPONENT ${INTEGRATION_SERVICE_DOCS_BUILDER}
    PATTERN ".buildinfo" EXCLUDE)
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Integration-Service-docs ${INTEGRATION_SERVICE_DOCS_BUILDER}")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
    "eProsima Integration Service documentation in ${INTEGRATION_SERVICE_DOCS_BUILDER} format")
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${INTEGRATION_SERVICE_DOCS_BUILDER})
