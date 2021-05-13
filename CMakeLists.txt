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

# Integration Service and all the SHs are required for:
#    * Generating the doxygen XML files that are then used for the API reference

find_package(is-core REQUIRED)
get_target_property(IS_CORE_INCLUDE_DIR is::is-core INTERFACE_INCLUDE_DIRECTORIES)

message(STATUS "Integration Service include directories: ${IS_CORE_INCLUDE_DIR}")
file(GLOB_RECURSE HPP_FILES "${IS_CORE_INCLUDE_DIR}/**/*.h*")

find_package(Doxygen REQUIRED)
# Create doxygen directories
add_custom_target(doc-dirs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/html/doxygen
    COMMENT "Creating documentation directories" VERBATIM)

# Doxygen related variables
set(DOXYGEN_INPUT_DIR "${IS_CORE_INCLUDE_DIR}/")
set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/doxygen")
set(DOXYGEN_INDEX_FILE "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml")
set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/docs/api_reference/doxygen-config.in")
set(DOXYFILE_OUT ${PROJECT_BINARY_DIR}/doxygen-config)

# Configure doxygen
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen command
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
    DEPENDS ${HPP_FILES}
    MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
    COMMENT "Generating doxygen documentation")

# Generate API reference
add_custom_target(doxygen ALL
    DEPENDS ${DOXYGEN_INDEX_FILE}
    COMMENT "Generated API documentation with doxygen" VERBATIM)
add_dependencies(doxygen doc-dirs)

# Install doxygen generated XML files
install(DIRECTORY ${PROJECT_BINARY_DIR}/doxygen/xml
    DESTINATION doxygen
    COMPONENT doxygen)
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Integration-Service-docs doxygen")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
    "eProsima Integration Service doxygen documentation")
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} doxygen)

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
    -D breathe_projects.Integration-Service=${DOXYGEN_OUTPUT_DIR}/xml
    -d "${PROJECT_BINARY_DIR}/doctrees"
    ${SPHINX_SOURCE}
    ${PROJECT_BINARY_DIR}/${INTEGRATION_SERVICE_DOCS_BUILDER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS doxygen
    COMMENT "Generating documentation with Sphinx")

# Install the generated docs
install(DIRECTORY ${PROJECT_BINARY_DIR}/${INTEGRATION_SERVICE_DOCS_BUILDER}
    DESTINATION .
    COMPONENT ${INTEGRATION_SERVICE_DOCS_BUILDER}
    PATTERN ".buildinfo" EXCLUDE)
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Integration-Service-docs ${INTEGRATION_SERVICE_DOCS_BUILDER}")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
    "eProsima Integration Service documetation in ${INTEGRATION_SERVICE_DOCS_BUILDER} format")
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${INTEGRATION_SERVICE_DOCS_BUILDER})
