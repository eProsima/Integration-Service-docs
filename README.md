# eProsima Integration Service Documentation

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Linux Build Status](http://jenkins.eprosima.com:8080/job/nightly_integration-service-docs_master/badge/icon)](http://jenkins.eprosima.com:8080/job/nightly_integration-service-docs_master)

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

*eProsima Integration-Service* is a tool based on [SOSS](https://soss.docs.eprosima.com/en/latest/index.html)
and [SOSS-DDS](https://github.com/eProsima/SOSS-DDS) that allows intercommunicating any *DDS*-based system with any
other protocol, including other *DDS* systems, integrating them into a larger, more complex system.

*eProsima Integration-Service* can be configured with a YAML text file, through which the user can provide a mapping
between the topics and services on the *DDS*-based middleware and those on the system(s) to which the user
wants to bridge it.

For more information, check out the
[eProsima Integration Service documentation](https://integration-service.docs.eprosima.com/en/latest/).
You can find all the source code on our GitHub repositories: [SOSS](https://github.com/eProsima/soss_v2) and
[SOSS-DDS](https://github.com/eProsima/SOSS-DDS).

1. [Installation Guide](#installation-guide)
1. [Getting Started](#getting-started)
1. [Generating documentation in other formats](#generating-documentation-in-other-formats)
1. [Running documentation tests](#running-documentation-tests)

## Installation Guide

1. In order to build and test the documentation, some dependencies must be installed beforehand:

    ```bash
    sudo apt update
    sudo apt install -y \
        git \
        python3 \
        python3-pip \
        python3-venv \
        python3-sphinxcontrib.spelling \
    ```

1. Clone the repository

    ```bash
    cd ~
    git clone https://github.com/eProsima/Integration-Service-docs integration-service-docs
    ```

1. Create a virtual environment and install python3 dependencies

    ```bash
    cd ~/integration-service-docs
    python3 -m venv integration-service-docs-venv
    source integration-service-docs-venv/bin/activate
    pip3 install -r docs/requirements.txt
    ```

## Getting Started

To generate the documentation in a HTML format for a specific branch of eProsima Integration Service run:

```bash
cd ~/integration-service-docs
source integration-service-docs-venv/bin/activate
make html
```

## Generating documentation in other formats

The documentation can be generated in several formats such as HTML, PDF, LaTex, etc. For a complete list of targets run:

```bash
cd ~/integration-service-docs
make help
```

Once you have selected a format, generate the documentation with:

```bash
cd ~/integration-service-docs
source integration-service-docs-venv/bin/activate
make <output_format>
```

## Running documentation tests

This repository provides a set of tests that verify that:

1. The RST follows the style guidelines
1. There are no spelling errors
1. The HTML is built correctly

Run the tests by:

```bash
cd ~/integration-service-docs
source integration-service-docs-venv/bin/activate
make test
```
## Contributing

If you are interested in making some contributions, either in the form of an issue or a pull request, please refer to
our [Contribution Guidelines](https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md).
