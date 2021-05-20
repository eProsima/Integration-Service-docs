<a href="https://integration-service.docs.eprosima.com/"><img src="https://github.com/eProsima/Integration-Service/blob/main/docs/images/logo.png?raw=true" hspace="8" vspace="2" height="100" ></a>

# Documentation

[![Integration Service CI Status](https://github.com/eProsima/Integration-Service-docs/actions/workflows/ci.yml/badge.svg)](https://github.com/eProsima/Integration-Service-docs/actions)


[*eProsima Integration Service*](https://github.com/eProsima/Integration-Service) is a Linux tool that enables communication among an arbitrary number of protocols that speak different languages.

If one has a number of complex systems and wills to combine them to create a larger, even more complex system, Integration Service can act as an intermediate message-passing tool that, by speaking a common language, centralizes and mediates the integration.

Integration Service is configured by means of a YAML text file, through which the user can provide a mapping between the topics and services handled by the middlewares of the systems involved.

## Installation Guide

To generate the *eProsima Integration Service* documentation you need to install some dependencies:

```bash
    sudo apt update
    sudo apt install -y git python3 python3-pip python3-sphinxcontrib.spelling doxygen
```

Once the requirements are fulfilled, you can generate the documentation with `colcon` following the next steps:

1. Clone the repository:

    ```bash
        mkdir ~/is_docs_ws
        cd ~/is_docs_ws
        git clone https://github.com/eProsima/Integration-Service-docs src/integration-service-docs
    ```

2. Install the `python3` dependencies:

    ```bash
        pip3 install -r src/integration-service-docs/docs/requirements.txt
    ```

3. Use the `repos` file to download *Integration Service* and all the *System Handles*:

    ```bash
        vcs import src < src/integration-service-docs/integration-service-docs.repos
    ```

4. Use `colcon` to compile and generate the documentation:

    ```bash
        colcon build --cmake-args -DBUILD_LIBRARY=OFF -DBUILD_API_REFERENCE=ON
    ```

It is recommended to use `colcon` to generate the documentation, but you can also create it using `sphinx` by following the next instructions:

```bash
    git clone https://github.com/eProsima/Integration-Service-docs integration-service-docs
    mkdir integration-service-docs/build
    cd integration-service-docs/build
    sphinx-build -b html -d build/doctrees docs build/html
```


## Read the Docs

The official documentation for the *eProsima Integration Service* can be accessed on [Read the Docs](https://integration-service.docs.eprosima.com/en/latest/).

## License

This repository is open-sourced under the *Apache-2.0* license. See the [LICENSE](LICENSE) file for more details.

## Getting help

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.
