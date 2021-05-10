.. _is_core:

Integration Service Core
========================

The :code:`is-core` library defines a set of abstract interfaces and provides some utility classes
that form a plugin-based framework.
A single :code:`integration-service` executable instance can connect `N` middlewares,
where each middleware has a plugin, or *System Handle* associated with it.
The *System Handle* for a middleware is a lightweight wrapper around that middleware (e.g. a *ROS* node or a *WebSocket*
server/client). The :code:`is-core` library provides CMake functions that allow these middleware
*System Handles* to be discovered by the :code:`integration-service` executable at runtime after the *System Handle*
has been installed.
A single :code:`integration-service` instance can route any number of topics or services to/from any number of
middlewares.
Because of this, downstream users can extend *Integration Service* to communicate with any middleware.

This section explains the main aspects of this library, and it is organized as follows:

.. add TOC.


.. TODO:

    What I would put here:

    * A section (w/ eventual subsection and everything that might be needed) explaining the functioning of the core library
      (classes, ..)

    * XTypes description (types inheritance and so on)

    * Useful diagrams based on those that were presented on December IS meetings