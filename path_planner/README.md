# Path Planner

This package is for calculating paths with an A* algorithm, both for Cartesian and geographic coordinates. It can build automatically the obstacles map, draw or plot results, and take into account elevation irregularities of the ground.

Developed and used for the projects MULTIDRONE and AERIAL-CORE.

This library can optionally use the following extra software for additional features (all with MIT Licenses):
* "ajnisbet/opentopodata", version 1.5.0. Open Topo Data is a REST API server for your elevation data. Copyright (c) 2020 Andrew Nisbet.
* "nlohmann/json" library, version 3.9.1. JSON parser used to extract the elevation data from Open Topo Data localhost responses. Copyright (c) Copyright (c) 2013-2021 Niels Lohmann.
* "lava/matplotlib-cpp". Plotting library that works by wrapping the popular python plotting library matplotlib. Needs a working python installation. Copyright (c) 2014 Benno Evers.

<!--- TODO: improve explanation, include examples. --->

### Dependencies ###

* [ROS Melodic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [geodesy (from geographic_info)](http://wiki.ros.org/geodesy): `sudo apt install -y ros-${ROS_VERSION}-geodesy`