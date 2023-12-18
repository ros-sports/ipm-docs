.. Inverse Perspective Mapping documentation master file, created by
   sphinx-quickstart on Fri Mar  4 19:52:06 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Inverse Perspective Mapping (IPM)
=================================

Inverse Perspective Mapping (IPM) is the process of converting 2D points from images into 3D points
without using depth information, by assuming the points lie on an arbitrary plane.

.. figure:: images/ipm2_crop.png
   :width: 100%
   :align: center
   :alt: IPM applied to project the camera image onto the ground plane in a soccer playing scenario.

   IPM applied to project the camera image as well as some detected objects onto the ground plane in a soccer robot scenario.

This package provides a general implementation of inverse perspective mapping for ROS. 
For a downstream package that uses this package to project balls etc. for soccer playing robots, see :ref:`SoccerIPM`.

The project is hosted on `Github`_ by ROS Sports. **Issues and Pull Requests are welcome!**

To get started look at the :ref:`Tutorial`.

.. toctree::
   :hidden:

   installation
   tutorial
   soccer

.. _Github: https://github.com/ros-sports/ipm
