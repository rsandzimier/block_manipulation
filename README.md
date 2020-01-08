# Rearranging blocks into densely packed configurations
6.881 Project
Ryan Sandzimier and Filippos Sotiropoulos

In this project we implement task and motion planning for an object rearrangement problem. A collection of cube blocks are arranged in a densely packed configuration by utilising a combination of pick-and-place as well as non-prehensile pushing actions. Utilising a Probabilistic Roadmap planner to choose control actions to relocate single blocks and a recursive algorithm, called Piecewise Linear Non-Monotone Rearrangement Search [1], to plan the sequence of blocks to move, we are able to successfully plan a sequence of actions to rearrange handfuls of blocks from a random original configuration to the densely packed grid. The method is demonstrated on an experimental setup.

[1] A.  Krontiris  and  K.  E.  Bekris,  “Dealing  with  difficult  instances  of object rearrangement,” in Robotics: Science and Systems (RSS) , Rome, Italy,   07/2015   2015.

![Pick and place test](https://github.com/rsandzimier/block_manipulation/blob/master/pick_place_and_push.gif)
