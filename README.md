navigation_experimental
-----------------------

A collection of navigation plugins and tools: Various recovery behaviors,
local and global planner plugins for move_base, a teleop filter for
obstacle avoidance, and a simple control-based move_base replacement.

The most useful package in this repo is
[sbpl_lattice_planner](sbpl_lattice_planner), a global planner plugin
for move_base that wraps the SBPL library to produce kinematically feasible
paths.

Each package in this repo has its own README file:

* [assisted_teleop](assisted_teleop)
* [goal_passer](goal_passer)
* [pose_base_controller](pose_base_controller)
* [pose_follower](pose_follower)
* [sbpl_lattice_planner](sbpl_lattice_planner)
* [sbpl_recovery](sbpl_recovery)
* [twist_recovery](twist_recovery)

ROS wiki page: http://wiki.ros.org/navigation_experimental
