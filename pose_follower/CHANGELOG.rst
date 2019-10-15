^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pose_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2019-10-15)
------------------
* Add dynamic reconfigure to pose_follower (`#40 <https://github.com/ros-planning/navigation_experimental/issues/40>`_)
  Similar to the other available local planners, this commit adds dynamic reconfigure to pose_follower. In addition to this, the collision_planner parameters (used for detecting illegal trajectory) have been moved to the `PoseFollower/collision_planner` namespace.
  Major ROS API changes:
  * all internal TrajectoryPlannerROS parameters moved into the collision_planner namespace
  * all internal TrajectoryPlannerROS publishers (cost_cloud, global_plan, local_plan) moved into the collision_planner namespace
  * there is still a global_plan topic without namespace, which is now only published by the pose_follower itself and no longer shared with the internal TrajectoryPlannerROS.
* unused publisher removed (`#41 <https://github.com/ros-planning/navigation_experimental/issues/41>`_)
* Add READMEs
* Contributors: Martin Günther, Pavel Shumejko

0.2.1 (2019-01-16)
------------------
* max rotation vel in in-place rotation limited (`#27 <https://github.com/ros-planning/navigation_experimental/issues/27>`_)
* Add visualization of global plan (`#26 <https://github.com/ros-planning/navigation_experimental/issues/26>`_)
* Use non deprecated pluginlib macro + headers
* Contributors: Martin Günther, Pavel, sumejko92

0.2.0 (2018-09-03)
------------------
* Initial release into indigo, kinetic, lunar and melodic
* Contributors: Martin Günther, David V. Lu!!, Dave Hershberger, Howard Cochran, Jon Binney, Kaijen Hsiao, Michael Ferguson, cratliff, Eitan Marder-Eppstein, Wim Meeussen
