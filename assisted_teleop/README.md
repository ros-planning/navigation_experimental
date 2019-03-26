assisted_teleop
===============

The assisted_teleop node subscribes to a desired trajectory topic
(geometry_msgs/Twist) and uses TrajectoryPlannerROS to find a valid trajectory
close to the desired trajectory before republishing. Useful for filtering
teleop commands while avoiding obstacles. This package also contains
LaserScanMaxRangeFilter, which is a LaserScan filter plugin that takes max
range values in a scan and turns them into valid values that are slightly less
than max range.
