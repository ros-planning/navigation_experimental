^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sbpl_lattice_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2022-08-24)
------------------
* Implement allow_unknown feature (`#60 <https://github.com/ros-planning/navigation_experimental/issues/60>`_)
* debug move_base launch files: Fix warnings, track unknown space
* Contributors: Martin Günther, Martin Peris

0.3.5 (2022-03-07)
------------------
* Add option to publish the sbpl footprint plan (`#58 <https://github.com/ros-planning/navigation_experimental/issues/58>`_)
* Contributors: Lotfi Zeghmi, Martin Günther

0.3.4 (2020-06-19)
------------------
* Initial release into noetic
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

0.3.3 (2019-10-15)
------------------
* Add READMEs
* Contributors: Martin Günther

0.3.2 (2019-01-16)
------------------
* Reinit on map size, footprint and costmap changes
* Add warning when cost_scaling_factor is too large
  Also see `#33 <https://github.com/ros-planning/navigation_experimental/issues/33>`_.
* Fix example config for TF2 (`#30 <https://github.com/ros-planning/navigation_experimental/issues/30>`_)
* sbpl_lattice_planner: Add missing DEPENDS SBPL
* Contributors: Jonathan Meyer, Martin Günther

0.3.1 (2018-09-05)
------------------

0.3.0 (2018-09-04)
------------------
* sbpl_lattice_planner: Update to tf2, add dependency
* Use non deprecated pluginlib macro + headers
* Contributors: Martin Günther

0.2.0 (2018-09-03)
------------------
* Initial release into indigo, kinetic, lunar and melodic
* Contributors: Martin Günther, David V. Lu!!, Dave Hershberger, E. Gil Jones, Eitan Marder-Eppstein, Felix Widmaier, Johannes Meyer, Jon Binney, Vincent Rabaud, Austin Hendrix
