^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package assisted_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2022-08-24)
------------------

0.3.5 (2022-03-07)
------------------

0.3.4 (2020-06-19)
------------------
* Initial release into noetic* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

0.3.3 (2019-10-15)
------------------
* assisted_teleop: Rename library laser_scan_filters
  This fixes a name clash with the package laser_filters.
  Fixes `#42 <https://github.com/ros-planning/navigation_experimental/issues/42>`_ .
* Add READMEs
* Contributors: Martin Günther

0.3.2 (2019-01-16)
------------------
* Fix some includes
* Contributors: Martin Günther

0.3.1 (2018-09-05)
------------------
* assisted_teleop: Don't link against Eigen_LIBRARIES
  That variable is not set. Eigen is a header-only library.
* Contributors: Martin Günther

0.3.0 (2018-09-04)
------------------
* Convert to TF2 + new navigation API (for melodic)
* Use non deprecated pluginlib macro + headers
* Contributors: Martin Günther

0.2.0 (2018-09-03)
------------------
* Initial release into indigo, kinetic, lunar and melodic
* Contributors: Martin Günther, V. David Lu!!, Dave Hershberger, Eitan Marder-Eppstein, Jon Binney, Kaijen Hsiao, Vincent Rabaud, Brian Gerkey, Ken Conley
