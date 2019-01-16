^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package assisted_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
