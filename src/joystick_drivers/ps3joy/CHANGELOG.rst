^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ps3joy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2020-10-12)
-------------------

1.14.0 (2020-07-07)
-------------------
* Fixing linter errors for Noetic. (`#174 <https://github.com/ros-drivers/joystick_drivers/issues/174>`_)
* Make sure to import struct where it is used. (`#162 <https://github.com/ros-drivers/joystick_drivers/issues/162>`_)
* roslint and Generic Clean-Up (`#161 <https://github.com/ros-drivers/joystick_drivers/issues/161>`_)
* Contributors: Chris Lalancette, Joshua Whitley

1.13.0 (2019-06-24)
-------------------
* Merge pull request `#128 <https://github.com/ros-drivers/joystick_drivers/issues/128>`_ from ros-drivers/fix/tab_errors
* Cleaning up Python indentation.
* Merge pull request `#123 <https://github.com/ros-drivers/joystick_drivers/issues/123>`_ from cclauss/modernize-python2-code
* Modernize Python 2 code to get ready for Python 3
* Merge branch 'master' into indigo-devel
* Contributors: Joshua Whitley, Matthew, cclauss

1.12.0 (2018-06-11)
-------------------
* Addressed numerous outstanding PRs.
* Created bluetooth_devices.md
* Created testing guide for ps3joy.
* Create procedure_test.md
* Let ps3joy_node not quit on inactivity-timeout.
* Refine diagnostics message usage in ps3joy_node
* Improve ps3joy_node with rospy.init_node and .is_shutdown
* Remove quit on failed root level check, part one of issue `#53 <https://github.com/ros-drivers/joystick_drivers/issues/53>`_
* Create README
* Changed package xml to format 2
* Contributors: Alenso Labady, Felix Kolbe, Jonathan Bohren, alab288, jprod123

1.11.0 (2017-02-10)
-------------------
* Update dependencies to remove warnings
* Contributors: Mark D Horn

1.10.1 (2015-05-24)
-------------------
* Remove stray architechture_independent flags
* Contributors: Jonathan Bohren, Scott K Logan

1.10.0 (2014-06-26)
-------------------
* First indigo reelase
* Update ps3joy/package.xml URLs with github user ros to ros-drivers
* Prompt for sudo password when required
* Contributors: Felix Kolbe, Jonathan Bohren, dawonn
