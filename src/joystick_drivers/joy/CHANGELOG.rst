^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy
^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.1 (2021-06-07)
-------------------

1.15.0 (2020-10-12)
-------------------
* Added autodetection for force-feedback devices. (`#169 <https://github.com/ros-drivers/joystick_drivers/issues/169>`_)
  * Added autodetection for force-feedback devices.
  * RAII for closedir
* joy: Little fixes for force feedback. (`#167 <https://github.com/ros-drivers/joystick_drivers/issues/167>`_)
  This commit increases the maximum magnitude of the FF effects to double the previous maximum.
* Print out joystick name on initialization. (`#168 <https://github.com/ros-drivers/joystick_drivers/issues/168>`_)
  This helps figuring out what string to give to the `dev_name` parameter.
* Contributors: Martin Pecka

1.14.0 (2020-07-07)
-------------------
* frame_id in the header of the joystick msg (`#166 <https://github.com/ros-drivers/joystick_drivers/issues/166>`_)
* roslint and Generic Clean-Up (`#161 <https://github.com/ros-drivers/joystick_drivers/issues/161>`_)
* Merge pull request `#158 <https://github.com/ros-drivers/joystick_drivers/issues/158>`_ from clalancette/ros1-cleanups
  ROS1 joy cleanups
* Greatly simplify the sticky_buttons support.
* Small fixes to rumble support.
* Use C++ style casts.
* Use empty instead of length.
* joy_def_ff -> joy_dev_ff
* Cleanup header includes.
* Use size_t appropriately.
* NULL -> nullptr everywhere.
* Style cleanup in joy_node.cpp.
* Merge pull request `#154 <https://github.com/ros-drivers/joystick_drivers/issues/154>`_ from zchen24/master
  Minor: moved default to right indent level
* Contributors: Chris Lalancette, Joshua Whitley, Mamoun Gharbi, Zihan Chen

1.13.0 (2019-06-24)
-------------------
* Merge pull request `#120 <https://github.com/ros-drivers/joystick_drivers/issues/120>`_ from furushchev/remap
  add joy_remap and its sample
* Merge pull request `#128 <https://github.com/ros-drivers/joystick_drivers/issues/128>`_ from ros-drivers/fix/tab_errors
  Cleaning up Python indentation.
* Merge pull request `#111 <https://github.com/ros-drivers/joystick_drivers/issues/111>`_ from matt-attack/indigo-devel
  Add Basic Force Feedback Support
* Merge pull request `#126 <https://github.com/ros-drivers/joystick_drivers/issues/126>`_ from clalancette/minor-formatting
* Put brackets around ROS\_* macros.
  In some circumstances they may be defined to empty, so we need
  to have brackets to ensure that they are syntactically valid.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Merge pull request `#122 <https://github.com/ros-drivers/joystick_drivers/issues/122>`_ from lbucklandAS/fix-publish-timestamp
  Add timestamp to all joy messages
* Change error messages and set ps3 as default controller
* Better handle device loss
  Allow for loss and redetection of device with force feedback
* Add basic force feedback over usb
  Addresses `#89 <https://github.com/ros-drivers/joystick_drivers/issues/89>`_
* Contributors: Chris Lalancette, Furushchev, Joshua Whitley, Lucas Buckland, Matthew, Matthew Bries

1.12.0 (2018-06-11)
-------------------
* Update timestamp when using autorepeat_rate
* Added dev_name parameter to select joystick by name
* Added Readme for joy package with description of new device name parameter
* Fixed numerous outstanding PRs.
* Added sticky buttons
* Changed package xml to format 2
* Fixed issue when the joystick data did not got send until changed.
* Changed messaging to better reflect what the script is doing
* Contributors: Dino Hüllmann, Jonathan Bohren, Joshua Whitley, Miklos Marton, Naoki Mizuno, jprod123, psimona

1.11.0 (2017-02-10)
-------------------
* fixed joy/Cmakelists for osx
* Update dependencies to remove warnings
* Contributors: Marynel Vazquez, Mark D Horn

1.10.1 (2015-05-24)
-------------------
* Remove stray architechture_independent flags
* Contributors: Jonathan Bohren, Scott K Logan

1.10.0 (2014-06-26)
-------------------
* First indigo release
