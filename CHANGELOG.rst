^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rig_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* create config directory if it doesnt exist (`#40 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/40>`_)
* Persist window size via .ini file (`#36 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/36>`_)
* Replace linear node list with tree representation (`#34 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/34>`_)
* Fixes for default parameters (`#33 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/33>`_)
* Contributors: Dominik, Jonas Otto

1.4.0 (2023-12-27)
------------------
* Add dependency on ament_index_cpp to fix build on rolling (`#29 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/29>`_)
* Place imgui.ini within users home directory (`#28 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/28>`_)
* improved input handling of numeric values (`#27 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/27>`_)
* Contributors: Chris Lalancette, Dominik

1.3.2 (2023-09-01)
------------------
* add dominik as maintainer
* Contributors: Jonas Otto

1.3.1 (2023-09-01)
------------------
* remove git version information in info window (`#24 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/24>`_)
* Contributors: Jonas Otto

1.3.0 (2023-08-30)
------------------
* Only list nodes providing the parameter service (`#23 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/23>`_)
* Cleanup (`#22 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/22>`_)
* Info window (`#21 <https://github.com/teamspatzenhirn/rig_reconfigure/issues/21>`_)
* Contributors: Dominik

1.2.0 (2023-08-11)
------------------
* Improvements for handling string parameters (sending update only once editing is complete)
* Add window icon
* Add desktop file
* Contributors: Dominik, Jonas Otto

1.1.0 (2023-04-24)
------------------
* Accept more parameter path separators
* allow scientific notation for float-parameters
* Manual FPS limiting for VNC sessions
* add support for string parameters
* allow clicking on node text for folding/unfolding
* added buttons for expanding / collapsing all parameter nodes
* sort node names in alphabetical order
* periodic node refreshing + warning about died nodes
* improved sizing of the list box
* Contributors: Dominik, Jonas Otto, Scott K Logan

1.0.0 (2023-02-25)
------------------
* initial release
