^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stag_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2021-03-19)
------------------
* Added message_runtime dependency
* Added message_generation dependency
* Added custom messages and publishing results as an array
* Update README.md
* Added ros buildfarm status
* Contributors: Brennan Cain, MikeK4y

0.3.7 (2020-12-14)
------------------
* increased cmake version
* removed reference to nodelet implementation in runnable. fixed nodelet namespacing warning
* removed old tag configuration
* added libraryhd exception handling to nodelet
* Solved pose overwrite for multiple markers
* added exception checking for the HD for stag
* Fixed single launch file
* bumped melodic branch to 0.2.0
* added URLS to package xml
* Contributors: Brennan Cain, Mike Kalaitzakis, MikeK4y

0.3.6 (2020-12-01)
------------------

0.3.5 (2020-12-01)
------------------
* Merge branch 'melodic-devel' into noetic-devel
* removed installation of bags directory, requires downloading bags first which is bad on the build farm
* added checking that the output folder is not empty, bad to save to root (if someone wants that, that's their problem)
* removed dependence on opencv4.2, should now work with 3.2 and 4.2
* Updated READMe
* Fixed download_examples.launch
* Fixed download_examples.launch
* Contributors: Brennan Cain, MikeK4y

0.3.4 (2020-11-17)
------------------
* Merge branch 'melodic-devel' into noetic-devel
* removed bags and added download script
* Contributors: Brennan Cain

0.3.3 (2020-11-16)
------------------
* Merge branch 'melodic-devel' into noetic-devel
* removed dependence on swri_nodelet
* Contributors: Brennan Cain

0.3.2 (2020-11-13)
------------------
* Updated changelog
* changed version to 0.3.1
* Contributors: Brennan Cain

* changed version to 0.3.1
* Contributors: Brennan Cain

0.0.1 (2020-11-13)
------------------
* added swri_nodelet to package.xml
* Merge remote-tracking branch 'origin/noetic-devel' into noetic-devel
* Changed files to match Yaml changes
* Looks like it's working. Do more tests
* Changed files to match Yaml changes
* renamed for hopefully better quality and changed launch files
* yaml loading successful on single tag
* Looks like it's working. Do more tests
* Changed files to match Yaml changes
* beginning move, invalid return from parameter server
* Looks like it's working. Do more tests
* Changed files to match Yaml changes
* Looks like it's working. Do more tests
* Changed files to match Yaml changes
* added yaml config files
* renamed for hopefully better quality and changed launch files
* yaml loading successful on single tag
* Looks like it's working. Do more tests
* Changed files to match Yaml changes
* beginning move, invalid return from parameter server
* Added function to check if points are coplanar
* Looks like it's working. Do more tests
* Found the ED functions used by STag and commented out the rest to start updating the ones we need
* Moved to release and added installation
* Changed files to match Yaml changes
* Looks like it's working. Do more tests
* removed lingering references to json and bundle generation
* removed lingering references to json and bundle generation
* Merge remote-tracking branch 'origin/noetic-devel' into noetic-devel
  # Conflicts:
  #	launch/stagNode_single.launch
  #	launch/stagNodelet_single.launch
* Changed files to match Yaml changes
* beginning move, invalid return from parameter server
* Added function to check if points are coplanar
* Looks like it's working. Do more tests
* Found the ED functions used by STag and commented out the rest to start updating the ones we need
* Updated README
* Fixed config files
* Changed files to match Yaml changes
* Changed files to match Yaml changes
* removed bundle_gen config (unused)
* removed old json files
* updated launch files for yaml
* added yaml config files
* renamed for hopefully better quality and changed launch files
* added change to nodelet
* single node successful
* yaml loading successful on single tag
* beginning move, invalid return from parameter server
* removed bundle_generator. was unimplemented
* removed bundle_gen config (unused)
* removed old json files
* updated launch files for yaml
* added yaml config files
* renamed for hopefully better quality and changed launch files
* added change to nodelet
* single node successful
* yaml loading successful on single tag
* Added function to check if points are coplanar
* beginning move, invalid return from parameter server
* removed bundle_generator. was unimplemented
* Looks like it's working. Do more tests
* Found the ED functions used by STag and commented out the rest to start updating the ones we need
* Changed the configuration. Moved publishing to common
* Updated README for git-lfs installation for earlier Ubuntu versions
* beginning work on bundle generator
* beginning work on bundle generator
* added nodelet documentation
* added additional nodelet launch files
* Merge remote-tracking branch 'origin/master'
* adding nodelet fixes and launch files
* fixed git lfs documentation
* added documentation
* removed message generation
* pathing is all local and examples should work out of the box
* added bags into git lfs
* renamed launch and config files to provide multiple examples
* mirroring node and nodelet
* fixed clang-format file
* added cmake default directory to gitignore
* Removed imaes and scripts folders
* Removed comments from package.xml
* Removed old STag messages
* Remove duplicate markers now uses the projective distortion. Reduced quad duplicates
* Fixed typo
* Update README.md
* Update README.md
* Update README.md
* Added more info in the README
* Cleaned up code. Added Instrunment inside DEBUG
* Used clang-format on our code
* Changed config files to meters
* Fixed async. Results look ok now
* Node is up to speed with nodelet
* Bug fixes. Nodelet works but pose results are wrong
* Bug fixes
* added local file to gitignore
* removed unnecessary depend, good for catkin build
* added a lil documentation
* add license, renamed tag_json_loader
* Added bundle config files
* Fixed some warnings
* added utility header
* reorganized to reflect tighter coupling
* removed build from repo
* Restructuring
* fixed package.xml
* added async, moved to double
* added bundle support, removed markers in  nodelet
* added tf w/ tf added to bag, instrumentation
* added image methods, move to swri nodelet
* added nodelet, moved stag to a library
* added namespacing to node
* formatted files with clang-format
* added clang format
* added bag launch file
* added rviz configuration
* added gitignore
* bug fix
* Covid19 quarantine push
* Removed duplicate markers
* ROS node working and publish marker pose
* Added msgs cfg launch files and other stuff
* First ROS node test
* Added CMake file and changed a few parts to run with OCV 4 on Linux
* Merge pull request `#7 <https://github.com/usrl-uofsc/stag_ros/issues/7>`_ from bbenligiray/add-license-1
  Create LICENSE
* Create LICENSE
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Added ED
* refactored all
* added paper link
* initial commit
* Create README.md
* Contributors: Brennan Cain, Burak Benligiray, MikeK4y, bbenligiray
