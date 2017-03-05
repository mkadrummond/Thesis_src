# Thesis_src

These are the source files used to program the BeagleBone Black used in my bachelor thesis: 
"Development of a test rig for the automated measurement of the static force characteristic of 
pneumatic muscles".

The folders pru, dts, and can-utils reside in the home directory of the 
BeagleBone Black itself. Of these three only the location of pru/ is important as timer.bin is 
called by a function in deviceControl.cpp.

dts/ and can-utils/ are largely for reference, although dts includes backup dtbo (compiled device 
tree overlays) that can be moved to /lib/firmware should they be needed (for example after reinstalling Debian).
