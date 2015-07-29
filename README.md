This is a fork of https://github.com/teknotus/depthview/tree/remote_control

I removed all QT specific code and kicked out all the GUI (screenshot, image display) stuff aswell.
main.cpp is a kind of demo for using this stuff.

Note that there are some issues:
- after initializing it takes some time until the camera is ready, hence the sleep command in the the demo
- RGB image is in wrong colors
- image is buffered for 3 frames.
- update data design is messed up. call update data with 1 cv::mat for RGB and with 2 for depth/ir
- camera controls are not accessible as i haven't figured out yet what is QT related and what is not
- there is a bunch of unused and unarranged code in the cameradriver files.

For the IR/Depth Camera to give both images at once you still need the kernel patch from: https://github.com/teknotus/depthview/tree/kernelpatchfmt

