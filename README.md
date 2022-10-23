# PicameraViewerForAstro
Front end of Raspberry pi HQ camera and controls live view image and motorized equatorial mounts

[Due to my bits and pieces coding, there are many untidy way arounds in the code. I will sort them out gradually.]


Developed on Raspberry Pi 4B 2GB and HQ camera

Required:
python 3, opencv, dcraw with raspberry pi camera raw process (https://github.com/6by9/dcraw)
temp directory in the same place of the .py resides.
for repeating writing image file to the temp directory, it is better to have it as a ramdrive.

Flow:
Frame grab in calling raspistill command
Store jpg file with raw data attached.
Call raspistill again for next frame.
Meanwhile, raw conversion starts in calling dcraw.
16bit tiff file is loaded then image process will be applied.
Raw conversion takes another jpg file and new frame will be captured.

The scheme of a ring buffer with two jpg files is realized.

16bit image data is automatically stacked.
You can track a single star as a guide star.
In the zoom window an isolated star should be detected as a white circle/blob.
Any pixel shift of the guide star in the zoom window will be compensated in image stacking when track mode is enabled.
After predetemined number of frames are stacked, 32bit floating point tiff file will be stored.

By repeating the procedure above, deep sky object can be observed on the screen, as well as saved into a 32bit tiff image.

It also provides GPIO signal controls for stepping motors for equatorial mounts. The signals are 5V high-low so should be connected directly to motor driver circuit board.

Images taken with the software is complied in videos and uploaded in https://www.youtube.com/channel/UCxLBTU-MXtfclGAINipCrsw



Known issues:
It crashes when the exposure time is short. Annoying for planetary photography.
Compensation direction in declination might be opposite. It should be changeable while running the software.



