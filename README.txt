Centroid Track Program

Searches for red,blue,green,yellow centroids and stores coordination in shared memory. 
Thresholds should be adjusted to account for lighting,camera, and shade of objects

STEPS:

1.Run the code and ensure your camera number is appropriately set

2.You will be prompted in the console to press space to start capturing images

3.Open Imageview.exe to view image output (recommend pinning to task bar)

4.At any time you may press 'enter' in the console to see the following info:

-centroid positions
-colour and label size thresholds 
-time for last image processing loop to complete

5.Press 'x' in the console to terminate session. 
The last frames are saved under rgb1,rgb2,and gscale1
rgb1 is the coloured image after scaling and applying lowpass/highpass filters
rgb2 is the greyscale image after applying thresholding and inverting 

*Most of the loops seem to execute within 0.05s meaning at max velocity of 0.5 m/s we are losing 2.5cm of accuracy. 
Given the dimensions of the sandwich box being ~15cm in width/length, this seems to be acceptable for now  

IMAGE PROCESSING:

The current search algorithm filters label image based on expected target size


SHARED DATA: 

The centroid data is stored with the CentroidData struct and then comitted to shared memory object CentroidSharedMem
It can be accessed with FILE_MAP_READ as per https://learn.microsoft.com/en-us/windows/win32/memory/creating-named-shared-memory

