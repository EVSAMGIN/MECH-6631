Centroid Search Program

Searches for red,blue,green,yellow centroids and stores location in shared memory

STEPS

-Run the code and ensure your camera number is appropriately set

-You will be prompted to press space to start capturing images

-Open Imageview.exe to view image output (recommend pinning to task bar)

-At any time you may press enter in the console to see the following info:

*centroid positions
*colour thresholds for each filter 
*time for last image processing loop to complete

**Most of the loops seem to execute within 0.05s meaning at max velocity of 0.5 m/s we are losing 2.5cm of accuracy. 
Given the dimensions of the sandwich box being 15cm in width/length, this seems to be acceptable for now  

-The colour thresholds can be adjusted in the console by pressing 'r','g','b','y' for red,green,blue,yellow respectively
-You will then be prompted to choose again for r,g,b or primary/secondary for yellow

-The threshold for the neighbouring pixel check can also be adjusted by entering 'p' into the console (current default is 1 pixel)


IMAGE PROCESSING:
This current version does not do any greyscale processing of the image for reduced complexity 

The current search algorithm filters pixels for desired colours
The threshold will need to be adjusted depending on lighting and shade of objects used. More improvements to come there 
There is a check for neighbouring pixels up/down of the same colour to avoid including one-off pixels in the centroid calculation


SHARED DATA: 

The centroid data is stored with the CentroidData struct and then comitted to shared memory object CentroidSharedMem
It can be accessed with FILE_MAP_READ as per https://learn.microsoft.com/en-us/windows/win32/memory/creating-named-shared-memory

