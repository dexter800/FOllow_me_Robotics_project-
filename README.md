
# Project: Search and Sample Return

[//]: ## (Image References)
[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[image4]: ./calibration_images/map_bw.jpg 
[image5]: ./calibration_images/example_grid2.jpg 
[image6]: ./calibration_images/color_thresh.png
[image7]: ./calibration_images/rock_pix.png 
[image8]: ./calibration_images/coordinate_transformation
[image9]: ./misc/Robo_simulator_mode.png 
[image10]:./misc/autonomous_mode.jpg
[image11]:/misc/applied_threshold.png


## Goal for this project 

In this project, I will write the code to develop the autonomous robot which can map and search the sample rock at simulated environment of Mars. 
This project is modelled after the NASA sample return challenge (https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and it will give you firsthand experience with the three essential elements of robotics, which are perception, decision making and actuation.  We will carry out this project in a simulator environment built with the Unity game engine.  

## Setting the simulator and running the autonomous mode
![image9]
I used Windows simulator Build, as I use windows computer. Make sure you have the environment setup. If you haven't, click [here](https://github.com/ryan-keenan/RoboND-Python-Starterkit).
Navigate to the `code` folder, activate your environment, and run:

'''
python drive_rover.py
'''

 Then, launched the simulator with high resolution (1024*768) and fantastic graphics quality.

## Notebook Analysis 

### 1. Perspective Transform 

I have modified the ‘Perspect_transform’ function to turn a Rover image navigable ground.


''' def perspect_transform(img, src, dst):
          M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
   mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask
'''

'''
dst_size = 5 
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
warped, mask = perspect_transform(grid_img, source, destination) 
''' 

For example 
Before   ![image5]
After applied warped and mask at Perspective Transfer
![image 11]

To Find colour threshold for rocks 

The written code to find rock pixel as below. 

'''   
def find_rocks(img, levels=(110, 110, 50)):
    rockpix = ((img[:,:,0] > levels[0]) \
    & (img[:,:,1] > levels[1]) \
    & (img[:,:,2] < levels[2]))
    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1 
    return color_select
'''

for example:
![rock_imge][image 3]
![rock_pix][image 7]

### 2. Process Image


Here, process_image(img) function is defined which perform to identify navigable terrain, obstacles and rocks sample into the world map. The destination value is defined in perspective transfer, here I will have modified some of the value as below. 

To Apply color threshold to identify navigable terrain/obstacles/ rock samples 

'''
threshed = color_thresh(warped)
obs_map = np.absolute(np.float32(threshed) - 1) * mask 
xpix, ypix = rover_coords(threshed)
'''

To Convert map image pixel values to rover-centric coords

'''
xpix, ypix = rover_coords(threshed)
'''

To Convert rover-centric pixel values to world coordinates

''' 
world_size = Rover.worldmap.shape[0]
scale = 2 *dst_size
obsxpix, obsypix = rover_coords(obs_map)
obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
'''

To Convert rover-centric pixel positions to polar coordinates

'''
dist, angles = to_polar_coords(xpix,ypix)
'''

## Autonomous Navigation and Mapping

### 1. Perception Step

This step is similar as  `process_image`  method described above in the "Notebook Analysis - Mapping" section. Here,  I have made 4 modifications. 

I added the function called found_obstacle_ahead to avoid  big rocks when it come on path of Rover.

'''
 def found_obstacle_ahead(Rover, range = 15, bearing=0):
	idx_in_front = np.where(np.abs(Rover.obs_angles - bearing) < deg2rad(15) \
		& (rover.obs_dists< range))[0]
	return len(idx_in_front) >5 
	''' 

I update the world map when Rover has a normal view, by updating the world map. To reduce fidelity of Rover I introduced roll and pitch angle smaller than `0.7` or bigger than `359.5`, which help lot to improve the fidelity of rover. 

'''
Yaw,roll, pitch =  Rover.yaw, Rover.roll, Rover.pitch
if roll <= 0.7 or roll >= 359.5:
        if pitch <= 0.7 or pitch >= 359.5:
 Rover.worldmap[y_world, x_world, 2] += 255
    Rover.worldmap[obs_y_world, obs_x_world, 0] +=255
 Rover.worldmap[rock_ycen, rock_xcen,1] += 255 
 '''

I update a vision image in RGB colour red, blue, yellow indicates obstacles, navigable terrain and rocks respectively. 

''''
Rover.vision_image[:,:,2] = threshed * 255
Rover.vision_image[:,:,0] = obs_map * 255
Rover.vision_image[:,:,1] = rock_map * 255
'''

To find the rocks in world map I define the function and update coordinate. 

'''
rock_map = find_rocks(warped, levels=(110,110,50))
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0],
                                                     Rover.pos[1], Rover.yaw, world_size,scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        '''
### 2.   Decision Step

To control the steering angle of rover or another word to follow the rover left side of wall, I add +5-degree angle at function Rover. Steer in decision.py. it did help Rover to dive in path without repetition. Set steering to average angle clipped to the range +/- 15
'''
Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) + 5, -15, 15)
'''


I didn’t complete picking sample rock form the map, so I left rest of the other code as it was before. 


## Lunching Autonomous Mode, Experience and Result, and Potential Improvement 

The  simulator was launched with high resolution (1024*768) and fantastic graphics quality, while Rover drove at autonomous mode, I need to switch once to manual mode to get Rover follow right path.
Rover mapped 88% of the world mapped on average 65% fidelity  at 390 seconds.

![Autonomous Result][image11]

Some potential improvements include but not limited to:

1. I could improve, writing better code to follow the path without repeating same path again. 
2. To avoid the rough surface, small other rock which make rover stuck or drive slow, it can improve rover to drive in the middle of path.
3. More clever ways to get unstuck in different situations.
4. To finding the solution to pick sample rocks and return home to Rover. 
5. We can rewrite descision.py file to make rover to do better decision. 


## my rover_drive video can be watch by clicking following YouTube link, https://www.youtube.com/watch?v=uPyv6F5I33M&feature=youtu.be
