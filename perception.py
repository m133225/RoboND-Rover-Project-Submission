import numpy as np
import cv2
import time

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
    
# Identify pixels above the threshold
def rock_thresh(img, rgb_thresh=(110, 110, 50)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img), M, (img.shape[1], img.shape[0]))
    
    return warped, mask

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    pitch_margin = 0.5
    roll_margin = 0.5
    rover_x = Rover.pos[0]
    rover_y = Rover.pos[1]
    rover_yaw = Rover.yaw
    world_size = 200
    
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    warped_img, mask = perspect_transform(img, source, destination)
    navi_threshed = color_thresh(warped_img)
    obs_threshed = (1 - navi_threshed) * mask[:,:,0]
    rock_threshed = rock_thresh(warped_img)
    navi_coords = navi_threshed.nonzero()
    
    Rover.vision_image[:, :, 0] = mask[:,:,0] * 255 # base warped image
    Rover.vision_image[:, :, 2] = 0                 # clear all blue
    Rover.vision_image[navi_coords[0], navi_coords[1], 0] = 0
    Rover.vision_image[navi_coords[0], navi_coords[1], 2] = 255
    
    x_rover_coords, y_rover_coords = rover_coords(navi_threshed)
    world_x_coords, world_y_coords = pix_to_world(x_rover_coords, y_rover_coords, \
                                                  rover_x, rover_y, rover_yaw, \
                                                  world_size, 2 * dst_size)

    x_obs_coords, y_obs_coords = rover_coords(obs_threshed)
    world_x_obs_coords, world_y_obs_coords = pix_to_world(x_obs_coords, y_obs_coords, \
                                                  rover_x, rover_y, rover_yaw, \
                                                  world_size, 2 * dst_size)   
                                                  
    x_rock_coords, y_rock_coords = rover_coords(rock_threshed)
    world_x_rock_coords, world_y_rock_coords = pix_to_world(x_rock_coords, y_rock_coords, \
                                                  rover_x, rover_y, rover_yaw, \
                                                  world_size, 2 * dst_size)

    dist, angles = to_polar_coords(x_rover_coords, y_rover_coords)
    Rover.nav_dists = dist
    Rover.nav_angles = angles                                     
    
    if len(x_rock_coords) > 0:
        est_rock_x = np.mean(x_rock_coords)
        est_rock_y = np.mean(y_rock_coords)
        Rover.rock_x = int(est_rock_x)
        Rover.rock_y = int(est_rock_y)
    else:
        Rover.rock_x = None
        Rover.rock_y = None
    
    Rover.worldmap[world_y_rock_coords, world_x_rock_coords, 1] = 255 # possible that when the rock is seen, the map is invalid until the point the rock is picked up. so just.. mark everything as a rock :p
    if Rover.pitch > pitch_margin and Rover.pitch < 360 - pitch_margin:
        return Rover
    if Rover.roll > roll_margin and Rover.roll < 360 - roll_margin:
        return Rover
               
    # Update world map only if perspective transformation is valid
    Rover.worldmap[world_y_obs_coords, world_x_obs_coords, 0] = 255
    Rover.worldmap[world_y_coords, world_x_coords, 2] = 255
    
    Rover.worldmap[Rover.worldmap[:,:,2] > 0, 0] = 0

    return Rover