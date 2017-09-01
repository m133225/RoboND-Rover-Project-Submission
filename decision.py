import numpy as np
import random
import time


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    bias = -12
    radius_considered = 35

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    if Rover.picking_up:
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = Rover.brake_set
        Rover.rock_x = None
        Rover.rock_y = None
    elif Rover.near_sample and Rover.vel != 0: # Just attempt to slow down and pick it up if we happen to be near it
        Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = Rover.brake_set
    elif Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("Picking rock up and setting mode to forward")
        Rover.send_pickup = True
        Rover.mode = 'forward'
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        considered_angles = Rover.nav_angles[Rover.nav_dists < radius_considered] * 180/np.pi
        if len(considered_angles) ==  0:
            average_angle = 0
        else:
            average_angle = np.mean(considered_angles)
        if Rover.mode == 'forward': 
            if Rover.rock_x != None:
                print("Rock found, changing mode to go to rock")
                Rover.mode = 'gotorock'
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                numberOfMidNavigablePathOnRight = sum((Rover.nav_dists > 15) & (Rover.nav_dists < 25) & (Rover.nav_angles * 180 / np.pi < 15))            
                numberOfMidNavigablePath = sum((Rover.nav_dists > 15) & (Rover.nav_dists < 25) & (Rover.nav_angles * 180 / np.pi < 15) & (Rover.nav_angles * 180 / np.pi > -15))            
                numberOfNearNavigablePath = sum(Rover.nav_dists < 8)
                if numberOfNearNavigablePath < 13 or numberOfMidNavigablePathOnRight < 80:
                    if Rover.vel > 0:
                        print("Obstacle? Braking")
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0 
                    else:
                        print("Obstacle? Turning to find more navigable paths.")
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        Rover.steer = 15 # Could be more clever here about which way to turnvigable terrain looks good 
                # and velocity is below max, then throttle 
                elif Rover.vel > 1 and numberOfMidNavigablePath < 80:
                    print("Approaching obstacle too quickly. Slowing down")
                    Rover.throttle = -Rover.throttle_set * Rover.vel
                    Rover.steer = 0
                    Rover.brake = 0#
                elif Rover.vel < 0:
                    print("Stop going backwards...")
                    # Set throttle value to throttle setting
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                elif Rover.vel < Rover.max_vel  :
                    print("Accelerating...")
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    
                    Rover.steer = np.clip(average_angle + bias, -15, 15)
                else: # Else coast
                    print("Passive...")
                    Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(average_angle + bias, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    print("Insufficient navigable path. Changing to stop mode.")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                print("Braking...")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                
                    print("Turning to find more navigable paths.")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    print("Navigable path found. Changing to forward mode")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(average_angle + bias, -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'gotorock':
            print("Moving to rock")
            if Rover.rock_x == None:
                Rover.mode = 'forward'
            else:
                rock_dist = np.sqrt(Rover.rock_x**2 + Rover.rock_y**2)
                steer = np.arctan2(Rover.rock_y, Rover.rock_x)*180/np.pi
                Rover.steer = np.clip(np.arctan2(Rover.rock_y, Rover.rock_x)*180/np.pi, -15, 15)
                print("Steer value:", steer)
                Rover.brake = 0
                if rock_dist > 7:
                    if Rover.vel < 0.5:
                        Rover.throttle = Rover.throttle_set
                    else:
                        Rover.throttle = -Rover.throttle_set
                else:
                    Rover.throttle = 0
                
            return Rover
                
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print("Blindly accelerating...")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    return Rover

