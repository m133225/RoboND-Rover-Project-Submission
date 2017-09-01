## Project: Search and Sample Return

---

### Notebook Analysis

#### Changes
Bulk of the code changes are in the `process_image` function as indicated by the to-dos. There are a couple of modifications to the helper functions, where the perspective transform returns a mask which is used to get the warped terrain visible to the rover. Similar to the `color_thresh` function, a `rock_thresh` function has also been added to identify the yellow rocks pixels.

#### Pixel Identification

The main additions to the `process_image` mostly followed the given to-do cues, and are very similar to the previous exercises. Obstacle identification was done by getting the 0s (instead of the 1s for navigable terrain), and making sure that the pixels are part of the visible terrain before painting it red. Rock identification used the `rock_thresh` to identify possible rocks in the image. All pixels are then converted to world coordinates, and then the world map is painted using the converted pixels. Any navigable terrain identified before takes priority if the same pixel is later identified to be an obstacle.

### Autonomous Navigation and Mapping

#### Perception
Most of the code in `perception_step()` are copied from the completed notebook, with a couple of changes. The world map is only painted if the `pitch` and `roll` values are within certain thresholds, since perspective transform is theoretically only valid on flat surfaces (or non-moving camera rotations). If the surface changes its orientation too drastically from the test image (where the source points were gotten from), it lowers the accuracy of the world map. Of course, the drawback to this is that a significant amount of frames can be invalid, especially when at high velocity or turning, which may cause it to not map some segments. This is mitigated by controlling the speed in `decision_step()`, to ensure that a significant amount of time is spent in valid orientation.

#### Decision

Changes were also made to the `decision_step()` function. The initial large focus in this section was to map most of the navigable terrain in the shortest time with acceptable accuracy. One problem identified from the very beginning was that the rover tends to get stuck IN the rocks, if it is going too quickly, or even on or between the smaller rocks. The way used to deal with this problem was to count the number of navigable pixels in a certain radius range. If there are few rocks in the close distance, assume that it is blocked from moving forward - so rotate to find more navigable paths. If the rover is moving at a significant speed and there are few navigable terrain in the mid distance, start to slow down instead of colliding into the incoming obstacle. This, of course slowed the exploration process down, but had higher reliability. In general, reversal was avoided if possible since it made the rover tracks appear on camera. Since they are of similar color to the obstacles, they may cause the navigable paths identified to be incorrect. Following the cues in the project challenge, a bias was also added to the average angle to allow the rover to prefer moving to the right, and thus travelling along the right wall. The strategy added at the end to pick the rocks was simple: if the rock is in its vision, steer and move slowly towards it. The drawback was possibly skipping a section of the map, and the rover needs to traverse another round before coming back to explore the missed section.

Of course, the above was just the finalized version of submission. There were many trial and errors using other methods such as moving slower if the rover is in invalid orientation for too long, or even halt decision making until a valid orientation is obtained. They either took too long to map the area or didn't solve the problem of getting stuck since they were several factors causing them. (How much to slow down? Use brakes, which causes the rover to jerk more?)

#### Performance
With the finalized version, it is possible to obtain 98% mapping in ~12 minutes, with about 75% fidelity and all rocks located (this could technically be done by just putting a positive value everywhere in the rock map....). Number of rocks collected is usually 5 or 6. The FPS obtained in most of my trials are about 30-40 FPS on 1024x768 resolution. With reasonable assumptions about the starting location, the rover will eventually reach (but not stop at) the starting position just by continuing its wall crawling. There are still occasions where the rover gets stuck, but it should be able to unstuck itself unless it is caught in the unseen holes near rocks.

#### Conclusion and Possible extensions
Of course, other than optimizing the decision making to improve its reliability, the other aim was to return to the starting position and stop there. Even then, further improvements to the code would seem rather hackish and tuned only for the given terrain, which does not provide much learning value.