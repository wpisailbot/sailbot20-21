
# Autonomous Systems
This directory houses the alorithms for the A* and point-to-point algorithms

# Algorithms Explaination


## A*

## P2P
The P2P (point to point) algorithm is used to travel between waypoints set by the A* algorithm. It takes the GPS coordinates of a desired destination and data provided by the Airmar and uses them to determine how to adjust the sail and rudders. It is controlled primarily through the getAction function, which is what you will call to update the sail and rudders. It is intended to be called repetedly to ensure the boat stays on course. The code switches among activating 3 states depending on what its state variable is set to. The first state controls the trim tab and recognizes whether or not the destination coordinates fall within the no sail zone of the boat's current location. The second state activates if the destination is within the no sail zone and instructs the rudders to follow a course that will take the boat as far upwind as possible. The third state activates if the destination is not within the no sail zone and instructs the rudders to follow a course that leads straight to the destination.

NOTE: because computer vision and collision avoidance are not yet incorporated, the boat will go directly toward its destination regardles of what is in the way. Once computer vision is added you will need to add code to the P2P algorithm to operate the rudders such that the boat doesn't crash, perhaps even a new state. Additionally, the second state instructs the boat to try to reach a destination in the no sail zone by only tacking once. When this is not possible, such as when the boat is trying to sail up a narrow chanel, you will need to instruct the boat to tack before it collides with anything. This will also require computer vision.
