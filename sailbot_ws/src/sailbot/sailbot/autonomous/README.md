
# Autonomous Systems
This directory houses the alorithms for the A* and point-to-point algorithms

# Algorithms Explaination


## A*

## P2P
The P2P (point to point) algorithm is used to travel between waypoints set by the A* algorithm. It takes the GPS coordinates of a desired destination and data provided by the Airmar and uses them to determine how to adjust the sail and rudders. It is called repetedly to ensure the sail and rudders are set correctly even as wind conditions change. The code switches among 3 states depending on what its state variable is set to. The first state controls the trim tab and recognizes whether or not the destination coordinates fall within the no sail zone of the boat's current location. The second state activates if the destination is within the no sail zone and instructs the rudders to follow a course that will take the boat as far upwind as possible.
