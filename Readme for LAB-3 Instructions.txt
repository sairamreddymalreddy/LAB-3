The Lab-3 Physics-Based Motion Control System

In this lab I have considered bounding spheres as the objects in the scene.
The input:
1. Data like number of objects(NUM_SPHERES) and dt = 0.016(change dt as per requirement) 
2.mass
3.radius of sphere(radius)
4.restitution
5.floorLevel
6.groundFriction
7.moment of Inertia is calculated on formula I=2/5 m r^2 which is for solid sphere
The above values are taken static can be modified if needed. 
8.others like Linear velocity,Position,Angular velocity,Inital oriention(roll,pitch,yaw) are taken random using rand() func with in defined intervals.
9.for more accurate display of animation I have made use of color component which RGB values that are generated random for each rigidbody
