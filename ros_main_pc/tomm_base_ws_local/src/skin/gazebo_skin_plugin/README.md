# gazebo_skin_plugin

Plugin to simulate skin in gazebo

Currently only proximity information published as pointcloud

### Issues
- The collision shapes of the robot are sometimes bigger than the visual shape, causing the sensor to detect itself
- The MultiRayCastShape is created via a PhysicsFactory inside of Gazbeo, so I can not modify its insides. Unfortunately I can't ask for the collision object of a ray, only its distance... 
- Collision forces sill are missing
- Accelerometer is missing