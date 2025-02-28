Header header

#input
uint64 random_seed 
geometry_msgs/Vector3 velocity_picture
geometry_msgs/Vector3 acceleration_picture
geometry_msgs/Vector3 gravity_picture
geometry_msgs/Vector3 goal_odom
int8 flight_stage

#output
#trajectory statistics
uint64 trajectory_id 
time trajectory_reset_time
geometry_msgs/Transform trajectory_transform

bool  trajectory_found
float64 speed_cost
float64 perception_cost
int64 NumCollisionFree
int64 NumPyramids
int64 NumVelocityChecks
int64 NumCollisionChecks
int64 NumCostChecks 
int64 NumTrajectoriesGenerated

#trajectory coeffs
geometry_msgs/Vector3 coeff0
geometry_msgs/Vector3 coeff1
geometry_msgs/Vector3 coeff2
geometry_msgs/Vector3 coeff3
geometry_msgs/Vector3 coeff4
geometry_msgs/Vector3 coeff5
time duration
