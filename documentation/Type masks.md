# Type masks 

*A topic of numerous of bugs within Ardupilot.* 

Type masks specify what the Pixhawk should regulate. When we're sending a PositionTarget via Mavros (on the topic 
"mavros/setpoint_raw/local"), one of the fields in that type is the type mask (http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html). The type mask is a flag where one can say that the Pixhawk should ignore the velocity 
setpoint, the acceleration setpoint etc. In Fluid FSM we specify that we should ignore velocity, acceleration and yaw 
rates, because we only send position setpoints.

This sounds all good right? Well no, because the Pixhawk is quite picky about these type masks. If you change the type 
mask frequently, the drone will respond with flying all over the place *as* it's receiving different position 
setpoints.

TL;DR:

When working with type masks:

* Don't change type mask frequenctly (a couple of times per second).