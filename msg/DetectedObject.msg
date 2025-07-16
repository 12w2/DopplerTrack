std_msgs/Header                 header

uint32                          id
string                          label
int8				 label_id
float32                         velocity_r   #velocity_r in polar axis
int8                            class_id
float32                         conf
bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering
geometry_msgs/Pose2D[]          target_trajectory 


############### 2D box
string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
float32                         x           # X 
float32                         y           # Y 
float32                         width       # box Width 
float32                         height      # box Height 
float32                         box_angle       # Angle [0 to 2*PI), allow rotated rects

############### 3D box
float32                         z
float32                         z_height 


########## track
float32                         direction  # Angle [0 to 2*PI), allow rotated rects
float32                         velocity
int32                           life       #track box life
int32                           miss_time
float32[]                       directions_history #  history directions


float32                         last_x           # X 
float32                         last_y           # Y 
float32                         last_velocity_r   #velocity_r in polar axis
float32		      	 loss
