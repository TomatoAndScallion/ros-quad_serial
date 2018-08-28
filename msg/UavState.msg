#Current autopilot state
#
# mode :
#       usermode
#       velmode
#       posmode
#       altmode
#       manual
#       takeoff
#       landing
#if is_user is true the pc can send order to uav
std_msgs/Header header
bool connected
bool armed
bool is_user
string mode
uint8 system_status
