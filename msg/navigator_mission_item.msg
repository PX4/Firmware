uint64 timestamp                 # time since system start (microseconds)

uint32 instance_count            # Instance count of this mission. Increments monotonically whenever the mission is modified

uint16 sequence_current          # Sequence of the current mission item

uint16 nav_cmd

float32 latitude
float32 longitude

float32 time_inside              # time that the MAV should stay inside the radius before advancing in seconds
float32 acceptance_radius        # default radius in which the mission is accepted as reached in meters
float32 loiter_radius            # loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise
float32 yaw                      # in radians NED -PI..+PI, NAN means don't change yaw
float32 altitude                 # altitude in meters (AMSL)

uint8 frame                      # mission frame
uint8 origin                     # mission item origin (onboard or mavlink)

bool loiter_exit_xtrack          # exit xtrack location: 0 for center of loiter wp, 1 for exit location
bool force_heading               # heading needs to be reached
bool altitude_is_relative        # true if altitude is relative from start point
bool autocontinue                # true if next waypoint should follow after this one
bool vtol_back_transition        # part of the vtol back transition sequence
