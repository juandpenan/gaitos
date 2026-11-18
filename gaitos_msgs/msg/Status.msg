# State of a gaitos robot, published at publish_rate
std_msgs/Header header
string mode                  # active operation mode
string controller            # controller used by the mode
bool estimate_valid          # the state estimate can be used
bool hardware_ok             # last read and write succeeded
bool dry_run                 # the hardware is never written
string[] legs                # leg names (model order)
bool[] contacts              # contact of each foot
float64[] foot_forces        # force of each foot [N or sensor units]
string last_safety_event     # last automatic transition to the safe mode
