uint64 timestamp    		# time since system start (microseconds)
uint32 tow          		# GPS Time of Week
float32 pos_x       		# Current position X (m)
float32 pos_y       		# Current position Y (m)
float32 pos_z       		# Current position Z (m)
float64 lat         		# Latitude, (degrees)
float64 lon         		# Longitude, (degrees)
float32 alt         		# Altitude AMSL, (meters)
float64 ref_lat     		# Reference point latitude, (degrees)
float64 ref_lon     		# Reference point longitude, (degrees)
float32 ref_alt     		# Reference altitude AMSL, (metres)
float32 head        		# Heading (degree)
float32 roll        		# Roll
float32 pitch       		# Pitch
uint32 status1      		# status #1
uint32 status2      		# status #2 (RESERVED)

uint8 satellites_used     	#  Number of satellites used
uint8 battery       		# Battery
float32 rtk_n       		# Rtk Baseline North coordinate (m)
float32 rtk_e       		# Rtk Baseline East coordinate (m)
float32 rtk_d       		# Rtk Baseline Down coordinate (m)

uint8 nav_state                                 # Currently active mode
uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
uint8 NAVIGATION_STATE_UNUSED3 = 8              # Free slot
uint8 NAVIGATION_STATE_UNUSED = 9               # Free slot
uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
uint8 NAVIGATION_STATE_UNUSED1 = 11             # Free slot
uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
uint8 NAVIGATION_STATE_UNUSED2 = 16             # Free slot
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
uint8 NAVIGATION_STATE_MAX = 23
