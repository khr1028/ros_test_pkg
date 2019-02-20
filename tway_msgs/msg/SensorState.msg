########################################
# CONSTANTS
########################################
# Bumper states (states are combined, when multiple bumpers are pressed)
uint8 BUMPER_RIGHT  = 1
uint8 BUMPER_CENTER = 2
uint8 BUMPER_LEFT   = 4

# Button states (only one button can be triggered at a time)
uint8 BUTTON0 = 1
uint8 BUTTON1 = 2
uint8 BUTTON2 = 4

# Motor errors
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Motor torque
uint8 TORQUE_ON  = 1
uint8 TORQUE_OFF = 2

########################################
# Messages
########################################
Header header
#IO Status
uint8  nBumperSts
uint8  nEMSBtnSts
uint8  nPowerOffSts
uint8  nLiftHomeSts
uint8  nLiftPosiSts
uint8  nLiftDownLimitSts
uint8  nLiftUpLimitSts
uint8  nBatteryChargeSts

#Motor Status
bool   bTorqueSts
int32  nLeftEncoder       # (-2,147,483,648 ~ 2,147,483,647)
int32  nRightEncoder      # (-2,147,483,648 ~ 2,147,483,647)
int32  nLeftDistance      # (-2,147,483,648 ~ 2,147,483,647)
int32  nRightDistance     # (-2,147,483,648 ~ 2,147,483,647)
int32  nLeftRPM           # (-2,147,483,648 ~ 2,147,483,647)
int32  nRightRPM          # (-2,147,483,648 ~ 2,147,483,647)
uint16 nMtrSts

#Battery Status(Simple)

