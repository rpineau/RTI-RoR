//
// RoR commands
//
//  Copyright © 2020 Rodolphe Pineau. All rights reserved.
//
const char ABORT                        = 'a'; // Tell everything to STOP!
const char ETH_RECONFIG                 = 'b'; // reconfigure ethernet
const char CALIBRATE_ROOF               = 'c'; // Calibrate the dome
const char ACCELERATION_ROTATOR         = 'e'; // Get/Set stepper acceleration
const char ETH_MAC_ADDRESS              = 'f'; // get the MAC address.
const char IP_ADDRESS                   = 'j'; // get/set the IP address
const char IP_SUBNET                    = 'p'; // get/set the ip subnet
const char IP_GATEWAY                   = 'u'; // get/set default gateway IP
const char IP_DHCP                      = 'w'; // get/set DHCP mode
const char RESTORE_NET_DEFAULT			= 'x'; //
const char COND_ROOF                 	= 'F'; // Get rain status
const char RESET_ALL					= 'A'; // resets all setting to factory default

const char CLOSE_ROOF                    = 'C'; // Close shutter
const char ROOF_RESTORE_MOTOR_DEFAULT    = 'D'; // Restore default values for motor control.
const char ACCELERATION_ROOF             = 'E'; // Get/Set stepper acceleration
const char VOLTS_ROOF                    = 'K'; // Get volts and set cutoff voltage (close if bellow)
const char STATE_ROOF                    = 'M'; // Get shutter state
const char OPEN_ROOF                     = 'O'; // Open the shutter
const char POSITION_ROOF                 = 'P'; // Get step position
const char SPEED_ROOF                    = 'R'; // Get/Set step rate (speed)
const char STEPSPER_ROOF                 = 'T'; // Get/Set steps per stroke
const char VERSION_ROOF                  = 'V'; // Get version string
const char REVERSED_ROOF                 = 'Y'; // Get/Set stepper reversed status
