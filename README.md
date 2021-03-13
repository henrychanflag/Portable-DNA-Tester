# Portable-DNA-Tester
The platform is NUC-131
Release notes:
R0.1: 1st release

R0.2:
- Laser LED will be turned on continuously instead of pulse if set to be ON
- LED flashing is handled by new method in each 0.5ms timer cycle so that the flashing can be in-sync

R0.3:
- There is no laser on/off key in the APPs but Measurement START/STOP key
- Heater will be on by default after power on
- Laser LED will be off upon box open but turned on if box is close and measurement is still in progress

R0.4:
- Tested with the real heating element
- Temp_ok status flag and displayed temp should be averaged over the LED_TIMER_COUNT period to ensure stability of the values

R0.5
- Change the counter variables to be in unit of msec for easy manipulation
- Add heater pwm max and min values upon different range of target temp settings. This can minimize the variation around the target temp.
- Add heater circuit (sensor + heater) check in SW. If the temp is lower than 5degC after heater is on for a period of time
  (TEMP_ERROR_LIMIT), then heater will be turned off and HEATER LED will fast flash.

R0.6
- TEMP_timer is changed to PID_timer as the PID is found to be smaller due to the slow response of the timer per heater pwm
- The basic heart beat is still keeping 50ms.

R0.7
- TEMP_timer is used back to isolate with PID_timer as the average temp should not be the same as PID_timer.
- "current_temp" is measured per TMR0 and will be used in PID and averaging temp according to PID_timer and TEMP_timer
- PID will use the instantaneous "current_temp" measured, not average temp over PID_timer frame.
- The basic heart beat is still keeping at TMR0(50ms) independently.

R0.8
- Change BOD level from 4.4V to 3.7V as the supply may be marginal at 4.7V when considered with cable loss.
- Change temp parameters to int type to avoid overflow.
- Deliver the ADC values to UART by using two bytes to swap the high & low bytes in correct order for user to easily check the values on screen.
- Power failure will fast-flash the POWER LED instead of slow flashing.

R0.9
- Change IO ports per 1st layout
- LASER_LED has only one single output
- Do the same averaging in laser reading as temp averaging to remove the frustrating values

R1.0
- Adding Bluetooth Write instead of UART
- Use agreed protocol to pack the data over BT transmission

R1.1
- Modified the display_temp & display_laser into degC & mV respectively
- Names for temperatures are added with ADC suffix
- VREF is added in the DEFINEs

R1.2
- Changed the UART RX with the Bluetooth RX protocol

R1.3
- Add one more packet type - COMMAND_STATUS to serve for data ACK to mobile

R1.4
- Changes in main() start-up phase: Heater set to PWM=0 first to prevent high current at start-up; wait 15ms before BOD is enabled

R1.5
- Increase the BAUD rate from 9600 to 38400 to save MCU time in UART write

R1.5.1
- Updated heater_status according to the "HEATER ON/OFF" flag from the mobile, not from the "MEASURE ON/OFF" flag

R1.6
- Add temp offset error (temp sensor & real chamber difference); affect calculation of display_temp & user_temp_adc
- Heater status will follow mobile heater_command bit to change other than turning on when measure_command bit is ON

R1.6 -Test
- No temp offset error
- Bluetooth_write every 30sec (set REALTERM with ROW=70, COL=40)for testing

R1.6.1
- Add temp offset error (temp sensor & real chamber difference); affect calculation of display_temp & user_temp_adc
- Updated heater_status according to the "HEATER ON/OFF" flag from the mobile, not from the "MEASURE ON/OFF" flag
- Add flag "rising_temp_reached" to allow :
>> the system to use 100% PWM for the heater in the 1st heater cycle;
>> the system to turn on HEATER LED only if target temp is reached for once, not only when the temp is within tolerance

R1.6.2
- Correct LED flashing and temperature thresholds for PWM max, min values

R1.7
- Pack the data packet into one single packet: COMMAND TYPE = 0; length = 'E' (=14)
- Data sequence is: laserA, laserB, temp, status (packed in 1byte)
- It will be sent in the routine LED_status_update 

R1.7.1
- bug fix on status data array number

R1.7.2-test
- set temp_offset = 0 to check the raw temp adc data
- Move "rising_temp_reached" to check with average_temp, not current_error
- Correct ADC_CLK divider to 7

R1.7.2
- bug fix on status data to handle temp_ok flag which value will be of two bits: 0,1,2
- Change TX_NACK_count from 2 to 5 to avoid BT LED flash too frequently

R1.7.3
- Move "rising_temp_reached" to check with average_temp, not current_error
- Correct ADC_CLK divider to 7

R.1.7.4
- correct rising temp bug, correct TEMP_ERROR_COUNT routine to limit the max value
- correct status bit "laser on" to mobile
- correct fatal bugs in bluetooth_write and generation of data packet
- status byte changed to 2-byte data in bluetooth_write

R1.7.4-test
- temp_offset always = 0 for testing/evaluation of raw temp sensor

R2.0
- Modify for the 6-capsule version and temporary for EV board
- Major impacts:
- UART rate change from 38400 to 115200
- ADC0-5 : corresponding to laser detectors; ADC6 : temp sensor
- Data packet size increase from 18 to 34; length defines changed from character to decimal
- IO are modified to fit the EV board instead of the actual product

R2.1
- Modify the IO to fit the final schematics
- VREF = 3.3V
- Release PC0,1,2,3 as debug LEDs
- Temp offset = 0 for testing
- Disable BOD for large current test

R2.2
- Enable BOD
- Correct the temp sensor detection bug; need to restart a new ADC for each temp sensor measurement
- Add comments about temp_offset calculation

R2.2-T
- remove temp_offset

R2.3
- modify temp_offset equation due to new 6 capsule device

R2.4
- Add laser adjustment factor
