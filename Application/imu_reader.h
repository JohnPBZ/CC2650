/******************************************************************************

 @file  simple_peripheral.h

 @brief This file contains the Simple BLE Peripheral sample application
        definitions and prototypes.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************/

#ifndef IMUREADER_H
#define IMUREADER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple BLE Peripheral.
 */
extern void IMUReader_init(void);
extern void IMU_Reader_processEvent(void);
extern void IMU_Show_processEvent(void);
extern void IMU_Reader_processEvent_Loop(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IMUREADER_H */
