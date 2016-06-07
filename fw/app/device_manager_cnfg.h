/* 
 *  Copyright (c) 2014 Robin Callender. All Rights Reserved.
 */
#ifndef DEVICE_MANAGER_CNFG_H__
#define DEVICE_MANAGER_CNFG_H__

/*
 * device_manager_inst Device Manager Instances
 */

/*
 *  Maximum applications that Device Manager can support.
 *
 *  Maximum application that the Device Manager can support.
 *          Currently only one application can be supported.
 *          Minimum value : 1
 *          Maximum value : 1
 *          Dependencies  : None.
 */
#define DEVICE_MANAGER_MAX_APPLICATIONS  1

/*
 *  Maximum connections that Device Manager should simultaneously manage.
 *
 *  Maximum connections that Device Manager should simultaneously manage.
 *          Minimum value : 1
 *          Maximum value : Maximum links supported by SoftDevice.
 *          Dependencies  : None.
 */
#define DEVICE_MANAGER_MAX_CONNECTIONS   1


/*
 *  Maximum bonds that Device Manager should manage.
 *
 *  Maximum bonds that Device Manager should manage.
 *          Minimum value : 1
 *          Maximum value : 254.
 *          Dependencies  : None.
 *
 *  NOTE: In case of GAP Peripheral role, the Device Manager will accept 
 *        bonding procedure requests from peers even if this limit is reached, 
 *        but bonding information will not be stored. 
 *        In such cases, application will be notified with DM_DEVICE_CONTEXT_FULL
 *        as event result at the completion of the security procedure.
 */
#define DEVICE_MANAGER_MAX_BONDS         7


/*
 *  Maximum Characteristic Client Descriptors used for GATT Server.
 *
 *  Maximum Characteristic Client Descriptors used for GATT Server.
 *          Minimum value : 1
 *          Maximum value : 254.
 *          Dependencies  : None. 
 */
#define DM_GATT_CCCD_COUNT               4


/*
 *  Size of application context.
 *
 *  Size of application context that Device Manager should manage for each bonded device.
 *          Size has to be a multiple of word size.
 *          Minimum value : 4.
 *          Maximum value : 254. 
 *          Dependencies  : Needed only if Application Context saving is used 
 *                          by the application.
 *
 *  NOTE: If set to zero, its an indication that application context is not 
 *        required to be managed by the module.
 */
#define DEVICE_MANAGER_APP_CONTEXT_SIZE    0


#endif // DEVICE_MANAGER_CNFG_H__

