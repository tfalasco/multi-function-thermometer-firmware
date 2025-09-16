/*
 * constants.h
 *
 *  Created on: Feb 3, 2017
 *      Author: TFalasco
 *
 *  Modified for 11-001321 Rev3 8/9/2017 PMG..
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// LED values
#define ledMASK			0x7000
#define ledWHITE		0x0000
#define ledCYAN			0x1000
#define ledYELLOW		0x2000
#define ledGREEN		0x3000
#define ledMAGENTA		0x4000
#define ledBLUE			0x5000
#define ledRED			0x6000
#define ledOFF			0x7000

// VReg2.7
#define regMASK			0x0020
#define regON			0x0020
#define regOFF			0x0000

//// VMEM2.7						// The circuit was changed to eliminate the VMEM CTRL; now wire directly to 2.7VREG.. PMG
//#define memMASK			0x0001
//#define memON			0x0001	//turn on eeprom array 2.7V supply.. PMG
//#define memOFF			0x0000  //turn off eeprom array 2.7V supply.. PMG

// VDS2482
#define ds2482MASK			0x1000
#define ds2482ON			0x1000
#define ds2482OFF			0x0000

// VMeasureTC2.7
#define tcMASK			0x0100
#define tcON			0x0100	//turn on TC measurement ckt 2.7V supply.. PMG
#define tcOFF			0x0000  //turn off TC measurement ckt 2.7V supply.. PMG

// VMeasureIR2.7
#define irMASK			0x0100
#define irON			0x0100
#define irOFF			0x0000

// VLaser
#define lasMASK			0x0080
#define lasON			0x0080
#define lasOFF			0x0000

// VBuzzer
#define buzMASK			0x0200
#define buzON			0x0200
#define buzOFF			0x0000

// Buzzer Profiles
#define bpNONE				0x00
#define bpQUICK_BEEP		0x01
#define bpDOUBLE_BEEP		0x02
#define bpHIGH_LOW			0x03
#define bpLOW_HIGH			0x04
#define bpCHIP				0x05
#define bpLONG_BEEP			0x06
#define bpMAX_PROFILE		bpLONG_BEEP
#define bpCONTINUOUS_BEEP	0xFF		// Intentionally above max profile

// Timer Handles and related
#define thADVERTISEMENT		0x01
#define thAUTO_SLEEP		0x02
#define thSLEEP_TIME		0x03
#define thMEASUREMENT		0x04
#define thWAIT_FOR_VMEAS_TC	0x05
#define thWAIT_FOR_TC		0x06
#define thSETTLE_FOR_TM		0x07
#define thWAIT_FOR_TM		0x08
#define thCALC_TEMP			0x09
#define thSLEEP_FLASH		0x0A
#define thDISPLAY			0x0B
#define thLASER				0x0C
#define thREAD_CAL_VALUES	0x0D
#define thWAIT_FOR_VMEAS_IR	0x0E
#define thWAIT_TO_SET_EMISS	0x0F
#define thWAIT_TO_GET_EMISS	0x10
#define thHOLD_TO_SCROLL_S2	0x11
#define thWAIT_TO_RETRY_EMS	0x12
#define thFAST_READ			0x13
#define thBOOT_BEEP_START	0x14
#define thBOOT_BEEP_STOP	0x15
#define thRITS_FLASH_ON		0x16
#define thRITS_FLASH_OFF	0x17
//#define thSHOW_MEM_CLEAR	0x18
#define thENABLE_INTRPTS	0x19
#define thMIN_HANDLE		thADVERTISEMENT
#define thMAX_HANDLE		thBOOT_BEEP_STOP
#define thREENABLE_INTRPTS	0x80				// Intentionally above thMAX_HANDLE
#define thFLASH_OFF			0x81				// Intentionally above thMAX_HANDLE
#define thSWITCH			0x82				// Intentionally above thMAX_HANDLE
#define thBUZZER_STEP_1		0x83				// Intentionally above thMAX_HANDLE
#define thBUZZER_STEP_2		0x84				// Intentionally above thMAX_HANDLE
#define thBUZZER_STEP_3		0x85				// Intentionally above thMAX_HANDLE
#define thBUZZER_STEP_4		0x86				// Intentionally above thMAX_HANDLE
#define thBUZZER_STEP_5		0x87				// Intentionally above thMAX_HANDLE
#define thUPDATE_BATTERY	0x88				// Intentionally above thMAX_HANDLE
#define thSHUTDOWN_GPIO		0x89				// Intentionally above thMAX_HANDLE

#define tmMINUTE 			1966080
#define tmSECOND			32768
#define tmQUARTERSECOND		8192
#define tmTENTHSECOND		3277
#define tm75MILLISECONDS	2457
#define tmFLASH				2048
#define tm60HZ				546
#define tmFIFTEENMS			492
#define tmTENMILLISEC		328					// This is actually the minimum value allowed.

// Persistant Storage
//#define psAUTO_SLEEP		0x4000
//#define psSLEEP_TIME		0x4001
#define psDISPLAY_SCALE		0x4002
#define psMEAS_INTERVAL		0x4003
#define psTHERM_MODE		0x4004
#define psEMISSIVITY		0x4005
#define psSERIAL_NUMBER		0x4006
// end psSERIAL_NUMBER		0x4022
#define psLASER				0x4023
#define psAUTO_SLEEP		0x4024
#define psSLEEP_TIME		0x4025
#define ps_key_not_found	0x0502

// Calibration Code
#define CALIBRATION_CODE	0x33

// Hardware Interrupts
#define intSWITCH2			0x0010
#define intSWITCH1			0x0008

// Switch Pins
#define pinSWITCH2			0x04
#define pinSWITCH1			0x03

// Switch De-bounce value
#define DEBOUNCE			0x64

// Switch States
#define ssNONE_PRESSED		0x00
#define	ssSW2_PRESSED		0x01
#define ssSW1_PRESSED		0x02
#define ssBOTH_PRESSED		0x03

// Switch press lengths
#define splUNDETERMINED		0x00
#define splSHORT			0x01
#define splLONG				0x02
#define splTIME				0x10	// time (in roughly 1/10th seconds) switch must be held down for long switch press

// EEPROM Chip 0x00 addresses
#define eeOFFSET			0x00
#define eeGAIN				0x04
#define eeCALIBRATED		0x08
#define eeCAL_VALID			0xA5	// value of 0xA5 means the calibration values are valid.

// Battery control line
#define batMASK				0x0800
#define batON				0x0800
#define batOFF				0x0000

// Display
//#define dspON_OFF_MASK		0x40	//these settings work for WM unit, but kNGHt DISPLAY pin is moved to PB11.. PMG
//#define dspCS_MASK			0x20
//#define dspON				0x40
//#define dspOFF				0x00
//#define dspCS_SELECT		0x20
//#define dspCS_DESELECT		0x00
//#define dspVBIT				0x40
//#define dspVBIT				0x40
////#define dspVBIT				0x20

// Display
#define dspON_OFF_MASK		0x8000
#define dspCS_MASK			0x0001
#define dspON				0x8000
#define dspOFF				0x0000
#define dspCS_SELECT		0x0001
#define dspCS_DESELECT		0x0000
//#define dspVBIT				0x40

// DS28E05 1-Wire ROM Function Commands	// PMG
//#define Read_ROM			0x33
//#define Match_ROM			0x55
//#define Search_ROM			0xF0
//#define Skip_ROM			0xCC
//#define Resume				0xA5
//#define DS28E05				0x0D	// DS28E05 device family code..
//
//// DS28E05 1-Wire MEMORY Function Commands	// PMG
//#define Write_MEM			0x55
//#define Read_MEM			0xF0
//
//// DS2482-100 Commands				// PMG
//#define DS2482_Addr			0x30
//#define DEVReset			0xF0	// Reset the DS2482-100
//#define Set_Read_Ptr		0xE1	//
//#define Write_Config		0xD2	//
//#define OWReset				0xB4	// Reset the 1-Wire bus
//#define OWSingleBit			0x87	//
//#define OWWriteByte			0xA5	//
//#define OWReadByte			0x96	//
//#define OWTriplet			0x78	// Only used with multiple 1-Wire devices present to perform Search ROM...
//
//// DS2482-100 Valid Pointer Codes	// Used by the Set_Read_Ptr command.. PMG
//#define Status_Register			0xF0
//#define Read_Data_Register		0xE1
//#define Configuration_Register	0xC3
//
//// DS2482-100 Configuration Register Map
//#define OWS			0x08	// 1-Wire speed
//#define SPU			0x04	// Strong pull-up
//#define APU			0x01	// Active pull-up



#define smOFF					0x00
#define smSTANDALONE			0x01
#define smADVERTISING			0x02
#define smCONNECTED				0x03
#define	smSLEEPING				0x04

/**************************************
 * No longer used as of version 0.30
 *
#define smSTND_NOTSTB			0x01
#define smADVT_NOTSTB			0x02
#define smCONN_NOTSTB			0x03

#define smSTND_STABLE			0x04
#define smADVT_STABLE			0x05
#define smCONN_STABLE			0x06

#define smSLEEPING				0x07

#define stSTABLE_TO_NOTSTB		(-0x03)
#define stNOTSTB_TO_STABLE		(0x03)
#define stADVT_TO_STND			(-0x01)
#define stSTND_TO_ADVT			(0x01)
#define stADVT_TO_CONN			(0x01)
 *
****************************************/

/***************************************
 * No longer used as of version 0.20
 *
// State Machine states
// Format is smBBB_MMMM_RRR
// Where BBB is the Bluetooth state:
//		STA = bluetooth off (STAndalone mode)
//		ADV = bluetooth ADVertising
//		CON = bluetooth CONnected
//		SLP = bluetooth connected but SLeePing
// And MMMM is the mode:
//		TEMP = TEMPerature mode (thermometer mode)
//		RITS = Rapid Intelligent Temperature Stabilization mode
//		MMFL = MeMory FuLl mode (scroll in standalone, wait for app to clear in connected)
// And RRR is the RITS sub-mode:
//		WT1 = WaiTing for 1st temp
//		WTN = WaiTing for Nth temp
//		STB	= STaBle temp
// And spectial state smOFF is when the Blue2-D2 is off
//
// These are organized so that the Bluetooth state can be
// gleaned by performing a modulo operation (% stBT_MOD)
// and states can be changed by adding a #defined state
// transition (eg. StateMachine += stSLP_TO_CON to wake up
// from any sleep state and transition to the corresponding
// connected state)

#define smOFF					0x00

#define smSTA_RITS_WT1			0x01
#define smADV_RITS_WT1			0x02
#define smCON_RITS_WT1			0x03
#define smSLP_RITS_WT1			0x04

#define smSTA_RITS_WTN			0x05
// intentionally skipped		0x06
#define smCON_RITS_WTN			0x07
#define smSLP_RITS_WTN			0x08

#define smSTA_RITS_STB			0x09
// intentionally skipped		0x0A
#define smCON_RITS_STB			0x0B
// intentionally skipped		0x0C

#define smSTA_MMFL_XXX			0x0D
// intentionally skipped		0x0E
#define smCON_MMFL_XXX			0x0F
// intentionally skipped		0x10

#define smSTA_TEMP_XXX			0x11
#define smADV_TEMP_XXX			0x12
#define smCON_TEMP_XXX			0x13
#define smSLP_TEMP_XXX			0x14

#define	stBT_MOD				0x04
#define stSTA					0x01
#define stADV					0x02
#define stCON					0x03
#define stSLP					0x00
#define stSLP_TO_CON			(-0x01)
#define stADV_TO_STA			(-0x01)
#define stSTA_TO_ADV			0x01
#define stCON_TO_SLP			0x01
#define stSLP_TO_CON			(-0x01)
#define stSTB_TO_WTN			(-0x04)
#define stSTB_TO_MMFL			(0x04)
 */

//#define smOFF					0x00
//#define smADVERTISING_TEMP		0x01
//#define smNOTCONNECTED_TEMP		0x02
//#define smCONNECTED_TEMP		0x03
//#define smADVERTISING_SETTINGS	0x04
//#define smNOTCONNECTED_SETTINGS	0x05
//#define smCONNECTED_SETTINGS	0x06
//#define smSLEEPING				0x07
//#define smADVERTISING_SET_MODE	0x08
//#define smNOTCONNECTED_SET_MODE	0x09
//#define smCONNECTED_SET_MODE	0x0A
//#define smADVERTISING_CONT		0x0B
//#define smNOTCONNECTED_CONT		0x0C
//#define smCONNECTED_CONT		0x0D
//
//// Settings Modes
//#define setFC					0x00
//#define setLASER				0x01
//#define setEMISSIVITY			0x02
//
//// ITS Modes
//#define imNONE					0x00
//#define imWAITING_FOR_STEP		0x01
//#define imSTABILIZING			0x02
//#define imSTABLE				0x03

// Arrow Characters
#define arw_DOWN				0x11
#define arw_UP					0x12
#define arw_DOWN_UP				0x13

#define symNULL					0x00
#define symON					0x01
#define symCONNECTED			0x02
#define symNO_CAL1				0x03
#define symHIGH					0x04
#define symLOW					0x05
#define symBLANK				0x06
#define symFAHRENHEIT			0x07
#define symCELSIUS				0x08
#define symBATT_ERR				0x09
#define symNO_PROBE1			0x0A
#define symSLEEP4				0x0B
#define symBLANK5				0x0C
#define symNOT_OKAY_LEFT		0x0D
#define symNOT_OKAY_RIGHT		0x0E
#define symOKAY_RIGHT			0x0F
#define symOKAY_LEFT			0x10

#endif /* CONSTANTS_H_ */
