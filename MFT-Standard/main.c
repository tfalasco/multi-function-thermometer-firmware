/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

//#define INCLUDE_IR						// Comment this line out for non-IR builds

//#define TEST_ROUNDING
//#define MCD_RITS
#define HOT_OR_COLD_RITS
//#define WHOLE_DEGS_F
//#define RITS_CHARACTERIZATION
#ifdef RITS_CHARACTERIZATION
#define HUNDREDTHS_DEG_F
#endif // RITS_CHARACTERIZATION
//#define BLACK_BORDERS
//#define TEST_DISP
//#define TEST_DISP_STRING		"-1oo.o"
//#define ADJUST_DISP


#define FIRMWARE_VER			"3.06"	// Must be 4 characters

#ifdef INCLUDE_IR
#define HARDWARE_VER			"006"	// Must be 3 characters
#define DEVICE_NAME				"Blue2-DIR"
#define DEVICE_NAME_LEN			9
#else
#define HARDWARE_VER			"009"	// Must be 3 characters
#define DEVICE_NAME				"Blue2-MFT"		// Maximum 11 characters
#define DEVICE_NAME_LEN			9
#endif //INCLUDE_IR

#define DEFAULT_VERT_OFFSET		3
#define DEFAULT_HORZ_OFFSET		2


/* Board headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
//#include "bg_dfu.h"
#include "gatt_db.h"
#include "aat.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_system.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#include "em_timer.h"
#endif

/* Device initialization header */
#include "InitDevice.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif

#include "em_gpio.h"
#include "em_i2c.h"
#include "em_adc.h"
#include "em_usart.h"
#include "em_timer.h"
#include "constants.h"
#include "display2.h"
#include "lookup_tables.h"
#include "em_cryotimer.h"
#include "em_letimer.h"
//#include "bsp_trace.h"

#include <stdio.h>
//#include <string.h>

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

I2C_TransferReturn_TypeDef ret;
uint8 i2c_read_buffer[9];
uint8 i2c_write_buffer[9];
I2C_TransferSeq_TypeDef seq;

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config =
{ .config_flags = 0,
		.sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
		.bluetooth.max_connections = MAX_CONNECTIONS,
		.bluetooth.heap = bluetooth_stack_heap,
		.bluetooth.heap_size = sizeof(bluetooth_stack_heap),
		.gattdb = &bg_gattdb_data,
		.ota.flags = 0,
//#ifdef INCLUDE_IR
//		.ota.device_name_len = 9,
//		.ota.device_name_ptr = "B2DIR-OTA",
//#else
		.ota.device_name_len = 7,
		.ota.device_name_ptr = HARDWARE_VER "-OTA",
//#endif //INCLUDE_IR
#ifdef FEATURE_PTI_SUPPORT
		.pti = &ptiInit,
#endif
	};

//uint8 OW_ROMID[9]; // Holds the smart probe device ID after calling void OWReadROMID(void);.. PMG
//uint8 OW_write_buffer[17];
//uint8 OW_read_buffer[17];

// State Machine
uint8 StateMachine;

uint8 StatusRegister[2];
// Bit  0 = Thermometer Mode : 0 = TC, 1 = IR
// Bit  1 = Display Scale: 0 = Celsius, 1 = Fahrenheit
// Bit  2 = Store Reading: 0 = Don't store, 1 = Store this reading
// Bit  3 = Calibration Mode: 0 = Measurement mode, 1 = Calibration mode
// Bit  4 = Calibration Step: 0 = Calibrating Offset, 1 = Calibrating Gain
// Bit  5 = Smart Probe: 0 = Not Found, 1 = Found
// Bit  6 = Reserved for future use
// Bit  7 = Reserved for future use
// Bit  8 = Probe missing : 0 = probe present, 1 = probe missing
// Bit  9 = Low : 0 = reading is not below range, 1 = reading is below range
// Bit 10 = High : 0 = reading is not above range, 1 = reading is above range
// Bit 11 = Sleeping : 0 = not in sleep mode, 1 = in sleep mode
// Bit 12 = Low Battery : 0 = battery is not low, 1 = battery is too low for accurate measurement
// Bit 13 = No Calibration: 0 = Calibrated, 1 = not calibrated
// Bit 14 = Not okay to record: 0 = Okay to record, 1 = not okay to record
// Bit 15 = Reserved for future use

//uint8 ItsMode;
//bool firstFastRead;
//uint8 ItsValue;
//int32 ItsStepSize;
//bool ItsIncreasing;

// Cryotimer counters
uint16 turnOffLEDs = 0;
uint8 buzzerStep1 = 0;
uint8 buzzerStep2 = 0;
uint8 buzzerStep3 = 0;
uint8 buzzerStep4 = 0;
uint8 shutDownCryo = 0;

// Buzzer profile
uint8 BuzzerProfile = bpNONE;

// Forward declarations
void StopAllTimers(void);
void ResetAutoSleep(uint8* AutoSleepTimer, uint8* SleepingTimer, uint8 MeasurementInterval);
void CalculateTemperature(int32* Temperature, uint8* Reading, int32 TEVolts, int16 Offset, int16 Gain);
void GetThermistorTEVolts(uint8* TMReading, int32* Temperature, int32* TEVolts);
void SetOffset(int32 TEVolts, uint8* TCReading, int32* Offset, uint8 * stateMachine);
void SetGain(int32 TEVolts, uint8* TCReading, int32* Gain, int32 Offset, uint8 MeasurementInterval, uint8 * stateMachine);
void UpdateBattery(uint8 *battPercentage, uint8 connectionHandle);
uint8 ReverseBits(uint8 b);
//void LoadDisplayBuffer(uint8 buffer[], char message[14]);
void LoadBigDisplayBuffer(uint8 buffer[], char message[14], int8 horzOffset);
void LoadMacDisplayBuffer(uint8 *buffer, char message[14], int8 horzOffset);
//void LoadBottomDisplayBuffer(uint8 *buffer, char leftMessage[6], char rightMessage[6]);
void LoadStatusIconBuffer(uint8 buffer[], char message[14], int8 horzOffset);
void AddBatteryIcon(uint8 *buffer, uint8 battPercentage);
void timer_WaitUs(uint8_t uDelay);
//void RemapBufferAndCountPixels(char * buffer, uint8 length, uint8 * pixelWidth);
void RemapMacBufferAndCountPixels(char * buffer, uint8 length, uint8 * pixelWidth);
//uint8 DS2482_Reset(void);
//void Wait_OW_Busy(void);
////void DS2482_Set_Read_Pointer(uint8 Pointer_Code);
//uint8 DS2482_Write_Configuration(uint8 Configuration_Byte);
//uint8 DS2482_OW_Reset(void);
//void DS2482_COMMAND(uint8 Command, uint8 Parameter);
//void DS2482_OW_WriteByte(uint8 Value);
//uint8 DS2482_OW_Read_Byte(void);
//uint8 OWReadROMID(uint8 *ID_ARRAY);
//uint8 calc_crc8(uint8 oldcrc, uint8 data);
//uint8 DS2482_OW_WriteMem(uint8 Parameters, uint8 *WriteBuff);  // Uses global array OW_write_buffer[].. PMG
//uint8 DS2482_OW_ReadMem(uint8 Parameters, uint8 Count, uint8 *ReadBuff);
//uint8 DS2482_Read_Status(void);
#ifdef INCLUDE_IR
void PowerLaser(bool powerState);
uint8 CalculateCRC(uint8 len, uint8* message);
#endif //INCLUDE_IR
void SetupLeTimer(void);
void SetupCryoTimer(void);
void BuzzerStart(bool start, uint8 level);
void ShutDownTasks(uint8 * buzzerProfile, uint8 * connectionHandle, uint8 * stateMachine, uint8 * advertisementTimer, uint8 * autoSleepTimer);
void StartUpTasks(void);
void PowerDisplay(bool powerState);
void GenerateTemperatureAsciiString(char* tempMessage, bool* useBigTemp, float tempInC);
void GenerateTemperatureDisplayString(char* tempDisplayMessage, bool* useBigTemp, float tempInC);
//void BeginITS(bool advertising);
//void StopITS(uint8 measIntvl, bool advertising);
void TakeImmediateTemperature(bool * ResettingMeasTimer);
void SetTemperatureDataToNaN(uint8* TemperatureData);
void SetLegacyTempAsFloatToNaN(uint8* LegacyTempAsFloat);
void RoundTo(float* input, unsigned int places);
void WriteBasicInfoToGatt(uint8* SerialNumber, uint8* SerialNumberString, uint8* GattString);

/**
 * @brief  Main function
 */
void main(void)
{
	uint8 advertisementTimer, autoSleepTimer, sleepingTimer;
	uint8 ConnectionHandle;
#ifdef INCLUDE_IR
	uint8 AutoSleepInterval, SleepTime, DisplayScale, measurementInterval, ThermometerMode, Emissivity, Laser;
#else
	uint8 AutoSleepInterval, SleepTime, DisplayScale, measurementInterval, ThermometerMode;
#endif //INCLUDE_IR
	int8 horizontalOffset, verticalOffset;
	uint8 SerialNumber[6], SerialNumberString[18], GattString[DEVICE_NAME_LEN + 1];
	uint8 TemperatureData[6]; // Four bytes of temperature value followed by 16 bits of status and error flags.
	uint8 LegacyTempAsFloat[5]; // Four bytes of temperature value followed by a null character
	int32 temperature;
	int32 teVolts;
#ifdef INCLUDE_IR
	uint8 tmReading[3], tcReading[3], irReading[3];
#else
	uint8 tmReading[3], tcReading[3];
#endif //INCLUDE_IR
	int32 offset, gain;
	struct gecko_msg_flash_ps_load_rsp_t* loadRsp;
	uint8 VBit;
	uint8 spiHeader[1] = "\x80";
	uint8 blankRow[18] = {0x00};
#ifdef BLACK_BORDERS
	uint8 filledRow[18] = {0x00};
#endif // BLACK_BORDERS
	uint8 tempLine[918] = {0x00};
	uint8 spiTrailer[1] = "\x00";
	char tempMessage[14] = {0x00};
	char tempDisplayMessage[14] = {0x00};
	bool useBigTemp = false;
	char statusIconMessage[14] = {0x00};
	uint8 switchTimerCounts = 0;
	uint8 SwitchState = ssNONE_PRESSED;
	uint8 switchPressLength = splUNDETERMINED;
	StateMachine = smOFF;
#ifdef INCLUDE_IR
	uint8 SettingMode = setFC;
	bool SetEmissivityViaGatt = false;
	uint8 DesiredEmissivity = 0x00;
	uint8 ConsecutiveScrolls = 0x00;
	bool SwitchingMode = false;
	uint8 SetEmissRetries = 0;
#endif // INCLUDE_IR
	bool StillBooting;
	uint8 BootToDFU = 0;
	uint8 length;
	uint8 AdvertisementData[31], ScanResponseData[31];
	uint8 ServiceUUIDOffset = 0;
	uint8 battPercentage;
	bool resettingMeasTimer = false;
	bool changeScale = false;

	bool WorkingClean;


	uint8 dataToWrite[1];



	union int32_bytes
	{
		int32 val;
		char bytes[sizeof(int32)];
	} int32Data;

	union float_bytes
	{
	   float val;
	   char bytes[sizeof(float)];
	} data;

	advertisementTimer = 0xFF;
	autoSleepTimer = 0x00;
	sleepingTimer = 0x00;

	AutoSleepInterval = 5;
	SleepTime = 5;
	DisplayScale = 1;
	measurementInterval = 1;
	ThermometerMode = 0;
	horizontalOffset = DEFAULT_HORZ_OFFSET;
	verticalOffset = DEFAULT_VERT_OFFSET;
//	ItsMode = imNONE;
//	firstFastRead = false;
#ifdef INCLUDE_IR
	Emissivity = 95;
	Laser = 0;
#endif
//	sprintf((char*) SerialNumberString, "NotImplemented");
	sprintf((char*) StatusRegister, "\x00\x00");

	ConnectionHandle = 0xFF;

#ifdef FEATURE_SPI_FLASH
	  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
	  MX25_init();
	  MX25_DP();
	  /* We must disable SPI communication */
	  USART_Reset(USART1);
#endif /* FEATURE_SPI_FLASH */


	// Make sure LEDs are off
	GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);

	/* Initialize peripherals */
	enter_DefaultMode_from_RESET();

	/* Initialize stack */
	gecko_init(&config);

	// Enable code correlation for Energy Profiler
//	BSP_TraceProfilerSetup();

	// Enable/Disable GATT characteristics
#ifdef INCLUDE_IR
	gecko_cmd_gatt_server_set_capabilities((Infrared | Legacy_Blue2  | Sound), 0);
#else
#ifdef MCD_RITS
	gecko_cmd_gatt_server_set_capabilities((Legacy_Blue2  | Sound | Rits), 0);
#else
#ifdef ADJUST_DISP
	gecko_cmd_gatt_server_set_capabilities((Legacy_Blue2  | Sound | Adjust_Display), 0);
#else
	gecko_cmd_gatt_server_set_capabilities((Legacy_Blue2  | Sound), 0);
#endif //ADJUST_DISP
#endif //MCD_RITS
#endif //INCLUDE_IR

	// Initialize GPIO
	GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);
	GPIO_PortOutSetVal(gpioPortA, dspCS_DESELECT, dspCS_MASK);
	PowerDisplay(false);
	GPIO_PortOutSetVal(gpioPortA, regOFF, regMASK);


	GPIO_PinModeSet(gpioPortB, 12, gpioModePushPull, 0);
	GPIO_PortOutSetVal(gpioPortB, ds2482OFF, ds2482MASK);


	// Disable Switch Interrupts until boot is done
	GPIO_IntDisable(0x0018);

	// Setup LeTimer for buzzer
	SetupLeTimer();

	// Setup Cryotimer
	SetupCryoTimer();

	// Initialize lookup tables
	InitializeThermocoupleTable();
	InitializeThermistorTable();


	while (1)
	{

		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;
		/* Check for stack event. */
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header))
		{
			case gecko_evt_system_boot_id:
				// Reinitialize variables
				advertisementTimer = 0xFF;
				autoSleepTimer = 0x00;
				sleepingTimer = 0x00;
				ConnectionHandle = 0xFF;
				VBit = 0x40;
				StateMachine = smOFF;
				StillBooting = true;
				BootToDFU = 0;

//				LockedOn = false;
//				HasMoved = true;
//				LockedTemp = 95.0;
//				FlashOffLockedTemp = false;

				WorkingClean = false;

				// Set bigger MTU to speed up transfer data
				gecko_cmd_gatt_set_max_mtu(58);

				// Write the serial number, fw, hw, and device name to GATT
				WriteBasicInfoToGatt(SerialNumber, SerialNumberString, GattString);

				// Read PS store values
				// Auto-sleep
				loadRsp = gecko_cmd_flash_ps_load(psAUTO_SLEEP);
				if (loadRsp->result == ps_key_not_found)
				{
					AutoSleepInterval = 5; // Default Value
					uint8 data[1];
					data[0] = AutoSleepInterval;
					gecko_cmd_flash_ps_save(psAUTO_SLEEP, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_auto_sleep_interval_characteristic, 0, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_auto_off_interval, 0, 1, data);

				}
				else
				{
					AutoSleepInterval = loadRsp->value.data[0];
					gecko_cmd_gatt_server_write_attribute_value(gattdb_auto_sleep_interval_characteristic, 0, 1, loadRsp->value.data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_auto_off_interval, 0, 1, loadRsp->value.data);
				}
				// Sleep Time
				loadRsp = gecko_cmd_flash_ps_load(psSLEEP_TIME);
				if (loadRsp->result == ps_key_not_found)
				{
					SleepTime = 5; // Default Value
					uint8 data[1];
					data[0] = SleepTime;
					gecko_cmd_flash_ps_save(psSLEEP_TIME, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_sleep_time_characteristic, 0, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_sleep_time, 0, 1, data);
				}
				else
				{
					SleepTime = loadRsp->value.data[0];
					gecko_cmd_gatt_server_write_attribute_value(gattdb_sleep_time_characteristic, 0, 1, loadRsp->value.data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_sleep_time, 0, 1, loadRsp->value.data);
				}
				// Display Scale
				loadRsp = gecko_cmd_flash_ps_load(psDISPLAY_SCALE);
				if (loadRsp->result == ps_key_not_found)
				{
					DisplayScale = 1; // Default Value (F)
					uint8 data[1];
					data[0] = DisplayScale;
					gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_display_scale_characteristic, 0, 1, data);
					// For legacy Blue2 support
					// Measurement Scale
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "F");
				}
				else
				{
					uint8 data[1];
					data[0] = loadRsp->value.data[0];
					if (changeScale)
					{
						if (data[0] == 1)
						{
							data[0] = 0;
						}
						else
						{
							data[0] = 1;
						}
					}
					DisplayScale = data[0];
					gecko_cmd_gatt_server_write_attribute_value(gattdb_display_scale_characteristic, 0, 1, data);
					// For legacy Blue2 support
					// Measurement Scale
					if (data[0] == 1)
					{
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "F");
					}
					else
					{
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "C");
					}
				}
				// Measurement Interval
				loadRsp = gecko_cmd_flash_ps_load(psMEAS_INTERVAL);
				if (loadRsp->result == ps_key_not_found)
				{
					measurementInterval = 1; // Default Value
					uint8 data[1];
					data[0] = measurementInterval;
					gecko_cmd_flash_ps_save(psMEAS_INTERVAL, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_measurement_interval_characteristic, 0, 1, data);
					// For legacy Blue2 support
					// Measurement Timing
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_timing, 0, 1, data);
				}
				else
				{
					measurementInterval = loadRsp->value.data[0];
					gecko_cmd_gatt_server_write_attribute_value(gattdb_measurement_interval_characteristic, 0, 1, loadRsp->value.data);
					// For legacy Blue2 support
					// Measurement Timing
					gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_timing, 0, 1, loadRsp->value.data);
				}
				// Horizontal Display Offset
				loadRsp = gecko_cmd_flash_ps_load(psDISP_HORZ_OFFSET);
				if (loadRsp->result == ps_key_not_found)
				{
					horizontalOffset = DEFAULT_HORZ_OFFSET;
					uint8 data[1];
					data[0] = horizontalOffset;
					gecko_cmd_flash_ps_save(psDISP_HORZ_OFFSET, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_vertical_offset, 0, 1, data);
				}
				else
				{
					horizontalOffset = (int8)(loadRsp->value.data[0]);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_vertical_offset, 0, 1, loadRsp->value.data);
				}
				// Vertical Display Offset
				loadRsp = gecko_cmd_flash_ps_load(psDISP_VERT_OFFSET);
				if (loadRsp->result == ps_key_not_found)
				{
					verticalOffset = DEFAULT_VERT_OFFSET;
					uint8 data[1];
					data[0] = verticalOffset;
					gecko_cmd_flash_ps_save(psDISP_VERT_OFFSET, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_horizontal_offset, 0, 1, data);
				}
				else
				{
					verticalOffset = (int8)(loadRsp->value.data[0]);
					gecko_cmd_gatt_server_write_attribute_value(gattdb_horizontal_offset, 0, 1, loadRsp->value.data);
				}
				// Thermometer Mode
				{
					ThermometerMode = 0; // Default Value (TC)
					uint8 data[1];
					data[0] = ThermometerMode;
					gecko_cmd_flash_ps_save(psTHERM_MODE, 1, data);
					gecko_cmd_gatt_server_write_attribute_value(
					gattdb_thermometer_mode_characteristic, 0, 1, data);
				}

				// Set up statusRegister
				if (ThermometerMode == 1)
				{
					StatusRegister[0] = StatusRegister[0] | 0x01;			// IR
				}
				else
				{
					StatusRegister[0] = StatusRegister[0] & 0xFE; 			// TC
				}
				if (DisplayScale == 1)
				{
					StatusRegister[0] = StatusRegister[0] | 0x02;			// Fahrenheit
				}
				else
				{
					StatusRegister[0] = StatusRegister[0] & 0xFD; 			// Celsius
				}

				// Set advertising parameters. 100ms advertisement interval. All channels used.
				gecko_cmd_le_gap_set_adv_parameters(160, 160, 7);

				// Set advertisment data.  Flags, Complete local name, Manufacturer-specific data (the mac address)
				for (uint8 i = 0; i < 31; i++)
				{
					AdvertisementData[i] = 0x00;
					ScanResponseData[i] = 0x00;
				}
				sprintf((char*)GattString, DEVICE_NAME);					// We will need this later, to set the name
				AdvertisementData[0]  = 0x02;								// Length of next parameter is 2 bytes
				AdvertisementData[1]  = 0x01;								// Flags
				AdvertisementData[2]  = 0x06;								// LE General Discoverable Mode & BR/EDR is not supported
				AdvertisementData[3]  = 0x10;								// Length of next parameter is 15 bytes
				AdvertisementData[4]  = 0xFF;								// Manufacturer specific data (we are sending the MAC address)
				AdvertisementData[5]  = 0x6A;								// First two octets are company identifier (0x016A)
				AdvertisementData[6]  = 0x01;								// First two octets are company identifier (0x016A)
				AdvertisementData[7]  = 0x01;								// Third octet is what we are sending (0x01 = MAC address)
				for (uint8 i = 0; i < 12; i++)
				{
					AdvertisementData[8 + i] = SerialNumberString[i];		// MAC Address in big-endian
				}
				if (DEVICE_NAME_LEN < 10)
				{
					// Device Name fits in adv packet
					AdvertisementData[20] = DEVICE_NAME_LEN + 1;				// Length of next parameter
					AdvertisementData[21] = 0x09;								// Complete Local Name
					for (uint8 i = 0; i < DEVICE_NAME_LEN; i++)
					{
						AdvertisementData[22 + i] = GattString[i];				// Device Name in big-endian
					}

					// The advertised Temperature Service UUID will start the scan response packet
					ServiceUUIDOffset = 0;
					ScanResponseData[0] =  0x11;								// Length of next parameter
					ScanResponseData[1] =  0x06;								// Complete List of 128-bit Service Class UUID
					ScanResponseData[2] =  0x96;								// Temperature Service UUID in little-endian
					ScanResponseData[3] =  0x94;								// Temperature Service UUID in little-endian
					ScanResponseData[4] =  0x11;								// Temperature Service UUID in little-endian
					ScanResponseData[5] =  0x72;								// Temperature Service UUID in little-endian
					ScanResponseData[6] =  0xb0;								// Temperature Service UUID in little-endian
					ScanResponseData[7] =  0xbb;								// Temperature Service UUID in little-endian
					ScanResponseData[8] =  0xaa;								// Temperature Service UUID in little-endian
					ScanResponseData[9] =  0x9e;								// Temperature Service UUID in little-endian
					ScanResponseData[10] = 0x61;								// Temperature Service UUID in little-endian
					ScanResponseData[11] = 0x47;								// Temperature Service UUID in little-endian
					ScanResponseData[12] = 0x2e;								// Temperature Service UUID in little-endian
					ScanResponseData[13] = 0x98;								// Temperature Service UUID in little-endian
					ScanResponseData[14] = 0x8b;								// Temperature Service UUID in little-endian
					ScanResponseData[15] = 0x90;								// Temperature Service UUID in little-endian
					ScanResponseData[16] = 0x35;								// Temperature Service UUID in little-endian
					ScanResponseData[17] = 0x64;								// Temperature Service UUID in little-endian

					gecko_cmd_le_gap_set_adv_data(0, 22 + DEVICE_NAME_LEN, AdvertisementData);
					gecko_cmd_le_gap_set_adv_data(1, 18, ScanResponseData);
				}
				else
				{
					// Device Name won't fit in adv packet.  Put it in scan response.
					ScanResponseData[0] = DEVICE_NAME_LEN + 1;				// Length of next parameter
					ScanResponseData[1] = 0x09;								// Complete Local Name
					for (uint8 i = 0; i < DEVICE_NAME_LEN; i++)
					{
						ScanResponseData[2 + i] = GattString[i];				// Device Name in big-endian
					}



					// The advertised Temperature Service UUID will start the scan response packet
					ServiceUUIDOffset = DEVICE_NAME_LEN + 2;
					ScanResponseData[ServiceUUIDOffset + 0] =  0x11;								// Length of next parameter
					ScanResponseData[ServiceUUIDOffset + 1] =  0x06;								// Complete List of 128-bit Service Class UUID
					ScanResponseData[ServiceUUIDOffset + 2] =  0x96;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 3] =  0x94;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 4] =  0x11;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 5] =  0x72;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 6] =  0xb0;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 7] =  0xbb;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 8] =  0xaa;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 9] =  0x9e;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 10] = 0x61;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 11] = 0x47;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 12] = 0x2e;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 13] = 0x98;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 14] = 0x8b;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 15] = 0x90;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 16] = 0x35;								// Temperature Service UUID in little-endian
					ScanResponseData[ServiceUUIDOffset + 17] = 0x64;								// Temperature Service UUID in little-endian

					gecko_cmd_le_gap_set_adv_data(0, 20, AdvertisementData);
					gecko_cmd_le_gap_set_adv_data(1, 20 + DEVICE_NAME_LEN, ScanResponseData);
				}

				GPIO_PortOutSetVal(gpioPortA, regON, regMASK);				// Turn on VReg2.7

				gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thREAD_CAL_VALUES, 1);
				break;

			case gecko_evt_le_connection_closed_id:

				ShutDownTasks(&BuzzerProfile, &ConnectionHandle, &StateMachine, &advertisementTimer, &autoSleepTimer);

				if (BootToDFU)
				{
					gecko_cmd_system_reset(2);
				}
				break;

			case gecko_evt_le_connection_opened_id:
				ConnectionHandle = evt->data.evt_le_connection_opened.connection;

				gecko_cmd_hardware_set_soft_timer(0, thADVERTISEMENT, 1); // Stop advertisement
				advertisementTimer = 0xFF;

				ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);	 // Start one-minute repeating auto-off timer

				// Negotiate connection parameters
				gecko_cmd_le_connection_set_parameters(ConnectionHandle, 12, 24, 0, 200);

				StateMachine =smCONNECTED;
				break;

			case gecko_evt_gatt_server_user_write_request_id:
				if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control)
				{
					BootToDFU = 1;
					gecko_cmd_gatt_server_send_user_write_response(ConnectionHandle, gattdb_ota_control, bg_err_success);

					gecko_cmd_endpoint_close(ConnectionHandle);
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_emissivity_characteristic)
				{
					gecko_cmd_gatt_server_write_attribute_value(gattdb_emissivity_characteristic, 0, 1, &evt->data.evt_gatt_server_user_write_request.value.data[0]);				// Update the GATT database
					gecko_cmd_gatt_server_send_user_write_response(ConnectionHandle, gattdb_emissivity_characteristic, 0);			// Send the user write response
				}
				break;

			case gecko_evt_gatt_server_user_read_request_id:
				if (evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_emissivity_characteristic)
				{
					uint8 Emissivity = 0x00;
					gecko_cmd_gatt_server_send_user_read_response(ConnectionHandle, gattdb_emissivity_characteristic, 0, 1, &Emissivity);
				}
				break;

			case gecko_evt_gatt_server_attribute_value_id:
				ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);

				if (StateMachine == smSLEEPING)
				{
					StateMachine = smCONNECTED;
					TakeImmediateTemperature(&resettingMeasTimer);
				}

				if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_auto_sleep_interval_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psAUTO_SLEEP, 1, dataToWrite);
						AutoSleepInterval = dataToWrite[0];
						// For legacy Blue2 support
						// Auto-off interval
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_auto_off_interval, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_auto_off_interval)
				{
					// For legacy Blue2 support
					// Auto-off Interval
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psAUTO_SLEEP, 1, dataToWrite);
						AutoSleepInterval = dataToWrite[0];
						gecko_cmd_gatt_server_write_attribute_value(gattdb_auto_sleep_interval_characteristic, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_sleep_time_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psSLEEP_TIME, 1, dataToWrite);
						SleepTime = dataToWrite[0];
						// For legacy Blue2 support
						// Sleep time
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_sleep_time, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_sleep_time)
				{
					// For legacy Blue2 support
					// Sleep time
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psSLEEP_TIME, 1, dataToWrite);
						SleepTime = dataToWrite[0];
						gecko_cmd_gatt_server_write_attribute_value(gattdb_sleep_time_characteristic, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_force_sleep_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len > 0)
					{
						autoSleepTimer = AutoSleepInterval;
						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thAUTO_SLEEP, 0);
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_force_sleep)
				{
					// For legacy Blue2 support
					// Force Sleep
					if (evt->data.evt_gatt_server_user_write_request.value.len > 0)
					{
						autoSleepTimer = AutoSleepInterval;
						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thAUTO_SLEEP, 0);
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_disconnect_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len > 0)
					{
						// Disconnect
						gecko_cmd_endpoint_close(ConnectionHandle);
						StateMachine = smOFF;
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_disconnect_peripheral)
				{
					// For legacy Blue2 support
					// Disconnect peripheral
					if (evt->data.evt_gatt_server_user_write_request.value.len > 0)
					{
						// Disconnect
						gecko_cmd_endpoint_close(ConnectionHandle);
						StateMachine = smOFF;
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_display_scale_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						if (evt->data.evt_gatt_server_user_write_request.value.data[0] == 1)
						{
							StatusRegister[0] = StatusRegister[0] | 0x02; // Fahrenheit
							uint8 data[1] =	{ 1 };
							gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, data);
							DisplayScale = 1;
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_display_scale_characteristic, 1, data);	// Send notification
							// For leagacy Blue2 support
							// Measurement Scale
							gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "F");				// Update the GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "F");	// Send notification
						}
						else if (evt->data.evt_gatt_server_user_write_request.value.data[0] == 0)
						{
							StatusRegister[0] = StatusRegister[0] & 0xFD; // Celsius
							uint8 data[1] = { 0 };
							gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, data);
							DisplayScale = 0;
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_display_scale_characteristic, 1, data);	// Send notification
							// For leagacy Blue2 support
							// Measurement Scale
							gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "C");				// Update the GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "C");	// Send notification
						}
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_measurement_scale)
				{
					// For leagacy Blue2 support
					// Measurement Scale
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						if (evt->data.evt_gatt_server_user_write_request.value.data[0] == 0x46)
						{
							StatusRegister[0] = StatusRegister[0] | 0x02; // Fahrenheit
							uint8 data[1] =	{ 1 };
							gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, data);
							DisplayScale = 1;
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "F");	// Send notification
							gecko_cmd_gatt_server_write_attribute_value(gattdb_display_scale_characteristic, 0, 1, data);				// Update the GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_display_scale_characteristic, 1, data);	// Send notification
						}
						else if (evt->data.evt_gatt_server_user_write_request.value.data[0] == 0x43)
						{
							StatusRegister[0] = StatusRegister[0] & 0xFD; // Celsius
							uint8 data[1] = { 0 };
							gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, data);
							DisplayScale = 0;
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "C");	// Send notification
							gecko_cmd_gatt_server_write_attribute_value(gattdb_display_scale_characteristic, 0, 1, data);				// Update the GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_display_scale_characteristic, 1, data);	// Send notification
						}
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_measurement_interval_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psMEAS_INTERVAL, 1, dataToWrite);
						measurementInterval = dataToWrite[0];
						gecko_cmd_hardware_set_soft_timer(tmSECOND * measurementInterval, thMEASUREMENT, 0);		// Restart measurement timer with new value

						// For legacy Blue2 support
						// Mesurement Interval
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_timing, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_legacy_measurement_timing)
				{
					// For leagacy Blue2 support
					// Measurement Interval
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						if (dataToWrite[0] < 1)
						{
							dataToWrite[0] = 1;
						}
						else if (dataToWrite[0] > 60)
						{
							dataToWrite[0] = 60;
						}
						gecko_cmd_flash_ps_save(psMEAS_INTERVAL, 1, dataToWrite);
						measurementInterval = dataToWrite[0];
						gecko_cmd_hardware_set_soft_timer(tmSECOND * measurementInterval, thMEASUREMENT, 0);		// Restart measurement timer with new value

						gecko_cmd_gatt_server_write_attribute_value(gattdb_measurement_interval_characteristic, 0, 1, dataToWrite);				// Update the GATT database
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_thermometer_mode_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						StatusRegister[0] = StatusRegister[0] & 0xFE;	// TC
						uint8 data[1] =
						{ 0 };
						gecko_cmd_flash_ps_save(psTHERM_MODE, 1, data);
						ThermometerMode = 0x00;
						TakeImmediateTemperature(&resettingMeasTimer);

					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_begin_calibration_characteristic)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len > 0)
					{
						if (evt->data.evt_gatt_server_user_write_request.value.data[0] == CALIBRATION_CODE)
						{
							// Change the cal mode to 'Calibrate'
							StatusRegister[0] = StatusRegister[0] | 0x08;

							// Change cal step to 'Offset'
							StatusRegister[0] = StatusRegister[0] & 0xEF;
						}
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_laser_characteristic)
				{

				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_play_sound_characteristic)
				{
					uint8 buzzerProfile = evt->data.evt_gatt_server_user_write_request.value.data[0];
					if ((buzzerProfile > bpNONE) && (buzzerProfile <= bpMAX_PROFILE))
					{
						BuzzerProfile = buzzerProfile;
						buzzerStep1 = 1;
//						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thBUZZER_STEP_1, 1);		// Start buzzer profile
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_vertical_offset)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						int8 signedData;
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						signedData = (int8)dataToWrite[0];
						if (signedData < -10)
						{
							signedData = -10;
						}
						else if (signedData > 10)
						{
							signedData = 10;
						}
						gecko_cmd_flash_ps_save(psDISP_VERT_OFFSET, 1, dataToWrite);
						verticalOffset = signedData;
					}
				}
				else if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_horizontal_offset)
				{
					if (evt->data.evt_gatt_server_user_write_request.value.len == 1)
					{
						uint8 dataToWrite[1];
						int8 signedData;
						dataToWrite[0] = evt->data.evt_gatt_server_user_write_request.value.data[0];
						signedData = (int8)dataToWrite[0];
						if (signedData < -10)
						{
							signedData = -10;
						}
						else if (signedData > 10)
						{
							signedData = 10;
						}
						gecko_cmd_flash_ps_save(psDISP_HORZ_OFFSET, 1, dataToWrite);
						horizontalOffset = signedData;
					}
				}
				break;

			case gecko_evt_system_external_signal_id:
				// Disable switch interrupts
//				GPIO_IntDisable(0x0018);
				WorkingClean = false;

				if (evt->data.evt_system_external_signal.extsignals & intSWITCH1)
				{
					// Debounce Switch 1
					uint8 lastSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH1);
					uint8 currentSwitchState;
					uint16 consecutiveSwitchStates = 0;
					while (consecutiveSwitchStates < DEBOUNCE)
					{
						timer_WaitUs(100);
						currentSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH1);
						if (currentSwitchState == lastSwitchState)
						{
							consecutiveSwitchStates++;
						}
						else
						{
							lastSwitchState = currentSwitchState;
							consecutiveSwitchStates = 0;
						}
					}

					if (lastSwitchState == 1)													// If just a transient spike, then break
					{
						// Re-enable switch interrupts
//						GPIO_IntEnable(0x0180);
//						GPIO_IntClear(0x0180);
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);
						break;
					}
					if (StateMachine != smOFF)
					{
//						for (uint8 i = 0; i < 5; i++)
//						{
//							GPIO_PortOutSet(gpioPortD, buzMASK);
//							GPIO_PortOutClear(gpioPortD, buzMASK);
//						}
						BuzzerProfile = bpCHIP;
						buzzerStep1 = 1;
//						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thBUZZER_STEP_1, 1);		// Start buzzer profile
					}

					SwitchState = ssSW1_PRESSED;
					switchPressLength = splUNDETERMINED;
				}
				else if (evt->data.evt_system_external_signal.extsignals & intSWITCH2)
				{
					// Debounce Switch 2
					uint8 lastSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH2);
					uint8 currentSwitchState;
					uint16 consecutiveSwitchStates = 0;
					while (consecutiveSwitchStates < DEBOUNCE)
					{
						timer_WaitUs(100);
						currentSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH2);
						if (currentSwitchState == lastSwitchState)
						{
							consecutiveSwitchStates++;
						}
						else
						{
							lastSwitchState = currentSwitchState;
							consecutiveSwitchStates = 0;
						}
					}

					if (lastSwitchState == 1)													// If just a transient spike, then break
					{
						// Re-enable switch interrupts
//						GPIO_IntEnable(0x0180);
//						GPIO_IntClear(0x0180);
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);

						break;
					}
					if (StateMachine != smOFF)
					{
						BuzzerProfile = bpCHIP;
						buzzerStep1 = 1;
//						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thBUZZER_STEP_1, 1);		// Start buzzer profile
					}

					SwitchState = ssSW2_PRESSED;
					switchPressLength = splUNDETERMINED;
				}

				switchTimerCounts = 0;
				gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thSWITCH, 1);
				break;

			case gecko_evt_hardware_soft_timer_id:
				if (evt->data.evt_hardware_soft_timer.handle == thADVERTISEMENT)
				{
					advertisementTimer++;
					if (advertisementTimer >= 20)
					{
						gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_non_connectable);  // Stop advertising
						advertisementTimer = 0xFF;
						gecko_cmd_hardware_set_soft_timer(0, thADVERTISEMENT, 1);

//						ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);

						StateMachine = smSTANDALONE;
						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);				// Update the display
					}
					else
					{
						GPIO_PortOutSetVal(gpioPortD, ledGREEN, ledMASK);	// Flash the green LED
						turnOffLEDs = 7;
//						gecko_cmd_hardware_set_soft_timer(tmFLASH, thFLASH_OFF, 1);
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thAUTO_SLEEP)
				{
					autoSleepTimer++;
					if (autoSleepTimer >= AutoSleepInterval)
					{
						if (StateMachine == smCONNECTED)
						{
							StatusRegister[1] = StatusRegister[1] | 0x08;								// Set the "Sleeping" flag.
							// Set up TemperatureData to send to Temperature Reading characteristic
							sprintf((char*) TemperatureData, "%c%c%c%c%c%c", 0x7F, 0xFF, 0xFF, 0xFF, StatusRegister[0], StatusRegister[1]);	// Blank out the temperature reading (NaN) and set the flags
							gecko_cmd_gatt_server_write_attribute_value(gattdb_temperature_reading_characteristic, 0, 6, TemperatureData);			// Write to GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_temperature_reading_characteristic, 6, TemperatureData);	// Send notification

							// For legacy Blue2 support
							// Send "Sleeping" for TempAsAscii characteristic
							GenerateTemperatureAsciiString(tempMessage, &useBigTemp, data.val);

							for (length = 0; length < 14; length++)
							{
								if (tempMessage[length] == '\0')
								{
									break;
								}
							}
							gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_ascii, 0, length, (uint8*)tempMessage);	// Write to GATT database
							gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_ascii, length, (uint8*)tempMessage);		// Send notification

							sleepingTimer = 0;
							gecko_cmd_hardware_set_soft_timer(tmMINUTE, thSLEEP_TIME, 0); 				// Start sleeping timer
							gecko_cmd_hardware_set_soft_timer(tmSECOND * 4, thSLEEP_FLASH, 0); 			// Start 4 second timer to flash yellow led
							gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);				// Update the display
							StateMachine = smSLEEPING;
							TakeImmediateTemperature(&resettingMeasTimer);

						}
						else if (StateMachine  == smSTANDALONE)
						{
							ShutDownTasks(&BuzzerProfile, &ConnectionHandle, &StateMachine, &advertisementTimer, &autoSleepTimer);
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thSLEEP_TIME)
				{
					sleepingTimer++;
					if (sleepingTimer >= SleepTime)
					{
						if (ConnectionHandle < 0xFF)
						{
							gecko_cmd_endpoint_close(ConnectionHandle);
						}
						else
						{
							ShutDownTasks(&BuzzerProfile, &ConnectionHandle, &StateMachine, &advertisementTimer, &autoSleepTimer);
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thMEASUREMENT)
				{
					WorkingClean = true;
					if (StatusRegister[1] & 0x08)												// Sleeping
					{
						StopAllTimers();
						// Set up temperatureData to send to Temperature Reading characteristic
						sprintf((char*) TemperatureData, "%c%c%c%c%c%c", 0x7F, 0xFF, 0xFF, 0xFF, (char) StatusRegister[0], (char) StatusRegister[1]);// Blank out the temperature reading (NaN) and set the flags
						gecko_cmd_gatt_server_write_attribute_value(gattdb_temperature_reading_characteristic, 0, 6, TemperatureData);		// Write to GATT database
						gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_temperature_reading_characteristic, 6, TemperatureData);			// Send notification

						// For legacy Blue2 support
						// TempAsFloat characteristic
						SetLegacyTempAsFloatToNaN(LegacyTempAsFloat);
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_float, 0, 5, LegacyTempAsFloat);	// Write to GATT database
						gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_float, 5, LegacyTempAsFloat);		// Send notification

						sleepingTimer = 0;
						gecko_cmd_hardware_set_soft_timer(tmMINUTE, thSLEEP_TIME, 0); 			// Start sleeping timer
						GPIO_PortOutSetVal(gpioPortD, ledYELLOW, ledMASK); 						// Flash the yellow LED
//						gecko_cmd_hardware_set_soft_timer(tmFLASH, thFLASH_OFF, 1);
						turnOffLEDs = 7;
						gecko_cmd_hardware_set_soft_timer(tmSECOND * 4,	thSLEEP_FLASH, 0); 		// Start 4 second timer to flash yellow LED again
						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC,	thDISPLAY, 1);			// Update the display
						GPIO_PortOutSetVal(gpioPortC, irOFF, irMASK);							// Turn off VMeasIR2.7
					}
					else
					{
						if (StateMachine == smCONNECTED)
						{
							// If we are advertising, the advertisement timer flashes the green LED.
							// If we are not connected (stand-alone mode) we don't flash an LED.
							// So we only flash the blue LED if we are connected.
//							GPIO_PortOutSetVal(gpioPortA, ledBLUE, 0x20); 						// Flash the blue LED
							GPIO_PortOutSetVal(gpioPortD, ledBLUE, ledMASK); 					// Flash the blue LED
						}
//						gecko_cmd_hardware_set_soft_timer(tmFLASH, thFLASH_OFF, 1);
						turnOffLEDs = 7;

						GPIO_PortOutSetVal(gpioPortC, tcON, tcMASK);							// Turn on VMeasTC2.7
						if ((StateMachine != smOFF) && (WorkingClean == true))
						{
							gecko_cmd_hardware_set_soft_timer(tmQUARTERSECOND, thWAIT_FOR_VMEAS_TC, 1); 	// Wait  for VMeasTC2.7
						}

						if (resettingMeasTimer)
						{
							resettingMeasTimer = false;
							if (StateMachine != smOFF)
							{
#ifdef TEST_ROUNDING
								gecko_cmd_hardware_set_soft_timer(tmSECOND * 4, thMEASUREMENT, 0);
#else
								gecko_cmd_hardware_set_soft_timer(tmSECOND * measurementInterval, thMEASUREMENT, 0);
#endif	// TEST_ROUNDING
							}
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thWAIT_FOR_VMEAS_TC)
				{
					if (WorkingClean == true)
					{
						I2C_TransferReturn_TypeDef ret;
						I2C_TransferSeq_TypeDef seq;
						seq.flags = I2C_FLAG_WRITE;
						seq.addr = 0xD0;
						seq.buf[0].len = 1;
						i2c_write_buffer[0] = 0xA8;
						seq.buf[0].data = i2c_write_buffer;
						ret = I2C_TransferInit (I2C0, &seq);								// Start the I2C transfer
						while (ret == i2cTransferInProgress)
						{
							if (StateMachine == smOFF)
							{
								break;
							}
							ret = I2C_Transfer(I2C0);
						}
						if ((StateMachine != smOFF) && (WorkingClean == true))
						{
							gecko_cmd_hardware_set_soft_timer(tm75MILLISECONDS, thWAIT_FOR_TC, 1); 			// Wait 70mS for A2D conversion
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thWAIT_FOR_TC)
				{
					if (WorkingClean == true)
					{
						seq.flags = I2C_FLAG_READ;
						seq.addr = 0xD0;
						seq.buf[0].len = 3;
						seq.buf[0].data = i2c_read_buffer;
						ret = I2C_TransferInit (I2C0, &seq);										// Start the I2C transfer
						while (ret == i2cTransferInProgress)
						{
							if (StateMachine == smOFF)
							{
								break;
							}
							ret = I2C_Transfer(I2C0);
						}
						tcReading[0] = i2c_read_buffer[0];
						tcReading[1] = i2c_read_buffer[1];

						if ((StateMachine != smOFF) && (WorkingClean == true))
						{
							gecko_cmd_hardware_set_soft_timer(512, thSETTLE_FOR_TM, 1); 				// Settle a bit before sending the next conversion request
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thSETTLE_FOR_TM)
				{
					if (WorkingClean == true)
					{
						seq.flags = I2C_FLAG_WRITE;
						seq.addr = 0xD0;
						seq.buf[0].len = 1;
						i2c_write_buffer[0] = 0x88;
						seq.buf[0].data = i2c_write_buffer;
						ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
						while (ret == i2cTransferInProgress)
						{
							if (StateMachine == smOFF)
							{
								break;
							}
							ret = I2C_Transfer(I2C0);
						}

						if ((StateMachine != smOFF) && (WorkingClean == true))
						{
							gecko_cmd_hardware_set_soft_timer(tm75MILLISECONDS, thWAIT_FOR_TM, 1); 					// Wait 70mS for A2D conversion
						}
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thWAIT_FOR_TM)
				{
					if (WorkingClean == true)
					{
						seq.flags = I2C_FLAG_READ;
						seq.addr = 0xD0;
						seq.buf[0].len = 3;
						seq.buf[0].data = i2c_read_buffer;
						ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
						while (ret == i2cTransferInProgress)
						{
							if (StateMachine == smOFF)
							{
								break;
							}
							ret = I2C_Transfer(I2C0);
						}
						tmReading[0] = i2c_read_buffer[0];
						tmReading[1] = i2c_read_buffer[1];

						GPIO_PortOutSetVal(gpioPortC, tcOFF, tcMASK);							// Turn off VMeasTC2.7
						if ((StateMachine != smOFF) && (WorkingClean == true))
						{
							gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thCALC_TEMP, 1);
						}
					}
				}
				if (evt->data.evt_hardware_soft_timer.handle == thCALC_TEMP)
				{
					GetThermistorTEVolts(tmReading, &temperature, &teVolts);
					if ((StatusRegister[0] & 0x08) == 0x00)									// Not in calibration mode
					{
						if (WorkingClean == true)
						{
							CalculateTemperature(&temperature, tcReading, teVolts, offset, gain);	// 'temperature' is now cjc compensated TC Temp in C x 100


#ifdef TEST_ROUNDING
							if (data.val == -26.26f)
							{
								data.val = -25.25f;
							}
							else if (data.val == -25.25f)
							{
								data.val = -24.24f;
							}
							else if (data.val == -24.24f)
							{
								data.val = -16.16f;
							}
							else if (data.val == -16.16f)
							{
								data.val = -15.15f;
							}
							else if (data.val == -15.15f)
							{
								data.val = -14.14f;
							}
							else if (data.val == -14.14f)
							{
								data.val = 14.14f;
							}
							else if (data.val == 14.14f)
							{
								data.val = 15.15f;
							}
							else if (data.val == 15.15f)
							{
								data.val = 16.16f;
							}
							else if (data.val == 16.16f)
							{
								data.val = 24.24f;
							}
							else if (data.val == 24.24f)
							{
								data.val = 25.25f;
							}
							else if (data.val == 25.25f)
							{
								data.val = 26.26f;
							}
							else
							{
								data.val = -26.26f;
							}
#else
							data.val = temperature / 100.0f;
#endif // TEST_ROUNDING

							TemperatureData[0] = data.bytes[0];
							TemperatureData[1] = data.bytes[1];
							TemperatureData[2] = data.bytes[2];
							TemperatureData[3] = data.bytes[3];
							StatusRegister[1] &= 0xF8;											// Clear the High, Low, and Probe flags
							if (data.val > 537.78)
							{
								StatusRegister[1] |= 0x04;										// If too high, set the "High" error flag
							}
							else if (data.val < -80.00)
							{
								StatusRegister[1] |= 0x01;										// If way too low, set the "Probe" error flag
							}
							else if (data.val < -73.33)
							{
								StatusRegister[1] |= 0x02;										// If too low, set the "Low" error flag
							}

							TemperatureData[4] = StatusRegister[0];					// Set the status flags
							TemperatureData[5] = StatusRegister[1];					// Set the error flags

							if (!
							   ((StatusRegister[1] & 0x08) 									// Sleeping
							|| ((StatusRegister[1] & 0x20) && (StatusRegister[0] | 0xFE)) 	// No calibration and TC mode
							|| ( StatusRegister[1] & 0x10)									// Low battery
							|| ((StatusRegister[1] & 0x01) && !(StatusRegister[0] & 0x01))	// No probe and TC mode
							|| ((StatusRegister[1] & 0x02) && !(StatusRegister[0] & 0x01))	// Low and TC mode
							|| ((StatusRegister[1] & 0x04) && !(StatusRegister[0] & 0x01)))	// High and TC mode
							)	// If no errors
							{
								TemperatureData[4] = StatusRegister[0];					// Set the status flags
								TemperatureData[5] = StatusRegister[1];					// Set the error flags

								// Send temperature
								gecko_cmd_gatt_server_write_attribute_value(gattdb_temperature_reading_characteristic, 0, 6, TemperatureData);	// Write to GATT database
								if (StateMachine == smCONNECTED)
								{
									gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_temperature_reading_characteristic, 6, TemperatureData);		// Send notification
								}

								// Legacy Blue2
								if (StatusRegister[0] & 0x02)										// Fahrenheit
								{
									int32Data.val = temperature * 9 / 5 + 3200;
								}
								else																// Celsius
								{
									int32Data.val = temperature;
								}
								LegacyTempAsFloat[0] = int32Data.bytes[0];
								LegacyTempAsFloat[1] = int32Data.bytes[1];
								LegacyTempAsFloat[2] = int32Data.bytes[2];
								LegacyTempAsFloat[3] = 0xFE;
								LegacyTempAsFloat[4] = 0x00;
								gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_float, 0, 5, LegacyTempAsFloat);	// Write to GATT database
								if (StateMachine == smCONNECTED)
								{
									gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_float, 5, LegacyTempAsFloat);		// Send notification
								}
							}
							else // There are errors in the reading
							{
								TemperatureData[4] = StatusRegister[0];					// Set the status flags
								TemperatureData[5] = StatusRegister[1];					// Set the error flags

								// Temperature not available, send NAN
								SetTemperatureDataToNaN(TemperatureData);
								gecko_cmd_gatt_server_write_attribute_value(gattdb_temperature_reading_characteristic, 0, 6, TemperatureData);	// Write to GATT database
								if (StateMachine == smCONNECTED)
								{
									gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_temperature_reading_characteristic, 6, TemperatureData);		// Send notification
								}

								SetLegacyTempAsFloatToNaN(LegacyTempAsFloat);
								gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_float, 0, 5, LegacyTempAsFloat);	// Write to GATT database
								if (StateMachine == smCONNECTED)
								{
									gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_float, 5, LegacyTempAsFloat);		// Send notification
								}
							}
						}
					}
					else																	// In calibration mode
					{
						StatusRegister[1] &= 0xBF;			// Clear Temperature not stable flag
						if ((StatusRegister[0] & 0x10) == 0)								// Calibrating offset
						{
							ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);
							gecko_cmd_hardware_set_soft_timer(0, thMEASUREMENT, 1); 		// Stop measurement timer
							SetOffset(teVolts, tcReading, &offset, &StateMachine);
						}
						else																// Calibrating Gain
						{
							SetGain(teVolts, tcReading, &gain, offset, measurementInterval, &StateMachine);
							gecko_cmd_hardware_set_soft_timer(tmSECOND * measurementInterval, thMEASUREMENT, 0);
						}
					}

					if (WorkingClean == true)
					{
						// For legacy Blue2 support
						// TempAsAscii characteristic
						if (StillBooting)
						{
							sprintf(tempMessage, "--");
							useBigTemp = true;
						}
						else
						{
							GenerateTemperatureAsciiString(tempMessage, &useBigTemp, data.val);
						}
						for (length = 0; length < 14; length++)
						{
							if (tempMessage[length] == '\0')
							{
								break;
							}
						}
						gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_ascii, 0, length, (uint8*)tempMessage);	// Write to GATT database
						gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_ascii, length, (uint8*)tempMessage);		// Send notification

						gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thDISPLAY)
				{
					GPIO_PortOutSetVal(gpioPortA, dspCS_SELECT, dspCS_MASK);
					spiHeader[0] = 0x80;
					spiTrailer[0] = 0x00;
					uint8 rowPointer = 0;
					uint8 rowPointerPlaceholder = 0;

					// Clear display strings
					for (uint8 i = 0; i < 13; i++)
					{
						tempDisplayMessage[i] = 0x00;
//						modeMessage[i] = 0x00;
						statusIconMessage [i] = 0x00;
					}

					for (uint16 i = 0; i < 918; i++)
					{
						tempLine[i] = 0xFF;
					}

					for (uint8 i = 0; i <  17; i++)											// Last byte intentionally left as 0x00
					{
						blankRow[i] = 0xFF;
//						filledRow[i] = 0x00;
					}

#ifdef BLACK_BORDERS
					// Add borders to blank row
					blankRow[1] = 0x00;
					blankRow[2] = 0x1F;
					blankRow[15] = 0xE0;
					blankRow[16] = 0x00;
#endif //BLACK_BORDERS

					// Toggle VBit
					if (VBit == 0x40)
					{
						VBit = 0xBF;
						spiHeader[0] &= VBit;
					}
					else
					{
						VBit = 0x40;
						spiHeader[0] |= VBit;
					}

					// Write to the display
					USART_SpiTransfer(USART0, spiHeader[0]);

#ifdef BLACK_BORDERS
					// Top border
					for (; rowPointer < 12; rowPointer++)
					{
						filledRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, filledRow[byte]);
						}
					}
#endif //BLACK_BORDERS

//					// Blank Lines
//					for (; rowPointer < 17; rowPointer++)
//					{
//						blankRow[0] = ReverseBits(rowPointer + 1);
//						for (uint8 byte = 0; byte < 18; byte++)
//						{
//							USART_SpiTransfer(USART0, blankRow[byte]);
//						}
//					}
//
//					// Mac Address Line
//					LoadMacDisplayBuffer(tempLine, (char*)SerialNumberString);
//					rowPointerPlaceholder = rowPointer;
//					for (; rowPointer < 27; rowPointer++)
//					{
//						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 0] = ReverseBits(rowPointer + 1);
//						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 17] = 0x00;
//						for (uint8 byte = 0; byte < 18; byte++)
//						{
//							USART_SpiTransfer(USART0, tempLine[((rowPointer - rowPointerPlaceholder) * 18) + byte]);
//						}
//					}
//
//					// Clear out tempLine
//					for (uint16 i = 0; i < 918; i++)
//					{
//						tempLine[i] = 0xFF;
//					}
					if (StateMachine == smADVERTISING)
					{
						// Blank Lines
						for (; rowPointer < 17 + verticalOffset; rowPointer++)
						{
							blankRow[0] = ReverseBits(rowPointer + 1);
							for (uint8 byte = 0; byte < 18; byte++)
							{
								USART_SpiTransfer(USART0, blankRow[byte]);
							}
						}

						// Mac Address Line
						LoadMacDisplayBuffer(tempLine, (char*)SerialNumberString, horizontalOffset);
						rowPointerPlaceholder = rowPointer;
						for (; rowPointer < 27 + verticalOffset; rowPointer++)
						{
							tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 0] = ReverseBits(rowPointer + 1);
							tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 17] = 0x00;
							for (uint8 byte = 0; byte < 18; byte++)
							{
								USART_SpiTransfer(USART0, tempLine[((rowPointer - rowPointerPlaceholder) * 18) + byte]);
							}
						}

						// Clear out tempLine
						for (uint16 i = 0; i < 918; i++)
						{
							tempLine[i] = 0xFF;
						}
					}

					// Blank Lines
					for (; rowPointer < 34 + verticalOffset; rowPointer++)
					{
						blankRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, blankRow[byte]);
						}
					}

					// Status Line
					for (uint8 i = 0; i < 14; i++)
					{
						statusIconMessage[i] = 0x00;
					}

					// Bluetooth Icon
					if ((StateMachine == smSLEEPING)
							|| (StateMachine == smCONNECTED))
					{
						statusIconMessage[0] = symCONNECTED;
					}
					else if (StateMachine == smADVERTISING)
					{
						statusIconMessage[0] = symON;
					}
					else if (StateMachine == smSTANDALONE)
					{
						statusIconMessage[0] = symBLANK;
					}
					else
					{
						statusIconMessage[0] = symBLANK;
					}

					statusIconMessage[1] = symBLANK;

					// Error Icon
					if (StatusRegister[1] > 0)
					{
						if (StatusRegister[1] & 0x08)											// Sleeping
						{
							statusIconMessage[2] = symSLEEP4;
						}
						else if ((StatusRegister[1] & 0x20) && !(StatusRegister[0] & 0x01))		// No calibration and TC mode
						{
							statusIconMessage[2] = symNO_CAL1;
						}
						else if (StatusRegister[1] & 0x10)										// Low battery
						{
							statusIconMessage[2] = symBATT_ERR;
						}
						else if ((StatusRegister[1] & 0x01) && !(StatusRegister[0] & 0x01))		// No probe and TC mode
						{
							statusIconMessage[2] = symNO_PROBE1;
						}
						else if ((StatusRegister[1] & 0x02) && !(StatusRegister[0] & 0x01))		// Low and TC mode
						{
							statusIconMessage[2] = symLOW;
						}
						else if ((StatusRegister[1] & 0x04) && !(StatusRegister[0] & 0x01))		// High and TC mode
						{
							statusIconMessage[2] = symHIGH;
						}
						else
						{
							statusIconMessage[2] = symBLANK;
						}
					}
					else
					{
						statusIconMessage[2] = symBLANK;
					}

					statusIconMessage[3] = symBLANK;


					// Temperature Scale Icon
					if (StatusRegister[0] & 0x02)		// Fahrenheit
					{
						statusIconMessage[4] = symFAHRENHEIT;
					}
					else								// Celsius
					{
						statusIconMessage[4] = symCELSIUS;
					}

					statusIconMessage[7] = 0x00;

					// Display the bluetooth string
					LoadStatusIconBuffer(tempLine, statusIconMessage, horizontalOffset);
					rowPointerPlaceholder = rowPointer;
					for (; rowPointer < 54 + verticalOffset; rowPointer++)
					{
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 0] = ReverseBits(rowPointer + 1);
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 17] = 0x00;
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, tempLine[((rowPointer - rowPointerPlaceholder) * 18) + byte]);
						}
					}

					// Clear out tempLine
					for (uint16 i = 0; i < 918; i++)
					{
						tempLine[i] = 0xFF;
					}

					// Blank Lines
					for (; rowPointer < 56 + verticalOffset; rowPointer++)
					{
						blankRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, blankRow[byte]);
						}
					}

					// Temp Line
					if (StateMachine == smSLEEPING)
					{
						sprintf(tempDisplayMessage, "!");
						useBigTemp = true;
					}
					else if (StatusRegister[1] & 0xBF)
					{
						sprintf(tempDisplayMessage, "!");
						useBigTemp = true;
					}
					else //if (StateMachine < smSTA_MMFL_XXX)
					{
						if (StillBooting)
						{
							sprintf(tempDisplayMessage, "--");
							useBigTemp = true;
							StillBooting = false;;
						}
						else
						{
							useBigTemp = true;
#ifdef TEST_DISP
							sprintf(tempDisplayMessage, TEST_DISP_STRING);
#else
							GenerateTemperatureDisplayString(tempDisplayMessage, &useBigTemp, data.val);
#endif //TEST_DISP
							GenerateTemperatureAsciiString(tempMessage, &useBigTemp, data.val);
						}
					}

					// Display the temperature string
					// Temp Line
					LoadBigDisplayBuffer(tempLine, tempDisplayMessage, horizontalOffset);
					rowPointerPlaceholder = rowPointer;
					for (; rowPointer < 107 + verticalOffset; rowPointer++)
					{
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 0] = ReverseBits(rowPointer + 1);
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 17] = 0x00;
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, tempLine[((rowPointer - rowPointerPlaceholder) * 18) + byte]);
						}
					}

					// Blank Lines
					for (; rowPointer < 107 + verticalOffset; rowPointer++)
					{
						blankRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, blankRow[byte]);
						}
					}

					// Clear out tempLine
					for (uint16 i = 0; i < 918; i++)
					{
						tempLine[i] = 0xFF;
					}

					// Battery Icon Line
					AddBatteryIcon(tempLine, battPercentage);
					rowPointerPlaceholder = rowPointer;
					for (; rowPointer < 115 + verticalOffset; rowPointer++)
					{
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 0] = ReverseBits(rowPointer + 1);
						tempLine[((rowPointer - rowPointerPlaceholder) * 18) + 17] = 0x00;
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, tempLine[((rowPointer - rowPointerPlaceholder) * 18) + byte]);
						}
					}

					// Clear out tempLine
					for (uint16 i = 0; i < 918; i++)
					{
						tempLine[i] = 0xFF;
					}

#ifdef BLACK_BORDERS
					// Blank Line
					for (; rowPointer < 116; rowPointer++)
					{
						blankRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, blankRow[byte]);
						}
					}

					// Bottom Border
					for (; rowPointer < 128; rowPointer++)
					{
						filledRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, filledRow[byte]);
						}
					}
#else
					// Blank Line
					for (; rowPointer < 128; rowPointer++)
					{
						blankRow[0] = ReverseBits(rowPointer + 1);
						for (uint8 byte = 0; byte < 18; byte++)
						{
							USART_SpiTransfer(USART0, blankRow[byte]);
						}
					}
#endif //BLACK_BORDERS

					USART_SpiTransfer(USART0, spiTrailer[0]);
					PowerDisplay(true);
					GPIO_PortOutSetVal(gpioPortA, dspCS_DESELECT, dspCS_MASK);

				}

				if (evt->data.evt_hardware_soft_timer.handle == thFLASH_OFF)
				{
					GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);
				}

				if (evt->data.evt_hardware_soft_timer.handle == thSLEEP_FLASH)
				{
					GPIO_PortOutSetVal(gpioPortD, ledYELLOW, ledMASK); // Flash the yellow LED
//					gecko_cmd_hardware_set_soft_timer(tmFLASH, thFLASH_OFF, 1);
					turnOffLEDs = 7;
				}

				if (evt->data.evt_hardware_soft_timer.handle == thSHUTDOWN_GPIO)
				{
					if (StateMachine == smOFF)
					{
						GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);				// Turn off LEDs
						GPIO_PortOutSetVal(gpioPortC, tcOFF, tcMASK);				// Turn off VMeasTC2.7
						PowerDisplay(false);										// Turn off display
						GPIO_PortOutSetVal(gpioPortA, regOFF, regMASK);				// Turn off VReg2.7
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thREENABLE_INTRPTS)
				{
					GPIO_IntClear(0x0018);
					GPIO_IntEnable(0x0018);
				}


				if (evt->data.evt_hardware_soft_timer.handle == thSWITCH)
				{

					// This switch statement waits (repeatedly checks) for
					// a short or long switch press on SW1, SW2, or both.
					switch(SwitchState)
					{
					case ssBOTH_PRESSED:
						if (++switchTimerCounts > splTIME)
						{
							// Long press is true
							switchPressLength = splLONG;
							break;
						}
						if ((GPIO_PinInGet(gpioPortA, pinSWITCH1) == 0) && (GPIO_PinInGet(gpioPortA, pinSWITCH2) == 0))
						{
							// Long press not yet true, but switches are still held down.
							// Keep waiting.
							gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thSWITCH, 1);
						}
						else
						{
							// Short press is true
							switchPressLength = splSHORT;
						}
						break;
					case ssSW1_PRESSED:
						// Special case for immediate response when turning on unit
						// If the unit is off, don't check for long press or dual
						// button press, just register as splSHORT on the down
						// stroke and break;
						if (StateMachine == smOFF)
						{
							switchPressLength = splSHORT;
							break;
						}
						if (GPIO_PinInGet(gpioPortA, pinSWITCH2) == 0)
						{
							// Debounce Switch 2
							uint8 lastSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH2);
							uint8 currentSwitchState;
							uint16 consecutiveSwitchStates = 0;
							while (consecutiveSwitchStates < DEBOUNCE)
							{
								timer_WaitUs(100);
								currentSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH2);
								if (currentSwitchState == lastSwitchState)
								{
									consecutiveSwitchStates++;
								}
								else
								{
									lastSwitchState = currentSwitchState;
									consecutiveSwitchStates = 0;
								}
							}

							if (lastSwitchState == 0)													// both switches are held down
							{
								SwitchState = ssBOTH_PRESSED;
								switchTimerCounts++;
								gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thSWITCH, 1);
								break;
							}
						}
						if (++switchTimerCounts > splTIME)
						{
							// Long press is true
							switchPressLength = splLONG;
							break;
						}
						if (GPIO_PinInGet(gpioPortA, pinSWITCH1) == 0)
						{
							// Long press not yet true, but switch is still held down.
							// Keep waiting.
							gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thSWITCH, 1);
							break;
						}
						else
						{
							// Short press is true
							switchPressLength = splSHORT;
							break;
						}
						break;
					case ssSW2_PRESSED:
						if (GPIO_PinInGet(gpioPortA, pinSWITCH1) == 0)
						{
							// Debounce Switch 1
							uint8 lastSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH1);
							uint8 currentSwitchState;
							uint16 consecutiveSwitchStates = 0;
							while (consecutiveSwitchStates < DEBOUNCE)
							{
								timer_WaitUs(100);
								currentSwitchState = GPIO_PinInGet(gpioPortA, pinSWITCH1);
								if (currentSwitchState == lastSwitchState)
								{
									consecutiveSwitchStates++;
								}
								else
								{
									lastSwitchState = currentSwitchState;
									consecutiveSwitchStates = 0;
								}
							}

							if (lastSwitchState == 0)													// both switches are held down
							{
								SwitchState = ssBOTH_PRESSED;
								switchTimerCounts++;
								gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thSWITCH, 1);
								break;
							}
						}
						if (++switchTimerCounts > splTIME)
						{
							// Long press is true
							switchPressLength = splLONG;
							break;
						}
						if (GPIO_PinInGet(gpioPortA, pinSWITCH2) == 0)
						{
							// Long press not yet true, but switch is still held down.
							// Keep waiting.
							gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thSWITCH, 1);
						}
						else
						{
							// Short press is true
							switchPressLength = splSHORT;
							break;
						}
						break;
					case ssNONE_PRESSED:
						switchTimerCounts = 0;
						switchPressLength = splUNDETERMINED;
#ifdef INCLUDE_IR
						ConsecutiveScrolls = 0;
#endif //INCLUDE_IR

						// Re-enable switch interrupts
//						GPIO_IntEnable(0x0180);
//						GPIO_IntClear(0x0180);
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);
						break;
					default:
						SwitchState = ssNONE_PRESSED;
						switchPressLength = splUNDETERMINED;
						switchTimerCounts = 0;

						// Re-enable switch interrupts
//						GPIO_IntEnable(0x0180);
//						GPIO_IntClear(0x0180);
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);
						break;
					}

					// This switch statement handles the short or long
					// switch press on SW1, SW2, or both, which was
					// determined in the above switch statement.
					switch (switchPressLength)
					{
					case splSHORT:
						switch (SwitchState)
						{
						case ssSW1_PRESSED:
							switch (StateMachine)
							{
							case smOFF:
								StateMachine = smADVERTISING;
								StillBooting = true;

								gecko_cmd_le_gap_set_adv_data(0, 22 + DEVICE_NAME_LEN, AdvertisementData);
								gecko_cmd_le_gap_set_adv_data(1, 18, ScanResponseData);
								gecko_cmd_le_gap_set_mode(le_gap_user_data, le_gap_undirected_connectable); // Start advertising

								UpdateBattery(&battPercentage, ConnectionHandle);
								advertisementTimer = 0;
								gecko_cmd_hardware_set_soft_timer(tmSECOND, thADVERTISEMENT, 0); 		// Start advertisement timer

								GPIO_PortOutSetVal(gpioPortA, regON, regMASK);							// Turn on VReg2.7
								gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);			// Turn on Display

								BuzzerProfile = bpLOW_HIGH;
								buzzerStep1 = 1;

								TakeImmediateTemperature(&resettingMeasTimer);
								StartUpTasks();

								// Refresh the serial number, fw, hw, and device name
								WriteBasicInfoToGatt(SerialNumber, SerialNumberString, GattString);

								// Remove this line to restore serial number string
								// sprintf((char*)SerialNumberString, "%d:%d", offset, gain);

								break;
							case smCONNECTED:
								break;
							case smADVERTISING:
								StopAllTimers();
								StateMachine = smSTANDALONE;
								gecko_cmd_hardware_set_soft_timer(0, thADVERTISEMENT, 1); 		// Stop advertisement timer
								advertisementTimer = 0xFF;
								gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_non_connectable);
								gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);				// Update the display
								TakeImmediateTemperature(&resettingMeasTimer);
								break;
							case smSTANDALONE:
								StopAllTimers();															// Start advertising
								StateMachine = smADVERTISING;

								gecko_cmd_le_gap_set_adv_data(0, 22 + DEVICE_NAME_LEN, AdvertisementData);
								gecko_cmd_le_gap_set_adv_data(1, 18, ScanResponseData);
								gecko_cmd_le_gap_set_mode(le_gap_user_data, le_gap_undirected_connectable); // Start advertising

								advertisementTimer = 0;
//								gecko_cmd_hardware_set_soft_timer(0, thAUTO_SLEEP, 1);					// Stop the auto-sleep timer
								gecko_cmd_hardware_set_soft_timer(tmSECOND, thADVERTISEMENT, 0); 		// Start advertisement timer
								GPIO_PortOutSetVal(gpioPortD, ledGREEN, ledMASK); 						// Flash the green LED
								gecko_cmd_hardware_set_soft_timer(tmFLASH, thFLASH_OFF, 1);
								gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);				// Update the display
								TakeImmediateTemperature(&resettingMeasTimer);
								break;
							case smSLEEPING:
								StateMachine = smCONNECTED;
								break;
							default:
								break;
							}
							ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);
							break;
						case ssSW2_PRESSED:
							switch (StateMachine)
							{
							case smSLEEPING:
								StateMachine = smCONNECTED;
								break;
							case smSTANDALONE:
								break;
							case smADVERTISING:
								break;
							case smCONNECTED:
								// Set the record flag and send the temperature one time
								StatusRegister[0] = StatusRegister[0] | 0x04;							// Set the Record flag
								TemperatureData[4] = StatusRegister[0];									// Set the status flags
								TemperatureData[5] = StatusRegister[1];									// Set the error flags
								gecko_cmd_gatt_server_write_attribute_value(gattdb_temperature_reading_characteristic, 0, 6, TemperatureData);		// Write to GATT database
								gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_temperature_reading_characteristic, 6, TemperatureData);	// Send notification

								// For legacy Blue2 support
								// Send the 'S' with TempAsAscii characteristic
								GenerateTemperatureAsciiString(tempMessage, &useBigTemp, data.val);
								for (length = 0; length < 13; length++)
								{
									if (tempMessage[length] == '\0')
									{
										break;
									}
								}
								tempMessage[length] = 'S';
								tempMessage[length + 1] = '\0';
								length++;
								gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_temperature_as_ascii, 0, length, (uint8*)tempMessage);	// Write to GATT database
								gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_temperature_as_ascii, length, (uint8*)tempMessage);		// Send notification

								StatusRegister[0] = StatusRegister[0] & 0xFB;							// Clear the Record flag
								TemperatureData[4] = StatusRegister[0];									// Set the status flags

								break;
							}
							ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);
							break;
						case ssBOTH_PRESSED:
							break;
						case ssNONE_PRESSED:
//#ifdef INCLUDE_IR
//							ConsecutiveScrolls = 0;
//#endif //INCLUDE_IR
							break;
						default:
							break;
						}

						// Reset switch variables
						SwitchState = ssNONE_PRESSED;
						switchPressLength = splUNDETERMINED;
						switchTimerCounts = 0;
						// Re-enable switch interrupts
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);
						break;
					case splLONG:
						BuzzerProfile = bpCHIP;
						buzzerStep1 = 1;
						switch (SwitchState)
						{
						case ssSW1_PRESSED:
							if ((StateMachine == smCONNECTED)
								|| (StateMachine == smSLEEPING))
							{
								gecko_cmd_endpoint_close(ConnectionHandle);
							}
							ShutDownTasks(&BuzzerProfile, &ConnectionHandle, &StateMachine, &advertisementTimer, &autoSleepTimer);

							break;
						case ssSW2_PRESSED:
							if (StateMachine != smOFF)
							{
								if (DisplayScale == 1)										// Fahrenheit
								{
									DisplayScale = 0;
									StatusRegister[0] = StatusRegister[0] & 0xFD;
									dataToWrite[0] = 0x00;
									gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, dataToWrite);
								}
								else														// Celsius
								{
									DisplayScale = 1;
									StatusRegister[0] = StatusRegister[0] | 0x02;
									dataToWrite[0] = 0x01;
									gecko_cmd_flash_ps_save(psDISPLAY_SCALE, 1, dataToWrite);
								}

								if (StateMachine == smCONNECTED)
								{
									// Write the display scale setting value to the GATT.
									gecko_cmd_gatt_server_write_attribute_value(gattdb_display_scale_characteristic, 0, 1, dataToWrite);				// Update the GATT database
									gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_display_scale_characteristic, 1, dataToWrite);	// Send notification

									// For legacy Blue2 support
									// Measurement Scale
									if (DisplayScale == 1)				// Fahrenheit
									{
										// Write the display scale setting value to the GATT.
										gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "F");				// Update the GATT database
										gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "F");	// Send notification
									}
									else								// Celsius
									{
										// Write the display scale setting value to the GATT.
										gecko_cmd_gatt_server_write_attribute_value(gattdb_legacy_measurement_scale, 0, 1, "C");				// Update the GATT database
										gecko_cmd_gatt_server_send_characteristic_notification(ConnectionHandle, gattdb_legacy_measurement_scale, 1, "C");	// Send notification
									}
								}

								//Update the display
								gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thDISPLAY, 1);
								ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);
							}
							break;
						case ssBOTH_PRESSED:
							ResetAutoSleep(&autoSleepTimer, &sleepingTimer, measurementInterval);
						case ssNONE_PRESSED:
							break;
						default:
							break;
						}

						// Reset switch variables
						SwitchState = ssNONE_PRESSED;
						switchPressLength = splUNDETERMINED;
						switchTimerCounts = 0;
						// Re-enable switch interrupts
						GPIO_IntClear(0x0018);
						GPIO_IntEnable(0x0018);
						break;
					case splUNDETERMINED:
						break;
					default:
						break;
					}
				}

				if (evt->data.evt_hardware_soft_timer.handle == thREAD_CAL_VALUES)
				{
					// Read calibration values
					seq.flags = I2C_FLAG_WRITE;
					seq.addr = 0xA4;
					i2c_write_buffer[0] = eeOFFSET;
					seq.buf[0].data = i2c_write_buffer;
					seq.buf[0].len = 1;
					ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
					while (ret == i2cTransferInProgress)
					{
						ret = I2C_Transfer(I2C0);
					}

					for (uint8 i = 0; i < 100; i++); // Short delay

					seq.flags = I2C_FLAG_READ;
					seq.addr = 0xA4;
					seq.buf[0].data = i2c_read_buffer;
					seq.buf[0].len = 9;
					ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
					while (ret == i2cTransferInProgress)
					{
						ret = I2C_Transfer(I2C0);
					}
					if (i2c_read_buffer[eeCALIBRATED] == eeCAL_VALID)
					{
						offset = i2c_read_buffer[eeOFFSET];
						offset <<= 8;
						offset += i2c_read_buffer[eeOFFSET + 1];
						offset <<= 8;
						offset += i2c_read_buffer[eeOFFSET + 2];
						offset <<= 8;
						offset += i2c_read_buffer[eeOFFSET + 3];
						gain = i2c_read_buffer[eeGAIN];
						gain <<= 8;
						gain += i2c_read_buffer[eeGAIN + 1];
						gain <<= 8;
						gain += i2c_read_buffer[eeGAIN + 2];
						gain <<= 8;
						gain += i2c_read_buffer[eeGAIN + 3];
						StatusRegister[1] &= 0xDF;											// Clear "No Calibration" error flag
					}
					else
					{
						StatusRegister[1] |= 0x20;											// Set "No Calibration" error flag
					}
#ifdef INCLUDE_IR
					gecko_cmd_hardware_set_soft_timer(tmQUARTERSECOND, thWAIT_TO_GET_EMISS, 1);
#else
					GPIO_PortOutSetVal(gpioPortA, regOFF, regMASK);							// Turn off VReg2.7
					gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thBOOT_BEEP_START, 1);		// Start buzzer profile
#endif //INCLUDE_IR
				}

				if (evt->data.evt_hardware_soft_timer.handle == thBOOT_BEEP_START)
				{
					BuzzerStart(true, 7);
					gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thBOOT_BEEP_STOP, 1);
				}

				if (evt->data.evt_hardware_soft_timer.handle == thBOOT_BEEP_STOP)
				{
					BuzzerStart(false, 0);
					gecko_cmd_hardware_set_soft_timer(tmTENTHSECOND, thENABLE_INTRPTS, 1);
				}

				if (evt->data.evt_hardware_soft_timer.handle == thENABLE_INTRPTS)
				{
					// It is now safe to enable switch interrupts
					// Setup Switch Interrupts
					GPIO_IntDisable(0x0018);
					GPIO_ExtIntConfig(gpioPortA, 3, 3, false, true, true);
					GPIO_ExtIntConfig(gpioPortA, 4, 4, false, true, true);
					NVIC_EnableIRQ(GPIO_ODD_IRQn);
					NVIC_EnableIRQ(GPIO_EVEN_IRQn);
					GPIO_IntEnable(0x0018);
				}

				if (evt->data.evt_hardware_soft_timer.handle == thUPDATE_BATTERY)
				{
					UpdateBattery(&battPercentage, ConnectionHandle);
				}

				break;

			default:
				break;
		}
	}
}
//---------------------------------------------------------------------------

void StopAllTimers()
{
	for (uint8 i = thMIN_HANDLE; i <= thMAX_HANDLE; i++)
	{
		gecko_cmd_hardware_set_soft_timer(0, i, 1);
	}
}
//---------------------------------------------------------------------------

void ResetAutoSleep(uint8* AutoSleepTimer, uint8* SleepingTimer, uint8 MeasurementInterval)
{
	if (StateMachine != smOFF)
	{
		gecko_cmd_hardware_set_soft_timer(0, thAUTO_SLEEP, 1); 						// Stop Auto sleep timer
		gecko_cmd_hardware_set_soft_timer(0, thSLEEP_TIME, 1); 						// Stop Sleep timer
		*AutoSleepTimer = 0; 														// reset autoSleepTimer count
		*SleepingTimer = 0;															// reset sleepingTimer count
		gecko_cmd_hardware_set_soft_timer(tmMINUTE, thAUTO_SLEEP, 0); 				// Start a repeating one-minute auto-sleep timer
		if (StatusRegister[1] & 0x08)
		{
			StatusRegister[1] = StatusRegister[1] & 0xF7;							// Clear the sleeping flag
			gecko_cmd_hardware_set_soft_timer(0, thSLEEP_FLASH, 1);					// stop the sleep flash timer
		#ifdef TEST_ROUNDING
			gecko_cmd_hardware_set_soft_timer(tmSECOND * 4, thMEASUREMENT, 0);
		#else
		#endif // TEST_ROUNDING
			gecko_cmd_hardware_set_soft_timer(tmSECOND * MeasurementInterval, thMEASUREMENT, 0);
		}
	}
}
//---------------------------------------------------------------------------

void CalculateTemperature(int32* Temperature, uint8* Reading, int32 TEVolts, int16 Offset, int16 Gain)
{
	uint16 adcCounts;
	int32 work;
	int16 countLower, countUpper;
#ifdef INCLUDE_IR
	uint16 irReading;

	if (StatusRegister[0] & 0x01)										// if IR
	{
		irReading = Reading[1];
		irReading <<= 8;
		irReading += Reading[0];
		*Temperature = (irReading * 2) - 27315;
	}
	else
	{
#endif //INCLUDE_IR
		// Reverse the byte order of TC Reading
		adcCounts = Reading[0];
		adcCounts <<= 8;
		adcCounts += Reading[1];

		// Convert thermistor equivalent TEVolts to ADC counts
		work = TEVolts;
		work *= 968615;
		work /= 100000;
		work += 5;  							// Round up
		work /= 10; // work is now ADC (equivalent) counts of CJC (s/b approx. 968 @ 77°F)

		// Add CJC (equivalent) counts to TC counts
		work += adcCounts;

		// Subtract the offset
		work -= Offset;

		// Apply the  gain
		work *= 10000;
		if (Gain != 0)
		{
			work /= Gain;
		}

		// Convert counts to uV
		// these are magic numbers from Blue2 code
		work *= 10000;
		work /= 16;
		work /= 605;

		// work is now the temperature compensated thermoelectric microvolts of the hot junction

		// Find where in the TC lookup table work falls
		uint8 tcIndex = 0;
		for (uint8 i = 1; i < 64; i++)
		{
			if (TCLookupTable[i] > work)
			{
				countUpper = TCLookupTable[i];
				countLower = TCLookupTable[i - 1];
				tcIndex = i;
				i = 64;
			}
		}

		// Calculate the temperature in Celsius (x100)
		// Table values are every 10 degC, so interpolate between countUpper and countLower
		*Temperature = tcIndex;
		*Temperature -= 1;
		*Temperature *= 10;
		*Temperature -= 80;
		*Temperature *= 100;

		work -= countLower;
		work *= 1000;
		work /= (countUpper - countLower);
		*Temperature += work;
#ifdef INCLUDE_IR
	}
#endif //INCLUDE_IR
}
//---------------------------------------------------------------------------

void GetThermistorTEVolts(uint8* TMReading, int32* Temperature, int32* TEVolts)
{
	uint16 adcCounts;
	int16 countLower, countUpper;
	int32 work;
	int32 tens, ones, tenths;

	// Reverse the byte order of TM_reading()
	adcCounts = TMReading[0];
	adcCounts <<= 8;
	adcCounts += TMReading[1];

	// Find where in the TM lookup table ADCCounts falls
	uint8 tmIndex = 81;
	for (uint8 i = 0; i < 81; i++)
	{
		if (TMLookupTable[i] > adcCounts)
		{
			tmIndex = i;
			break;
		}
	}

	if (tmIndex == 81)
	{
		*Temperature = -25000;						// Cap at -5degC
	}
	else if (tmIndex == 0)
	{
		*Temperature = 55000;					// Cap at 55degC (x1000)
	}
	else
	{
		// Calculate the temperature
		countLower = TMLookupTable[tmIndex - 1];
		countUpper = TMLookupTable[tmIndex];
		work = countUpper - adcCounts;
		countUpper = countUpper - countLower;
		*Temperature = (1000 / countUpper) * work;
		work = (55 - tmIndex) * 1000;
		*Temperature += work; 					// Temperature is now the thermistor temp in Celsius x 1000
	}

	// Determine TEVolts due to cold junction
	// Extract decimals from integer
	work = *Temperature / 100; 						// work is thermistor temp in Celsius X10
	tens = (work / 100);							// 2X.X
	ones = (work / 10) - (tens * 10);				// X4.X
	tenths = (work - (tens * 100)) - (ones * 10);	// XX.8

	tmIndex = tens + 8;
	countLower = TCLookupTable[tmIndex];
	countUpper = TCLookupTable[tmIndex + 1];
	countUpper -= countLower;
	work = (ones * 10) + tenths;
	*TEVolts = (countUpper * work) / 100;
	*TEVolts += countLower;
}
//---------------------------------------------------------------------------

void SetOffset(int32 TEVolts, uint8* TCReading, int32* Offset, uint8 * stateMachine)
{
	uint32 work;
	uint16 adcCounts;
	uint8 i2c_write_buffer[5];

	// Turn VMEAS back on for the write operation
	GPIO_PortOutSetVal(gpioPortC, tcON, tcMASK);							// Turn on VMeasTC2.7

	// Convert TEVolts to A2D counts..
	work = TEVolts;
	work *= 968615;
	work /= 100000;
	work += 5;  									// Round up (ambient operating range is >= 32°F, so we never need to round down)
	work /= 10;										// work is now A2D counts of CJC; (approx. 968 @ 77°F)

	// Reverse the byte order of TM_reading()
	adcCounts = TCReading[0];
	adcCounts <<= 8;
	adcCounts += TCReading[1];
	*Offset = adcCounts;

	*Offset += work;

	// We need a slight delay here (?)
	for (uint8 bytes = 0; bytes < 5; bytes++)
	{
		for (uint8 i = 0; i < 20; i++)
		{
			timer_WaitUs(250);
		}
	}

	// Write Offset to EEPROM
	i2c_write_buffer[0] = eeOFFSET;
	i2c_write_buffer[1] = (uint8)((*Offset) >> 24);
	i2c_write_buffer[2] = (uint8)((*Offset) >> 16);
	i2c_write_buffer[3] = (uint8)((*Offset) >> 8);
	i2c_write_buffer[4] = (uint8)(*Offset);
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef seq;
	seq.flags = I2C_FLAG_WRITE;
	seq.addr = 0xA4;
	seq.buf[0].len = 5;
	seq.buf[0].data = i2c_write_buffer;
	ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
		if (*stateMachine == smOFF)
		{
			break;
		}
	}

	// Turn VMEAS back off after the write operation
	GPIO_PortOutSetVal(gpioPortC, tcOFF, tcMASK);							// Turn off VMeasTC2.7

	GPIO_PortOutSetVal(gpioPortD, ledRED, ledMASK); 						// Turn on red LED
	StatusRegister[0] = StatusRegister[0] | 0x10;							// Change the calibration step to 'Gain'
	gecko_cmd_hardware_set_soft_timer(tmSECOND * 10, thMEASUREMENT, 1);		// 10 second timer to allow operator to set Ectron to 842
}
//---------------------------------------------------------------------------

void SetGain(int32 TEVolts, uint8* TCReading, int32* Gain, int32 Offset, uint8 MeasurementInterval, uint8 * stateMachine)
{
	int32 work;
	uint16 adcCounts;
	uint8 i2c_write_buffer[5];

	// Turn VMEAS back on for the write operation
	GPIO_PortOutSetVal(gpioPortC, tcON, tcMASK);							// Turn on VMeasTC2.7

	// Convert TEVolts to A2D counts..
	work = TEVolts;															// work is approx. 964uV @ 75.5°F..
	work *= 968615;
	work /= 100000;
	work += 5; 																// Round up (ambient operating range is >= 32°F, so we never need to round down)
	work /= 10;																// work is now A2D counts of CJC; (approx. 968 @ 77°F)

	// Reverse the byte order of TC_reading()
	adcCounts = TCReading[0];
	adcCounts <<= 8;
	adcCounts += TCReading[1];
	*Gain = adcCounts;

	*Gain += work;
	*Gain -= Offset; 														// systemOffset is approx. 6500 counts
	// Now SystemGain is approx. 18551 A2D counts

	*Gain *= 10000;
	*Gain /= 17931; 														// A2D counts
	*Gain += 6;																// Fudge

	// We need a slight delay here (?)
	for (uint8 bytes = 0; bytes < 5; bytes++)
	{
		for (uint8 i = 0; i < 20; i++)
		{
			timer_WaitUs(250);
		}
	}

	// Write Gain to EEPROM
	i2c_write_buffer[0] = eeGAIN;
	i2c_write_buffer[1] = (uint8)((*Gain) >> 24);
	i2c_write_buffer[2] = (uint8)((*Gain) >> 16);
	i2c_write_buffer[3] = (uint8)((*Gain) >> 8);
	i2c_write_buffer[4] = (uint8)(*Gain);
	I2C_TransferReturn_TypeDef ret;
	I2C_TransferSeq_TypeDef seq;
	seq.flags = I2C_FLAG_WRITE;
	seq.addr = 0xA4;
	seq.buf[0].len = 5;
	seq.buf[0].data = i2c_write_buffer;
	ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
		if (*stateMachine == smOFF)
		{
			break;
		}
	}

	// We need a slight delay here (?)
	for (uint8 bytes = 0; bytes < 5; bytes++)
	{
		for (uint8 i = 0; i < 20; i++)
		{
			timer_WaitUs(250);
		}
	}

	// Write $A5 to EEPROM to indicate system is calibrated.
	i2c_write_buffer[0] = eeCALIBRATED;
	i2c_write_buffer[1] = eeCAL_VALID;
	seq.flags = I2C_FLAG_WRITE;
	seq.addr = 0xA4;
	seq.buf[0].len = 2;
	seq.buf[0].data = i2c_write_buffer;
	ret = I2C_TransferInit (I2C0, &seq);									// Start the I2C transfer
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
		if (*stateMachine == smOFF)
		{
			break;
		}
	}

	// Turn VMEAS back off after the write operation
	GPIO_PortOutSetVal(gpioPortC, tcOFF, tcMASK);							// Turn off VMeasTC2.7

	GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK); 						// Turn off red LED
	StatusRegister[0] = StatusRegister[0] & 0xE7;							// Change the calibration step to 'Offset' and change calibration mode to 'Measurement'
	StatusRegister[1] = StatusRegister[1] & 0xDF;							// Clear the "No Calibration" error flag
	gecko_cmd_hardware_set_soft_timer(tmSECOND * MeasurementInterval, thMEASUREMENT, 0);		// Restart measurement timer
}
//---------------------------------------------------------------------------

void UpdateBattery(uint8* battPercentage, uint8 connectionHandle)
{
//	uint32 adcBattCntsTrials[4];
//	uint8 minIndex,  maxIndex;
	uint32 adcBattCnts;

	GPIO_PortOutSetVal(gpioPortB, batON, batMASK);

	ADC_Start(ADC0, adcStartSingle);
	while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
	adcBattCnts =  ADC_DataSingleGet(ADC0);

//	// Take four readings
//	ADC_Start(ADC0, adcStartSingle);
//	while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//	adcBattCntsTrials[0] =  ADC_DataSingleGet(ADC0);
//
//	timer_WaitUs(250);
//
//	ADC_Start(ADC0, adcStartSingle);
//	while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//	adcBattCntsTrials[1] =  ADC_DataSingleGet(ADC0);
//
//	timer_WaitUs(250);
//
//	ADC_Start(ADC0, adcStartSingle);
//	while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//	adcBattCntsTrials[2] =  ADC_DataSingleGet(ADC0);
//
//	timer_WaitUs(250);
//
//	ADC_Start(ADC0, adcStartSingle);
//	while ( ( ADC0->STATUS & ADC_STATUS_SINGLEDV ) == 0 ){}
//	adcBattCntsTrials[3] =  ADC_DataSingleGet(ADC0);
//
//	// Find the min and max
//	minIndex = 0;
//	maxIndex = 0;
//
//	for (uint8 i = 1; i < 4; i++)
//	{
//		if (adcBattCntsTrials[i] < adcBattCntsTrials[minIndex])
//		{
//			minIndex = i;
//		}
//		if (adcBattCntsTrials[i] > adcBattCntsTrials[maxIndex])
//		{
//			maxIndex = i;
//		}
//	}
//
//	// Discard the min and max and average the remaining two readings
//	for (uint8 i = 0; i < 4; i++)
//	{
//		if ((i == minIndex) || (i == maxIndex))
//		{
//			continue;
//		}
//		adcBattCnts = adcBattCntsTrials[i] / 2;
//	}

	GPIO_PortOutSetVal(gpioPortB, batOFF, batMASK);

	if (adcBattCnts > 2457)													// 1.5V, 100%
	{
		adcBattCnts = 2457;
	}
	else if (adcBattCnts < 1638)											// 1.0V, 0%
	{
		adcBattCnts = 1638;
	}

	*battPercentage = ((adcBattCnts - 1638) * 100) / 819;
	gecko_cmd_gatt_server_write_attribute_value(gattdb_battery_level, 0, 1, battPercentage);	// Write to GATT database
	gecko_cmd_gatt_server_send_characteristic_notification(connectionHandle, gattdb_battery_level, 1, battPercentage);	// Send notification
}
//---------------------------------------------------------------------------

#ifdef INCLUDE_IR
uint8 CalculateCRC(uint8 len, uint8* message)
{
	uint8 crc = 0;

	// The CRC polynomial is X^8 + x^2 + X + 1 (from the MLX90614 datasheet)
	// which equates to a binary generator of 0b 1 0000 0111.
	// The msb is always 1, so we can drop that and use the least significant
	// 8 bits (0x07) as the polynomial.
#define POLYNOMIAL 0x07

	for (uint8 byteNum = 0; byteNum < len; byteNum++)
	{
		crc ^= message[byteNum];
		for (uint8 bitNum = 8; bitNum > 0; bitNum--)
		{
			if (crc & 0x80)
			{
				crc <<= 1;
				crc ^= POLYNOMIAL;
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc;
}
//---------------------------------------------------------------------------
#endif //INCLUDE_IR

uint8 ReverseBits(uint8 b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
//---------------------------------------------------------------------------

//void LoadDisplayBuffer(uint8 *buffer, char message[14])
//{
//	uint8 pixelPosition = 0;						// Used to compress unused space in two-byte characters
//	uint8 totalSpace = 0;							// Used to count total space used by message so we can center it
//	uint8 byteToCopy1, byteToCopy2;
//
//	RemapBufferAndCountPixels(message, 14, &totalSpace);
//
//	pixelPosition = (127 - totalSpace) / 2;			// Left margin
//
//	for (int charNum = 0; charNum < 14; charNum++)
//	{
//		for (uint8 rowNum = 0; rowNum < 20; rowNum++)
//		{
//			// Read the two bytes for this character out of display2.h array
//			byteToCopy1 = characters[message[charNum]][rowNum * 2];
//			byteToCopy2 = characters[message[charNum]][rowNum * 2 + 1];
//
//			// We have to NOT (~) the character bytes because  they write white-on-black instead of black-on-white.
//			// We also have to shift the bits into preceding bytes in order to use up the blank bits.  Otherwise
//			// The characters are way too  s p a c e d   o u t .
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~((byteToCopy1 << (8 - (pixelPosition % 8))) + (byteToCopy2 >> (pixelPosition % 8)));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 2] = ~(byteToCopy2 << (8 - (pixelPosition % 8)));
//		}
//		// After each character, update the pixel position
//		pixelPosition += widths[message[charNum]];
//	}
//
//#ifdef BLACK_BORDERS
//	//  Add black borders
//	for (int rowNum = 0; rowNum < 20; rowNum++)
//	{
//		buffer[1 + (rowNum * 18)] = 0x00;
//		buffer[2 + (rowNum * 18)] &= 0x1F;
//		buffer[15 + (rowNum * 18)] &= 0xE0;
//		buffer[16 + (rowNum * 18)] = 0x00;
//	}
//#endif //BLACK_BORDERS
//}
//---------------------------------------------------------------------------

void LoadStatusIconBuffer(uint8 *buffer, char message[14], int8 horzOffset)
{
	uint8 pixelPosition = 0;						// Used to compress unused space in two-byte characters
	uint8 totalSpace = 0;							// Used to count total space used by message so we can center it
	uint8 byteToCopy1, byteToCopy2;
	uint8 numChars = 0;

	for (numChars = 0; numChars < 14; numChars++)
	{
		if (message[numChars] == 0x00)
			break;
		totalSpace += symbolWidths[message[numChars]];
	}

	pixelPosition = ((127 - totalSpace) / 2) + horzOffset;			// Left margin
	for (uint8 charNum = 0; charNum < numChars; charNum++)
	{
		for (uint8 rowNum = 0; rowNum < 20; rowNum++)
		{
			// Read the two bytes for this character out of display2.h array
			byteToCopy1 = symbols[message[charNum]][rowNum * 2];
			byteToCopy2 = symbols[message[charNum]][rowNum * 2 + 1];

			// NOT (~) the character bytes and apply kearning
			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~((byteToCopy1 << (8 - (pixelPosition % 8))) + (byteToCopy2 >> (pixelPosition % 8)));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 2] = ~(byteToCopy2 << (8 - (pixelPosition % 8)));
		}

		pixelPosition += symbolWidths[message[charNum]];
	}

#ifdef BLACK_BORDERS
	//  Add black borders
	for (int rowNum = 0; rowNum < 20; rowNum++)
	{
		buffer[1 + (rowNum * 18)] = 0x00;
		buffer[2 + (rowNum * 18)] &= 0x1F;
		buffer[15 + (rowNum * 18)] &= 0xE0;
		buffer[16 + (rowNum * 18)] = 0x00;
	}
#endif //BLACK_BORDERS
}
//---------------------------------------------------------------------------

void AddBatteryIcon(uint8 *buffer, uint8 battPercentage)
{
	uint8 rowNumber = 0;
	uint8 gaugeByte1, gaugeByte2;

	// Draw frame
	buffer[1 + (rowNumber * 18) + 3] = 0x80;
	buffer[1 + (rowNumber * 18) + 4] = 0x1F;

	rowNumber = 1;
	buffer[1 + (rowNumber * 18) +3] = 0x7F;
	buffer[1 + (rowNumber * 18) +4] = 0xEF;

	rowNumber = 2;
	buffer[1 + (rowNumber * 18) +3] = 0x7F;
	buffer[1 + (rowNumber * 18) +4] = 0xE3;

	rowNumber = 3;
	buffer[1 + (rowNumber * 18) +3] = 0x7F;
	buffer[1 + (rowNumber * 18) +4] = 0xE3;

	rowNumber = 4;
	buffer[1 + (rowNumber * 18) +3] = 0x7F;
	buffer[1 + (rowNumber * 18) +4] = 0xE3;

	rowNumber = 5;
	buffer[1 + (rowNumber * 18) +3] = 0x7F;
	buffer[1 + (rowNumber * 18) +4] = 0xEF;

	rowNumber = 6;
	buffer[1 + (rowNumber * 18) +3] = 0x80;
	buffer[1 + (rowNumber * 18) +4] = 0x1F;

	// Fill in gauge
	switch ((battPercentage + 7) / 12)
	{
	case 8:
		gaugeByte1 = 0xC0;
		gaugeByte2 = 0x3F;
		break;
	case 7:
		gaugeByte1 = 0xC0;
		gaugeByte2 = 0x7F;
		break;
	case 6:
		gaugeByte1 = 0xC0;
		gaugeByte2 = 0xFF;
		break;
	case 5:
		gaugeByte1 = 0xC1;
		gaugeByte2 = 0xFF;
		break;
	case 4:
		gaugeByte1 = 0xC3;
		gaugeByte2 = 0xFF;
		break;
	case 3:
		gaugeByte1 = 0xC7;
		gaugeByte2 = 0xFF;
		break;
	case 2:
		gaugeByte1 = 0xCF;
		gaugeByte2 = 0xFF;
		break;
	case 1:
		gaugeByte1 = 0xDF;
		gaugeByte2 = 0xFF;
		break;
	case 0:
		gaugeByte1 = 0xFF;
		gaugeByte2 = 0xFF;
		break;
	default:
		gaugeByte1 = 0xFF;
		gaugeByte2 = 0xFF;
		break;
	}

	for (uint8 rowNumber = 2; rowNumber < 5; rowNumber++)
	{
		buffer[1 + (rowNumber * 18) + 3] &= gaugeByte1;
		buffer[1 + (rowNumber * 18) + 4] &= gaugeByte2;
	}

#ifdef BLACK_BORDERS
	//  Add black borders
	for (int rowNum = 0; rowNum < 8; rowNum++)
	{
		buffer[1 + (rowNum * 18)] = 0x00;
		buffer[2 + (rowNum * 18)] &= 0x1F;
		buffer[15 + (rowNum * 18)] &= 0xE0;
		buffer[16 + (rowNum * 18)] = 0x00;
	}
#endif //BLACK_BORDERS
}
//---------------------------------------------------------------------------

void LoadBigDisplayBuffer(uint8 *buffer, char message[14], int8 horzOffset)
{
	uint8 pixelPosition = 0;						// Used to compress unused space in three-byte characters
	uint8 totalSpace = 0;							// Used to count total space used by message so we can center it
	uint8 byteToCopy1, byteToCopy2, byteToCopy3;
	bool noMoreCharacters = false;

	for (int charNum = 0; charNum < 14; charNum++)
	{
		if (message[charNum] == '\0')
		{
			noMoreCharacters = true;
		}

		if (noMoreCharacters)
		{
			message[charNum] = 0x00;
		}
		else
		{
			// Remap the minus symbol
			if (message[charNum] == 0x2D)
			{
				message[charNum] = 0x01;
			}

			// Remap the dot symbol
			else if (message[charNum] == 0x2E)
			{
				message[charNum] = 0x02;
			}

			// Remap the exclamation to the error symbol
			else if (message[charNum] == 0x21)
			{
				message[charNum] = 0x0D;
			}

			// Remap the letter 'o' to a skinny zero
			else if (message[charNum] == 0x6f)
			{
				message[charNum] = 0x0E;
			}

			// Remap the numbers
			else if ((message[charNum] >= 0x30) && (message[charNum] <= 0x39))
			{
				message[charNum] -= 0x2D;
			}

			// Use blank for everything else
			else
			{
				message[charNum] = 0x00;
			}
		}

		totalSpace += bigWidths[message[charNum]];
	}
	pixelPosition = ((127 - totalSpace) / 2) + horzOffset;

	for (int charNum = 0; charNum < 14; charNum++)
	{
		for (uint8 rowNum = 0; rowNum < 51; rowNum++)
		{
			// Read the three bytes for this character out of display2.h array
			byteToCopy1 = bigCharacters[message[charNum]][rowNum * 3];
			byteToCopy2 = bigCharacters[message[charNum]][rowNum * 3 + 1];
			byteToCopy3 = bigCharacters[message[charNum]][rowNum * 3 + 2];

			// We have to NOT (~) the character bytes.
			// We also have to shift the bits into preceding bytes in order to use up the blank bits.  Otherwise
			// The characters are way too  s p a c e d   o u t .
			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~((byteToCopy1 << (8 - (pixelPosition % 8))) + (byteToCopy2 >> (pixelPosition % 8)));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 2] = ~((byteToCopy2 << (8 - (pixelPosition % 8))) + (byteToCopy3 >> (pixelPosition % 8)));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 3] = ~(byteToCopy3 << (8 - (pixelPosition % 8)));
		}
		// After each character, update the pixel position
		pixelPosition += bigWidths[message[charNum]];
	}

#ifdef BLACK_BORDERS
	//  Add black borders
	for (int rowNum = 0; rowNum < 51; rowNum++)
	{
		buffer[1 + (rowNum * 18)] = 0x00;
		buffer[2 + (rowNum * 18)] &= 0x1F;
		buffer[15 + (rowNum * 18)] &= 0xE0;
		buffer[16 + (rowNum * 18)] = 0x00;
	}
#endif //BLACK_BORDERS
}
//---------------------------------------------------------------------------

void LoadMacDisplayBuffer(uint8 *buffer, char message[13], int8 horzOffset)
{
	uint8 pixelPosition = 0;						// Used to compress unused space in two-byte characters
	uint8 totalSpace = 0;							// Used to count total space used by message so we can center it
	uint8 byteToCopy1;

	RemapMacBufferAndCountPixels(message, 13, &totalSpace);

	pixelPosition = ((128 - totalSpace) / 2) + horzOffset;			// Left margin

	for (int charNum = 0; charNum < 17; charNum++)
	{
		for (uint8 rowNum = 0; rowNum < 10; rowNum++)
		{
			// Read the one byte for this character out of display2.h array
			byteToCopy1 = macCharacters[message[charNum]][rowNum];

			// We have to NOT (~) the character bytes because I am shamelessly copying the characters from the
			// Ramtex "Vardana_20x13.sym" font file and they write white-on-black instead of black-on-white.
			// We also have to shift the bits into preceding bytes in order to use up the blank bits.  Otherwise
			// The characters are way too  s p a c e d   o u t .
			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~(byteToCopy1 << (8 - (pixelPosition % 8)));
		}
		// After each character, update the pixel position
		pixelPosition += macWidths[message[charNum]];
	}

#ifdef BLACK_BORDERS
	//  Add black borders
	for (int rowNum = 0; rowNum < 10; rowNum++)
	{
		buffer[1 + (rowNum * 18)] = 0x00;
		buffer[2 + (rowNum * 18)] &= 0x1F;
		buffer[15 + (rowNum * 18)] &= 0xE0;
		buffer[16 + (rowNum * 18)] = 0x00;
	}
#endif //BLACK_BORDERS
}
//---------------------------------------------------------------------------

//void LoadBottomDisplayBuffer(uint8 *buffer, char leftMessage[6], char rightMessage[6])
//{
//	uint8 pixelPosition = 0;						// Used to compress unused space in two-byte characters
//	uint8 totalSpace = 0;							// Used to count total space used by message so we can center it
//	uint8 byteToCopy1, byteToCopy2;
//
//	// Left message
//	RemapBufferAndCountPixels(leftMessage, 6, &totalSpace);
//	pixelPosition = 5;
//	for (int charNum = 0; charNum < 6; charNum++)
//	{
//		for (uint8 rowNum = 0; rowNum < 20; rowNum++)
//		{
//			// Read the two bytes for this character out of display2.h array
//			byteToCopy1 = characters[leftMessage[charNum]][rowNum * 2];
//			byteToCopy2 = characters[leftMessage[charNum]][rowNum * 2 + 1];
//
//			// We have to NOT (~) the character bytes because I am shamelessly copying the characters from the
//			// Ramtex "narrow20.sym" font file and they write white-on-black instead of black-on-white.
//			// We also have to shift the bits into preceding bytes in order to use up the blank bits.  Otherwise
//			// The characters are way too  s p a c e d   o u t .
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~((byteToCopy1 << (8 - (pixelPosition % 8))) + (byteToCopy2 >> (pixelPosition % 8)));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 2] = ~(byteToCopy2 << (8 - (pixelPosition % 8)));
//		}
//		// After each character, update the pixel position
//		pixelPosition += widths[leftMessage[charNum]];
//	}
//
//	// Right message
//	RemapBufferAndCountPixels(rightMessage, 6, &totalSpace);
//	pixelPosition = 122 - totalSpace;
//	for (int charNum = 0; charNum < 6; charNum++)
//	{
//		for (uint8 rowNum = 0; rowNum < 20; rowNum++)
//		{
//			// Read the two bytes for this character out of display2.h array
//			byteToCopy1 = characters[rightMessage[charNum]][rowNum * 2];
//			byteToCopy2 = characters[rightMessage[charNum]][rowNum * 2 + 1];
//
//			// We have to NOT (~) the character bytes because I am shamelessly copying the characters from the
//			// Ramtex "narrow20.sym" font file and they write white-on-black instead of black-on-white.
//			// We also have to shift the bits into preceding bytes in order to use up the blank bits.  Otherwise
//			// The characters are way too  s p a c e d   o u t .
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8)] &= ~(byteToCopy1 >> (pixelPosition % 8));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 1] = ~((byteToCopy1 << (8 - (pixelPosition % 8))) + (byteToCopy2 >> (pixelPosition % 8)));
//			buffer[1 + (rowNum * 18) + (pixelPosition / 8) + 2] = ~(byteToCopy2 << (8 - (pixelPosition % 8)));
//		}
//		// After each character, update the pixel position
//		pixelPosition += widths[rightMessage[charNum]];
//	}
//
//#ifdef BLACK_BORDERS
//	//  Add black borders
//	for (int rowNum = 0; rowNum < 20; rowNum++)
//	{
//		buffer[1 + (rowNum * 18)] = 0x00;
//		buffer[2 + (rowNum * 18)] &= 0x1F;
//		buffer[15 + (rowNum * 18)] &= 0xE0;
//		buffer[16 + (rowNum * 18)] = 0x00;
//	}
//#endif //BLACK_BORDERS
//}
//---------------------------------------------------------------------------

void timer_WaitUs(uint8_t uDelay) {

  /* copied from http://community.silabs.com/t5/32-bit-MCU/Delay-in-microseconds/m-p/104321#M2443
   * adjustment factor for 14MHz oscillator, based on the timing of this whole function
   * with speed optimization on, could probably be done in a prettier way. */

  uint16_t cycle_delay = uDelay * 14 - 28;

  /* Reset Timer */

  TIMER_CounterSet(TIMER0, 0);

  /* Start TIMER0 */

  TIMER0->CMD = TIMER_CMD_START;

  /* Wait until counter value is over top */

  while(TIMER0->CNT < cycle_delay){

  /* Do nothing, just wait */

  }

  TIMER0->CMD = TIMER_CMD_STOP;

}
//---------------------------------------------------------------------------

void GPIO_ODD_IRQHandler()
{
	uint32_t flags = GPIO_IntGet();
	GPIO_IntClear(flags);
	GPIO_IntDisable(0x18);
	//Send gecko_evt_system_external_signal_id event to the main loop
	gecko_external_signal(flags);
}
//---------------------------------------------------------------------------

void GPIO_EVEN_IRQHandler()
{
	uint32_t flags = GPIO_IntGet();
	GPIO_IntClear(flags);
	GPIO_IntDisable(0x18);
	//Send gecko_evt_system_external_signal_id event to the main loop
	gecko_external_signal(flags);
}
//---------------------------------------------------------------------------

void CRYOTIMER_IRQHandler()
{
	CRYOTIMER_IntClear(CRYOTIMER_IFC_PERIOD);
	CRYOTIMER_IntDisable(CRYOTIMER_IEN_PERIOD);

	// Perform buzzer and LED actions
	if (turnOffLEDs > 0)
	{
		if (--turnOffLEDs == 0)
		{
			GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);
		}
	}

	if (buzzerStep1 > 0)
	{
		if (--buzzerStep1 == 0)
		{
			switch(BuzzerProfile)
			{
			case bpNONE:
				BuzzerStart(false, 0);
				break;
			case bpQUICK_BEEP:
				BuzzerStart(true, 7);
				buzzerStep2 = 13;
				break;
			case bpDOUBLE_BEEP:
				BuzzerStart(true, 7);
				buzzerStep2 = 13;
				break;
			case bpHIGH_LOW:
				BuzzerStart(true, 7);
				buzzerStep2 = 13;
				break;
			case bpLOW_HIGH:
				BuzzerStart(true, 7);
				buzzerStep2 = 13;
				break;
			case bpCONTINUOUS_BEEP:
				BuzzerStart(true, 7);
				buzzerStep2 = 125;
				break;
			case bpCHIP:
				BuzzerStart(true, 4);
				buzzerStep2 = 2;
				break;
			case bpLONG_BEEP:
				BuzzerStart(true, 7);
				buzzerStep2 = 32;
				break;
			default:
				BuzzerStart(false, 0);
				break;
			}
		}
	}

	if (buzzerStep2 > 0)
	{
		if (--buzzerStep2 == 0)
		{
			switch(BuzzerProfile)
			{
			case bpNONE:
				BuzzerStart(false, 0);
				break;
			case bpQUICK_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpDOUBLE_BEEP:
				BuzzerStart(false, 0);
				buzzerStep3 = 13;
				break;
			case bpHIGH_LOW:
				BuzzerStart(true, 32);
				buzzerStep3 = 13;
				break;
			case bpLOW_HIGH:
				BuzzerStart(true, 4);
				buzzerStep3 = 13;
				break;
			case bpCONTINUOUS_BEEP:
				BuzzerStart(false, 0);
				buzzerStep1 = 125;
				break;
			case bpCHIP:
				BuzzerStart(false, 0);
				break;
			case bpLONG_BEEP:
				BuzzerStart(false, 0);
				break;
			default:
				BuzzerStart(false, 0);
				break;
			}
		}
	}

	if (buzzerStep3 > 0)
	{
		if (--buzzerStep3 == 0)
		{
			switch(BuzzerProfile)
			{
			case bpNONE:
				BuzzerStart(false, 0);
				break;
			case bpQUICK_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpDOUBLE_BEEP:
				BuzzerStart(true, 7);
				buzzerStep4 = 13;
				break;
			case bpHIGH_LOW:
				BuzzerStart(false, 0);
				break;
			case bpLOW_HIGH:
				BuzzerStart(false, 0);
				break;
			case bpCONTINUOUS_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpCHIP:
				BuzzerStart(false, 0);
				break;
			case bpLONG_BEEP:
				BuzzerStart(false, 0);
				break;
			default:
				BuzzerStart(false, 0);
				break;
			}
		}
	}

	if (buzzerStep4 > 0)
	{
		if (--buzzerStep4 == 0)
		{
			switch(BuzzerProfile)
			{
			case bpNONE:
				BuzzerStart(false, 0);
				break;
			case bpQUICK_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpDOUBLE_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpHIGH_LOW:
				BuzzerStart(false, 0);
				break;
			case bpLOW_HIGH:
				BuzzerStart(false, 0);
				break;
			case bpCONTINUOUS_BEEP:
				BuzzerStart(false, 0);
				break;
			case bpCHIP:
				BuzzerStart(false, 0);
				break;
			case bpLONG_BEEP:
				BuzzerStart(false, 0);
				break;
			default:
				BuzzerStart(false, 0);
				break;
			}
		}
	}

	if ((shutDownCryo > 0) && (StateMachine == smOFF))
	{
		if (--shutDownCryo == 0)
		{
			CRYOTIMER_Enable(false);
			CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false);
		}
	}

	CRYOTIMER_IntEnable(CRYOTIMER_IEN_PERIOD);
}
//---------------------------------------------------------------------------

#ifdef INCLUDE_IR
// Turn the laser power on or off.
// powerState true = on, false = off.
void PowerLaser(bool powerState)
{
//	CMU_ClockEnable(cmuClock_CRYOTIMER, powerState);
	CRYOTIMER_Enable(powerState);
	if (!powerState)
	{
		GPIO_PortOutClear(gpioPortC, lasMASK);
	}
}
//---------------------------------------------------------------------------
#endif //INCLUDE_IR

// Remap the buffer and count the pixel width
// buffer is a null-terminated char array
// length is the length of the char array
// pixelWidth is a pointer to a uint8 to hold the calculated pixel width of the string in the buffer
//void RemapBufferAndCountPixels(char * buffer, uint8 length, uint8 * pixelWidth)
//{
//	bool noMoreCharacters;
//
//	noMoreCharacters = false;
//	*pixelWidth = 0;
//
//	for (int charNum = 0; charNum < 14; charNum++)
//	{
//		if (buffer[charNum] == '\0')
//		{
//			noMoreCharacters = true;
//		}
//
//		if (noMoreCharacters)
//		{
//			buffer[charNum] = 0x00;
//		}
//		else
//		{
//			// Remap space
//			if (buffer[charNum] == 0x20)
//			{
//				buffer[charNum] = 0x45;
//			}
//
//			// Remap open parenthesis
//			else if (buffer[charNum] == 0x28)
//			{
//				buffer[charNum] = 0x01;
//			}
//
//			// Remap close parenthesis
//			else if (buffer[charNum] == 0x29)
//			{
//				buffer[charNum] = 0x02;
//			}
//
//			// Remap plus sign
//			else if (buffer[charNum] == 0x2B)
//			{
//				buffer[charNum] = 0x03;
//			}
//
//			// Remap the minus sign
//			else if (buffer[charNum] == 0x2D)
//			{
//				buffer[charNum] = 0x04;
//			}
//
//			// Remap the dot symbol
//			else if (buffer[charNum] == 0x2E)
//			{
//				buffer[charNum] = 0x05;
//			}
//
//			// Remap the dot symbol
//			else if (buffer[charNum] == 0x2F)
//			{
//				buffer[charNum] = 0x46;
//			}
//
//			// Remap the numbers
//			else if ((buffer[charNum] >= 0x30) && (buffer[charNum] <= 0x39))
//			{
//				buffer[charNum] = buffer[charNum] - 42;
//			}
//
//			// Remap the upper case letters
//			else if ((buffer[charNum] >= 0x41) && (buffer[charNum] <= 0x5A))
//			{
//				buffer[charNum] = buffer[charNum] - 49;
//			}
//
//			// Remap the lower case letters
//			else if ((buffer[charNum] >= 0x61) && (buffer[charNum] <= 0x7A))
//			{
//				buffer[charNum] = buffer[charNum] - 55;
//			}
//
//			// Remap the degree symbol
//			else if (buffer[charNum] == 0xB0)
//			{
//				buffer[charNum] = 0x44;
//			}
//		}
//
//		(*pixelWidth) += widths[buffer[charNum]];
//	}
//}
//---------------------------------------------------------------------------

void RemapMacBufferAndCountPixels(char * buffer, uint8 length, uint8 * pixelWidth)
{
	bool noMoreCharacters;

	noMoreCharacters = false;
	*pixelWidth = 0;

	for (int charNum = 0; charNum < 18; charNum++)
	{
		if (buffer[charNum] == '\0')
		{
			noMoreCharacters = true;
		}

		if (noMoreCharacters)
		{
			buffer[charNum] = 0x00;
		}
		else
		{
			// Remap the numbers
			if ((buffer[charNum] >= 0x30) && (buffer[charNum] <= 0x39))
			{
				buffer[charNum] = buffer[charNum] - 47;
			}

			// Remap the upper case letters
			else if ((buffer[charNum] >= 0x41) && (buffer[charNum] <= 0x46))
			{
				buffer[charNum] = buffer[charNum] - 54;
			}

			// Remap the colon symbol
			else if (buffer[charNum] == 0x3A)
			{
				buffer[charNum] = 0x11;
			}
		}

		(*pixelWidth) += macWidths[buffer[charNum]];
	}
}
//---------------------------------------------------------------------------

void SetupLeTimer(void)
{
	// Enable clocks
//	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_HFRCO);
	CMU_ClockEnable(cmuClock_HFLE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

//	// Configure the buzzer pin as push-pull

	LETIMER0->CNT = 1;

	LETIMER_RepeatSet(LETIMER0, 0, 0x01);	// Set to zero so it won't trigger
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);	// Set to zero so it won't trigger

	LETIMER_CompareSet(LETIMER0, 0, 16);
	LETIMER_CompareSet(LETIMER0, 1, 8);

//	// Route the LETIMER0 output to the buzzer on PD11 (which is LOC19 according to EFR32BG1 Family Data Sheet).
//	LETIMER0->ROUTELOC0 = (LETIMER0->ROUTELOC0 & ~_LETIMER_ROUTELOC0_OUT0LOC_MASK) | LETIMER_ROUTELOC0_OUT0LOC_LOC19;

	// Route the LETIMER0 output to the buzzer on PC9 (which is LOC13 according to EFR32BG1 Family Data Sheet, but is actually LOC14).
	LETIMER0->ROUTELOC0 = (LETIMER0->ROUTELOC0 & ~_LETIMER_ROUTELOC0_OUT0LOC_MASK) | LETIMER_ROUTELOC0_OUT0LOC_LOC14;
	LETIMER0->ROUTEPEN |= LETIMER_ROUTEPEN_OUT0PEN;

	/* Set configurations for LETIMER 0 */
	const LETIMER_Init_TypeDef letimerInit =
	{
			.enable 		= false,                  /* Don't start counting when init completed. */
			.debugRun       = true,                   /* Counter shall keep running during debug halt. */
			.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
			.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
			.out0Pol        = 0,                      /* Idle value for output 0. */
			.out1Pol        = 0,                      /* Idle value for output 1. */
			.ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
			.ufoa1          = letimerUFOAPwm,         /* PWM output on output 1*/
			.repMode        = letimerRepeatFree       /* Count until stopped */
	};

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);
}
//---------------------------------------------------------------------------

void SetupCryoTimer(void)
{
	// Setup Cryotimer
	CRYOTIMER_IntEnable(CRYOTIMER_IEN_PERIOD);
	NVIC_EnableIRQ(CRYOTIMER_IRQn);

	CRYOTIMER_Init_TypeDef init;
	init.enable = false;
	init.debugRun = true;
	init.em4Wakeup = true;
	init.osc = cryotimerOscULFRCO;			// 1000 Hz
	init.presc = cryotimerPresc_8;		// Divide osc by 8 (freq = 125 Hz, period = 8ms)
	init.period = cryotimerPeriod_1;	// Trigger every cycle
//	init.period = cryotimerPeriod_128;	// Trigger every 128 cycles (freq = .976 Hz, period = 1.024 s)
	CRYOTIMER_Init(&init);

//	CMU_ClockSelectSet(cmuClock_CRYOTIMER, cmuSelect_ULFRCO);
	// Enable CRYOTIMER clock
	CMU_ClockEnable(cmuClock_CRYOTIMER, true);

	// Clear CRYOTIMER_IF PERIOD flag; it will be set upon EM4 wake
	CRYOTIMER_IntClear(CRYOTIMER_IF_PERIOD);


}
//---------------------------------------------------------------------------

void BuzzerStart(bool start, uint8 level)
{
	if (start)
	{
		LETIMER_RepeatSet(LETIMER0, 0, 0x01);
		LETIMER_RepeatSet(LETIMER0, 1, 0x01);

		LETIMER_CompareSet(LETIMER0, 0, level);
		LETIMER_CompareSet(LETIMER0, 1, level/2);

		LETIMER0->ROUTEPEN |= LETIMER_ROUTEPEN_OUT0PEN;
		LETIMER_Enable(LETIMER0, true);
	}
	else
	{
		LETIMER_Enable(LETIMER0, false);
		LETIMER0->ROUTEPEN = _LETIMER_ROUTEPEN_RESETVALUE;
		GPIO_PortOutClear(gpioPortC, buzMASK);
	}
}
//---------------------------------------------------------------------------

void ShutDownTasks(uint8 * buzzerProfile, uint8 * connectionHandle, uint8 * stateMachine, uint8 * advertisementTimer, uint8 * autoSleepTimer)
{
	*stateMachine = smOFF;

	StopAllTimers();
	gecko_cmd_hardware_set_soft_timer(0, thUPDATE_BATTERY, 1);

	gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_non_connectable);

	*buzzerProfile = bpHIGH_LOW;
	buzzerStep1 = 1;

	*connectionHandle = 0xFF;

	*autoSleepTimer = 0xFF;
	*advertisementTimer = 0xFF;

	StatusRegister[1] = StatusRegister[1] & 0xF7;								// Clear the "Sleeping" flag.

	I2C_Enable(I2C0, false);
	USART_Enable(USART0, usartDisable);
	LETIMER_Enable(LETIMER0, false);
	TIMER0->CMD = TIMER_CMD_STOP;
	TIMER_Enable(TIMER0, false);

	GPIO_PortOutSetVal(gpioPortD, ledOFF, ledMASK);				// Turn off LEDs
	GPIO_PortOutSetVal(gpioPortC, tcOFF, tcMASK);				// Turn off VMeasTC2.7
	PowerDisplay(false);										// Turn off display
	GPIO_PortOutSetVal(gpioPortA, regOFF, regMASK);				// Turn off VReg2.7
	//	GPIO_PortOutSetVal(gpioPortB, ds2482OFF, ds2482MASK);		// Turn off DS2482-100 ..PMG
	gecko_cmd_hardware_set_soft_timer(tmSECOND, thSHUTDOWN_GPIO, 1);	// Make sure any remaining running code doesn't leave a GPIO high

	shutDownCryo = 30;

}
//---------------------------------------------------------------------------

void StartUpTasks(void)
{
	CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
	I2C_Enable(I2C0, true);
	USART_Enable(USART0, usartEnableTx);
	TIMER_Enable(TIMER0, true);
	LETIMER_Enable(LETIMER0, true);
	gecko_cmd_hardware_set_soft_timer(tmMINUTE, thUPDATE_BATTERY, 0);
	CRYOTIMER_Enable(true);
}
//---------------------------------------------------------------------------

void PowerDisplay(bool powerState)
{
	if (powerState)
	{
		GPIO_PortOutSetVal(gpioPortD, dspON, dspON_OFF_MASK);
		GPIO_PortOutSetVal(gpioPortA, dspCS_SELECT, dspCS_MASK);
	}
	else
	{
		GPIO_PortOutSetVal(gpioPortD, dspOFF, dspON_OFF_MASK);
		GPIO_PortOutSetVal(gpioPortA, ~dspCS_SELECT, dspCS_MASK);
	}
}
//---------------------------------------------------------------------------

void GenerateTemperatureAsciiString(char* tempMessage, bool* useBigTemp, float tempInC)
{
	float tempInF;

	if (StatusRegister[1] & 0x08)											// Sleeping
	{
		sprintf(tempMessage, "Sleeping");
//		*useBigTemp = false;
	}
	else if ((StatusRegister[1] & 0x20) && !(StatusRegister[0] & 0x01))		// No calibration and TC mode
	{
		sprintf(tempMessage, "No Cal");
//		*useBigTemp = false;
	}
	else if (StatusRegister[1] & 0x10)										// Low battery
	{
		sprintf(tempMessage, "Low Batt");
//		*useBigTemp = false;
	}
	else if ((StatusRegister[1] & 0x01) && !(StatusRegister[0] & 0x01))		// No probe and TC mode
	{
		sprintf(tempMessage, "No Probe");
//		*useBigTemp = false;
	}
	else if ((StatusRegister[1] & 0x02) && !(StatusRegister[0] & 0x01))		// Low and TC mode
	{
		sprintf(tempMessage, "Low");
//		*useBigTemp = false;
	}
	else if ((StatusRegister[1] & 0x04) && !(StatusRegister[0] & 0x01))		// High and TC mode
	{
		sprintf(tempMessage, "High");
//		*useBigTemp = false;
	}
//	else if (StatusRegister[1] & 0x40)
//	{
//		sprintf(tempMessage, "--");
////		*useBigTemp = true;
//	}
	else																	// Show Temperature
	{
		if (StatusRegister[0] & 0x02)										// Fahrenheit
		{
			tempInF = tempInC;
			tempInF *= 9;
			tempInF /= 5;
			tempInF += 32;

			RoundTo(&tempInF, 1);
			sprintf(tempMessage, "%.1f%cF", tempInF, 0xB0);
//			*useBigTemp = true;
		}
		else																// Celsius
		{
			RoundTo(&tempInC,  1);
			sprintf(tempMessage, "%.1f%cC", tempInC, 0xB0);
//			*useBigTemp = true;
		}
	}
}
//---------------------------------------------------------------------------

void GenerateTemperatureDisplayString(char* tempDisplayMessage, bool* useBigTemp, float tempInC)
{
	float tempInF;
#ifdef WHOLE_DEGS_F
	int tempInFInt;
#endif // WHOLE_DEGS_F

	if (StatusRegister[1] & 0x08)											// Sleeping
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
	else if ((StatusRegister[1] & 0x20) && !(StatusRegister[0] & 0x01))		// No calibration and TC mode
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
	else if (StatusRegister[1] & 0x10)										// Low battery
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
	else if ((StatusRegister[1] & 0x01) && !(StatusRegister[0] & 0x01))		// No probe and TC mode
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
	else if ((StatusRegister[1] & 0x02) && !(StatusRegister[0] & 0x01))		// Low and TC mode
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
	else if ((StatusRegister[1] & 0x04) && !(StatusRegister[0] & 0x01))		// High and TC mode
	{
		sprintf(tempDisplayMessage, "!");
		*useBigTemp = true;
	}
//	else if (StatusRegister[1] & 0x40)
//	{
//		sprintf(tempDisplayMessage, "--");
//		*useBigTemp = true;
//	}
	else																	// Show Temperature
	{
		if (StatusRegister[0] & 0x02)										// Fahrenheit
		{
			tempInF = tempInC;
			tempInF *= 9;
			tempInF /= 5;
			tempInF += 32;
#ifdef WHOLE_DEGS_F
			// Round to whole degs
			RoundTo(&tempInF, 0);

			sprintf(tempDisplayMessage, "%.0f", tempInF);

#else
#ifdef HUNDREDTHS_DEG_F
			RoundTo(&tempInF, 2);
			sprintf(tempDisplayMessage, "%.2f", tempInF);
#else

			RoundTo(&tempInF, 1);
			if (tempInF == 1000.0)
			{
				// Use the character 'o' for zeros, which remaps to a skinny zero
				sprintf(tempDisplayMessage, "1ooo.o");
			}
			else if (tempInF == -100.0)
			{
				// Use the character 'o' for zeros, which remaps to a skinny zero
				sprintf(tempDisplayMessage, "-1oo.o");
			}
			else
			{
				sprintf(tempDisplayMessage, "%.1f", tempInF);
			}
#endif //HUNDREDTHS_DEG_F
#endif //WHOLE_DEGS_F
			*useBigTemp = true;
		}
		else																// Celsius
		{
			RoundTo(&tempInC, 1);
			sprintf(tempDisplayMessage, "%.1f", tempInC);
			*useBigTemp = true;
		}
	}
}
//---------------------------------------------------------------------------

//void BeginITS(bool advertising)
//{
//	StopAllTimers();
//	if(advertising)
//	{
//		gecko_cmd_hardware_set_soft_timer(tmSECOND, thADVERTISEMENT, 0); 			// Restart advertisement timer
//	}
////	ItsMode = imWAITING_FOR_STEP;
//	ItsStepSize = 0;
//	firstFastRead = true;
//	GPIO_PortOutSetVal(gpioPortC, irON, irMASK);
//	GPIO_PortOutSetVal(gpioPortC, tcON, tcMASK);
//	gecko_cmd_hardware_set_soft_timer(tmQUARTERSECOND, thWAIT_FOR_VMEAS_TC, 1); 	// Wait  for VMeasTC2.7
//}
////---------------------------------------------------------------------------
//
//void StopITS(uint8 measIntvl, bool advertising)
//{
//	StopAllTimers();
//	if(advertising)
//	{
//		gecko_cmd_hardware_set_soft_timer(tmSECOND, thADVERTISEMENT, 0); 			// Restart advertisement timer
//	}
//	gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thWAIT_FOR_VMEAS_TC, 1);  		// Get an immediate measurement
//
//}
//---------------------------------------------------------------------------

void TakeImmediateTemperature(bool * ResettingMeasTimer)
{
	// Stop all measurement related timers
	gecko_cmd_hardware_set_soft_timer(0, thMEASUREMENT, 1);
	gecko_cmd_hardware_set_soft_timer(0, thWAIT_FOR_VMEAS_TC, 1);
	gecko_cmd_hardware_set_soft_timer(0, thWAIT_FOR_TC, 1);
	gecko_cmd_hardware_set_soft_timer(0, thSETTLE_FOR_TM, 1);
	gecko_cmd_hardware_set_soft_timer(0, thWAIT_FOR_TM, 1);
	gecko_cmd_hardware_set_soft_timer(0, thCALC_TEMP, 1);
	gecko_cmd_hardware_set_soft_timer(0, thWAIT_FOR_VMEAS_IR, 1);

	// Restart measurements
	*ResettingMeasTimer = true;
	gecko_cmd_hardware_set_soft_timer(tmTENMILLISEC, thMEASUREMENT, 1);
}
//---------------------------------------------------------------------------


void SetTemperatureDataToNaN(uint8* TemperatureData)
{
	TemperatureData[0] = 0xFF;								// Set temperature to NaN
	TemperatureData[1] = 0xFF;								// Set temperature to NaN
	TemperatureData[2] = 0xFF;								// Set temperature to NaN
	TemperatureData[3] = 0xFF;								// Set temperature to NaN
}
//---------------------------------------------------------------------------


void SetLegacyTempAsFloatToNaN(uint8* LegacyTempAsFloat)
{
	LegacyTempAsFloat[0] = 0xFF;								// Set temperature to NaN
	LegacyTempAsFloat[1] = 0xFF;								// Set temperature to NaN
	LegacyTempAsFloat[2] = 0x7F;								// Set temperature to NaN
	LegacyTempAsFloat[3] = 0x00;								// Set temperature to NaN
	LegacyTempAsFloat[4] = 0x00;								// Set temperature to NaN
}
//---------------------------------------------------------------------------


void RoundTo(float* input, unsigned int places)
{
	int multiplier = 1;
	int inputInt;

	for (unsigned int i = 0; i < places; i++)
	{
		multiplier *= 10;
	}

	*input = *input * multiplier;

	if (*input > 0)
	{
		*input += 0.5;
	}
	else if (*input < 0)
	{
		*input -= 0.5;
	}

	inputInt = (int)(*input);

	*input = (float)inputInt / (float)multiplier;
}
//---------------------------------------------------------------------------

void WriteBasicInfoToGatt(uint8* SerialNumber, uint8* SerialNumberString, uint8* GattString)
{
	struct gecko_msg_system_get_bt_address_rsp_t* BtAddress;

// Get the Bluetooth address
	BtAddress = gecko_cmd_system_get_bt_address();
	for (uint8 i = 0; i < 6; i++)
	{
		SerialNumber[i] = BtAddress->address.addr[i];
	}
//				sprintf((char*)SerialNumberString, "%02X:%02X:%02X:%02X:%02X:%02X", SerialNumber[5], SerialNumber[4], SerialNumber[3], SerialNumber[2], SerialNumber[1], SerialNumber[0]);
	sprintf((char*) SerialNumberString, "%02X%02X%02X%02X%02X%02X",
			SerialNumber[5], SerialNumber[4], SerialNumber[3], SerialNumber[2],
			SerialNumber[1], SerialNumber[0]);

	// Write firmware and hardware versions, the serial number, and the device name to GATT
	sprintf((char*) GattString, FIRMWARE_VER);
	gecko_cmd_gatt_server_write_attribute_value(gattdb_firmware_revision_string,
			0, 4, GattString);				// Update the GATT database
	sprintf((char*) GattString, HARDWARE_VER);
	gecko_cmd_gatt_server_write_attribute_value(gattdb_hardware_revision_string,
			0, 3, GattString);				// Update the GATT database
	sprintf((char*) GattString, DEVICE_NAME);
	gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0,
			DEVICE_NAME_LEN, GattString);			// Update the GATT database
	gecko_cmd_gatt_server_write_attribute_value(gattdb_serial_number_string, 0,
			12, SerialNumberString);		// Update the GATT database
}
//---------------------------------------------------------------------------




/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
