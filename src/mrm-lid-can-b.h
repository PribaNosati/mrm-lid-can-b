#pragma once
#include "Arduino.h"
#include "mrm-board.h"

/**
Purpose: mrm-lid-can-b interface to CANBus.
@author MRMS team
@version 0.2 2019-08-15
Licence: You can use this code any way you like.
*/

#define CAN_ID_LID_CAN_B0_IN 0x0180
#define CAN_ID_LID_CAN_B0_OUT 0x0181
#define CAN_ID_LID_CAN_B1_IN 0x0182
#define CAN_ID_LID_CAN_B1_OUT 0x0183
#define CAN_ID_LID_CAN_B2_IN 0x0184
#define CAN_ID_LID_CAN_B2_OUT 0x0185
#define CAN_ID_LID_CAN_B3_IN 0x0186
#define CAN_ID_LID_CAN_B3_OUT 0x0187
#define CAN_ID_LID_CAN_B4_IN 0x0188
#define CAN_ID_LID_CAN_B4_OUT 0x0189
#define CAN_ID_LID_CAN_B5_IN 0x018A
#define CAN_ID_LID_CAN_B5_OUT 0x018B
#define CAN_ID_LID_CAN_B6_IN 0x018C
#define CAN_ID_LID_CAN_B6_OUT 0x018D
#define CAN_ID_LID_CAN_B7_IN 0x018E
#define CAN_ID_LID_CAN_B7_OUT 0x018F

#define CAN_ID_LID_CAN_B8_IN 0x0280
#define CAN_ID_LID_CAN_B8_OUT 0x0281
#define CAN_ID_LID_CAN_B9_IN 0x0282
#define CAN_ID_LID_CAN_B9_OUT 0x0283
#define CAN_ID_LID_CAN_B10_IN 0x0284
#define CAN_ID_LID_CAN_B10_OUT 0x0285
#define CAN_ID_LID_CAN_B11_IN 0x0286
#define CAN_ID_LID_CAN_B11_OUT 0x0287
#define CAN_ID_LID_CAN_B12_IN 0x0288
#define CAN_ID_LID_CAN_B12_OUT 0x0289
#define CAN_ID_LID_CAN_B13_IN 0x028A
#define CAN_ID_LID_CAN_B13_OUT 0x028B
#define CAN_ID_LID_CAN_B14_IN 0x028C
#define CAN_ID_LID_CAN_B14_OUT 0x028D
#define CAN_ID_LID_CAN_B15_IN 0x028E
#define CAN_ID_LID_CAN_B15_OUT 0x028F

//CANBus commands
#define COMMAND_LID_CAN_B_CALIBRATE 0x05
#define COMMAND_LID_CAN_B_RANGING_TYPE 0x42

#define MRM_LID_CAN_INACTIVITY_ALLOWED_MS 10000


class Mrm_lid_can_b : public SensorBoard
{
	std::vector<uint16_t>* readings; // Analog readings of all sensors

	/** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	bool started(uint8_t deviceNumber);
	
public:
	
	/** Constructor
	@param robot - robot containing this board
	@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
	@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
	@param maxNumberOfBoards - maximum number of boards
	*/
	Mrm_lid_can_b(Robot* robot = NULL, uint8_t maxNumberOfBoards = 14);

	~Mrm_lid_can_b();

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	void add(char * deviceName = (char*)"");
	
	/** Calibration, only once after production
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void calibration(uint8_t deviceNumber = 0);

	/** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param sampleCount - Number or readings. 40% of the c, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	uint16_t distance(uint8_t deviceNumber = 0, uint8_t sampleCount = 0, uint8_t sigmaCount = 1);

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	@param length - number of data bytes
	*/
	bool messageDecode(uint32_t canId, uint8_t data[8], uint8_t dlc = 8);

	/** Enable plug and play
	@param enable - enable or disable
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	void pnpSet(bool enable = true, uint8_t deviceNumber = 0);

	/** Ranging type
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param value - long range 0, high speed 1, high accuracy 2
	*/
	void rangingType(uint8_t deviceNumber, uint8_t value = 0);

	/** Analog readings
	@param receiverNumberInSensor - always 0
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0);

	/** Print all readings in a line
	*/
	void readingsPrint();

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	void test(uint8_t deviceNumber = 0xFF, uint16_t betweenTestsMs = 0);

};


