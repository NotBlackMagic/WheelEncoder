#include "stdio.h"
#include "stdint.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "MKE16Z4.h"

#include "fsl_gpio.h"
#include "fsl_lpi2c.h"
#include "fsl_lpuart.h"
#include "fsl_mscan.h"
#include "fsl_port.h"

//Output Pin Defines
#define GPIO_OUTPUT_LED_PORT			GPIOC
#define GPIO_OUTPUT_LED_PIN				7U
#define GPIO_OUTPUT_CAN_STB_PORT		GPIOB
#define GPIO_OUTPUT_CAN_STB_PIN			7U
#define GPIO_OUTPUT_CAN_EN_PORT			GPIOD
#define GPIO_OUTPUT_CAN_EN_PIN			0U

//Input Pin Defines
#define GPIO_INPUT_HAL_LH_PORT			GPIOA
#define GPIO_INPUT_HAL_LH_PIN			0U
#define GPIO_INPUT_HAL_LL_PORT			GPIOA
#define GPIO_INPUT_HAL_LL_PIN			1U
#define GPIO_INPUT_HAL_RH_PORT			GPIOC
#define GPIO_INPUT_HAL_RH_PIN			2U
#define GPIO_INPUT_HAL_RL_PORT			GPIOC
#define GPIO_INPUT_HAL_RL_PIN			3U
#define GPIO_INPUT_CAN_ERR_PORT			GPIOB
#define GPIO_INPUT_CAN_ERR_PIN			6U

//Interface properties
#define DEVICE_UART_BAUDRATE			115200
#define DEVICE_I2C_ADDR   				'N'
#define DEVICE_ID         				'N'
#define DEVICE_TX_RATE_MS				100		//Rate to send data over UART/CAN, every N ms

void Delay(uint32_t ms);

typedef enum Register {
	EncoderID = 0x00,
	EncoderLeftHigh = 0x01,
	EncoderRightHigh = 0x02,
	EncoderLeftLow = 0x03,
	EncoderRightLow = 0x04,
	EncoderPPR = 0x05
} Register;

typedef enum EncoderMode {
	Independent_2W_Low,
	Independent_2W_High,
	Independent_4W,
	Quadrature_2W
} EncoderMode;

typedef struct WheelEncoder {
	int16_t ticksPerEvolution;
	int32_t tickCount;
	int16_t rpm;
} WheelEncoder;

EncoderMode encoderMode;
WheelEncoder encoderLeftHigh;
WheelEncoder encoderLeftLow;
WheelEncoder encoderRightHigh;
WheelEncoder encoderRightLow;

//Pin Interrupt Handler for GPIO Ports A and E
void PORTAE_IRQHandler(void) {
	if((GPIO_PortGetInterruptFlags(GPIO_INPUT_HAL_LH_PORT) & (1U << GPIO_INPUT_HAL_LH_PIN)) == (1U << GPIO_INPUT_HAL_LH_PIN)) {
		//GPIOA Pin 0 (PTA0) interrupt
		if(encoderMode == Quadrature_2W) {
			//Quadrature Mode, status of LL gives rotation direction
			if(GPIO_PinRead(GPIO_INPUT_HAL_LL_PORT, GPIO_INPUT_HAL_LL_PIN) == 0x01) {
				encoderLeftHigh.tickCount += 1;
			}
			else {
				encoderLeftHigh.tickCount -= 1;
			}
		}
		else {
			encoderLeftHigh.tickCount += 1;
		}

		//Clear Flag
		GPIO_PortClearInterruptFlags(GPIO_INPUT_HAL_LH_PORT, (1U << GPIO_INPUT_HAL_LH_PIN));
	}
	if((GPIO_PortGetInterruptFlags(GPIO_INPUT_HAL_LL_PORT) & (1U << GPIO_INPUT_HAL_LL_PIN)) == (1U << GPIO_INPUT_HAL_LL_PIN)) {
		//GPIOA Pin 1 (PTA1) interrupt
		if(encoderMode != Quadrature_2W) {
			encoderLeftLow.tickCount += 1;
		}

		//Clear Flag
		GPIO_PortClearInterruptFlags(GPIO_INPUT_HAL_LL_PORT, (1U << GPIO_INPUT_HAL_LL_PIN));
	}
}

//Pin Interrupt Handler for GPIO Ports B, C and D
void PORTBCD_IRQHandler(void) {
	if((GPIO_PortGetInterruptFlags(GPIO_INPUT_HAL_RH_PORT) & (1U << GPIO_INPUT_HAL_RH_PIN)) == (1U << GPIO_INPUT_HAL_RH_PIN)) {
		//GPIOC Pin 2 (PTC2) interrupt
		if(encoderMode == Quadrature_2W) {
			//Quadrature Mode, status of LL gives rotation direction
			if(GPIO_PinRead(GPIO_INPUT_HAL_RH_PORT, GPIO_INPUT_HAL_RH_PIN) == 0x01) {
				encoderRightHigh.tickCount += 1;
			}
			else {
				encoderRightHigh.tickCount -= 1;
			}
		}
		else {
			encoderRightHigh.tickCount += 1;
		}

		//Clear Flag
		GPIO_PortClearInterruptFlags(GPIO_INPUT_HAL_RH_PORT, (1U << GPIO_INPUT_HAL_RH_PIN));
	}
	if((GPIO_PortGetInterruptFlags(GPIO_INPUT_HAL_RL_PORT) & (1U << GPIO_INPUT_HAL_RL_PIN)) == (1U << GPIO_INPUT_HAL_RL_PIN)) {
		//GPIOC Pin 3 (PTC3) interrupt
		if(encoderMode != Quadrature_2W) {
			encoderRightLow.tickCount += 1;
		}

		//Clear Flag
		GPIO_PortClearInterruptFlags(GPIO_INPUT_HAL_RL_PORT, (1U << GPIO_INPUT_HAL_RL_PIN));
	}
}

//UART0 Interrupt Handler
void LPUART0_IRQHandler(void) {

}

uint8_t g_slave_buff[64];
lpi2c_slave_handle_t g_s_handle;
volatile bool g_SlaveCompletionFlag = false;
static void lpi2c_slave_callback(LPI2C_Type *base, lpi2c_slave_transfer_t *xfer, void *param) {
	switch (xfer->event) {
		case kLPI2C_SlaveAddressMatchEvent: {
			//Address match event
			xfer->data = NULL;
			xfer->dataSize = 0;
			break;
		}
		case kLPI2C_SlaveTransmitEvent: {
			//Transmit request
			//Check received data for what to send back
			uint8_t reqRegister = g_slave_buff[0];

			uint8_t txIndex = 0;
			switch(reqRegister) {
				case EncoderID:
					g_slave_buff[txIndex++] = DEVICE_ID;
					break;
				case EncoderLeftHigh:
					//First: Data for Encoder Left High input
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount;
					g_slave_buff[txIndex++] = encoderLeftHigh.rpm >> 8;
					g_slave_buff[txIndex++] = encoderLeftHigh.rpm;
					//Second: Data for Encoder Right High input
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm;
					//Second: Data for Encoder Left Low input
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm;
					//Second: Data for Encoder Right Low input
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount;
					g_slave_buff[txIndex++] = encoderRightLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.rpm;
					break;
				case EncoderRightHigh:
					//Second: Data for Encoder Right High input
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm;
					//Second: Data for Encoder Left Low input
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm;
					//Second: Data for Encoder Right Low input
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount;
					g_slave_buff[txIndex++] = encoderRightLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.rpm;
					break;
				case EncoderLeftLow:
					//Second: Data for Encoder Left Low input
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.tickCount;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderLeftLow.rpm;
					//Second: Data for Encoder Right Low input
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount;
					g_slave_buff[txIndex++] = encoderRightLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.rpm;
					break;
				case EncoderRightLow:
					//Second: Data for Encoder Right Low input
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.tickCount;
					g_slave_buff[txIndex++] = encoderRightLow.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightLow.rpm;
					break;
				case EncoderPPR:
					g_slave_buff[txIndex++] = encoderLeftHigh.ticksPerEvolution >> 8;
					g_slave_buff[txIndex++] = encoderLeftHigh.ticksPerEvolution;
					break;
			}

			//Update information for transmit process
			xfer->data = &g_slave_buff[0];
			xfer->dataSize = txIndex;
			break;
		}
		case kLPI2C_SlaveReceiveEvent: {
			//Receive request
			//Update information for received process
			xfer->data = g_slave_buff;
			xfer->dataSize = 64;
			break;
		}
		case kLPI2C_SlaveCompletionEvent: {
			//Transfer done
			g_SlaveCompletionFlag = true;
			xfer->data = NULL;
			xfer->dataSize = 0;
			break;
		}
		default: {
			g_SlaveCompletionFlag = false;
			break;
		}
	}
}

mscan_handle_t mscanHandle;
mscan_mb_transfer_t txXfer, rxXfer;
mscan_frame_t txFrame;
int8_t canTXFrameStatus = 0;
volatile bool txComplete = false;
volatile bool rxComplete = false;
static void mscan_callback(MSCAN_Type *base, mscan_handle_t *handle, status_t status, void *userData) {
	switch (status) {
		case kStatus_MSCAN_RxIdle: {
			//MSCAN RX event
			rxComplete = true;
			break;
		}
		case kStatus_MSCAN_TxIdle: {
			//MSCAN TX event
			txComplete = true;
			switch(canTXFrameStatus) {
				case 1:
					//Second: Data for Encoder Right High input
					//Prepare CAN TX Frame
					//CAN Frame IDs (11-Bit long)
					//Constructed as follows: (DEVICE_ID << 3) + Register
					txFrame.ID_Type.ID = (DEVICE_ID << 3) + EncoderRightHigh;
					txFrame.format = kMSCAN_FrameFormatStandard;
					txFrame.type= kMSCAN_FrameTypeData;
					//Frame payload
					txFrame.dataByte0 = encoderRightHigh.tickCount >> 24;
					txFrame.dataByte1 = encoderRightHigh.tickCount >> 16;
					txFrame.dataByte2 = encoderRightHigh.tickCount >> 8;
					txFrame.dataByte3 = encoderRightHigh.tickCount;
					txFrame.dataByte4 = encoderRightHigh.rpm >> 8;
					txFrame.dataByte5 = encoderRightHigh.rpm;
					txFrame.DLR = 6;
					//Send data through Tx Message Buffer
					txXfer.frame = &txFrame;
					txXfer.mask  = kMSCAN_TxEmptyInterruptEnable;
					MSCAN_TransferSendNonBlocking(MSCAN, &mscanHandle, &txXfer);
					txComplete = false;
					canTXFrameStatus = 2;
					break;
				case 2:
					//Third: Data for Encoder Left Low input
					//Prepare CAN TX Frame
					//CAN Frame IDs (11-Bit long)
					//Constructed as follows: (DEVICE_ID << 3) + Register
					txFrame.ID_Type.ID = (DEVICE_ID << 3) + EncoderLeftLow;
					txFrame.format = kMSCAN_FrameFormatStandard;
					txFrame.type= kMSCAN_FrameTypeData;
					//Frame payload
					txFrame.dataByte0 = encoderLeftLow.tickCount >> 24;
					txFrame.dataByte1 = encoderLeftLow.tickCount >> 16;
					txFrame.dataByte2 = encoderLeftLow.tickCount >> 8;
					txFrame.dataByte3 = encoderLeftLow.tickCount;
					txFrame.dataByte4 = encoderLeftLow.rpm >> 8;
					txFrame.dataByte5 = encoderLeftLow.rpm;
					txFrame.DLR = 6;
					//Send data through Tx Message Buffer
					txXfer.frame = &txFrame;
					txXfer.mask  = kMSCAN_TxEmptyInterruptEnable;
					MSCAN_TransferSendNonBlocking(MSCAN, &mscanHandle, &txXfer);
					txComplete = false;
					canTXFrameStatus = 3;
					break;
				case 3:
					//Forth: Data for Encoder Right Low input
					//Prepare CAN TX Frame
					//CAN Frame IDs (11-Bit long)
					//Constructed as follows: (DEVICE_ID << 3) + Register
					txFrame.ID_Type.ID = (DEVICE_ID << 3) + EncoderRightLow;
					txFrame.format = kMSCAN_FrameFormatStandard;
					txFrame.type= kMSCAN_FrameTypeData;
					//Frame payload
					txFrame.dataByte0 = encoderRightLow.tickCount >> 24;
					txFrame.dataByte1 = encoderRightLow.tickCount >> 16;
					txFrame.dataByte2 = encoderRightLow.tickCount >> 8;
					txFrame.dataByte3 = encoderRightLow.tickCount;
					txFrame.dataByte4 = encoderRightLow.rpm >> 8;
					txFrame.dataByte5 = encoderRightLow.rpm;
					txFrame.DLR = 6;
					//Send data through Tx Message Buffer
					txXfer.frame = &txFrame;
					txXfer.mask  = kMSCAN_TxEmptyInterruptEnable;
					MSCAN_TransferSendNonBlocking(MSCAN, &mscanHandle, &txXfer);
					txComplete = false;
					canTXFrameStatus = 4;
					break;
				case 4:
					canTXFrameStatus = 0;
					return;
				default:
					canTXFrameStatus = 0;
					return;
			}
			break;
		}
		default: {
			break;
		}
	}
}

uint8_t rxBuffer[256];
uint8_t rxBufferLength;
uint8_t txBuffer[256];
uint8_t txBufferLength;
int main(void) {
	//Init
	BOARD_InitBootPins();
	BOARD_InitBootClocks();

	SysTick_Config(CLOCK_GetFreq(kCLOCK_ScgFircClk) / 1000);	//SysTick configured with 1ms timing

	//Init GPIO Output Pins
	gpio_pin_config_t gpioOutputConfig = {
		kGPIO_DigitalOutput,
		0,
	};
	GPIO_PinInit(GPIO_OUTPUT_LED_PORT, GPIO_OUTPUT_LED_PIN, &gpioOutputConfig);				//LED Output
	GPIO_PinInit(GPIO_OUTPUT_CAN_STB_PORT, GPIO_OUTPUT_CAN_STB_PIN, &gpioOutputConfig);		//CAN (TJA1463) Standby
	GPIO_PinInit(GPIO_OUTPUT_CAN_EN_PORT, GPIO_OUTPUT_CAN_EN_PIN, &gpioOutputConfig);		//CAN (TJA1463) Enable

	//Init GPIO Input Pins
	gpio_pin_config_t gpioInputConfig = {
		kGPIO_DigitalInput,
		0,
	};
	GPIO_PinInit(GPIO_INPUT_HAL_LH_PORT, GPIO_INPUT_HAL_LH_PIN, &gpioInputConfig);			//HAL Sensor Left High
	GPIO_PinInit(GPIO_INPUT_HAL_LL_PORT, GPIO_INPUT_HAL_LL_PIN, &gpioInputConfig);			//HAL Sensor Left Low
	GPIO_PinInit(GPIO_INPUT_HAL_RH_PORT, GPIO_INPUT_HAL_RH_PIN, &gpioInputConfig);			//HAL Sensor Right High
	GPIO_PinInit(GPIO_INPUT_HAL_RL_PORT, GPIO_INPUT_HAL_RL_PIN, &gpioInputConfig);			//HAL Sensor Right Low
	GPIO_PinInit(GPIO_INPUT_CAN_ERR_PORT, GPIO_INPUT_CAN_ERR_PIN, &gpioInputConfig);		//CAN (TJA1463) Error

	//Init GPIO Interrupts
	PORT_SetPinInterruptConfig(PORTA, GPIO_INPUT_HAL_LH_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTA, GPIO_INPUT_HAL_LL_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, GPIO_INPUT_HAL_RH_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, GPIO_INPUT_HAL_RL_PIN, kPORT_InterruptFallingEdge);
	EnableIRQ(PORTAE_IRQn);
	EnableIRQ(PORTBCD_IRQn);

	//Set GPIO Output Pins
	GPIO_PinWrite(GPIO_OUTPUT_LED_PORT, GPIO_OUTPUT_LED_PIN, 0);				//LED Output
	GPIO_PinWrite(GPIO_OUTPUT_CAN_STB_PORT, GPIO_OUTPUT_CAN_STB_PIN, 0);		//CAN (TJA1463) Standby
	GPIO_PinWrite(GPIO_OUTPUT_CAN_EN_PORT, GPIO_OUTPUT_CAN_EN_PIN, 0);			//CAN (TJA1463) Disabled

	//CAN transceiver (TJA1463) to normal mode
	GPIO_PinWrite(GPIO_OUTPUT_CAN_STB_PORT, GPIO_OUTPUT_CAN_STB_PIN, 1);
	GPIO_PinWrite(GPIO_OUTPUT_CAN_EN_PORT, GPIO_OUTPUT_CAN_EN_PIN, 1);

	//Init UART
	/*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kLPUART_ParityDisabled;
	 * config.stopBitCount = kLPUART_OneStopBit;
	 * config.txFifoWatermark = 0;
	 * config.rxFifoWatermark = 0;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	lpuart_config_t config;
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = DEVICE_UART_BAUDRATE;
	config.enableTx     = true;
	config.enableRx     = true;
	LPUART_Init(LPUART0, &config, CLOCK_GetFreq(kCLOCK_ScgFircClk));

	//Init I2C
	/*
	 * slaveConfig.address0 = 0U;
	 * slaveConfig.address1 = 0U;
	 * slaveConfig.addressMatchMode = kLPI2C_MatchAddress0;
	 * slaveConfig.filterDozeEnable = true;
	 * slaveConfig.filterEnable = true;
	 * slaveConfig.enableGeneralCall = false;
	 * slaveConfig.ignoreAck = false;
	 * slaveConfig.enableReceivedAddressRead = false;
	 * slaveConfig.sdaGlitchFilterWidth_ns = 0;
	 * slaveConfig.sclGlitchFilterWidth_ns = 0;
	 * slaveConfig.dataValidDelay_ns = 0;
	 * slaveConfig.clockHoldTime_ns = 0;
	 */
	lpi2c_slave_config_t slaveConfig;
	LPI2C_SlaveGetDefaultConfig(&slaveConfig);
	slaveConfig.address0 = DEVICE_I2C_ADDR;
	LPI2C_SlaveInit(LPI2C0, &slaveConfig, CLOCK_GetIpFreq(kCLOCK_Lpi2c0));
	//Create the LPI2C handle for the non-blocking transfer
	LPI2C_SlaveTransferCreateHandle(LPI2C0, &g_s_handle, lpi2c_slave_callback, NULL);
	//Start accepting I2C transfers on the LPI2C slave peripheral
	int32_t reVal = LPI2C_SlaveTransferNonBlocking(LPI2C0, &g_s_handle, kLPI2C_SlaveCompletionEvent | kLPI2C_SlaveAddressMatchEvent);
	if(reVal != kStatus_Success) {

	}

	//Init (MS)CAN
	/*
	 * mscanConfig.baudRate = 1000000U;
	 * mscanConfig.enableTimer = false;
	 * mscanConfig.enableWakeup = false;
	 * mscanConfig.clkSrc = kMSCAN_ClkSrcOsc;
	 * mscanConfig.enableLoopBack = false;
	 * mscanConfig.enableListen = false;
	 * mscanConfig.busoffrecMode = kMSCAN_BusoffrecAuto;
	 * mscanConfig.filterConfig.filterMode = kMSCAN_Filter32Bit;
	 */
	mscan_config_t mscanConfig;
	MSCAN_GetDefaultConfig(&mscanConfig);
	//Clock source configuration
	mscanConfig.clkSrc = kMSCAN_ClkSrcBus;
	//Free running timer (timestamp timer) configuration
	mscanConfig.enableTimer = true;
	//Acceptance filter configuration
	mscanConfig.filterConfig.u32IDAR0 = MSCAN_RX_MB_STD_MASK(0x123);
	mscanConfig.filterConfig.u32IDAR1 = MSCAN_RX_MB_STD_MASK(0x123);
	//To receive standard identifiers in 32-bit filter mode, program the last three bits ([2:0]) in the mask registers CANIDMR1 and CANIDMR5 to don't care.
	mscanConfig.filterConfig.u32IDMR0 = 0x00070000;
	mscanConfig.filterConfig.u32IDMR1 = 0x00070000;
	//Initialize MSCAN module
	MSCAN_Init(MSCAN, &mscanConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	//Create MSCAN handle structure and set call back function
	MSCAN_TransferCreateHandle(MSCAN, &mscanHandle, mscan_callback, NULL);

	//Send over UART
	txBufferLength = sprintf(txBuffer, "Wheel Encoder Boot Complete!");
	LPUART_WriteBlocking(LPUART0, txBuffer, txBufferLength);
//	LPUART_WriteByte(LPUART0, demoRingBuffer[txIndex]);

	//Enable RX Interrupt
//	LPUART_EnableInterrupts(LPUART0, kLPUART_RxDataRegFullInterruptEnable);
//	EnableIRQ(LPUART0_IRQn);

	//Init encoder
	encoderMode = Independent_4W;
	encoderLeftHigh.ticksPerEvolution = 2;
	encoderLeftHigh.tickCount = 0;
	encoderLeftHigh.rpm = 0;
	encoderLeftLow.ticksPerEvolution = 2;
	encoderLeftLow.tickCount = 0;
	encoderLeftLow.rpm = 0;
	encoderRightHigh.ticksPerEvolution = 2;
	encoderRightHigh.tickCount = 0;
	encoderRightHigh.rpm = 0;
	encoderRightLow.ticksPerEvolution = 2;
	encoderRightLow.tickCount = 0;
	encoderRightLow.rpm = 0;

	uint8_t ledState = 0;
	while(1) {
		//Send over UART and (MS)CAN at a rate of 1/DEVICE_TX_RATE_MS (every DEVICE_TX_RATE_MS)
		txBufferLength = sprintf(txBuffer, "LH: %d, RH: %d, LL: %d, RL: %d\n", encoderLeftHigh.tickCount, encoderLeftLow.tickCount, encoderRightHigh.tickCount, encoderRightLow.tickCount);
		LPUART_WriteBlocking(LPUART0, txBuffer, txBufferLength);

		//First: Data for Encoder Left High input
		//Prepare CAN TX Frame
		//CAN Frame IDs (11-Bit long)
		//Constructed as follows: (DEVICE_ID << 3) + Register
		txFrame.ID_Type.ID = (DEVICE_ID << 3) + EncoderLeftHigh;
		txFrame.format = kMSCAN_FrameFormatStandard;
		txFrame.type= kMSCAN_FrameTypeData;
		//Frame payload
		txFrame.dataByte0 = encoderLeftHigh.tickCount >> 24;
		txFrame.dataByte1 = encoderLeftHigh.tickCount >> 16;
		txFrame.dataByte2 = encoderLeftHigh.tickCount >> 8;
		txFrame.dataByte3 = encoderLeftHigh.tickCount;
		txFrame.dataByte4 = encoderLeftHigh.rpm >> 8;
		txFrame.dataByte5 = encoderLeftHigh.rpm;
		txFrame.DLR = 6;
		//Send data through Tx Message Buffer
		txXfer.frame = &txFrame;
		txXfer.mask  = kMSCAN_TxEmptyInterruptEnable;
		MSCAN_TransferSendNonBlocking(MSCAN, &mscanHandle, &txXfer);
		txComplete = false;
		canTXFrameStatus = 1;

		GPIO_PinWrite(GPIO_OUTPUT_LED_PORT, GPIO_OUTPUT_LED_PIN, ledState);
		ledState = !ledState;
		Delay(DEVICE_TX_RATE_MS);
	}
}

void HardFault_Handler(void) {

}

volatile uint32_t systick = 0;
void SysTick_Handler(void) {
	systick += 1;
}

void Delay(uint32_t ms) {
	uint32_t timestamp = systick + ms;
	while(systick < timestamp);
}
