#include "stdio.h"
#include "stdint.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "MKE16Z4.h"

#include "fsl_gpio.h"
#include "fsl_lpi2c.h"
#include "fsl_lpuart.h"
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
#define DEVICE_I2C_ADDR   				0x02
#define DEVICE_CAN_ADDR					0x00
#define DEVICE_ID         				'N'

void Delay(uint32_t ms);

typedef enum Register {
	EncoderID = 0x00,
	EncoderData = 0x01,
	EncoderPPR = 0x02
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
				case EncoderData:
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderLeftHigh.tickCount;
					g_slave_buff[txIndex++] = encoderLeftHigh.rpm >> 8;
					g_slave_buff[txIndex++] = encoderLeftHigh.rpm;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 24;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 16;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.tickCount;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm >> 8;
					g_slave_buff[txIndex++] = encoderRightHigh.rpm;
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
	GPIO_PinInit(GPIO_INPUT_CAN_ERR_PORT, GPIO_INPUT_CAN_ERR_PIN, &gpioInputConfig);			//CAN (TJA1463) Error

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
	GPIO_PinWrite(GPIO_OUTPUT_CAN_EN_PORT, GPIO_OUTPUT_CAN_EN_PIN, 0);			//CAN (TJA1463) Enable

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
		txBufferLength = sprintf(txBuffer, "LH: %d, LL: %d, RH: %d, RL: %d\n", encoderLeftHigh.tickCount, encoderLeftLow.tickCount, encoderRightHigh.tickCount, encoderRightLow.tickCount);
		LPUART_WriteBlocking(LPUART0, txBuffer, txBufferLength);

		GPIO_PinWrite(GPIO_OUTPUT_LED_PORT, GPIO_OUTPUT_LED_PIN, ledState);
		ledState = !ledState;
		Delay(1000);
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
