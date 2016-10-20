/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#include <string.h>

#include "stm32L0xx_hal.h"
#include "ws2812b.h"

extern WS2812_Struct ws2812b;

// WS2812 framebuffer - buffer for 2 LEDs - two times 24 bits
uint8_t ws2812bDmaBitBuffer[24 * 2];
#define BUFFER_SIZE		(sizeof(ws2812bDmaBitBuffer)/sizeof(uint8_t))

TIM_HandleTypeDef    Tim2Handle;
TIM_OC_InitTypeDef tim2OC1;
TIM_OC_InitTypeDef tim2OC2;

static uint8_t timCompareLogic0;
static uint8_t timCompareLogic1;

uint32_t tim_period;

// Gamma correction table
const uint8_t gammaTable[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

static void ws2812b_gpio_init(void)
{
	// WS2812B outputs
	WS2812B_GPIO_CLK_ENABLE();
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin       = WS2812B_PINS;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WS2812B_PORT, &GPIO_InitStruct);

	// Enable output pins for debuging to see DMA Full and Half transfer interrupts
	#if defined(LED4_PORT) && defined(LED5_PORT)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_InitStruct.Pin = LED4_PIN;
		HAL_GPIO_Init(LED4_PORT, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = LED5_PIN;
		HAL_GPIO_Init(LED5_PORT, &GPIO_InitStruct);
	#endif
}

static void TIM2_init(void)
{
	// TIM2 Periph clock enable
	__HAL_RCC_TIM2_CLK_ENABLE();

	// This computation of pulse length should work ok,
	// at some slower core speeds it needs some tuning.
	tim_period =  SystemCoreClock / 800000; // 0,125us period (10 times lower the 1,25us period to have fixed math below)

	timCompareLogic0 = (10 * tim_period) / 36;
	timCompareLogic1 = (10 * tim_period) / 15;

	Tim2Handle.Instance = TIM2;

	Tim2Handle.Init.Period            = tim_period;
	//Tim2Handle.Init.RepetitionCounter = 0;
	Tim2Handle.Init.Prescaler         = 0x00;
	Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&Tim2Handle);

	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	tim2OC1.OCMode       = TIM_OCMODE_PWM1;
	tim2OC1.OCPolarity   = TIM_OCPOLARITY_HIGH;
	tim2OC1.Pulse        = timCompareLogic0;
	tim2OC1.OCFastMode   = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&Tim2Handle, &tim2OC1, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&Tim2Handle, TIM_CHANNEL_1);

	(&Tim2Handle)->Instance->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_1TRANSFER;

}


DMA_HandleTypeDef     dmaUpdate;


static void DMA_init(void)
{

	// TIM2 Update event - Channel 2
	__HAL_RCC_DMA1_CLK_ENABLE();

	dmaUpdate.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dmaUpdate.Init.PeriphInc = DMA_PINC_DISABLE;
	dmaUpdate.Init.MemInc = DMA_MINC_ENABLE;
	dmaUpdate.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaUpdate.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	dmaUpdate.Init.Mode = DMA_CIRCULAR;
	dmaUpdate.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	dmaUpdate.Instance = DMA1_Channel2;
	dmaUpdate.Init.Request = DMA_REQUEST_8;

	dmaUpdate.XferCpltCallback  = DMA_TransferCompleteHandler;
	dmaUpdate.XferHalfCpltCallback = DMA_TransferHalfHandler;

    __HAL_LINKDMA(&Tim2Handle, hdma[TIM_DMA_ID_UPDATE], dmaUpdate);

	HAL_DMA_Init(&dmaUpdate);


	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	//HAL_DMA_Start_IT(&dmaUpdate, (uint32_t)timDmaTest, (uint32_t)&(&Tim2Handle)->Instance->DMAR, 4);
	HAL_DMA_Start_IT(&dmaUpdate, (uint32_t)ws2812bDmaBitBuffer, (uint32_t)&TIM2->CCR1, BUFFER_SIZE);

}



void DMA1_Channel2_3_IRQHandler(void)
{
  // Check the interrupt and clear flag
  HAL_DMA_IRQHandler(&dmaUpdate);
}



static void loadNextFramebufferData(WS2812_BufferItem *bItem, uint32_t row)
{

	uint32_t r = bItem->frameBufferPointer[bItem->frameBufferCounter++];
	uint32_t g = bItem->frameBufferPointer[bItem->frameBufferCounter++];
	uint32_t b = bItem->frameBufferPointer[bItem->frameBufferCounter++];

	if(bItem->frameBufferCounter == bItem->frameBufferSize)
		bItem->frameBufferCounter = 0;

	ws2812b_set_pixel(bItem->channel, row, r, g, b);
}


// Transmit the framebuffer
static void WS2812_sendbuf()
{
	// transmission complete flag
	ws2812b.transferComplete = 0;

	uint32_t i;

	for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
	{
		ws2812b.item[i].frameBufferCounter = 0;

		loadNextFramebufferData(&ws2812b.item[i], 0); // ROW 0
		loadNextFramebufferData(&ws2812b.item[i], 1); // ROW 0
	}

	HAL_TIM_Base_Stop(&Tim2Handle);
	(&Tim2Handle)->Instance->CR1 &= ~((0x1U << (0U)));


	// clear all DMA flags
	__HAL_DMA_CLEAR_FLAG(&dmaUpdate, DMA_FLAG_TC2 | DMA_FLAG_HT2 | DMA_FLAG_TE2);

	// configure the number of bytes to be transferred by the DMA controller
	dmaUpdate.Instance->CNDTR = BUFFER_SIZE;

	// clear all TIM2 flags
	__HAL_TIM_CLEAR_FLAG(&Tim2Handle, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);

	// enable DMA channels
	__HAL_DMA_ENABLE(&dmaUpdate);

	// IMPORTANT: enable the TIM2 DMA requests AFTER enabling the DMA channels!
	__HAL_TIM_ENABLE_DMA(&Tim2Handle, TIM_DMA_UPDATE);

	TIM2->CNT = tim_period-1;

	// Set zero length for first pulse because the first bit loads after first TIM_UP
	TIM2->CCR1 = 0;

	__HAL_DBGMCU_FREEZE_TIM2();

	// start TIM2
	__HAL_TIM_ENABLE(&Tim2Handle);
}





void DMA_TransferHalfHandler(DMA_HandleTypeDef *DmaHandle)
{
	#if defined(LED4_PORT)
		LED4_PORT->BSRR = LED4_PIN;
	#endif

	// Is this the last LED?
	if(ws2812b.repeatCounter != (WS2812B_NUMBER_OF_LEDS / 2 - 1))
	{
		uint32_t i;

		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			loadNextFramebufferData(&ws2812b.item[i], 0);
		}

	} else {
		// If this is the last pixel, set the next pixel value to zeros, because
		// the DMA would not stop exactly at the last bit.
		ws2812b_set_pixel(0, 0, 0, 0, 0);
	}

	#if defined(LED4_PORT)
		LED4_PORT->BRR = LED4_PIN;
	#endif
}

void DMA_TransferCompleteHandler(DMA_HandleTypeDef *DmaHandle)
{
	#if defined(LED5_PORT)
		LED5_PORT->BSRR = LED5_PIN;
	#endif

	ws2812b.repeatCounter++;

	if(ws2812b.repeatCounter == WS2812B_NUMBER_OF_LEDS / 2)
	{
		// Transfer of all LEDs is done, disable DMA but enable tiemr update IRQ to stop the 50us pulse
		ws2812b.repeatCounter = 0;

		// Enable TIM2 Update interrupt for 50us Treset signal
		__HAL_TIM_ENABLE_IT(&Tim2Handle, TIM_IT_UPDATE);
		// Disable DMA
		__HAL_DMA_DISABLE(&dmaUpdate);
		// Disable the DMA requests
		__HAL_TIM_DISABLE_DMA(&Tim2Handle, TIM_DMA_UPDATE);

	} else {

		// Load bitbuffer with next RGB LED values
		uint32_t i;
		for( i = 0; i < WS2812_BUFFER_COUNT; i++ )
		{
			loadNextFramebufferData(&ws2812b.item[i], 1);
		}

	}

	#if defined(LED5_PORT)
		LED5_PORT->BRR = LED5_PIN;
	#endif
}


void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim2Handle);
}

// TIM2 Interrupt Handler gets executed on every TIM2 Update if enabled
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// I have to wait 50us to generate Treset signal
	if (ws2812b.timerPeriodCounter < (uint8_t)WS2812_RESET_PERIOD)
	{
		// count the number of timer periods
		ws2812b.timerPeriodCounter++;
	}
	else
	{
		ws2812b.timerPeriodCounter = 0;
		__HAL_TIM_DISABLE(&Tim2Handle);
		TIM2->CR1 = 0; // disable timer

		// disable the TIM2 Update
		__HAL_TIM_DISABLE_IT(&Tim2Handle, TIM_IT_UPDATE);
		// set TransferComplete flag
		ws2812b.transferComplete = 1;
	}

}



static void ws2812b_set_pixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue)
{

	// Apply gamma
	red = gammaTable[red];
	green = gammaTable[green];
	blue = gammaTable[blue];

	uint32_t calcCol = (column*24);

#if defined(SETPIX_1)

	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+i)] = (((((green)<<i) & 0x80)>>7)<<row) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+i)] = (((((red)<<i) & 0x80)>>7)<<row) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+i)] = (((((blue)<<i) & 0x80)>>7)<<row) ? timCompareLogic1 : timCompareLogic0;
	}

#elif defined(SETPIX_2)



		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+0)] = (((green)<<0) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+0)] = (((red)<<0) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+0)] = (((blue)<<0) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+1)] = (((green)<<1) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+1)] = (((red)<<1) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+1)] = (((blue)<<1) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+2)] = (((green)<<2) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+2)] = (((red)<<2) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+2)] = (((blue)<<2) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+3)] = (((green)<<3) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+3)] = (((red)<<3) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+3)] = (((blue)<<3) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+4)] = (((green)<<4) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+4)] = (((red)<<4) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+4)] = (((blue)<<4) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+5)] = (((green)<<5) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+5)] = (((red)<<5) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+5)] = (((blue)<<5) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+6)] = (((green)<<6) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+6)] = (((red)<<6) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+6)] = (((blue)<<6) & 0x80) ? timCompareLogic1 : timCompareLogic0;

		// write new data for pixel
		ws2812bDmaBitBuffer[(calcCol+7)] = (((green)<<7) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+8+7)] = (((red)<<7) & 0x80) ? timCompareLogic1 : timCompareLogic0;
		ws2812bDmaBitBuffer[(calcCol+16+7)] = (((blue)<<7) & 0x80) ? timCompareLogic1 : timCompareLogic0;

#elif defined(SETPIX_3)

		uint8_t *bitBuffer = &ws2812bDmaBitBuffer[calcCol];

		// write new data for pixel
		*bitBuffer++ = (green & 0x80) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x40) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x20) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x10) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x08) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x04) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x02) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (green & 0x01) ? timCompareLogic1 : timCompareLogic0;

		*bitBuffer++ = (red & 0x80) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x40) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x20) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x10) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x08) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x04) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x02) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (red & 0x01) ? timCompareLogic1 : timCompareLogic0;

		*bitBuffer++ = (blue & 0x80) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x40) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x20) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x10) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x08) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x04) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x02) ? timCompareLogic1 : timCompareLogic0;
		*bitBuffer++ = (blue & 0x01) ? timCompareLogic1 : timCompareLogic0;

#endif
}


void ws2812b_init()
{
	ws2812b_gpio_init();
	DMA_init();
	TIM2_init();

	// Need to start the first transfer
	ws2812b.transferComplete = 1;
}


void ws2812b_handle()
{
	if(ws2812b.startTransfer) {
		ws2812b.startTransfer = 0;
		WS2812_sendbuf();
	}

}
