/*****************************************************************************/
/**
 * \file ws2812b.c
 *
 * \author Соколов Сергей <sokoloff@gmail.com>
 * \author Пащенко Андрей <temnislav@gmail.com>
 *
 * \date 14.01.2017
 * 
 * \brief Модуль управления лентой.
 *
 * \details     Для реализации управления лентой будет использоваться DMA и 
 *              прерывания от DMA, таймер TIM21 и прерывания по нему, SPI1.
 *              DMA передаёт данные в SPI1 из массива Ws281b_ArrayForTape.
 *              TIM21 используется для реализации 50мс задержки между
 *              итерациями передачи данных от DMA.
 *              
 *****************************************************************************/

/*****************************************************************************
 *                            Заголовочные файлы.
 *****************************************************************************/

#include "ws2812b.h"
#include "delay.h"

/*****************************************************************************
 *                         Локальные макроопределения.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       WS2812B_ARRAYFORTAPE_LENGTH Размер массива, соответствующий 
 *              кол-ву светодиодов в ленте умноженное на 24, так как на 1 диод 
 *              выделяется 3 байта — по 1 байту на цвет (RGB).
 *              - SPI_LOGIC_0 — 00000111 будет являться нулём для светодиода
 *              - SPI_LOGIC_1 — 11100000 будет являться единицей для светодиода
 *
 *-----------------------------------------------------------------------------
 */

#define WS2812B_ARRAYFORTAPE_LENGTH (10 * 3) * 8 //96
#define DIOD 3 * 8
#define SPI_LOGIC_0 0x03 //0x03 //
#define SPI_LOGIC_1 0xFC //0xFC // 

/*****************************************************************************
 *                           Локальные типы данных.
 *****************************************************************************/

/*****************************************************************************
 *                             Локальные данные.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       Ws2812b_timer — объявляем имя таймера, работающего на TIM21
 *              Ws2812b_spi — объявляем имя spi, работающего на SPI1
 *              Ws2812b_DMA — объявляем имя DMA, работающего на DMA1_Channel3
 *              GPIO_Ws2812b — объявляем имя для ПИНа, к которыму будем
 *              подключать ленту, используем GPIO_PIN_7
 *
 *-----------------------------------------------------------------------------
 */

TIM_HandleTypeDef Ws2812b_timer;
SPI_HandleTypeDef Ws2812b_spi;
DMA_HandleTypeDef Ws2812b_DMA;
GPIO_InitTypeDef GPIO_Ws2812b;

/**----------------------------------------------------------------------------
 *
 * \brief       Объявляем массив из которого будут передаваться данные на
 *              светодиодную ленту
 *
 *-----------------------------------------------------------------------------
 */

uint8_t Ws2812b_ArrayForTape[WS2812B_ARRAYFORTAPE_LENGTH];

/*****************************************************************************
 *                         Прототипы локальных функций.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - HAL_Ws2812b_DataStart — Передаём данные в Ws2812b_spi с 
 *              помощью Ws2812b_DMA из Ws2812b_ArrayForTape.
 *              - HAL_Ws2812b_NewData_1 — эксперименты, будут выпилены.
 *              - HAL_Ws2812b_NewData_2 — эксперименты, будут выпилены.
 *-----------------------------------------------------------------------------
 */

void TIM21_IRQHandler(void);
void Ws2812b_NewData_1(void);
void Ws2812b_NewData_2(void);
void DMA1_Channel2_3_IRQHandler(void);
void Ws2812b_DMA_Interupt_i(void);

/*****************************************************************************
 *                             Локальные функции.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - Ws2812b_DataStart — Передаём данные в Ws2812b_spi с помощью
 *              Ws2812b_DMA из Ws2812b_ArrayForTape.
 *
 *-----------------------------------------------------------------------------
 */

void TIM21_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&Ws2812b_timer);
  HAL_SPI_Transmit_DMA(&Ws2812b_spi, Ws2812b_ArrayForTape, WS2812B_ARRAYFORTAPE_LENGTH);
}

/**----------------------------------------------------------------------------
 *
 * \brief       Сброс флагов прерывания.
 *
 *-----------------------------------------------------------------------------
 */

void DMA1_Channel2_3_IRQHandler(void)
{
  /* Transfer Complete Interrupt management ***********************************/
  if(__HAL_DMA_GET_FLAG(&Ws2812b_DMA, __HAL_DMA_GET_TC_FLAG_INDEX(&Ws2812b_DMA)) != RESET)
  {
    Ws2812b_DMA_Interupt_i();    
  }
  HAL_DMA_IRQHandler(&Ws2812b_DMA); 
}


/**----------------------------------------------------------------------------
 *
 * \brief       - HAL_Ws2812b_DMA_Interupt функция для вектора прерываний от
 *              DMA, которая будет заряжать таймер Ws2812b_timer на 50мс.
 *              Вызывает прерывание от DMA внутри которого заряжается таймер,
 *              по истечении которого сработает прерывание, которое в свою
 *              очередь DMA начнёт передавать данные
 *
 *-----------------------------------------------------------------------------
 */

void Ws2812b_DMA_Interupt_i(void)
{

  __HAL_TIM_SetCounter(&Ws2812b_timer, 0);
  HAL_TIM_Base_Start(&Ws2812b_timer);
  HAL_TIM_Base_Start_IT(&Ws2812b_timer);
}

/*****************************************************************************
 *                        Глобальные функции
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - Ws2812b_timer_spi_dma_gpio_init — Конфигурируем Ws2812b_spi,
 *              GPIO_Ws2812b, Ws2812b_DMA, Ws2812b_timer, прерывания по таймеру
 *              и по DMA для реализации сброса в 50мс.
 *
 *-----------------------------------------------------------------------------
 */

void Ws2812b_timer_spi_dma_gpio_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  
  __HAL_RCC_TIM21_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  
  Ws2812b_spi.Instance                   = SPI1;
  Ws2812b_spi.Init.Mode                  = SPI_MODE_MASTER;
  Ws2812b_spi.Init.Direction             = SPI_DIRECTION_2LINES;
  Ws2812b_spi.Init.DataSize              = SPI_DATASIZE_8BIT;
  Ws2812b_spi.Init.CLKPolarity           = SPI_POLARITY_LOW;
  Ws2812b_spi.Init.CLKPhase              = SPI_PHASE_1EDGE;
  Ws2812b_spi.Init.NSS                   = SPI_NSS_SOFT; 
  Ws2812b_spi.Init.BaudRatePrescaler     = SPI_BAUDRATEPRESCALER_4;                     //делитель
  Ws2812b_spi.Init.FirstBit              = SPI_FIRSTBIT_MSB;                            // старший бит
  Ws2812b_spi.Init.TIMode                = SPI_TIMODE_DISABLED;                         // прерывание отключено
  Ws2812b_spi.Init.CRCCalculation        = SPI_CRCCALCULATION_DISABLED;                 // отключено црц
  Ws2812b_spi.Init.CRCPolynomial         = 10;                                          // прочитать про полином !!!!
  HAL_SPI_Init(&Ws2812b_spi);
  
  GPIO_Ws2812b.Pin                       = GPIO_PIN_7;                                  //GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; // нужен только 7 пин
  GPIO_Ws2812b.Mode                      = GPIO_MODE_AF_PP;                             //сконфигурирован в режим альтернативной функции
  GPIO_Ws2812b.Pull                      = GPIO_NOPULL;                                 //
  GPIO_Ws2812b.Speed                     = GPIO_SPEED_HIGH;                             //
  GPIO_Ws2812b.Alternate                 = GPIO_AF0_SPI1;                               // на пин повешен spi
  HAL_GPIO_Init(GPIOA, &GPIO_Ws2812b);                                                  //
  
  Ws2812b_DMA.Instance                   = DMA1_Channel3;                               //в main.s DMA1_Channel3 - именно для spi
  Ws2812b_DMA.Init.Request               = DMA_REQUEST_1;                               //пресет запроса для spi 1 и только для него
  Ws2812b_DMA.Init.Direction             = DMA_MEMORY_TO_PERIPH;                        //откуда-куда
  Ws2812b_DMA.Init.PeriphInc             = DMA_PINC_DISABLE;                            //отключено инкрементирование регистра переферии
  Ws2812b_DMA.Init.MemInc                = DMA_MINC_ENABLE;                             //инкрементирование памяти откуда пишется вся шляпа
  Ws2812b_DMA.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;                         //для переферии задаётся длинна слова для инкремента
  Ws2812b_DMA.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;                         //та же шляпа для источника
  Ws2812b_DMA.Init.Mode                  = DMA_NORMAL;                                  //режим работы передачи данных
  Ws2812b_DMA.Init.Priority              = DMA_PRIORITY_VERY_HIGH;			//приоритет
  HAL_DMA_Init(&Ws2812b_DMA);
  
  __HAL_LINKDMA(&Ws2812b_spi,hdmatx,Ws2812b_DMA);
  
  Ws2812b_timer.Instance                 = TIM21;
  Ws2812b_timer.Init.Prescaler           = 20;
  Ws2812b_timer.Init.CounterMode         = TIM_COUNTERMODE_UP;
  Ws2812b_timer.Init.Period              = 40;
  HAL_TIM_Base_Init(&Ws2812b_timer);
  
  sMasterConfig.MasterOutputTrigger     = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode         = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&Ws2812b_timer, &sMasterConfig);
  
  HAL_NVIC_SetPriority(TIM21_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(TIM21_IRQn);
  
  HAL_SPI_Transmit_DMA(&Ws2812b_spi, Ws2812b_ArrayForTape, WS2812B_ARRAYFORTAPE_LENGTH);
  
  HAL_TIM_Base_Start(&Ws2812b_timer);
  HAL_TIM_Base_Start_IT(&Ws2812b_timer);
}

/**----------------------------------------------------------------------------
 *
 * \brief       - HAL_Ws2812b_NewData_1 и HAL_Ws2812b_NewData_2(void)
 *              Серёга эксперементировал с лентой.
 *              HAL_Ws2812b_NewData_2(void) — для примера тут он зажёг зелёный,
 *              предварительно забив все 3 диода нолями.
 *
 *                    | 7  |0xF8|0xF8| 7  | 7  |
 *               сброс| 0  | 1  | 1  | 0  | 0  |
 *               50мс |    |    |    |    |    |
 *              -_____--___---__---__--___--___| и так далее.
 *              
 *              С помощью значения 7 мы передаём на конкретный цвет конкретного
 *              диода значение 0, то есть нам надо передать 8 байт со значением
 *              7 для того чтобы полностью выключить конкретный цвет.
 *              А с помощью значения 0xF8 мы передаём 1 и тем самым включаем
 *              необходимый нам цвет.
 *              Каждый бит необходимый диоду для работы мы передаём с помощью
 *              одного байта, имитируя некоторое подобие ШИМа.
 *              Сброс же в 50мс мы реализуем с помощью таймера Ws2812b_timer
 *
 *-----------------------------------------------------------------------------
 */

void Ws2812b_NewData_3(void)
{
  unsigned char temp_1 = 0;
  unsigned char temp_2 = 0;
  for (unsigned int i = 0; i < WS2812B_ARRAYFORTAPE_LENGTH; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_0;
  }
  for (unsigned char j = 0; j < (WS2812B_ARRAYFORTAPE_LENGTH / DIOD); ++j)
  {
    for (unsigned int l = ((temp_1 * DIOD) + 8); l < ((temp_1 * DIOD) + 16); l++)
    {
      Ws2812b_ArrayForTape[l] = SPI_LOGIC_1;
    }
    temp_1++;
  }
  /*for (unsigned int i = 33; i < 37; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }
  for (unsigned int i = 57; i < 61; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }*/
}

void Ws2812b_NewData_1(void)
{
  for (unsigned int i = 0; i < WS2812B_ARRAYFORTAPE_LENGTH; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_0;
  }
  for (unsigned int i = 8; i < 13; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }
  for (unsigned int i = 25; i < 27; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }
  for (unsigned int i = 50; i < 51; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }for (unsigned int i = 80; i < 82; i++)
  {
    Ws2812b_ArrayForTape[i] = SPI_LOGIC_1;
  }
}


void Ws2812b_NewData_2(void)
{
  
  if(get_status(delay_test) == delay_busy)
  {
    return;
  }
  else
  {
    unsigned char tape_end;
    tape_end = Ws2812b_ArrayForTape[0];
    for(unsigned int i = 0; i < (WS2812B_ARRAYFORTAPE_LENGTH - 1); ++i)
    {
      Ws2812b_ArrayForTape[i] = Ws2812b_ArrayForTape[(i + 1)];
    }
    Ws2812b_ArrayForTape[(WS2812B_ARRAYFORTAPE_LENGTH - 1)] = tape_end;
  }
  delay_set_ms(delay_test, 500);
}

/*****************************************************************************
 *                            Конец файла
 *****************************************************************************/