/*****************************************************************************/
/**
 * \file ws2812b.c
 *
 * \author ������� ������ <sokoloff@gmail.com>
 * \author ������� ������ <temnislav@gmail.com>
 *
 * \date 14.01.2017
 * 
 * \brief ������ ���������� ������.
 *
 * \details     ��� ���������� ���������� ������ ����� �������������� DMA � 
 *              ���������� �� DMA, ������ TIM21 � ���������� �� ����, SPI1.
 *              DMA ������� ������ � SPI1 �� ������� Ws281b_ArrayForTape.
 *              TIM21 ������������ ��� ���������� 50�� �������� �����
 *              ���������� �������� ������ �� DMA.
 *              
 *****************************************************************************/

/*****************************************************************************
 *                            ������������ �����.
 *****************************************************************************/

#include "ws2812b.h"
#include "delay.h"

/*****************************************************************************
 *                         ��������� ����������������.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       WS2812B_ARRAYFORTAPE_LENGTH ������ �������, ��������������� 
 *              ���-�� ����������� � ����� ���������� �� 24, ��� ��� �� 1 ���� 
 *              ���������� 3 ����� � �� 1 ����� �� ���� (RGB).
 *              - SPI_LOGIC_0 � 00000111 ����� �������� ���� ��� ����������
 *              - SPI_LOGIC_1 � 11100000 ����� �������� �������� ��� ����������
 *
 *-----------------------------------------------------------------------------
 */

#define WS2812B_ARRAYFORTAPE_LENGTH (10 * 3) * 8 //96
#define DIOD 3 * 8
#define SPI_LOGIC_0 0x03 //0x03 //
#define SPI_LOGIC_1 0xFC //0xFC // 

/*****************************************************************************
 *                           ��������� ���� ������.
 *****************************************************************************/

/*****************************************************************************
 *                             ��������� ������.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       Ws2812b_timer � ��������� ��� �������, ����������� �� TIM21
 *              Ws2812b_spi � ��������� ��� spi, ����������� �� SPI1
 *              Ws2812b_DMA � ��������� ��� DMA, ����������� �� DMA1_Channel3
 *              GPIO_Ws2812b � ��������� ��� ��� ����, � �������� �����
 *              ���������� �����, ���������� GPIO_PIN_7
 *
 *-----------------------------------------------------------------------------
 */

TIM_HandleTypeDef Ws2812b_timer;
SPI_HandleTypeDef Ws2812b_spi;
DMA_HandleTypeDef Ws2812b_DMA;
GPIO_InitTypeDef GPIO_Ws2812b;

/**----------------------------------------------------------------------------
 *
 * \brief       ��������� ������ �� �������� ����� ������������ ������ ��
 *              ������������ �����
 *
 *-----------------------------------------------------------------------------
 */

uint8_t Ws2812b_ArrayForTape[WS2812B_ARRAYFORTAPE_LENGTH];

/*****************************************************************************
 *                         ��������� ��������� �������.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - HAL_Ws2812b_DataStart � ������� ������ � Ws2812b_spi � 
 *              ������� Ws2812b_DMA �� Ws2812b_ArrayForTape.
 *              - HAL_Ws2812b_NewData_1 � ������������, ����� ��������.
 *              - HAL_Ws2812b_NewData_2 � ������������, ����� ��������.
 *-----------------------------------------------------------------------------
 */

void TIM21_IRQHandler(void);
void Ws2812b_NewData_1(void);
void Ws2812b_NewData_2(void);
void DMA1_Channel2_3_IRQHandler(void);
void Ws2812b_DMA_Interupt_i(void);

/*****************************************************************************
 *                             ��������� �������.
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - Ws2812b_DataStart � ������� ������ � Ws2812b_spi � �������
 *              Ws2812b_DMA �� Ws2812b_ArrayForTape.
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
 * \brief       ����� ������ ����������.
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
 * \brief       - HAL_Ws2812b_DMA_Interupt ������� ��� ������� ���������� ��
 *              DMA, ������� ����� �������� ������ Ws2812b_timer �� 50��.
 *              �������� ���������� �� DMA ������ �������� ���������� ������,
 *              �� ��������� �������� ��������� ����������, ������� � ����
 *              ������� DMA ����� ���������� ������
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
 *                        ���������� �������
 *****************************************************************************/

/**----------------------------------------------------------------------------
 *
 * \brief       - Ws2812b_timer_spi_dma_gpio_init � ������������� Ws2812b_spi,
 *              GPIO_Ws2812b, Ws2812b_DMA, Ws2812b_timer, ���������� �� �������
 *              � �� DMA ��� ���������� ������ � 50��.
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
  Ws2812b_spi.Init.BaudRatePrescaler     = SPI_BAUDRATEPRESCALER_4;                     //��������
  Ws2812b_spi.Init.FirstBit              = SPI_FIRSTBIT_MSB;                            // ������� ���
  Ws2812b_spi.Init.TIMode                = SPI_TIMODE_DISABLED;                         // ���������� ���������
  Ws2812b_spi.Init.CRCCalculation        = SPI_CRCCALCULATION_DISABLED;                 // ��������� ���
  Ws2812b_spi.Init.CRCPolynomial         = 10;                                          // ��������� ��� ������� !!!!
  HAL_SPI_Init(&Ws2812b_spi);
  
  GPIO_Ws2812b.Pin                       = GPIO_PIN_7;                                  //GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7; // ����� ������ 7 ���
  GPIO_Ws2812b.Mode                      = GPIO_MODE_AF_PP;                             //��������������� � ����� �������������� �������
  GPIO_Ws2812b.Pull                      = GPIO_NOPULL;                                 //
  GPIO_Ws2812b.Speed                     = GPIO_SPEED_HIGH;                             //
  GPIO_Ws2812b.Alternate                 = GPIO_AF0_SPI1;                               // �� ��� ������� spi
  HAL_GPIO_Init(GPIOA, &GPIO_Ws2812b);                                                  //
  
  Ws2812b_DMA.Instance                   = DMA1_Channel3;                               //� main.s DMA1_Channel3 - ������ ��� spi
  Ws2812b_DMA.Init.Request               = DMA_REQUEST_1;                               //������ ������� ��� spi 1 � ������ ��� ����
  Ws2812b_DMA.Init.Direction             = DMA_MEMORY_TO_PERIPH;                        //������-����
  Ws2812b_DMA.Init.PeriphInc             = DMA_PINC_DISABLE;                            //��������� ����������������� �������� ���������
  Ws2812b_DMA.Init.MemInc                = DMA_MINC_ENABLE;                             //����������������� ������ ������ ������� ��� �����
  Ws2812b_DMA.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;                         //��� ��������� ������� ������ ����� ��� ����������
  Ws2812b_DMA.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;                         //�� �� ����� ��� ���������
  Ws2812b_DMA.Init.Mode                  = DMA_NORMAL;                                  //����� ������ �������� ������
  Ws2812b_DMA.Init.Priority              = DMA_PRIORITY_VERY_HIGH;			//���������
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
 * \brief       - HAL_Ws2812b_NewData_1 � HAL_Ws2812b_NewData_2(void)
 *              ����� ����������������� � ������.
 *              HAL_Ws2812b_NewData_2(void) � ��� ������� ��� �� ���� ������,
 *              �������������� ����� ��� 3 ����� ������.
 *
 *                    | 7  |0xF8|0xF8| 7  | 7  |
 *               �����| 0  | 1  | 1  | 0  | 0  |
 *               50�� |    |    |    |    |    |
 *              -_____--___---__---__--___--___| � ��� �����.
 *              
 *              � ������� �������� 7 �� ������� �� ���������� ���� �����������
 *              ����� �������� 0, �� ���� ��� ���� �������� 8 ���� �� ���������
 *              7 ��� ���� ����� ��������� ��������� ���������� ����.
 *              � � ������� �������� 0xF8 �� ������� 1 � ��� ����� ��������
 *              ����������� ��� ����.
 *              ������ ��� ����������� ����� ��� ������ �� ������� � �������
 *              ������ �����, �������� ��������� ������� ����.
 *              ����� �� � 50�� �� ��������� � ������� ������� Ws2812b_timer
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
 *                            ����� �����
 *****************************************************************************/