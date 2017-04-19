/******************************************************************************/
/**
 * \file        input.c
 *
 * \author      Paschenko Andrey <temnislav@gmail.com>
 * \mentor      Sokolov Sergey <sokloff@gmail.com>
 *
 * \date        10.02.2017
 *
 * \brief       ����������� ���������� ��������� ������.
 *
 * \details     ������� ���� ��������� ������ �� ��������� �������, �����
 *              ������������ ����� �� ���������� ������ ����� �����
 *              ��������������.
 *
 *
 ******************************************************************************/

/******************************************************************************
 *                             ������������ �����
 ******************************************************************************/

#include "input.h"
#include "stdbool.h"

/******************************************************************************
 *                         ��������� ����������������
 ******************************************************************************/

#define INPUT_NUMBER 5
   
/******************************************************************************
 *                            ��������� ���� ������
 ******************************************************************************/

/******************************************************************************
 *                              ��������� ������
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      - TIM_HandleTypeDef debounce_tim � ������ ��� ������������ 
  *             - SignalStatus_t s_SignalStatus �
  *             - SignalStatus_t *p_SignalStatus = &s_SignalStatus � ���������
  *             �� ��������� �� ��������� ������ ��� ��������� �����.
  *             - active_gpio_name � ���������� ��� ������ ����� ���������
  *             (�������������) �����.
  *             - inactive_gpio_name � ���������� ��� ������ ����� �����������
  *             (��������������) �����.
  *             - gpio_active_flag � ���������� ��� �������� ������������
  *             �������� ������.
  *             - gpio_inactive_flag � ���������� ��� �������� ������������
  *             ���������� ������.
  *
  * ----------------------------------------------------------------------------
  */

TIM_HandleTypeDef debounce_tim;

_Bool array_signal_status[INPUT_NUMBER];

uint16_t active_gpio_name       = 0;
uint16_t inactive_gpio_name     = 0;
uint16_t gpio_active_flag       = 0;
uint16_t gpio_inactive_flag     = 0;

/******************************************************************************
 *                         ��������� ��������� �������
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      it_gpio_status � ���������� �� ������������ ����� (GPIO) 
  *             � ������� ��������� ����� ��� ����������� ���������
  *             ������������.
  *             gpio_debounce � ��������� ������������ � ������ ���������
  *             ������ � ��������� ��� ������������� � ������ ������.
  *
  * ----------------------------------------------------------------------------
  */

void EXTI4_15_IRQHandler(void);
void gpio_debounce(void);

/******************************************************************************
 *                              ��������� �������
 ******************************************************************************/

 
/** ----------------------------------------------------------------------------
  *
  * \brief      ���������� ���������� �� ������������ GPIO �� rising � falling
  *             � ������� ��������� ����� � ��������������� ����������. 
  *             ����������� ����� ��������� ����� ��� ��������� � ������������.
  *             ������ ������� � ���������� �� ������� ��� ���������
  *             ������������.
  *
  * ----------------------------------------------------------------------------
  */

void EXTI4_15_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(TURN_SIGNALS);
  __HAL_GPIO_EXTI_CLEAR_IT(STOP_SIGNALS);
  __HAL_GPIO_EXTI_CLEAR_IT(REVERSE);
  __HAL_GPIO_EXTI_CLEAR_IT(RUNNING_LIGHTS);
  __HAL_GPIO_EXTI_CLEAR_IT(CUSTOM_STATUS);
  
  /* GPIO_PIN_SET */
  if (HAL_GPIO_ReadPin(GPIOC, TURN_SIGNALS != GPIO_PIN_RESET))
  {
    active_gpio_name = TURN_SIGNALS;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, STOP_SIGNALS == GPIO_PIN_SET))
  {
    active_gpio_name = STOP_SIGNALS;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, REVERSE == GPIO_PIN_SET))
  {
    active_gpio_name = REVERSE;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, RUNNING_LIGHTS == GPIO_PIN_SET))
  {
    active_gpio_name = RUNNING_LIGHTS;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, CUSTOM_STATUS == GPIO_PIN_SET))
  {
    active_gpio_name = CUSTOM_STATUS;
  }
  
  /* GPIO_PIN_RESET */
  else if (HAL_GPIO_ReadPin(GPIOC, TURN_SIGNALS == GPIO_PIN_RESET))
  {
    inactive_gpio_name = TURN_SIGNALS;
  } 
  else if (HAL_GPIO_ReadPin(GPIOC, STOP_SIGNALS == GPIO_PIN_RESET))
  {
    inactive_gpio_name = STOP_SIGNALS;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, REVERSE == GPIO_PIN_RESET))
  {
    inactive_gpio_name = REVERSE;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, RUNNING_LIGHTS == GPIO_PIN_RESET))
  {
    inactive_gpio_name = RUNNING_LIGHTS;
  }
  else if (HAL_GPIO_ReadPin(GPIOC, CUSTOM_STATUS == GPIO_PIN_RESET))
  {
    inactive_gpio_name = CUSTOM_STATUS;
  }
  
  /* flag GPIO_PIN_SET */
  if (active_gpio_name != 0)
  {
    gpio_active_flag = 1;
    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    HAL_TIM_Base_Start_IT(&debounce_tim);
    HAL_TIM_Base_Start(&debounce_tim);
  }
  
  /* flag GPIO_PIN_RESET */
  if (inactive_gpio_name != 0)
  {
    gpio_inactive_flag = 1;
    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    HAL_TIM_Base_Start_IT(&debounce_tim);
    HAL_TIM_Base_Start(&debounce_tim);
  }
}

/** ----------------------------------------------------------------------------
  *
  * \brief      ���������� ���������� �� ������� � ���������� ������������ ��
  *             ������.
  *             ����� ������ ���������� �� �������.
  *             ����� ��������� ������ ��������� ������.
  *             �������� � ��������� SignalStatus_t �������� ��������� ������.
  *
  * ----------------------------------------------------------------------------
  */

void gpio_debounce(void)
{
  HAL_TIM_IRQHandler(&debounce_tim);
  HAL_TIM_Base_Stop(&debounce_tim);
  HAL_TIM_Base_Stop_IT(&debounce_tim);
  
  /* GPIO_PIN_SET Debounce */
  if (HAL_GPIO_ReadPin(GPIOC, active_gpio_name == GPIO_PIN_SET))
  {
    if(gpio_active_flag == 0)
    {
      HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
      return;      
    }
  }
  else
  {
    active_gpio_name = 0;
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    return;
  }
  
  /* GPIO_PIN_RESET Debounce */
  if (HAL_GPIO_ReadPin(GPIOC, inactive_gpio_name == GPIO_PIN_RESET))
  {
    if(gpio_inactive_flag == 0)
    {
      HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
      return;      
    }
  }
  else
  {
    inactive_gpio_name = 0;
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    return;
  }
  
  /* ���� �������� ������ �������, �� ��������� � ���� ����� �������*/
  
  gpio_active_flag      = 0;
  gpio_inactive_flag    = 0;
  
  /* ������� �������� ��������� ����� � ��������� SignalStatus_t */
  
  if(active_gpio_name == TURN_SIGNALS)
  {
    array_signal_status[turn_signals]           = true;
  }
  
  else if(active_gpio_name == STOP_SIGNALS)
  {
    array_signal_status[stop_signals]           = true;
  }
  
  else if(active_gpio_name == REVERSE)
  {
    array_signal_status[reverse]                = true;
  }
  
  else if(active_gpio_name == RUNNING_LIGHTS)
  {
    array_signal_status[running_lights]         = true;
  }
  
  else if(active_gpio_name == CUSTOM_STATUS)
  {
    array_signal_status[custom_status]          = true;
  }
  
  /* ������� ���������� ��������� ����� � ��������� SignalStatus_t */
  if(inactive_gpio_name == TURN_SIGNALS)
  {
    array_signal_status[turn_signals]           = false;
  }
  
  else if(inactive_gpio_name == STOP_SIGNALS)
  {
    array_signal_status[stop_signals]           = false;
  }
  
  else if(inactive_gpio_name == REVERSE)
  {
    array_signal_status[reverse]                = false;
  }
  
  else if(inactive_gpio_name == RUNNING_LIGHTS)
  {
    array_signal_status[running_lights]         = false;
  }
  
  else if(inactive_gpio_name == CUSTOM_STATUS)
  {
    array_signal_status[custom_status]          = false;
  }
  
  active_gpio_name   = 0;
  inactive_gpio_name = 0;
  
  HAL_TIM_Base_Start_IT(&debounce_tim);
  HAL_TIM_Base_Start(&debounce_tim);
  
  return;
}

/******************************************************************************
 *                             ���������� �������
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      ������������ GPIO ��� ������:
  *             TURN_SIGNALS            GPIO_PIN_5
  *             STOP_SIGNALS            GPIO_PIN_6
  *             REVERSE                 GPIO_PIN_7
  *             RUNNING_LIGHTS          GPIO_PIN_8
  *             CUSTOM_STATUS           GPIO_PIN_9
  *             ����� ������ �� ������������ � ����������.
  *             ��������� ���������� �� ������������ ������.
  *
  * ----------------------------------------------------------------------------
  */

void input_gpio_init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
   
  GPIO_InitStruct.Pin   = TURN_SIGNALS;
  GPIO_InitStruct.Pin   = STOP_SIGNALS;
  GPIO_InitStruct.Pin   = REVERSE;
  GPIO_InitStruct.Pin   = RUNNING_LIGHTS;
  GPIO_InitStruct.Pin   = CUSTOM_STATUS;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;//GPIO_PULLUP;//GPIO_NOPULL;//
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/** ----------------------------------------------------------------------------
  *
  * \brief      ������������ ������� ��� ��������� ����������.
  *             ������ ������������ ��� � 500��.
  *             ��������� ���������� �� �������.
  *
  * ----------------------------------------------------------------------------
  */

void input_tim_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  
  __HAL_RCC_TIM6_CLK_ENABLE();
  
  debounce_tim.Instance                 = TIM6;
  debounce_tim.Init.Prescaler           = 24000;
  debounce_tim.Init.CounterMode         = TIM_COUNTERMODE_UP;
  debounce_tim.Init.Period              = 500;
  HAL_TIM_Base_Init(&debounce_tim);
  
  sMasterConfig.MasterOutputTrigger     = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode         = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&debounce_tim, &sMasterConfig);
  
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/** ----------------------------------------------------------------------------
  *
  * \brief      get_input_status -
  *             
  *
  * ----------------------------------------------------------------------------
  */

_Bool get_input_status(signal_status_typedef name)
{
  return array_signal_status[name];
}

/******************************************************************************
 *                                ����� �����
 ******************************************************************************/
