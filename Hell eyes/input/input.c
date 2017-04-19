/******************************************************************************/
/**
 * \file        input.c
 *
 * \author      Paschenko Andrey <temnislav@gmail.com>
 * \mentor      Sokolov Sergey <sokloff@gmail.com>
 *
 * \date        10.02.2017
 *
 * \brief       Определение активности статусных входов.
 *
 * \details     Имеется пять статусных входов по состоянию которых, будет
 *              определяться какой из алгоритмов работы ленты будет
 *              использоваться.
 *
 *
 ******************************************************************************/

/******************************************************************************
 *                             ЗАГОЛОВОЧНЫЕ ФАЙЛЫ
 ******************************************************************************/

#include "input.h"
#include "stdbool.h"

/******************************************************************************
 *                         ЛОКАЛЬНЫЕ МАКРООПРЕДЕЛЕНИЯ
 ******************************************************************************/

#define INPUT_NUMBER 5
   
/******************************************************************************
 *                            ЛОКАЛЬНЫЕ ТИПЫ ДАННЫХ
 ******************************************************************************/

/******************************************************************************
 *                              ЛОКАЛЬНЫЕ ДАННЫЕ
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      - TIM_HandleTypeDef debounce_tim — таймер для антидребезга 
  *             - SignalStatus_t s_SignalStatus —
  *             - SignalStatus_t *p_SignalStatus = &s_SignalStatus — указатель
  *             на структуру со статусами входов для включения ленты.
  *             - active_gpio_name — переменная для записи имени активного
  *             (включившегося) входа.
  *             - inactive_gpio_name — переменная для записи имени неактивного
  *             (выключившегося) входа.
  *             - gpio_active_flag — переменная для проверки антидребезга
  *             активных входов.
  *             - gpio_inactive_flag — переменная для проверки антидребезга
  *             неактивных входов.
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
 *                         ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      it_gpio_status — прерывание по срабатыванию входа (GPIO) 
  *             и записью состояния входа для последующей обработки
  *             антидребезга.
  *             gpio_debounce — обработка антидребезга и запись состояния
  *             входов в структуру для использования в другом модуле.
  *
  * ----------------------------------------------------------------------------
  */

void EXTI4_15_IRQHandler(void);
void gpio_debounce(void);

/******************************************************************************
 *                              ЛОКАЛЬНЫЕ ФУНКЦИИ
 ******************************************************************************/

 
/** ----------------------------------------------------------------------------
  *
  * \brief      Обработчик прерывания по срабатыванию GPIO по rising и falling
  *             с записью состояния входа в соответствующую переменную. 
  *             Выставление флага состояния входа для обработки в антидребезге.
  *             Запуск таймера и прерывания по таймеру для обработки
  *             антидребезга.
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
  * \brief      Обработчик прерывания по таймеру с обработкой антидребезга от
  *             входов.
  *             Сброс флагов прерывания по таймеру.
  *             Сброс локальных флагов состояния входов.
  *             Передача в структуру SignalStatus_t текущего состояния входов.
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
  
  /* Если проверка прошла успешно, то переходим к этой части вектора*/
  
  gpio_active_flag      = 0;
  gpio_inactive_flag    = 0;
  
  /* Передаём активное состояние входа в структуру SignalStatus_t */
  
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
  
  /* Передаём неактивное состояние входа в структуру SignalStatus_t */
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
 *                             ГЛОБАЛЬНЫЕ ФУНКЦИИ
 ******************************************************************************/

/** ----------------------------------------------------------------------------
  *
  * \brief      Конфигурация GPIO для входов:
  *             TURN_SIGNALS            GPIO_PIN_5
  *             STOP_SIGNALS            GPIO_PIN_6
  *             REVERSE                 GPIO_PIN_7
  *             RUNNING_LIGHTS          GPIO_PIN_8
  *             CUSTOM_STATUS           GPIO_PIN_9
  *             Режим работы по срабатыванию и отключению.
  *             Включение прерывание по срабатыванию входов.
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
  * \brief      Конфигурация таймера для обработки прерывания.
  *             Переод срабатывания раз в 500мс.
  *             Включение прерывания по таймеру.
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
 *                                КОНЕЦ ФАЙЛА
 ******************************************************************************/
