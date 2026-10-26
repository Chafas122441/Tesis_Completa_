/**
 * @file gripper_manual.c
 * @brief Servicio PWM de giro Z, inclinacion X y compuerta de la pala.
 * Gestiona consignas por canal, limites, permisos y vencimiento de operacion.
 */
#include "gripper_manual.h"
#include "gripper_limits.h"
#include "main.h"
#include <stdlib.h>
#include <stdio.h>

static TIM_HandleTypeDef gripTim2, gripTim4;
static volatile unsigned channel;
static volatile uint32_t lastUpdate;
static unsigned lastPulse[3] = {1500,1500,1500};
static uint32_t lastStep;
static uint8_t initialized;

void GripperManual_Stop(void) {
  if (!initialized) return;
  /* Finish only an existing high pulse, never turn it into a short command.
     CCR is preloaded: use the largest legal pulse, not a newer CCR value.
     DWT bounds this wait to 4 ms even when a caller has masked interrupts. */
  const uint32_t startedCycles = DWT->CYCCNT;
  const uint32_t maxCycles = (SystemCoreClock / 1000u) * 4u;
  const uint8_t wasActive = channel != 0u;
  uint8_t forced = 0u;
  channel=0;
  for (;;) {
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    const uint8_t expired = !(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ||
      (uint32_t)(DWT->CYCCNT - startedCycles) >= maxCycles;
    /* Leave a 1 ms guard before wrap: the counter must not start a new
       high pulse between this check and switching the pin to GPIO LOW. */
    const uint32_t count2 = TIM2->CNT, count4 = TIM4->CNT;
    const uint8_t low2 = !(TIM2->CR1 & TIM_CR1_CEN) ||
      (count2 >= 2500u && count2 <= 19000u);
    const uint8_t low4 = !(TIM4->CR1 & TIM_CR1_CEN) ||
      (count4 >= 2500u && count4 <= 19000u);
    if (low2 || expired ||
        !(TIM2->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E))) {
      if (expired && !low2 && (TIM2->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E))) forced=1u;
      GPIOA->BSRR = (uint32_t)(GPIO_PIN_15|GPIO_PIN_1)<<16;
      MODIFY_REG(GPIOA->MODER,(3UL<<30)|(3UL<<2),(1UL<<30)|(1UL<<2));
      TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
      TIM2->CR1 &= ~TIM_CR1_CEN;
    }
    if (low4 || expired || !(TIM4->CCER & TIM_CCER_CC3E)) {
      if (expired && !low4 && (TIM4->CCER & TIM_CCER_CC3E)) forced=1u;
      GPIOB->BSRR = (uint32_t)GPIO_PIN_8<<16;
      MODIFY_REG(GPIOB->MODER,3UL<<16,1UL<<16);
      TIM4->CCER &= ~TIM_CCER_CC3E;
      TIM4->CR1 &= ~TIM_CR1_CEN;
    }
    const uint8_t stopped = !(TIM2->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E)) &&
                           !(TIM4->CCER & TIM_CCER_CC3E);
    __set_PRIMASK(saved);
    if (stopped) break;
  }
  if (wasActive && __get_PRIMASK() == 0u)
    printf("[GRIP STOP] PWM_OFF pulse_complete=%u us=%u,%u,%u; no new pose.\r\n",
           !forced, lastPulse[0], lastPulse[1], lastPulse[2]);
}
int GripperManual_Active(void) { return channel!=0; }
uint32_t GripperManual_UpdateAgeMs(void) {
  return channel ? HAL_GetTick()-lastUpdate : UINT32_MAX;
}
unsigned GripperManual_Channel(void) { return channel; }
unsigned GripperManual_Pulse(unsigned ch) {
  return ch>=1 && ch<=3 ? lastPulse[ch-1] : 0u;
}
void GripperManual_Tick(void) {
  if (channel && HAL_GetTick()-lastUpdate>GRIP_WATCHDOG_MS)
    GripperManual_Stop();
}
static void timer_init(TIM_HandleTypeDef *t,TIM_TypeDef *instance) {
  t->Instance=instance;
  uint32_t hz=HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1)!=0) hz*=2;
  if(hz%1000000u) Error_Handler();
  t->Init.Prescaler=hz/1000000u-1u;
  t->Init.CounterMode=TIM_COUNTERMODE_UP; t->Init.Period=19999;
  t->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  t->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
  if(HAL_TIM_PWM_Init(t)!=HAL_OK) Error_Handler();
  TIM_OC_InitTypeDef o={0};
  o.OCMode=TIM_OCMODE_PWM1; o.OCPolarity=TIM_OCPOLARITY_HIGH;
  o.OCFastMode=TIM_OCFAST_DISABLE;
  if(instance==TIM2) {
    if(HAL_TIM_PWM_ConfigChannel(t,&o,TIM_CHANNEL_1)!=HAL_OK) Error_Handler();
    if(HAL_TIM_PWM_ConfigChannel(t,&o,TIM_CHANNEL_2)!=HAL_OK) Error_Handler();
  } else if(HAL_TIM_PWM_ConfigChannel(t,&o,TIM_CHANNEL_3)!=HAL_OK) Error_Handler();
}
void GripperManual_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM4_CLK_ENABLE();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  initialized=1; GripperManual_Stop();
  timer_init(&gripTim2,TIM2); timer_init(&gripTim4,TIM4);
  GripperManual_Stop();
}
DeltaResultCode GripperManual_Begin(unsigned ch,unsigned us) {
  if(ch<1 || ch>3 || us<GRIP_MIN_US[ch-1] || us>GRIP_MAX_US[ch-1])
    return DELTA_RESULT_RAW_RANGE;
  if(channel) return DELTA_RESULT_BUSY;
  /* No jump on re-enable. Last commanded pulse retained until MCU reset. */
  if(us!=lastPulse[ch-1]) return DELTA_RESULT_RAW_STEP;
  TIM_TypeDef *t=ch==3?TIM4:TIM2;
  if(ch==1) t->CCR1=us; else if(ch==2) t->CCR2=us; else t->CCR3=us;
  t->EGR=TIM_EGR_UG; t->SR=0; t->CNT=0;
  GPIO_InitTypeDef g={0};
  g.Pin=ch==1?GPIO_PIN_15:ch==2?GPIO_PIN_1:GPIO_PIN_8;
  g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_PULLDOWN;g.Speed=GPIO_SPEED_FREQ_LOW;
  g.Alternate=ch==3?GPIO_AF2_TIM4:GPIO_AF1_TIM2;
  HAL_GPIO_Init(ch==3?GPIOB:GPIOA,&g);
  uint32_t saved=__get_PRIMASK(); __disable_irq();
  lastStep=lastUpdate=HAL_GetTick();
  t->CCER |= ch==1?TIM_CCER_CC1E:ch==2?TIM_CCER_CC2E:TIM_CCER_CC3E;
  t->CR1 |= TIM_CR1_CEN; channel=ch;
  __set_PRIMASK(saved);
  return DELTA_RESULT_OK;
}
DeltaResultCode GripperManual_Set(unsigned ch,unsigned us) {
  if(!channel || ch!=channel) return DELTA_RESULT_DISARMED;
  if(us<GRIP_MIN_US[ch-1] || us>GRIP_MAX_US[ch-1]) return DELTA_RESULT_RAW_RANGE;
  if(abs((int)us-(int)lastPulse[ch-1])>GRIP_MAX_STEP_US) return DELTA_RESULT_RAW_STEP;
  uint32_t saved=__get_PRIMASK(); __disable_irq();
  if(!channel || HAL_GetTick()-lastUpdate>GRIP_WATCHDOG_MS) {
    GripperManual_Stop(); __set_PRIMASK(saved); return DELTA_RESULT_DISARMED;
  }
  uint32_t now=HAL_GetTick();
  if(us!=lastPulse[ch-1] && now-lastStep<GRIP_PERIOD_MS) {
    __set_PRIMASK(saved); return DELTA_RESULT_STREAM_RATE;
  }
  if(us!=lastPulse[ch-1]) lastStep=now;
  lastPulse[ch-1]=us;
  if(ch==1) TIM2->CCR1=us; else if(ch==2) TIM2->CCR2=us; else TIM4->CCR3=us;
  lastUpdate=now;
  __set_PRIMASK(saved);
  return DELTA_RESULT_OK;
}

/* channel=4 means all three PWM outputs. No jumps at activation. */
DeltaResultCode GripperManual_GroupBegin(const uint16_t us[3]) {
  if(channel) return DELTA_RESULT_BUSY;
  for(unsigned i=0;i<3;++i) {
    if(us[i]<GRIP_MIN_US[i] || us[i]>GRIP_MAX_US[i]) return DELTA_RESULT_RAW_RANGE;
    if(us[i]!=lastPulse[i]) return DELTA_RESULT_RAW_STEP;
  }
  TIM2->CCR1=us[0]; TIM2->CCR2=us[1]; TIM4->CCR3=us[2];
  TIM2->EGR=TIM_EGR_UG; TIM4->EGR=TIM_EGR_UG;
  TIM2->SR=0; TIM4->SR=0; TIM2->CNT=0; TIM4->CNT=0;
  GPIO_InitTypeDef g={0};
  g.Pin=GPIO_PIN_15|GPIO_PIN_1; g.Mode=GPIO_MODE_AF_PP;
  g.Pull=GPIO_PULLDOWN;g.Speed=GPIO_SPEED_FREQ_LOW;g.Alternate=GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA,&g);
  g.Pin=GPIO_PIN_8;g.Alternate=GPIO_AF2_TIM4;HAL_GPIO_Init(GPIOB,&g);
  uint32_t saved=__get_PRIMASK();__disable_irq();
  lastStep=lastUpdate=HAL_GetTick();
  TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E;
  TIM4->CCER |= TIM_CCER_CC3E;
  TIM2->CR1 |= TIM_CR1_CEN;TIM4->CR1 |= TIM_CR1_CEN;channel=4;
  __set_PRIMASK(saved);
  return DELTA_RESULT_OK;
}
static DeltaResultCode GroupSetBounded(const uint16_t us[3], unsigned maxStepUs) {
  if(channel!=4) return DELTA_RESULT_DISARMED;
  int changed=0;
  for(unsigned i=0;i<3;++i) {
    if(us[i]<GRIP_MIN_US[i] || us[i]>GRIP_MAX_US[i]) return DELTA_RESULT_RAW_RANGE;
    if(abs((int)us[i]-(int)lastPulse[i])>(int)maxStepUs) return DELTA_RESULT_RAW_STEP;
    changed |= us[i]!=lastPulse[i];
  }
  uint32_t saved=__get_PRIMASK();__disable_irq();
  uint32_t now=HAL_GetTick();
  if(channel!=4 || now-lastUpdate>GRIP_WATCHDOG_MS) {
    GripperManual_Stop();__set_PRIMASK(saved);return DELTA_RESULT_DISARMED;
  }
  if(changed && now-lastStep<GRIP_GROUP_PERIOD_MS) {
    __set_PRIMASK(saved);return DELTA_RESULT_STREAM_RATE;
  }

  if(changed) lastStep=now;
  for(unsigned i=0;i<3;++i) lastPulse[i]=us[i];
  TIM2->CCR1=us[0];TIM2->CCR2=us[1];TIM4->CCR3=us[2];lastUpdate=now;
  __set_PRIMASK(saved);return DELTA_RESULT_OK;
}
DeltaResultCode GripperManual_GroupSet(const uint16_t us[3]) {
  return GroupSetBounded(us, GRIP_GROUP_MAX_STEP_US);
}
DeltaResultCode GripperManual_AutoGroupSet(const uint16_t us[3]) {
  return GroupSetBounded(us, GRIP_AUTO_MAX_STEP_US);
}
