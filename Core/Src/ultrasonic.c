#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stm32f767xx.h>

/*
 * TIM1 - PE9 GPIO - G0  //UL front
 * TIM9 - PE6 GPIO - G1  //UL left
 * TIM3 - PA6 GPIO - D0  //UL right
 * TIM4 - PB6 GPIO - D1  //UL back
 *
 */


//UL 1
uint32_t Value1_1 = 0;
uint32_t Value2_1 = 0;
uint32_t d_1 = 0;
uint8_t capture_value_first_1 = 0;
uint32_t distance_1 = 0;

//UL 2
uint32_t Value1_2 = 0;
uint32_t Value2_2 = 0;
uint32_t d_2 = 0;
uint8_t capture_value_first_2 = 0;
uint32_t distance_2 = 0;

//UL3
uint32_t Value1_3 = 0;
uint32_t Value2_3 = 0;
uint32_t d_3 = 0;
uint8_t capture_value_first_3 = 0;
uint32_t distance_3 = 0;

//UL4
uint32_t Value1_4 = 0;
uint32_t Value2_4 = 0;
uint32_t d_4 = 0;
uint8_t capture_value_first_4 = 0;
uint32_t distance_4 = 0;


//average variables.
uint32_t average_1 = 0;
uint32_t average_2 = 0;
uint32_t average_3 = 0;
uint32_t average_4 = 0;

uint8_t count1 = 0;
uint8_t count2 = 0;
uint8_t count3 = 0;
uint8_t count4 = 0;

volatile uint32_t final_value1 =0;
volatile uint32_t final_value2 =0;
volatile uint32_t final_value3 =0;
volatile uint32_t final_value4 =0;

#define  TIM_INPUTCHANNELPOLARITY_RISING     0x00000000U

void input_capture_callback_U1(void)
{
	       if (capture_value_first_1==0) // if the first value is not captured_1
			{
				Value1_1 = TIM1->CCR1;
				capture_value_first_1 = 1;
				TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM1->CCER |= (TIM_CCER_CC1P); // set to falling ed_1ge
			}

			else if (capture_value_first_1==1)   // if the first is alread_1y captured_1
			{
				Value2_1 = TIM1->CCR1;;  // read_1 second_1 value
				TIM1->CNT = 0; // reset counter

				if (Value2_1 > Value1_1)
					d_1 = Value2_1-Value1_1;

				if (Value1_1 > Value2_1)
					d_1 = (0xffff - Value1_1) + Value2_1;

				if(count1 < 5)
				{
					distance_1 = d_1 * .034/2;
					average_1 = average_1 + distance_1;
					++count1;
				}else{
					final_value1 = average_1;
					count1 = 0;
				}
				
				TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM1->CCER |= (TIM_INPUTCHANNELPOLARITY_RISING); // set to rising ed_1ge
				TIM1->DIER &= ~(TIM_DIER_CC1IE); //disable interrupt
				capture_value_first_1 = 0;
			}
}


void input_capture_callback_U2(void)
{
	       if (capture_value_first_2==0) // if the first value is not captured_1
			{
				Value1_2 = TIM9->CCR1;
				capture_value_first_2 = 1;
				TIM9->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM9->CCER |= (TIM_CCER_CC1P); // set to falling ed_1ge
			}

			else if (capture_value_first_2==1)   // if the first is alread_1y captured_1
			{
				Value2_2 = TIM9->CCR1;;  // read_1 second_1 value
				TIM9->CNT = 0; // reset counter

				if (Value2_2 > Value1_2)
					d_2 = Value2_2-Value1_2;

				if (Value1_2 > Value2_2)
					d_2 = (0xffff - Value1_2) + Value2_2;


				if(count2 < 5)
				{
					distance_2 = d_2 * .034/2;
					average_2 = average_2 + distance_2;
					++count2;
				}else{
					final_value2 = average_2;
					count2 = 0;
				}

				
				TIM9->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM9->CCER |= (TIM_INPUTCHANNELPOLARITY_RISING); // set to rising ed_1ge
				TIM9->DIER &= ~(TIM_DIER_CC1IE); //disable interrupt
				capture_value_first_2 = 0;
			}
}


void input_capture_callback_U3(void)
{
	       if (capture_value_first_3==0) // if the first value is not captured_1
			{
				Value1_3 = TIM3->CCR1;
				capture_value_first_3 = 1;
				TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM3->CCER |= (TIM_CCER_CC1P); // set to falling ed_1ge
			}

			else if (capture_value_first_3==1)   // if the first is alread_1y captured_1
			{
				Value2_3 = TIM3->CCR1;;  // read_1 second_1 value
				TIM3->CNT = 0; // reset counter

				if (Value2_3 > Value1_3)
					d_3 = Value2_3-Value1_3;

				if (Value1_3 > Value2_3)
					d_3 = (0xffff - Value1_3) + Value2_3;


				if(count3 < 5)
				{
					distance_3 = d_3 * .034/2;
					average_3 = average_3 + distance_3;
					++count3;
				}else{
					final_value3 = average_3;
					count3 = 0;
				}

				
				TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM3->CCER |= (TIM_INPUTCHANNELPOLARITY_RISING); // set to rising ed_1ge
				TIM3->DIER &= ~(TIM_DIER_CC1IE); //disable interrupt
				capture_value_first_3 = 0;
			}
}


void input_capture_callback_U4(void)
{
	       if (capture_value_first_4==0) // if the first value is not captured_1
			{
				Value1_4 = TIM4->CCR1;
				capture_value_first_4 = 1;
				TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM4->CCER |= (TIM_CCER_CC1P); // set to falling ed_1ge
			}

			else if (capture_value_first_4==1)   // if the first is alread_1y captured_1
			{
				Value2_4 = TIM4->CCR1;;  // read_1 second_1 value
				TIM4->CNT = 0; // reset counter

				if (Value2_4 > Value1_4)
					d_4 = Value2_4-Value1_4;

				if (Value1_4 > Value2_4)
					d_4 = (0xffff - Value1_4) + Value2_4;


				if(count4 < 5)
				{
					distance_4 = d_4 * .034/2;
					average_4 = average_4 + distance_4;
					++count4;
				}else{
					final_value4 = average_4;
					count4 = 0;
				}

				
				TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); //reset
				TIM4->CCER |= (TIM_INPUTCHANNELPOLARITY_RISING); // set to rising ed_1ge
				TIM4->DIER &= ~(TIM_DIER_CC1IE); //disable interrupt
				capture_value_first_4 = 0;
			}
}
