/**
  @page TIM_Input_Capture TIM Input Capture example 
  
  @verbatim
  ******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
  * @file    TIM/InputCapture/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Description of the TIM Input Capture example.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
   @endverbatim

@par Example Description 

This example shows how to use the TIM peripheral to measure the frequency of an 
external signal.

The TIMxCLK frequency is set to SystemCoreClock /2 (Hz), the Prescaler is 0 so the 
TIM1 counter clock is SystemCoreClock (Hz)/2.

SystemCoreClock is set to 120 MHz.

TIM1 is configured in Input Capture Mode: the external signal is connected to 
TIM1 Channel2 used as input pin.
To measure the frequency we use the TIM1 CC2 interrupt request,
so In the TIM1_CC_IRQHandler routine, the frequency of the external signal is computed. 
The "TIM1Freq" variable contains the external signal frequency:
TIM1Freq = TIM1 counter clock / Capture in Hz,
where the Capture is the difference between two consecutive TIM1 captures. 

The minimum frequency value to measure is 1100 Hz. 

@par Directory contents 

  - TIM/InputCapture/stm32f2xx_conf.h    Library Configuration file
  - TIM/InputCapture/stm32f2xx_it.c      Interrupt handlers
  - TIM/InputCapture/stm32f2xx_it.h      Interrupt handlers header file
  - TIM/InputCapture/main.c              Main program 
  - TIM/InputCapture/system_stm32f2xx.c  STM32F2xx system source file

@note The "system_stm32f2xx.c" is generated by an automatic clock configuration 
      tool and can be easily customized to your own configuration. 
      To select different clock setup, use the "STM32F2xx_Clock_Configuration_V1.0.0.xls" 
      provided with the AN3362 package available on <a href="http://www.st.com/internet/mcu/family/141.jsp">  ST Microcontrollers </a>
  
@par Hardware and Software environment 

  - This example runs on STM32F2xx Devices.
  
  - This example has been tested with STM322xG-EVAL RevB and can be easily tailored
    to any other development board.

  - STM322xG-EVAL Set-up 
    - Connect the external signal to measure to the TIM1 CH2 pin (PE.11).   
  
@par How to use it ? 

In order to make the program work, you must do the following :
 - Copy all source files from this example folder to the template folder under
   Project\STM32F2xx_StdPeriph_Template
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
   
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */


