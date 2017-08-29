/**
  @page Use of built-in bootloader to program an external QSPI memory
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics ************************
  * @file    readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-May-2016
  * @brief   Description of how to program an external memory using a user-bootcode 
  *          based on the built-in UART bootloader protocol.
  ***********************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This example is provided as an example firmware for application note AN4852
"Programming an external Flash memory using the UART bootloader built-in 
STM32 microcontrollers"

This example describes how to program an external QSPI flash memory using a developed
user boot-code based on the internal UART bootloader protocol.

Tx Pin: PA.09
Rx Pin: PA.10
   _______________________________________________
  |STM32_Board                            ________|                    _______________
  |                                      | USART1 |                   | HyperTerminal |
  |                                      |        |                   |               |
  |                                      |      TX|___________________|RX             |
  |                                      |        |                   |               |
  |                                      |        |                   |               |             
  |                                      |      RX|___________________|TX             |
  |                                      |________|                   |               |
  |__________                                     |                   |_______________|           
  |  System  |       ________        ___________  |  
  |  Memory  |      |  SRAM  |      |          |  |  
  |          |      |        |      |          |  |
  | built-in |      |        |      |   QSPI   |  |
  |bootloader|----->|Bootcode|----->|          |  |
  |_________ |______|________|______|__________|__|

The free ST flash loader demonstrator tool will be used in this application.
-> You can find the tool under the path below: Utilities/PC_Software/FlashLoaderDemonstrator.
or you can simply download it from ST website "www.st.com" 

Some steps must be taken to program the external QSPI flash using the built-in bootloader: 
1- Adding the necessary map files: 
   - The ST flash loader tool is configured by default for reprogramming the internal 
   device flash memory; So, in order to be able to load the bootcode image into the 
   RAM memory and to program the external QSPI flash memory, 2 new mapping descriptor  
   files should be added to the map directory: One for the SRAM memory and the other 
   for the QSPI memory.  
 
   - Two ready (modified) map descriptor files entitled "STM32F4_46_512K_RAM" 
   and "STM32F4_46_512K_QSPI" are available under this folder:
   Projects/STM32446E_EVAL/Prog_QSPI_Flash_Via_Bootloader/Map.
   The first give the user 116KB of SRAM where the bootcode will be downloaded;
   And the second one adds the first 256 sectors of the QSPI memory. 
  
   - Place the two files in the "Map" directory located in the installation 
   directory of the flash loader demonstrator: "C:\...\Flash Loader Demo\Map"			

2- Loading the code into RAM: Open the flash loader demonstrator and use it to load  
   the binary file of the project into SRAM memory and to jump to execute it.
									  
3- At this stage, the code is loaded in RAM memory and the CPU is waiting for the user 
   to send the command codes to perform read, write, Go or erase operations to/from 
   the QSPI memory.
   Just be careful, before writing in any memory area of the QSPI, you have to erase 
   its content.

Please refer to the application note AN4852 for more information: The three steps 
previously listed are further detailed in the second part titled : 
"How to use the user boot code?".

@note The user bootcode allows to program Only the first 256 sectors of the QSPI memory.

@note The mass erase is not supported: If you want to erase all Quad-SPI content, you have 
to erase it sector by sector.

STM32-Eval board's LEDs can be used to indicate:
 - That the jump to the user-bootcode is done correctly: LED1 is ON.
 - An error: LED3 is ON.

The UART is configured as follows:
    - BaudRate = 115200 baud  
    - Word Length = 9 Bits (8 data bit + 1 parity bit)
    - One Stop Bit
    - Even parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Reception and transmission are enabled in the time

@note USARTx/UARTx instance used and associated resources can be updated in "main.h"
      file depending hardware configuration used.
      
@par Directory contents 

  - Prog_QSPI_Flash_Via_Bootloader/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - Prog_QSPI_Flash_Via_Bootloader/Inc/main.h                  Main program header file  
  - Prog_QSPI_Flash_Via_Bootloader/Src/main.c                  Main program
  - Prog_QSPI_Flash_Via_Bootloader/Src/stm32f4xx_hal_msp.c     HAL MSP file
  - Prog_QSPI_Flash_Via_Bootloader/Src/system_stm32f4xx.c      STM32F4xx system source file

@par Hardware and Software environment

  - This example runs on STM32F446xx devices.
    
  - This example has been tested with STMicroelectronics STM32446E-EVAL board and can be
    easily tailored to any other supported device and development board.    
      
  - STM32446E-EVAL Set-up
    - If you will use RS232 cable: Connect a null-modem female/female RS232 cable between 
	  the DB9 connector CN25 (USART1) and PC serial port to ensure the UART communication.	  
         - Make sure that jumper JP4 pin1 (PA10) and pin2 (uart1_rx) are connected.   
         - Make sure that jumper JP6 pin2 (uart1_tx) and pin3 (rs232) are connected.   
         - Make sure that jumper JP7 pin1 (uart1_tx) and pin2 (PA9) are connected.   
         - Make sure that jumper JP8 pin1 (PA10) and pin2 (rs232) are connected.  
		 
    - If you will use STLINK virtual com: Jumpers JP4 and JP7 are the same as previously described.          
         - Make sure that jumper JP6 pin1 (stlk) and pin2 (uart1_tx) are connected.
         - Make sure that jumper JP8 pin5 (PA10) and pin6 (stlk) are connected.   

    - The STM32F446xx bootloader is activated by applying pattern 1:
         - Boot0(pin) = 1
         - Boot1(pin) = 0
		 
  - Flash loader demonstrator configuration:
    - Port Name = COM1
    - BaudRate = 115200 baud
    - Data bits = 8 Bits
    - Even parity
    - Echo: Disabled 
    - Timeout(s) = 5 (increase this parameter if you will erase a lot of sectors).

@par How to use it? 

In order to make the program work, you must do the following:

- EWARM:
    - Open the Project.eww workspace
    - Rebuild all files: Project->Rebuild all
    - A binary file "STM32446E_EVAL.bin" will be generated under "STM32446E_EVAL/Exe" folder. 
    - Use the flash loader demonstrator to load this (.bin) into SRAM memory and to jump to execute it. 
    - Send the convenient command code : Read, Write, Extended Erase...

- MDK-ARM:
    - Open the Project.uvproj project
    - Rebuild all files: Project->Rebuild all target files
    - A binary file "STM32446E_EVAL.bin" will be generated under "Binary" folder. 
    - Finally load this image with IAP application
    - Use the flash loader demonstrator to load this (.bin) into SRAM memory and to jump to execute it. 
    - Send the convenient command code : Read, Write, Extended Erase...

- SW4STM32
    - Open the SW4STM32 toolchain.
    - Browse to the SW4STM32 workspace directory, select the project (.project file in \SW4STM32\STM32446E_EVAL directory).
    - Rebuild all project files.
    - A binary file "STM32446E_EVAL.bin" will be generated under "STM32446E_EVAL\Debug" folder. 
    - Use the flash loader demonstrator to load this (.bin) into SRAM memory and to jump to execute it.
    - Send the convenient command code : Read, Write, Extended Erase...


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
