/**
  *********************************************************************************
  * @file    Prog_QSPI_Flash_Via_Bootloader/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-May-2016
  * @brief   This sample code will be downloaded in embedded SRAM and used to 
  *          program an external Quad-SPI flash memory via the UART bootloader 
  *          protocol.         
  *********************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
  *********************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* QSPI */
QSPI_HandleTypeDef QSPIHandle;
QSPI_CommandTypeDef sCommand;
QSPI_MemoryMappedTypeDef sMemMappedCfg;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Jump */
typedef  void (*pFunction)(void);
pFunction JumpToApplication;


/* flash */
FLASH_OBProgramInitTypeDef OBInit;

/* Private variables ---------------------------------------------------------*/
__IO uint32_t SectorsWRPStatus = 0xFFF;
uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void Send_Byte(uint8_t Byte);
void Select_Command(void);
void UART_Config(void);
void QuadSPI_Init (void);
uint8_t Get_Byte(void);
uint8_t Verify_Checksum(uint8_t *T,uint8_t size);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi); 

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization: global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock to 100 MHz */
  SystemClock_Config();
	
	/* Suspend Tick increment */
  HAL_SuspendTick();
	
  /* Configure leds */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
	
  /* Configure the UART peripheral */
	UART_Config();
		
	/* Led1 ON to signal the start of the application*/
	 BSP_LED_On(LED1);
	
	/* Initialize the QSPI peripheral */
	 QuadSPI_Init();
	
	/* Select the appropriate command to execute */
	 Select_Command();

  /* Infinite loop */  
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 100
  *            PLL_P                          = 2
  *            PLL_Q                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale3 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
	
 /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
	
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
		/* Toggle LED3 for error */
		while(1)
		{
			BSP_LED_Toggle(LED3);
		}
}
/**
  * @brief  This function receives a byte from the PC HyperTerminal and stores it in a buffer.
  * @param  None
  * @retval The received byte
  */
uint8_t Get_Byte(void)
{
	/* Buffer used for reception */
  uint8_t aRxBuffer[1];
	
	if(HAL_UART_Receive(&UartHandle, (uint8_t *)aRxBuffer, 1, 0xFFFF) != HAL_OK)
  {
			/* Transfer error in reception process */
			Error_Handler();
  }
  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
  {
  }
  return aRxBuffer[0];
}

/**
  * @brief  This function sends a byte to the PC HyperTerminal.
  * @param  The byte to be sent
  * @retval None
  */
void Send_Byte(uint8_t Byte)
{
	 uint8_t aTxBuffer[1];

	 aTxBuffer[0] = Byte;
	 if(HAL_UART_Transmit(&UartHandle, (uint8_t *) aTxBuffer , 1, 0xFFFF)!= HAL_OK)
	 {
			/* Transfer error in transmission process */
			Error_Handler();
	 }
	 while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	 {
	 }	
}

/**
  * @brief  This function calculates the checksum of the received address.
  * @param  the table where the address is stored and the size of this table.
  * @retval Calculated checksum.
  */
uint8_t Verify_Checksum(uint8_t *T,uint8_t size)
{  
	 uint8_t cnt;
   uint8_t checksum;
   checksum = T[0];
	
   for (cnt=1; cnt<size-1;cnt++)
	 {
		checksum = checksum ^ T[cnt];
	 }	 
   return checksum;
}

/**
  * @brief  This function Selects which command to apply (write, read etc...)
  * @param  None
  * @retval None
  */
void Select_Command(void)
{
uint8_t current_command =0;	
uint8_t counter, counter1, counter2;
uint8_t nb_minus1,bytes_number,nb_plus1;
uint8_t sector_number=0;
uint8_t step1=0, step2=0, nb_sector=0;                           
uint8_t size_sec=0;
uint8_t	read_check_host=0, read_check_MCU=0;
uint8_t complement, check_addr, pre_check, checksum;
static uint8_t a_address_buffer[5];
static uint8_t a_read_buffer[256];  /* The maximum length of the block to be read for the STM32 is 256 bytes */
static uint8_t a_write_buffer[257]; /* The maximum length of the block to be written for the STM32 is 256 bytes +1 checksum */
static uint8_t a_erase_buffer[1025];/* The used QSPI memory contains 512 sectors -> 1025 = 512*2 + 1checksum */
uint32_t erase_address=0x00, read_address=0x00, write_address=0x00, go_address=0x00;
uint32_t* p_read_address=&read_address;
	 
while (1)
{
    complement=0;
    current_command = Get_Byte();
    switch (current_command)
    {
			
        /**** wait until 0x7F is received on USARTx_RX : USART protocol is used ****/
        case (UART_CHECK):  
        {				
              /* Send the Achnowledgement byte : STM32 is ready to receive commands */
              Send_Byte(ACK_BYTE); 
              break;				
        }
			
        /*************** Get Command **************/
        case (GET_CMD):  
        {				
              complement = (0xFF ^ GET_CMD); /* calculate the Get command complemented byte*/
              if (Get_Byte()== complement)   /* receive the complemented byte from the host and check if it's equal to the calculated one */
              {						
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);
					
                    /* Send the number of bytes to be sent (1 Version + 6 Commands) - 1 */
                    Send_Byte(0x06); 
					
                    /* Send the version */
                    Send_Byte(VERSION_NUMBER);
					
                    /* Send the Supported Commands */
                    Send_Byte(0x00);  /* command 1 : Get Command */
                    Send_Byte(0x02);  /* command 2 : Get device ID */
                    Send_Byte(0x11);  /* command 3 : Read Memory */
                    Send_Byte(0x21);  /* command 4 : Go command */    
                    Send_Byte(0x31);  /* command 5 : Write command */
                    Send_Byte(0x44);  /* command 6 : Extended Erase */
					
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);						
              }
              else  /* The received GET command complement is not correct */
              {						 
                    /* Send the Not Achnowledgement byte */
                    Send_Byte(NACK_BYTE);							 
              }				 			
              break;	 
        }	
        /*************** End Get_Command **************/	
			
        /*************** Get ID Command **************/
        case (GID_CMD):  
        {
              complement = (0xFF ^ GID_CMD);  /* calculate the GetID complemented byte*/
              if (Get_Byte()== complement)    /* check if received complemented byte is equal to the calculated one */
              {						
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);
					
                    /* Send the number of bytes to be send - 1 */
                    Send_Byte(0x01);
					
                    /* Send the PID */
                    Send_Byte(0x04);
                    Send_Byte(0x21);
					
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);					
              }
              else  /* The received GET ID command complement is not correct */
              {
                    /* Send the Not Achnowledgement byte */
                    Send_Byte(NACK_BYTE);
              }
              break;				
        }	
        /*************** End Get ID Command **************/
			
        /***************  GO Command **************/
        case (GO_CMD):  
        {
             counter=0;
             check_addr=0;	
             complement = (0xFF ^ GO_CMD); /* calculate the Go command complemented byte*/
             if (Get_Byte()== complement)  /* check if received complemented byte is equal to the calculated one */
             {
					
                 /* Verify if Read protection is not active*/ 
                 HAL_FLASHEx_OBGetConfig(&OBInit);
                 SectorsWRPStatus = OBInit.RDPLevel;
                 if (SectorsWRPStatus == 0xAA)
                 {
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);
						
                    /* Receive the Go address and the checksum byte */
                    for (counter=0; counter<5;counter++)
                    {
                          a_address_buffer[counter]=Get_Byte();
                    }
							
                    /* Calculate the Go address checksum */
                    check_addr = Verify_Checksum(a_address_buffer,5);
							
                    /* Calculate Go address */							
                    go_address = (a_address_buffer[0]*0x1000000) + (a_address_buffer[1]*0x10000) + (a_address_buffer[2]*0x100) + a_address_buffer[3];

                    /* Check if the go address is within the QSPI memory area and whether the checksum is correct*/
                    if ((go_address<0x91FFFFFF) && (check_addr == a_address_buffer[4]))							       
                    {			
                          /* Send the Achnowledgement byte */
                          Send_Byte(ACK_BYTE);	

    											sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
                          sCommand.Instruction       = QUAD_INOUT_FAST_READ_CMD;
                          sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
                          sCommand.Address           = 0;
                          sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
                          sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
                          sCommand.DataMode          = QSPI_DATA_4_LINES;
                          sCommand.DummyCycles       = 10;
                          sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
                          sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
                          sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
                          sCommand.NbData            = 1;					
					
                          sMemMappedCfg.TimeOutPeriod = 0xFFFF;
                          sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
                          /* The user application to be executed is stored in the QSPI memory so memory mapped mode should be configured.*/
                          if (HAL_QSPI_MemoryMapped(&QSPIHandle, &sCommand, &sMemMappedCfg) != HAL_OK)
                          {
                               Error_Handler();
                          }						

                          /* Jump to user application */
                          JumpAddress = *(__IO uint32_t*) (go_address+ 4);
                          JumpToApplication = (pFunction) JumpAddress;
											
                          /* Initialize user application's Stack Pointer */
                          __set_MSP(*(__IO uint32_t*) go_address);
                          JumpToApplication();
											
                          while (1)
                          {
														  /* This loop will be reached only if an error occur :
														  jump doesn't happen or execution of program in the new erea has crashed*/
                          }	
                    }
                    else  /* Address not valid OR checksum incorrect */
                    {
                          /* Send the Not Achnowledgement byte */
                          Send_Byte(NACK_BYTE);
                    }
                 }
                 else  	/* Read protection is active*/ 
                 {
                    /* Send the Not Achnowledgement byte */	
                    Send_Byte(NACK_BYTE);
                 }
             }
             else   /* The received and the calculated Go command complement are not equal */
             {
                 /* Send the Not Achnowledgement byte */						
                 Send_Byte(NACK_BYTE);
             }
             break;
        }	
        /***************** End GO Command ****************/	
			
        /*************** Read Memory Command **************/		
        case (RM_CMD):         
        {
             counter=0;
             check_addr=0;
             nb_minus1=bytes_number=0;					
             complement = (0xFF ^ RM_CMD); /* calculate the Read command complemented byte*/
             if (Get_Byte()== complement)  /* check if received complemented byte is equal to the calculated one */
             {					
                 /* Verify if Read protection is not active*/ 
                 HAL_FLASHEx_OBGetConfig(&OBInit);
                 SectorsWRPStatus = OBInit.RDPLevel;
                 if (SectorsWRPStatus == 0xAA)
                 {
                    /* Send the Achnowledgement byte */
                    Send_Byte(ACK_BYTE);
						
                    /* Receive the start @ (4 Bytes) and the checksum */
                    for (counter=0; counter<5;counter++)
                    {
                          a_address_buffer[counter]=Get_Byte();
                    }
							
                    /* Calculate the Read address checksum */
                    check_addr = Verify_Checksum(a_address_buffer,5);
							
                    /* Calculate Read address */							
                    read_address = (a_address_buffer[0]*0x1000000) + (a_address_buffer[1]*0x10000) + (a_address_buffer[2]*0x100) + a_address_buffer[3];

                    /* Check if the read address is in a valid area and whether the checksum is correct*/
                    if ((((read_address>=0x08000000) && (read_address<0x081FFFFF)) || \
                          (read_address==0x1FFF76DE) || \
                         ((read_address>=0x20000000) && (read_address<0x2001FFFF)))&& \
                          (check_addr == a_address_buffer[4])) 
                    {
                          /* Send the Achnowledgement byte */
                          Send_Byte(ACK_BYTE);
								
                          /* Number of bytes to be read - 1 */
                          nb_minus1 = Get_Byte ();
                          /* Number of bytes to be read */
                          bytes_number = nb_minus1 + 1 ;
								
                          /* receive the checksum from the PC HyperTerminal */
                          read_check_host = Get_Byte ();	
								
                          /* Checksum calculated by STM32 */
                          read_check_MCU = (0xFF ^ nb_minus1);	
								
                          /* Check if the recieved and the calculated Read checksum are identical*/
                          if ( read_check_host == read_check_MCU)
                          {
                               /* Send the Achnowledgement byte */
                               Send_Byte(ACK_BYTE);	
										
                               /* Reading sequence : From flash or RAM memory */
                               if(HAL_UART_Transmit(&UartHandle, (uint8_t*)*p_read_address, bytes_number, 0xFFFF)!= HAL_OK)
                               {
                                     while(1)
                                     {
                                     }										
                               }
                          }
                          else  /* Recieved and calculated Read checksum not identical */
                          {
                               /* Send the Not Achnowledgement byte */
                               Send_Byte(NACK_BYTE);
                          }
                    }
                    /* Verify that the read address is within the QSPI memory area and that the received checksum is correct */
                    else if ((read_address<0x91FFFFFF) && (check_addr == a_address_buffer[4])) 	
                    {
                          read_address= read_address - 0x90000000;
								
                          /* Send the Achnowledgement byte */						
                          Send_Byte(ACK_BYTE);
								
                          /* Number of bytes to be read - 1 */
                          nb_minus1 = Get_Byte ();
                          /* Number of bytes to be read */
                          bytes_number = nb_minus1 + 1 ;	
								
                          /* receive the checksum from the PC HyperTerminal */
                          read_check_host = Get_Byte ();
								
                          /* Checksum calculated by STM32 */
                          read_check_MCU = (0xFF ^ nb_minus1);
                          
                          /* Check whether the recieved and the calculated checksum are identical*/
                          if ( read_check_host == read_check_MCU)
                          {
                               /* Send the Achnowledgement byte */									
                               Send_Byte(ACK_BYTE);	
										
                               /* Configure Volatile Configuration register (with new dummy cycles) */
                               QSPI_DummyCyclesCfg(&QSPIHandle);
											
                               /* Reading Sequence : From QSPI memory */
                               sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
                               sCommand.DataMode    = QSPI_DATA_4_LINES  ;
                               sCommand.Instruction = QUAD_INOUT_FAST_READ_CMD;
                               sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
                               sCommand.Address     = read_address;		
                               sCommand.NbData      = bytes_number;
                               if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
                               {
                                     Error_Handler();
                               }
											
                               /* STM32 receives "bytes_number" data from QSPI memory and stores it in "a_read_buffer" buffer */
                               if (HAL_QSPI_Receive(&QSPIHandle, a_read_buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
                               {
                                     Error_Handler();
                               }
											
                               /* STM32 sends "bytes_number" data from "a_read_buffer" to the PC HyperTerminal*/
                               if(HAL_UART_Transmit(&UartHandle, a_read_buffer, bytes_number, 0xFFFF)!= HAL_OK)
                               {
                                     while(1)
                                     {
                                     }      
                               }							     	
                          }	
                          else  /* Recieved and calculated Read checksum not identical */
                          {
                               /* Send the Not Achnowledgement byte */
                               Send_Byte(NACK_BYTE);
                          }									
                    }
                    else  /* Read address not valid OR checksum incorrect */
                    {
                          /* Send the Not Achnowledgement byte */								 
                          Send_Byte(NACK_BYTE);
                    }
                 }
                 else  /* Read protection is active*/ 
                 {
                    /* Send the Not Achnowledgement byte */	
                    Send_Byte(NACK_BYTE);
                 }
             }
             else  /* The received and the calculated Read command complement are not equal */
             {
                 /* Send the Not Achnowledgement byte */	
                 Send_Byte(NACK_BYTE);
             }
             break;
        }
        /*************** End RM_Command **************/	
		
        /************ Write Memory Command ***********/						
        case (WM_CMD):        
        {
             counter=0;
             counter1=0;
             check_addr=pre_check=checksum=0;		
             nb_minus1=bytes_number=nb_plus1=0;							
             complement = (0xFF ^ WM_CMD);  /* calculate the Write command complemented byte*/
             if (Get_Byte()== complement)   /* check if received complemented byte is equal to the calculated one */
             {
                 /* Verify if Read protection is not active*/ 
                 HAL_FLASHEx_OBGetConfig(&OBInit);
                 SectorsWRPStatus = OBInit.RDPLevel;
                 if (SectorsWRPStatus == 0xAA)
                 {
                    /* Send the Achnowledgement byte */						
                    Send_Byte(ACK_BYTE);
						
                    /* Receive the start address (4 Bytes) and the checksum */
                    for (counter=0; counter<5;counter++)
                    {
                          a_address_buffer[counter]=Get_Byte();
                    }
						
                    /* Checksum calculated by STM32 */
                    check_addr = Verify_Checksum(a_address_buffer,5);
                    /* Calculate Write address */														
                    write_address = (a_address_buffer[0]*0x1000000) + (a_address_buffer[1]*0x10000) + (a_address_buffer[2]*0x100) + a_address_buffer[3];  
                    /* Verify that the write address is within the QSPI memory area and that the received checksum is correct */
                    if ((check_addr == a_address_buffer[4]) && (write_address <0x91FFFFFF) )
                    {	
												 /* Send the Achnowledgement byte */													
												 Send_Byte(ACK_BYTE);
												 write_address= write_address - 0x90000000;
						
												 /* Receive number of bytes to be written - 1 */
												 nb_minus1 = Get_Byte ();
												 /* Number of bytes to be written */
												 bytes_number = nb_minus1 + 1 ;
												 /* Datas : Nb datas to be received : bytes_number + 1 checksum */
												 nb_plus1=bytes_number+1;
							
												 /* Receive the datas to be written and the checksum (XOR of all the received bytes) */ 									
												 for (counter1=0; counter1<nb_plus1; counter1++)
												 {												
															 a_write_buffer[counter1]=Get_Byte();
												 } 
							
												 /* Checksum calculated by STM32 */
												 pre_check = Verify_Checksum(a_write_buffer,nb_plus1);
												 checksum = pre_check ^ nb_minus1;

												 /* Check whether the recieved and the calculated checksum are identical :
												 checksum = XOR of all datas to be written in QSPI memory */
												 if (checksum == a_write_buffer[bytes_number])
												 {
															 /* Enable write operations : Deactivate the QSPI memory protection */
															 QSPI_WriteEnable(&QSPIHandle);
								
															 /* Writing Sequence : To QSPI */
															 sCommand.Instruction = EXT_QUAD_IN_FAST_PROG_CMD;
															 sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
															 sCommand.DataMode    = QSPI_DATA_4_LINES;
															 sCommand.NbData      = bytes_number;
															 sCommand.Address     = write_address;
															 sCommand.DummyCycles = 0;

															 if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
															 {
																		 Error_Handler();
															 }
									
															 /* STM32 writes "bytes_number" data saved in "a_write_buffer" buffer in the chosen QSPI address */												
															 if (HAL_QSPI_Transmit(&QSPIHandle, a_write_buffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
															 {
																		 Error_Handler();
															 }
									
															 /* Configure automatic polling mode to wait for end of program */  
															 QSPI_AutoPollingMemReady(&QSPIHandle);
									
															 /* Send the Achnowledgement byte */																						
															 Send_Byte(ACK_BYTE);	
												 }
												 else  /* Calculated and received checksums are diffrent */
												 {
															 /* Send the Not Achnowledgement byte */																																
															 Send_Byte(NACK_BYTE);
												 }
                    }
                    else   /* Write address not valid OR checksum incorrect */
                    {
                          /* Send the Not Achnowledgement byte */																																						 			 
                          Send_Byte(NACK_BYTE);
                    }
                 }
                 else  /* Read protection is active*/ 
                 {	
                    /* Send the Not Achnowledgement byte */																																						 			 
                    Send_Byte(NACK_BYTE);
                 }
             }
             else  /* The received and the calculated Write command complement are not equal */
             {
                 /* Send the Not Achnowledgement byte */																																						 			 	
                 Send_Byte(NACK_BYTE);
             }
             break;
        } 
        /*************** End WM_Command ***************/
		
        /********Extended Erase Memory Command ********/			
        case (ER_CMD):         
        {
             counter=0;
					   counter1=0;
					   counter2=0;
             pre_check=checksum=0;
             complement = (0xFF ^ ER_CMD);   /* calculate the Extended Erase command complemented byte*/
             if (Get_Byte()== complement)    /* check if received complemented byte is equal to the calculated one */
             {
                 /* Verify if Read protection is not active*/ 					
                 HAL_FLASHEx_OBGetConfig(&OBInit);
                 SectorsWRPStatus = OBInit.RDPLevel;
                 if (SectorsWRPStatus == 0xAA)
                 {
                    /* Send the Achnowledgement byte */														
                    Send_Byte(ACK_BYTE);	

									 /* Initialize Reception buffer : Erase all the previous contents of "a_read_buffer" */
                    for (counter2 = 0; counter2 < READ_BUFFER_SIZE; counter2++) 
                    {
											a_read_buffer[counter2] = 0;
                    }
									
                    /* Receive the number of sectors to be erased : coded on two bytes */
                    step1 = Get_Byte();
                    step2 = Get_Byte();
									
                    /* Special Erase command : mass erase (0xFFFF) 
                    -> Not supported */
                    if(step1==0xFF && step2==0xFF)
                    {
											if(Get_Byte()== 0) /* Checksum of 0xFFFF */
											{
													 /* Send the Achnowledgement byte */																			 
													 Send_Byte(NACK_BYTE);
													 break; 
											}
                    }	
                    else  /* Fixed number of sectors to erase */
                    {										
											/* Nb of sectors to erase-1 */								
											nb_sector=(step1*0x100)+step2;
											/* Receive each sector number to erase (coded on 2 bytes) + checksum of all the received bytes */									
											size_sec=(nb_sector+1)*2 + 1;
											for (counter=0; counter<size_sec;counter++)
											{
												 a_erase_buffer[counter]=Get_Byte();
											}	
										
											/* Checksum calculated by STM32 */
											pre_check = Verify_Checksum(a_erase_buffer,size_sec);
											checksum=pre_check ^ step1 ^ step2;
										
											/* Check whether the recieved and the calculated checksum (of all the received bytes) are identical */
											if ( checksum == a_erase_buffer[size_sec-1])
											{
												 /*Erase the chosen sectors : sector by sector */
												 for (counter1=0; counter1<(size_sec-1); counter1++)
												 {
														/* Calculate the number of the sector to erase */
														sector_number = a_erase_buffer[counter1]*0x100 + a_erase_buffer[counter1+1];
																				
														/* Calculate the start address of the sector to erase */
														erase_address = (sector_number * 64*1024); /* earch sector's size is 64KB*/
											
														counter1=counter1+1; /*used because number of sectors to erase and all sector codes are coded on two bytes.*/

														/* Enable write operations */
														QSPI_WriteEnable(&QSPIHandle);
																				
														/* Erasing Sequence : from QSPI */
														sCommand.Instruction = SECTOR_ERASE_CMD;
														sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
														sCommand.Address     = erase_address;
														sCommand.DataMode    = QSPI_DATA_NONE;
														sCommand.DummyCycles = 0;
														if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
														{
															Error_Handler();
														}
															/* Configure automatic polling mode to wait for end of erase */
														QSPI_AutoPollingMemReady(&QSPIHandle);

												 }																															 
												 /* Send the Achnowledgement byte : All chosen sectors are erased */
												 Send_Byte(ACK_BYTE);
											}
											else  /* Calculated and received checksums are different */
											{
													 /* Send the Not Achnowledgement byte */																						
													 Send_Byte(NACK_BYTE);
											}
                    }
                 }
                 else /* Read protection is active*/ 
                 {
                    /* Send the Not Achnowledgement byte */																						
                    Send_Byte(NACK_BYTE);
                 }
             }		
             else  /* The received and the calculated Extended Erase command complement are not equal */
             {
                 /* Send the Not Achnowledgement byte */																						
                 Send_Byte(NACK_BYTE);
             }
             break;
        } 
					
        /************ End EM_Command ***********/	
        default: 
        {
           break;	
        }
				
    } /* End switch */		
}
}		

/**
  * @brief  This function configures the UART peripheral.
  * @param  None
  * @retval None
  */
void UART_Config(void)
{
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 9 Bits (8 data bit + 1 parity bit) : 
	                    BE CAREFUL : Program 8 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = Even parity
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
			
  UartHandle.Instance          = USARTx;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_9B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_EVEN;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
			/* Initialization Error */
			Error_Handler();
  }
	
}	
/**
  * @brief  This function initializes the QuadSPI.
  * @param  None
  * @retval None
  */
void QuadSPI_Init(void)
{	
 /* Initialize QuadSPI */
  QSPIHandle.Instance = QUADSPI;
  HAL_QSPI_DeInit(&QSPIHandle);
        
  QSPIHandle.Init.ClockPrescaler     = 2;
  QSPIHandle.Init.FifoThreshold      = 4;
  QSPIHandle.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  QSPIHandle.Init.FlashSize          = QSPI_FLASH_SIZE;
  QSPIHandle.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  QSPIHandle.Init.ClockMode          = QSPI_CLOCK_MODE_0;

  if (HAL_QSPI_Init(&QSPIHandle) != HAL_OK)
  {
    Error_Handler();
  }

  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

}

/**
  * @brief  This function sends a Write Enable and waits it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure automatic polling mode to wait for write enabling */  
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&QSPIHandle, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function configures the dummy cycles on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Read Volatile Configuration register */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Receive(&QSPIHandle, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable write operations */
  QSPI_WriteEnable(&QSPIHandle);

  /* Write Volatile Configuration register (with new dummy cycles) */  
  sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
  MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ_QUAD << POSITION_VAL(0xF0)));
      
  if (HAL_QSPI_Command(&QSPIHandle, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Transmit(&QSPIHandle, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  This function reads the SR of the memory and waits the EOP.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready */  
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode         = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&QSPIHandle, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
