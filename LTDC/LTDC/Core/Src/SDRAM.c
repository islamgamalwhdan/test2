/*
 * SDRAM.c
 *
 *  Created on: Feb 22, 2021
 *      Author: Islam
 */

#include "SDRAM.h"

static SDRAM_HandleTypeDef hsdram1;
static FMC_SDRAM_CommandTypeDef command;

static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);


void Init_SD_RAM(void)
{
	FMC_SDRAM_TimingTypeDef SdramTiming = {0};


	  /** Perform the SDRAM1 memory initialization sequence
	  */
	  hsdram1.Instance = FMC_SDRAM_DEVICE;
	  /* hsdram1.Init */
	  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
	  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
	  /* SdramTiming */
	  SdramTiming.LoadToActiveDelay = 2;
	  SdramTiming.ExitSelfRefreshDelay = 7;
	  SdramTiming.SelfRefreshTime = 4;
	  SdramTiming.RowCycleDelay = 7;
	  SdramTiming.WriteRecoveryTime = 3;
	  SdramTiming.RPDelay = 2;
	  SdramTiming.RCDDelay = 2;

	  HAL_SDRAM_Init(&hsdram1, &SdramTiming);
	  SDRAM_Initialization_Sequence(&hsdram1, &command);

}

static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 4: Insert 100 ms delay */
  HAL_Delay(100);

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 4;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}



