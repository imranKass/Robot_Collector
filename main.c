/*****************************************************************************
*
* Copyright (C) 2013 - 2017 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the
*   distribution.
*
* * Neither the name of Texas Instruments Incorporated nor the names of
*   its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*
* MSP432 empty main.c template
*
******************************************************************************/



#include <stdint.h>
#include <math.h>
#include "msp432p401r.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/BLE_UART.h"
#include "inc/Robot_Arm.h"

#define PWM_NOMINAL 6000

#define PWM_PERIOD          60000   // 20ms period at 3MHz

void Process_BLE_UART_Data(char BLE_UART_Buffer[])
{
    /* LED Controls */
    if (Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED GREEN"))
    {
        LED2_Output(RGB_LED_GREEN);
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED OFF"))
    {
        LED2_Output(RGB_LED_OFF);
    }

    /* Motor Controls */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "MOVE FORWARD"))
    {
        Motor_Forward(4500, 4500);
        Clock_Delay1us(3000000);
        Motor_Stop();
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "MOVE BACKWARD"))
    {
        Motor_Backward(4500, 4500);
        Clock_Delay1us(3000000);
        Motor_Stop();
    }

    /* === HEIGHT SERVO TESTS === */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 0"))
    {
        Robot_Arm_Set_Height(0);
        BLE_UART_OutString("HEIGHT at 0 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 30"))
    {
        Robot_Arm_Set_Height(30);
        BLE_UART_OutString("HEIGHT at 30 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 60"))
    {
        Robot_Arm_Set_Height(60);
        BLE_UART_OutString("HEIGHT at 60 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 90"))
    {
        Robot_Arm_Set_Height(90);
        BLE_UART_OutString("HEIGHT at 90 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 120"))
    {
        Robot_Arm_Set_Height(120);
        BLE_UART_OutString("HEIGHT at 120 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 150"))
    {
        Robot_Arm_Set_Height(150);
        BLE_UART_OutString("HEIGHT at 150 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HEIGHT 180"))
    {
        Robot_Arm_Set_Height(180);
        BLE_UART_OutString("HEIGHT at 180 degrees\r\n");
    }

    /* === TILT SERVO TESTS === */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 0"))
    {
        Robot_Arm_Set_Tilt(0);
        BLE_UART_OutString("TILT at 0 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 30"))
    {
        Robot_Arm_Set_Tilt(30);
        BLE_UART_OutString("TILT at 30 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 60"))
    {
        Robot_Arm_Set_Tilt(60);
        BLE_UART_OutString("TILT at 60 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 90"))
    {
        Robot_Arm_Set_Tilt(90);
        BLE_UART_OutString("TILT at 90 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 120"))
    {
        Robot_Arm_Set_Tilt(120);
        BLE_UART_OutString("TILT at 120 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 150"))
    {
        Robot_Arm_Set_Tilt(150);
        BLE_UART_OutString("TILT at 150 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "TILT 180"))
    {
        Robot_Arm_Set_Tilt(180);
        BLE_UART_OutString("TILT at 180 degrees\r\n");
    }

    /* === GRIPPER SERVO TESTS === */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "GRIP 0"))
    {
        Robot_Arm_Set_Gripper(0);
        BLE_UART_OutString("GRIPPER at 0 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "GRIP 10"))
    {
        Robot_Arm_Set_Gripper(10);
        BLE_UART_OutString("GRIPPER at 10 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "GRIP 20"))
    {
        Robot_Arm_Set_Gripper(20);
        BLE_UART_OutString("GRIPPER at 20 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "GRIP 30"))
    {
        Robot_Arm_Set_Gripper(30);
        BLE_UART_OutString("GRIPPER at 30 degrees\r\n");
    }
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "GRIP 40"))
    {
        Robot_Arm_Set_Gripper(40);
        BLE_UART_OutString("GRIPPER at 40 degrees\r\n");
    }

    /* === AUTOMATED RANGE FINDER === */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "FIND HEIGHT"))
    {
        BLE_UART_OutString("=== FINDING HEIGHT SAFE RANGE ===\r\n");
        BLE_UART_OutString("Watch servo and note which angles work without buzzing\r\n\r\n");

        uint8_t test_angles[] = {0, 30, 60, 90, 120, 150, 180};
        int i;

        for(i = 0; i < 7; i++)
        {
            char msg[50];
            sprintf(msg, "Testing HEIGHT %d degrees...\r\n", test_angles[i]);
            BLE_UART_OutString(msg);

            Robot_Arm_Set_Height(test_angles[i]);
            Clock_Delay1ms(3000);  // 3 seconds per position
        }

        BLE_UART_OutString("\r\nHEIGHT range test complete!\r\n");
        BLE_UART_OutString("Note which angles worked without buzzing.\r\n\r\n");
    }

    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "FIND TILT"))
    {
        BLE_UART_OutString("=== FINDING TILT SAFE RANGE ===\r\n");
        BLE_UART_OutString("Watch servo and note which angles work without buzzing\r\n\r\n");

        uint8_t test_angles[] = {0, 10,20,30,40,50,60,70,80};
        int i;

        for(i = 0; i < 7; i++)
        {
            char msg[50];
            sprintf(msg, "Testing TILT %d degrees...\r\n", test_angles[i]);
            BLE_UART_OutString(msg);

            Robot_Arm_Set_Tilt(test_angles[i]);
            Clock_Delay1ms(3000);
        }

        BLE_UART_OutString("\r\nTILT range test complete!\r\n");
        BLE_UART_OutString("Note which angles worked without buzzing.\r\n\r\n");
    }

    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "FIND GRIP"))
    {
        BLE_UART_OutString("=== FINDING GRIPPER SAFE RANGE ===\r\n");
        BLE_UART_OutString("Watch servo and note which angles work without buzzing\r\n\r\n");

        uint8_t test_angles[] = {0, 30,60,90,120,150,180};
        int i;

        for(i = 0; i < 10; i++)
        {
            char msg[50];
            sprintf(msg, "Testing GRIPPER %d degrees...\r\n", test_angles[i]);
            BLE_UART_OutString(msg);

            Robot_Arm_Set_Gripper(test_angles[i]);
            Clock_Delay1ms(3000);
        }

        BLE_UART_OutString("\r\nGRIPPER range test complete!\r\n");
        BLE_UART_OutString("Note which angles worked without buzzing.\r\n\r\n");
    }

    /* Help Command */
    else if (Check_BLE_UART_Data(BLE_UART_Buffer, "HELP"))
    {
        BLE_UART_OutString("\r\n=== EMERGENCY DIAGNOSTIC COMMANDS ===\r\n");
        BLE_UART_OutString("\r\nIndividual Tests:\r\n");
        BLE_UART_OutString("  HEIGHT 0, HEIGHT 30, HEIGHT 60, HEIGHT 90, HEIGHT 120, HEIGHT 150, HEIGHT 180\r\n");
        BLE_UART_OutString("  TILT 0, TILT 30, TILT 60, TILT 90, TILT 120, TILT 150, TILT 180\r\n");
        BLE_UART_OutString("  GRIP 0, GRIP 10, GRIP 20, GRIP 30, GRIP 40\r\n");
        BLE_UART_OutString("\r\nAutomated Range Finders:\r\n");
        BLE_UART_OutString("  FIND HEIGHT - Tests HEIGHT 0-180 in steps\r\n");
        BLE_UART_OutString("  FIND TILT - Tests TILT 0-180 in steps\r\n");
        BLE_UART_OutString("  FIND GRIP - Tests GRIPPER 0-60 in steps\r\n");
        BLE_UART_OutString("\r\nWatch servos and note angles that work without buzzing!\r\n");
        BLE_UART_OutString("======================================\r\n\r\n");
    }

    else
    {
        printf("BLE Command Invalid!\n");
    }



}



/*void Timer_Init(void)
{
    P7->SEL0 |= 0x70;
    P7->SEL1 &= ~0x70;
    P7->DIR |= 0x70;

    TIMER_A1->CCR[0] = PWM_PERIOD - 1;
    TIMER_A1->CCTL[1] = 0x00E0;  // HEIGHT
    TIMER_A1->CCTL[2] = 0x00E0;  // TILT
    TIMER_A1->CCTL[3] = 0x00E0;  // GRIPPER

    // Start at middle
    TIMER_A1->CCR[1] = 4500;  // HEIGHT
    TIMER_A1->CCR[2] = 4500;  // TILT
    TIMER_A1->CCR[3] = 4500;  // GRIPPER

    TIMER_A1->CTL = 0x0214;
}*/

int main(void)
{
    int i;
    int string_size;
    char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    DisableInterrupts();
    Clock_Init48MHz();
    LED2_Init();
    EUSCI_A0_UART_Init_Printf();
    BLE_UART_Init();
    Motor_Init();
    Robot_Arm_Init();
    EnableInterrupts();

    Clock_Delay1ms(1000);
    BLE_UART_Reset();
    BLE_UART_OutString("\r\n");
    BLE_UART_OutString("========================================\r\n");
    BLE_UART_OutString("  EMERGENCY SERVO DIAGNOSTIC MODE\r\n");
    BLE_UART_OutString("========================================\r\n");
    BLE_UART_OutString("This will help find safe servo ranges\r\n");
    BLE_UART_OutString("Type HELP for commands\r\n");
    BLE_UART_OutString("========================================\r\n\r\n");

    BLE_UART_OutString("Servos initialized - ready for testing\r\n");
    BLE_UART_OutString("NOTE: Watch for buzzing and note safe angles!\r\n\r\n");

 
    while(1)
    {
        string_size = BLE_UART_InString(BLE_UART_Buffer, BLE_UART_BUFFER_SIZE);

        if (string_size > 0)
        {
            printf("BLE UART Data: ");
            for (i = 0; i < string_size; i++)
            {
                printf("%c", BLE_UART_Buffer[i]);
            }
            printf("\n");

            Process_BLE_UART_Data(BLE_UART_Buffer);
        }
    }
}






