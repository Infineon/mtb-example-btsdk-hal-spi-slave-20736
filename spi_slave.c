/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/******************************************************
 * This application demonstrates a second SPI interface
 * to communicate with the primary peer device as a SPI
 * slave.
 ******************************************************/

/** @file */
#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "spiffydriver.h"
#include "bleappconfig.h"
#include "devicelpm.h"
#include "hidddriversconfig.h"
#include "sparcommon.h"

/******************************************************
 *                      Constants
 ******************************************************/

// Comment out following definition to have Client run transmissions
// #define SPI_SLAVE_TRANSMIT_ON_TIMEOUT

// Use 1M speed
#define SPEED                                       1000000

// CS is active low
#define CS_ASSERT                                   0
#define CS_DEASSERT                                 1

// use GPIO P14 for output flow control
#define SPIFFY2_OUTPUT_FLOW_CTRL_PIN                14
#define SPIFFY2_OUTPUT_FLOW_CTRL_PORT               0

// use GPIO P2 for input flow control
#define SPIFFY2_INPUT_FLOW_CTRL_PIN                 2
#define SPIFFY2_INPUT_FLOW_CTRL_PORT                0

// Max transaction size
#define SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION 15
#define SPI_TRANSACTION_BUFFER_SIZE 16

#define DEBUG_PORT                                  1
#define DEBUG_PIN                                   12

#define SPI_COMM_CHECK_BYTE                         0xA0

#define SPI_TRANSFER_STATE_IDLE                     0
#define SPI_TRANSFER_STATE_MASTER                   1
#define SPI_TRANSFER_STATE_SLAVE                    2

#define SPI_TRANSFER_SUBSTATE_NONE                  0

#define SPI_TRANSFER_SUBSTATE_START                 1
#define SPI_TRANSFER_SUBSTATE_END                   2


/******************************************************
 *               Function Prototypes
 ******************************************************/

void        application_gpio_interrupt_handler(void* parameter, UINT8 u8);
int         application_spiffy2_send_bytes(void);
static UINT8 ringBufferAdd(UINT8* buffer, UINT8 length);
void        spi_comm_slave_create(void);
void        spi_comm_slave_timeout(UINT32 arg);
void        spi_comm_slave_fine_timeout(UINT32 arg);
static void application_spiffy2_init_in_slave_mode(void);

UINT8       spiProcessingCheck(void);
UINT32      device_lpm_queriable(LowPowerModePollType type, UINT32 context);

/******************************************************
 *               Variables Definitions
 ******************************************************/
UINT32  timer_count          = 0;
UINT8   count                = 0;
UINT8   spiTransferState     = SPI_TRANSFER_STATE_IDLE;
UINT8   spiTransferSubState  = SPI_TRANSFER_SUBSTATE_NONE;

UINT32  spiTxhwfifoHead      = 0;
UINT32  spiTxhwfifoTail      = 0;
UINT32  spiTxnumEmptyEntries = SPI_TRANSACTION_BUFFER_SIZE;

// The bytes we want to transmit as a slave.
UINT8 spi_slave_bytes_to_tx[SPI_TRANSACTION_BUFFER_SIZE][SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION];

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG spi_comm_slave_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG spi_comm_slave_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // GPIOs are not used
    },
    /*.gpio_flag =*/
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};

UINT16  spi_comm_slave_data_client_configuration    = 0;
UINT16  spi_comm_slave_control_client_configuration = 0;
UINT32  spi_comm_slave_timer_count                  = 0;
UINT32  spi_comm_slave_timer_timeout                = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization
APPLICATION_INIT()
{
    bleapp_set_cfg(NULL,
                   0,
                   NULL,
                   (void *)&spi_comm_slave_puart_cfg,
                   (void *)&spi_comm_slave_gpio_cfg,
                   spi_comm_slave_create);

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// Create spi comm slave
void spi_comm_slave_create(void)
{
    ble_trace0("spi_comm_slave_create()\n");
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    // Initialization for SPI2 interface
    application_spiffy2_init_in_slave_mode();

    bleprofile_regTimerCb(spi_comm_slave_fine_timeout, spi_comm_slave_timeout);
    bleprofile_StartTimer();

    devlpm_init();     // disable sleep
    devlpm_registerForLowPowerQueries(device_lpm_queriable, 0);

}


void spi_comm_slave_timeout(UINT32 arg)
{
    //ble_trace0("\rspi_comm_slave_timeout");
}

void spi_comm_slave_fine_timeout(UINT32 arg)
{
    UINT8 result;
    //ble_trace0("\rspi_comm_slave_fine_timeout");
#ifdef SPI_SLAVE_TRANSMIT_ON_TIMEOUT
    UINT8 i;
    UINT8 buffer[SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION - 1];


    for (i=0; i<SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION-1; i++)
    {
        buffer[i]=i;
    }
    timer_count++;

    if (timer_count == 10)
    {
        timer_count = 5;

        while (spiTxnumEmptyEntries > 0)
        {
            buffer[0] = count++;

            ringBufferAdd(buffer, SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION-1);
             ble_trace1("adding buffer, num left: %d\n", spiTxnumEmptyEntries);

        }

        result = application_spiffy2_send_bytes();
        if (result)
            ble_trace1("slave transmit: %d\n", result);
    }
#else
    ble_trace0("\rspi_comm_slave_fine_timeout");
    if (spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE)
    {
        result = application_spiffy2_send_bytes();
        if (result)
            ble_trace1("slave transmit: %d\n", result);
    }
#endif

    spi_comm_slave_timer_count++;

    spiProcessingCheck();
}

// receives data from the master
void masterToSlaveReceive(void)
{
    UINT8 length     = 0;
    UINT8 dump_count = 0;
    UINT8 junk;

    // try to receive 15 bytes at a time at most
    UINT8 buffer[SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION];

    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_LOW);

    memset(buffer, 0x00, sizeof(buffer));

    // CS is no longer asserted so some bytes will be available in the FIFO.
    // ble_trace1("Fifo Count at read: %02D\nBytes:\n", spiffyd_slaveGetRxFifoCount(SPIFFYD_2));

    // Get the first byte and check the length
    spiffyd_slaveRxData(SPIFFYD_2, 1, &length);

    if ((length & 0xF0) == SPI_COMM_CHECK_BYTE)
    {
        length = length & 0x0F;
        ble_trace1("Received Length: 0x%0x Bytes\n", length);
        if (length < SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION)
        {
            // M->S transaction falling edge transition.
            // First byte is the bytes in this transaction.
            if (length > 0)
            {
                spiffyd_slaveRxData(SPIFFYD_2, length, &buffer[1]);
                // Do something with buffer. Here we just trace out the bytes to debug uart.
                 ble_tracen((char *)&buffer[1], length);
            }

        }
    }


    // Clear out any extra data
    while (spiffyd_slaveGetRxFifoCount(SPIFFYD_2))
    {
        // Dump bytes
        spiffyd_slaveRxData(SPIFFYD_2, 1, &junk);
        dump_count++;
    }

    if (dump_count)
    {
        ble_trace1("Dumped %02D Bytes\n", dump_count);
    }

    spiTransferState = SPI_TRANSFER_STATE_IDLE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
}

// Initiate Slave to Master transaction
void slaveToMasterStart(void)
{
    spiTransferState = SPI_TRANSFER_STATE_SLAVE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_START;

    // Set FCO high to indicate that we want to transmit something to the master.
    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_HIGH);

    spi_comm_slave_timer_timeout = spi_comm_slave_timer_count + 2;

}

// Initiate Master to Slave transaction
void masterToSlaveStart(void)
{
    // Raise FCO indicating we are ready for transmit from master.
    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_HIGH);
    //ble_trace0("Raising FCO for M->S transaction\n");
    //ble_trace1("Fifo Count at raise FCO: 0x%02D\nBytes:\n", spiffyd_slaveGetRxFifoCount(SPIFFYD_2));

    spiTransferState = SPI_TRANSFER_STATE_MASTER;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_START;
    spi_comm_slave_timer_timeout = spi_comm_slave_timer_count + 2;
}

// End Master transaction
void masterToIdle(void)
{
    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_LOW);
    spiTransferState = SPI_TRANSFER_STATE_IDLE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
}

// End Slave transaction
void slaveToIdle(void)
{
    UINT8 junk;
    UINT8 result[1];
    UINT8 dump_count = 0;

    // Clear out data that got received when data was transmitted out
    while (spiffyd_slaveGetRxFifoCount(SPIFFYD_2))
    {
        // Dump bytes
        spiffyd_slaveRxData(SPIFFYD_2, 1, &junk);
        dump_count++;
    }

    if (dump_count)
          ble_trace1("Dumped %02D Bytes\n", dump_count);

    // S->M Transfer done, lower FCO
     if (dump_count)
         ble_trace0("Ending S->M transaction\n");

    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_LOW);
    spiTransferState = SPI_TRANSFER_STATE_IDLE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;

}

// Perform Slave to Master transaction
void slaveToMasterSend(void)
{
    UINT8 txlength     = 0;

    ble_trace1("Starting S->M transaction %d bytes\n", (int) spi_slave_bytes_to_tx[0]);

    txlength = spi_slave_bytes_to_tx[spiTxhwfifoTail][0]+1;

    spi_slave_bytes_to_tx[spiTxhwfifoTail][0] |= SPI_COMM_CHECK_BYTE;

    // FCO is high, so this is a S->M transaction. Put the bytes in the FIFO and lower FCO.
    spiffyd_slaveTxData(SPIFFYD_2, txlength, spi_slave_bytes_to_tx[spiTxhwfifoTail]);

    //ble_trace1("Fifo Count after write: %02D Bytes:\n", (int)spiffyd_slaveGetTxFifoCount(SPIFFYD_2));

    ble_tracen((char *)spi_slave_bytes_to_tx[spiTxhwfifoTail], txlength);

    // Lower FCO to indicate to master that data is ready to read.
    gpio_setPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN, GPIO_PIN_OUTPUT_LOW);

    // update ring buffer variables
    spiTxnumEmptyEntries++;
    spiTxhwfifoTail++;
    if (spiTxhwfifoTail == SPI_TRANSACTION_BUFFER_SIZE)
    {
        spiTxhwfifoTail = 0;
    }

    spiTransferSubState = SPI_TRANSFER_SUBSTATE_END;
    spi_comm_slave_timer_timeout = spi_comm_slave_timer_count + 2;

}

// Based on states and CS, perform next step in the transaction
UINT8 spiProcessingCheck(void)
{

    UINT8 retVal = 0;


    if (spiTransferState == SPI_TRANSFER_STATE_SLAVE)
    {
        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_START)
        {
            // Check CS is asserted indicating ack for S->M transaction
            if (gpio_getPinInput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_ASSERT)
            {
                slaveToMasterSend();
                //retVal = TRUE;
            }
            else if (spi_comm_slave_timer_count >= spi_comm_slave_timer_timeout)
            {
                masterToIdle();
            }
        }

        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_END)
        {
            // Check for end of S->M transaction.
            if (gpio_getPinInput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_DEASSERT)
            {
                slaveToIdle();
                retVal = TRUE;
            }
            else if (spi_comm_slave_timer_count >= spi_comm_slave_timer_timeout)
            {
                slaveToIdle();
            }
        }
    }

    if (spiTransferState == SPI_TRANSFER_STATE_MASTER)
    {
        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_START)
        {
            // Check for end of M->S transaction.
            if (gpio_getPinInput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_DEASSERT)
            {
                masterToSlaveReceive();
                //retVal = TRUE;
            }
            else if (spi_comm_slave_timer_count >= spi_comm_slave_timer_timeout)
            {
                masterToIdle();
            }
        }
    }

    if (spiTransferState == SPI_TRANSFER_STATE_IDLE)
    {
        if (gpio_getPinInput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_ASSERT)
        {
            masterToSlaveStart();
            retVal = TRUE;
        }
    }

    return(retVal);
}


// Thread context interrupt handler.
void application_gpio_interrupt_handler(void* parameter, UINT8 u8)
{
    UINT8 result = 1;

    // keep processisng to increase throughput
    while (result)
    {
        result = spiProcessingCheck();
    }

    // Initiate slave transmit of data if we have something to send and is idle
    while ((spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE) && (spiTransferState == SPI_TRANSFER_STATE_IDLE))
    {
        result = application_spiffy2_send_bytes();
        if (result)
        {
            ble_trace1("slave transmit: %d\n", result);
        }
    }

    // Pending interrupt on this GPIO will automatically be cleared by the driver.
}

// initialize spiffy2 as SPI slave
void application_spiffy2_init_in_slave_mode(void)
{
    // We will use an interrupt for the first rx byte and then start a state machine from then on.
    UINT16 interrupt_handler_mask[3] = {0, 0, 0};
    UINT16 cs_pin_config = 0 ;

    // Use SPIFFY2 interface as slave
    spi2PortConfig.masterOrSlave = SLAVE2_CONFIG;

    // pull for MISO for master, MOSI/CLOCK/CS if slave mode
    spi2PortConfig.pinPullConfig = INPUT_PIN_PULL_DOWN;

    // Use P3 for CLK, P0 for MOSI and P1 for MISO
    spi2PortConfig.spiGpioConfig = SLAVE2_P02_CS_P03_CLK_P00_MOSI_P01_MISO;

    // Initialize SPIFFY2 instance
    spiffyd_init(SPIFFYD_2);

    // Configure the SPIFFY2 HW block
    spiffyd_configure(SPIFFYD_2, 0, SPI_MSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_3);

    // enable SPIFFY2 slave receiver
    spiffyd_slaveEnableRx(SPIFFYD_2);

    // enable SPIFFY2 slave transmitter
    spiffyd_slaveEnableTx(SPIFFYD_2);

    interrupt_handler_mask[SPIFFY2_INPUT_FLOW_CTRL_PORT] |= (1 << SPIFFY2_INPUT_FLOW_CTRL_PIN);

    // Now register the interrupt handler.
    gpio_registerForInterrupt(interrupt_handler_mask, application_gpio_interrupt_handler, NULL);

    // Since we are using CS pin as flow control in and want to additionally enable interrupts, read and modify
    // the config because spiffyd_configure() would have already configured it for CS.
    cs_pin_config = gpio_getPinConfig(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN);

    // configure GPIO used for input flow control. Also enable interrupt on both edges.
    // Use the same pull as spi2PortConfig.pinPullConfig
    gpio_configurePin(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN,
        cs_pin_config | GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE, GPIO_PIN_OUTPUT_LOW);

    // Clear out any spurious interrupt status.
    gpio_clearPinInterruptStatus(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN);

    // configure GPIO used for output flow control (FCO)
    gpio_configurePin(SPIFFY2_OUTPUT_FLOW_CTRL_PORT,
                      SPIFFY2_OUTPUT_FLOW_CTRL_PIN,
                      GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
}

// Adds transmit data to the ring buffer
UINT8 ringBufferAdd(UINT8* buffer, UINT8 length)
{
    if (spiTxnumEmptyEntries == 0)
    {
        return(1);
    }

    // Copy over the data to a ring buffer so we can send it when the master is ready.
    spi_slave_bytes_to_tx[spiTxhwfifoHead][0] = length;
    memcpy(&spi_slave_bytes_to_tx[spiTxhwfifoHead][1], buffer, length);

    // update ring buffer variables
    spiTxnumEmptyEntries--;
    spiTxhwfifoHead++;
    if (spiTxhwfifoHead == SPI_TRANSACTION_BUFFER_SIZE)
    {
        spiTxhwfifoHead = 0;
    }

    return(0);
}

// Sends some bytes to the SPI master. Bytes will be sent asynchronously.
int application_spiffy2_send_bytes(void)
{
    // check to see if we are not already doing a SPI transfer
    if (spiTransferState != SPI_TRANSFER_STATE_IDLE)
    {
        spiProcessingCheck();
        ble_trace2("spiTransferState %d, %d\n", spiTransferState, spiTransferSubState);
        return 1;
    }

    // If FCO is already high, either we are either receiving or in the start of a transmit already
    if (gpio_getPinOutput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
    {
        spiProcessingCheck();
        return 2;
    }

    // read the input of CS to make sure it is not asserted so we are not already in another transaction
    if (gpio_getPinInput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_ASSERT)
    {
        spiProcessingCheck();
        return 3;
    }

    slaveToMasterStart();

    return 0;
}

// Callback called by the FW when ready to sleep/deep-sleep. Disable both by returning 0.
UINT32 device_lpm_queriable(LowPowerModePollType type, UINT32 context)
{
    // Disable sleep.
    return 0;
}
