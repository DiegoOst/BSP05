/**
  ******************************************************************************
  * @file    x_nucleo_nfc04a1.c
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file provides nfc05a1 specific functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty  
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */

#include "utils.h"
#include "rfal_rf.h"
#include "rfal_nfca.h"
#include "rfal_nfcb.h"
#include "rfal_nfcf.h"
#include "rfal_nfcv.h"
#include "rfal_st25tb.h"
#include "rfal_nfcDep.h"
#include "rfal_isoDep.h"
#include "DigitalOut.h"
#include "st25r3911_interrupt.h"


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/

/* Definition of possible states the demo state machine could have */
#define DEMO_ST_FIELD_OFF			        0
#define DEMO_ST_POLL_ACTIVE_TECH      1
#define DEMO_ST_POLL_PASSIV_TECH      2
#define DEMO_ST_WAIT_WAKEUP	          3

#define DEMO_BUF_LEN                  255

/* macro to cycle through states */
#define	NEXT_STATE()		             {state++; state %= sizeof(stateArray);}




/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/* State array of all possible states to be executed one after each other */
static uint8_t stateArray[] = { DEMO_ST_FIELD_OFF,
                                DEMO_ST_POLL_ACTIVE_TECH,
                                DEMO_ST_POLL_PASSIV_TECH,
                                DEMO_ST_WAIT_WAKEUP
                              };

/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

/* APDUs communication data */
static uint8_t ndefSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };
static uint8_t ccSelectFile[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static uint8_t readBynary[] = { 0x00, 0xB0, 0x00, 0x00, 0x0F };
/*static uint8_t ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };*/

/* P2P communication data */
static uint8_t ndefPing[] = {0x00, 0x00};
static uint8_t ndefInit[] = {0x05, 0x20, 0x06, 0x0F, 0x75, 0x72, 0x6E, 0x3A, 0x6E, 0x66, 0x63, 0x3A, 0x73, 0x6E, 0x3A, 0x73, 0x6E, 0x65, 0x70, 0x02, 0x02, 0x07, 0x80, 0x05, 0x01, 0x02};
static uint8_t ndefUriSTcom[] = {0x13, 0x20, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x19, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x12, 0x55, 0x00, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d};


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

/*! Transmit buffers union, only one interface is used at a time                                                            */
static union{
    rfalIsoDepApduBufFormat  isoDepTxBuf;                                /* ISO-DEP Tx buffer format (with header/prologue) */
    rfalNfcDepBufFormat      nfcDepTxBuf;                                /* NFC-DEP Rx buffer format (with header/prologue) */
    uint8_t                  txBuf[DEMO_BUF_LEN];                        /* Generic buffer abstraction                      */
}gTxBuf;


/*! Receive buffers union, only one interface is used at a time                                                             */
static union {
    rfalIsoDepApduBufFormat  isoDepRxBuf;                                /* ISO-DEP Rx buffer format (with header/prologue) */
    rfalNfcDepBufFormat      nfcDepRxBuf;                                /* NFC-DEP Rx buffer format (with header/prologue) */
    uint8_t                  rxBuf[DEMO_BUF_LEN];                        /* Generic buffer abstraction                      */
}gRxBuf;

static rfalIsoDepBufFormat   tmpBuf;          /* tmp buffer required for ISO-DEP APDU interface, I-Block interface does not */

/*! Receive buffers union, only one interface is used at a time                                                             */
static union {
    rfalIsoDepDevice  isoDepDev;                                         /* ISO-DEP Device details                          */
    rfalNfcDepDevice  nfcDepDev;                                         /* NFC-DEP Device details                          */
}gDevProto;

static bool doWakeUp = false;                /*!< by default do not perform Wake-Up               */
static uint8_t state = DEMO_ST_FIELD_OFF;    /*!< Actual state, starting with RF field turned off */



extern volatile bool wakeupFlag;




/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

static bool demoPollAP2P( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static bool demoPollNFCA( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static bool demoPollNFCB( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static bool demoPollST25TB( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static bool demoPollNFCF( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static bool demoPollNFCV( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static ReturnCode demoActivateP2P( uint8_t* nfcid, uint8_t nfidLen, bool isActive, rfalNfcDepDevice *nfcDepDev, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static ReturnCode demoNfcDepBlockingTxRx( rfalNfcDepDevice *nfcDepDev, const uint8_t *txBuf, uint16_t txBufSize, uint8_t *rxBuf, uint16_t rxBufSize, uint16_t *rxActLen, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static ReturnCode demoIsoDepBlockingTxRx( rfalIsoDepDevice *isoDepDev, const uint8_t *txBuf, uint16_t txBufSize, uint8_t *rxBuf, uint16_t rxBufSize, uint16_t *rxActLen, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static void demoSendNdefUri( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );
static void demoSendAPDUs( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 );


/*!
 *****************************************************************************
 * \brief Demo Cycle
 *
 *  This function executes the actual state of the demo state machine.
 *  Must be called cyclically
 *****************************************************************************
 */



void demoCycle(SPI* mspiChannel, ST25R3911* mST25, DigitalIn * uButton, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  bool found = false;

  /* Code below is commented since not implemented. It may be of interest to uncommented if going through the wake up mode */

  /* Check if USER button is pressed */
//  if( uButton -> read() == 1 )
  //{

			//doWakeUp = 1;
						//!doWakeUp;          /* enable/disable wakeup */
	//		state = DEMO_ST_FIELD_OFF;        /* restart loop          */

			/* Debounce button */
		//	while( uButton -> read() == 0 );
//  }

  switch( stateArray[state] )
  {
  	  //
  	  case DEMO_ST_FIELD_OFF:

  		  fieldLED_01 -> write(0);
  		  fieldLED_02 -> write(0);
  		  fieldLED_03 -> write(0);
  		  fieldLED_04 -> write(0);
  		  fieldLED_05 -> write(0);
  		  fieldLED_06 -> write(0);


      rfalFieldOff( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      rfalWakeUpModeStop( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      platformDelay(300);

      /* If WakeUp is to be executed, enable Wake-Up mode */
      if( doWakeUp )
      {
        //"Going to Wakeup mode.\r\n");

        rfalWakeUpModeStart( NULL, mST25, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        state = DEMO_ST_WAIT_WAKEUP;
        break;
      }
    
      NEXT_STATE();
      break;

      //
    case DEMO_ST_POLL_ACTIVE_TECH:
      demoPollAP2P( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      platformDelay(40);
      NEXT_STATE();
      break;

      //
    case DEMO_ST_POLL_PASSIV_TECH:
      found |= demoPollNFCA( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      found |= demoPollNFCB( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      found |= demoPollST25TB( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      found |= demoPollNFCF( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      found |= demoPollNFCV( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    
      platformDelay(300);
      state = DEMO_ST_FIELD_OFF;
      break;

      //
    case DEMO_ST_WAIT_WAKEUP:




		 st25r3911Isr(mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;


		 if( st25r3911ChipHasWoke() )
		 {
			 rfalSetWumState();
		 }


      /* Check if Wake-Up Mode has been awaked */
      if( rfalWakeUpModeHasWoke(mST25, mspiChannel, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 )  )
      {
        /* If awake, go directly to Poll */
        rfalWakeUpModeStop( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        state = DEMO_ST_POLL_ACTIVE_TECH;
      }
      break;

    default:
      break;
  }

}


/*!
 *****************************************************************************
 * \brief Poll NFC-AP2P
 *
 * Configures the RFAL to AP2P communication and polls for a nearby
 * device. If a device is found turns On a LED and logs its UID.
 * If the Device supports NFC-DEP protocol (P2P) it will activate
 * the device and try to send an URI record.
 *
 * This methid first tries to establish communication at 424kb/s and if
 * failed, tries also at 106kb/s
 *
 *
 *  \return true    : AP2P device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollAP2P( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode       err;
  bool             try106 = false;


  while (!try106)
  {
    /*******************************************************************************/
    /* NFC_ACTIVE_POLL_MODE                                                        */
    /*******************************************************************************/
    /* Initialize RFAL as AP2P Initiator NFC BR 424 */
    err = rfalSetMode(RFAL_MODE_POLL_ACTIVE_P2P, ((try106) ? RFAL_BR_106 : RFAL_BR_424), ((try106) ? RFAL_BR_106 : RFAL_BR_424), mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    rfalSetErrorHandling(RFAL_ERRORHANDLING_NFC);
    rfalSetFDTListen(RFAL_FDT_LISTEN_AP2P_POLLER);
    rfalSetFDTPoll(RFAL_TIMING_NONE);

    rfalSetGT( RFAL_GT_AP2P_ADJUSTED );
    err = rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;


    err = demoActivateP2P( NFCID3, RFAL_NFCDEP_NFCID3_LEN, true, &gDevProto.nfcDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    if (err == ERR_NONE)
    {
      /****************************************************************************/
      /* Active P2P device activated                                              */
      /* NFCID / UID is contained in : nfcDepDev.activation.Target.ATR_RES.NFCID3 */
    	// //("NFC Active P2P device found. NFCID3: %s\r\n", hex2Str(gDevProto.nfcDepDev.activation.Target.ATR_RES.NFCID3, RFAL_NFCDEP_NFCID3_LEN));
      fieldLED_05 -> write(1);
      /* Send an URI record */
      demoSendNdefUri( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
      return true;
    }

    /* AP2P at 424kb/s didn't found any device, try at 106kb/s */
    try106 = true;
    rfalFieldOff( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  }

  return false;
}

/*!
 *****************************************************************************
 * \brief Poll NFC-A
 *
 * Configures the RFAL to NFC-A (ISO14443A) communication and polls for a nearby
 * NFC-A device.
 * If a device is found turns On a LED and logs its UID.
 *
 * Additionally, if the Device supports NFC-DEP protocol (P2P) it will activate
 * the device and try to send an URI record.
 * If the device supports ISO-DEP protocol (ISO144443-4) it will
 * activate the device and try exchange some APDUs with PICC.
 *
 *
 *  \return true    : NFC-A device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollNFCA( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode        err;
  bool              found = false;
  uint8_t           devIt = 0;
  rfalNfcaSensRes   sensRes;

  rfalNfcaPollerInitialize( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;   /* Initialize for NFC-A */
  rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;      /* Turns the Field On if not already and start GT timer */

  err = rfalNfcaPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensRes, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  if(err == ERR_NONE)
  {
    rfalNfcaListenDevice nfcaDevList[1];
    uint8_t                   devCnt;

    err = rfalNfcaPollerFullCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, 1, nfcaDevList, &devCnt, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if ( (err == ERR_NONE) && (devCnt > 0) )
    {
      found = true;
      devIt = 0;

      fieldLED_03 -> write(1);

      /* Check if it is Topaz aka T1T */
      if( nfcaDevList[devIt].type == RFAL_NFCA_T1T )
      {
        /********************************************/
        /* NFC-A T1T card found                     */
        /* NFCID/UID is contained in: t1tRidRes.uid */
    	  //   //("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2Str(nfcaDevList[devIt].ridRes.uid, RFAL_T1T_UID_LEN));
      }
      else
      {
        /*********************************************/
        /* NFC-A device found                        */
        /* NFCID/UID is contained in: nfcaDev.nfcId1 */
    	  // //("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str(nfcaDevList[0].nfcId1, nfcaDevList[0].nfcId1Len));
      }


      /* Check if device supports P2P/NFC-DEP */
      if( (nfcaDevList[devIt].type == RFAL_NFCA_NFCDEP) || (nfcaDevList[devIt].type == RFAL_NFCA_T4T_NFCDEP))
      {
        /* Continue with P2P Activation .... */

        err = demoActivateP2P( NFCID3, RFAL_NFCDEP_NFCID3_LEN, false, &gDevProto.nfcDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        if (err == ERR_NONE)
        {
          /*********************************************/
          /* Passive P2P device activated              */
        	// //("NFCA Passive P2P device found. NFCID: %s\r\n", hex2Str(gDevProto.nfcDepDev.activation.Target.ATR_RES.NFCID3, RFAL_NFCDEP_NFCID3_LEN));

          /* Send an URI record */
          demoSendNdefUri( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        }
      }
      /* Check if device supports ISO14443-4/ISO-DEP */
      else if (nfcaDevList[devIt].type == RFAL_NFCA_T4T)
      {
        /* Activate the ISO14443-4 / ISO-DEP layer */

        rfalIsoDepInitialize();
        err = rfalIsoDepPollAHandleActivation((rfalIsoDepFSxI)RFAL_ISODEP_FSDI_DEFAULT, RFAL_ISODEP_NO_DID, RFAL_BR_424, &gDevProto.isoDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        if( err == ERR_NONE )
        {
        	//  //("ISO14443-4/ISO-DEP layer activated. \r\n");

          /* Exchange APDUs */
          demoSendAPDUs( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        }
      }
    }
  }
  return found;
}

/*!
 *****************************************************************************
 * \brief Poll NFC-B
 *
 * Configures the RFAL to NFC-B (ISO14443B) communication and polls for a nearby
 * NFC-B device.
 * If a device is found turns On a LED and logs its UID.
 * Additionally, if the Device supports ISO-DEP protocol (ISO144443-4) it will
 * activate the device and try exchange some APDUs with PICC
 *
 *  \return true    : NFC-B device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollNFCB( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode            err;
  rfalNfcbListenDevice  nfcbDev;
  bool                  found = false;
  uint8_t               devCnt = 0;
  
  /*******************************************************************************/
  /* ISO14443B/NFC_B_PASSIVE_POLL_MODE                                           */
  /*******************************************************************************/

  rfalNfcbPollerInitialize( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;   /* Initialize for NFC-B */
  rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;      /* Turns the Field On if not already and start GT timer */


  err = rfalNfcbPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, 1, &nfcbDev, &devCnt, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  if( (err == ERR_NONE) && (devCnt > 0) )
  {
    /**********************************************/
    /* NFC-B card found                           */
    /* NFCID/UID is contained in: sensbRes.nfcid0 */
    found = true;

    fieldLED_02 -> write(1);
    // //("ISO14443B/NFC-B card found. UID: %s\r\n", hex2Str(nfcbDev.sensbRes.nfcid0, RFAL_NFCB_NFCID0_LEN));

  }

  /* Check if device supports ISO14443-4/ISO-DEP */
  if( nfcbDev.sensbRes.protInfo.FsciProType & RFAL_NFCB_SENSB_RES_PROTO_ISO_MASK )
  {

    /* Activate the ISO14443-4 / ISO-DEP layer */
    rfalIsoDepInitialize();
    err = rfalIsoDepPollBHandleActivation((rfalIsoDepFSxI)RFAL_ISODEP_FSDI_DEFAULT, RFAL_ISODEP_NO_DID, RFAL_BR_424, RFAL_ISODEP_ATTRIB_REQ_PARAM1_DEFAULT, &nfcbDev, NULL, 0, &gDevProto.isoDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if( err == ERR_NONE )
    {
      //("ISO14443-4/ISO-DEP layer activated. \r\n");

      /* Exchange APDUs */
      demoSendAPDUs( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    }
  }
  return found;
}

/*!
 *****************************************************************************
 * \brief Poll ST25TB
 *
 * Configures the RFAL and polls for a nearby ST25TB device.
 * If a device is found turns On a LED and logs its UID.
 *
 *  \return true    : ST25TB device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollST25TB( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode              err;
  bool                    found = false;
  uint8_t                 devCnt = 0;
  rfalSt25tbListenDevice  st25tbDev;

  /*******************************************************************************/
  /* ST25TB_PASSIVE_POLL_MODE                                                    */
  /*******************************************************************************/

  rfalSt25tbPollerInitialize( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

  err = rfalSt25tbPollerCheckPresence(NULL, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  if( err == ERR_NONE )
  {
    err = rfalSt25tbPollerCollisionResolution(1, &st25tbDev, &devCnt, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if ((err == ERR_NONE) && (devCnt > 0))
    {
      /******************************************************/
      /* ST25TB card found                                  */
      /* NFCID/UID is contained in: st25tbDev.UID           */
      found = true;
      fieldLED_02 -> write(1);
      ////  //("ST25TB card found. UID: %s\r\n", hex2Str(st25tbDev.UID, RFAL_ST25TB_UID_LEN));

    }
  }
  return found;
}


/*!
 *****************************************************************************
 * \brief Poll NFC-F
 *
 * Configures the RFAL to NFC-F (FeliCa) communication and polls for a nearby
 * NFC-F device.
 * If a device is found turns On a LED and logs its UID.
 * Additionally, if the Device supports NFC-DEP protocol (P2P) it will
 * activate the device and try to send an URI record
 *
 *  \return true    : NFC-F device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollNFCF( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode            err;
  rfalNfcfListenDevice  nfcfDev;
  uint8_t               devCnt = 0;
  bool                  found = false;

  /*******************************************************************************/
  /* Felica/NFC_F_PASSIVE_POLL_MODE                                              */
  /*******************************************************************************/

  rfalNfcfPollerInitialize( RFAL_BR_212, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ; /* Initialize for NFC-F */
  rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;                 /* Turns the Field On if not already and start GT timer */

  err = rfalNfcfPollerCheckPresence( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  if( err == ERR_NONE )
  {
    err = rfalNfcfPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, 1, &nfcfDev, &devCnt, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    if( (err == ERR_NONE) && (devCnt > 0) )
    {
      /******************************************************/
      /* NFC-F card found                                   */
      /* NFCID/UID is contained in: nfcfDev.sensfRes.NFCID2 */
      found = true;
      fieldLED_01 -> write(1);
      //("Felica/NFC-F card found. UID: %s\r\n", hex2Str(nfcfDev.sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN));



      /* Check if device supports P2P/NFC-DEP */
      if( rfalNfcfIsNfcDepSupported( &nfcfDev ) )
      {
        /* Continue with P2P (NFC-DEP) activation */
        err = demoActivateP2P( nfcfDev.sensfRes.NFCID2, RFAL_NFCDEP_NFCID3_LEN, false, &gDevProto.nfcDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        if (err == ERR_NONE)
        {
          /*********************************************/
          /* Passive P2P device activated              */
          //("NFCF Passive P2P device found. NFCID: %s\r\n", hex2Str(gDevProto.nfcDepDev.activation.Target.ATR_RES.NFCID3, RFAL_NFCDEP_NFCID3_LEN));

          /* Send an URI record */
          demoSendNdefUri( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
        }
      }
    }
  }
  return found;
}


/*!
 *****************************************************************************
 * \brief Poll NFC-V
 *
 * Configures the RFAL to NFC-V (ISO15693) communication, polls for a nearby
 * NFC-V device. If a device is found turns On a LED and logs its UID
 *
 *
 *  \return true    : NFC-V device found
 *  \return false   : No device found
 *
 *****************************************************************************
 */
bool demoPollNFCV( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode            err;
  rfalNfcvListenDevice  nfcvDev;
  bool                  found = false;
  uint8_t               devCnt = 0;

  /*******************************************************************************/
  /* ISO15693/NFC_V_PASSIVE_POLL_MODE                                            */
  /*******************************************************************************/

  rfalNfcvPollerInitialize( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;           /* Initialize for NFC-F */
  rfalFieldOnAndStartGT( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;              /* Turns the Field On if not already and start GT timer */

  err = rfalNfcvPollerCollisionResolution(1, &nfcvDev, &devCnt, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  if( (err == ERR_NONE) && (devCnt > 0) )
  {
    /******************************************************/
    /* NFC-V card found                                   */
    /* NFCID/UID is contained in: invRes.UID */
    REVERSE_BYTES(nfcvDev.InvRes.UID, RFAL_NFCV_UID_LEN);

    found = true;
    fieldLED_04 -> write(1);
    //("ISO15693/NFC-V card found. UID: %s\r\n", hex2Str(nfcvDev.InvRes.UID, RFAL_NFCV_UID_LEN));

  }

  return found;
}

/*!
 *****************************************************************************
 * \brief Activate P2P
 *
 * Configures NFC-DEP layer and executes the NFC-DEP/P2P activation (ATR_REQ
 * and PSL_REQ if applicable)
 *
 * \param[in] nfcid      : nfcid to be used
 * \param[in] nfcidLen   : length of nfcid
 * \param[in] isActive   : Active or Passive communication
 * \param[out] nfcDepDev : If activation successful, device's Info
 *
 *  \return ERR_PARAM    : Invalid parameters
 *  \return ERR_TIMEOUT  : Timeout error
 *  \return ERR_FRAMING  : Framing error detected
 *  \return ERR_PROTO    : Protocol error detected
 *  \return ERR_NONE     : No error, activation successful
 *
 *****************************************************************************
 */
ReturnCode demoActivateP2P( uint8_t* nfcid, uint8_t nfidLen, bool isActive, rfalNfcDepDevice *nfcDepDev, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  rfalNfcDepAtrParam nfcDepParams;

  nfcDepParams.nfcid     = nfcid;
  nfcDepParams.nfcidLen  = nfidLen;
  nfcDepParams.BS        = RFAL_NFCDEP_Bx_NO_HIGH_BR;
  nfcDepParams.BR        = RFAL_NFCDEP_Bx_NO_HIGH_BR;
  nfcDepParams.LR        = RFAL_NFCDEP_LR_254;
  nfcDepParams.DID       = RFAL_NFCDEP_DID_NO;
  nfcDepParams.NAD       = RFAL_NFCDEP_NAD_NO;
  nfcDepParams.GBLen     = sizeof(GB);
  nfcDepParams.GB        = GB;
  nfcDepParams.commMode  = ((isActive) ? RFAL_NFCDEP_COMM_ACTIVE : RFAL_NFCDEP_COMM_PASSIVE);
  nfcDepParams.operParam = (RFAL_NFCDEP_OPER_FULL_MI_EN | RFAL_NFCDEP_OPER_EMPTY_DEP_DIS | RFAL_NFCDEP_OPER_ATN_EN | RFAL_NFCDEP_OPER_RTOX_REQ_EN);

  /* Initialize NFC-DEP protocol layer */
  rfalNfcDepInitialize();

  /* Handle NFC-DEP Activation (ATR_REQ and PSL_REQ if applicable) */
  return rfalNfcDepInitiatorHandleActivation( &nfcDepParams, RFAL_BR_424, nfcDepDev, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
}


/*!
 *****************************************************************************
 * \brief Send URI
 *
 * Sends a NDEF URI record 'http://www.ST.com' via NFC-DEP (P2P) protocol.
 *
 * This method sends a set of static predefined frames which tries to establish
 * a LLCP connection, followed by the NDEF record, and then keeps sending
 * LLCP SYMM packets to maintain the connection.
 *
 *
 *  \return true    : NDEF URI was sent
 *  \return false   : Exchange failed
 *
 *****************************************************************************
 */
void demoSendNdefUri( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  uint16_t   actLen = 0;
  ReturnCode err = ERR_NONE;

  //(" Initalize device .. ");
  if(ERR_NONE != demoNfcDepBlockingTxRx( &gDevProto.nfcDepDev, ndefInit, sizeof(ndefInit), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &actLen , mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) )
  {
    //("failed.");
    return;
  }
  //("succeeded.\r\n");

  actLen = 0;
  //(" Push NDEF Uri: www.ST.com .. ");
  if(ERR_NONE != demoNfcDepBlockingTxRx( &gDevProto.nfcDepDev, ndefUriSTcom, sizeof(ndefUriSTcom), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &actLen, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) )
  {
    //("failed.");
    return;
  }
  //("succeeded.\r\n");


  //(" Device present, maintaining connection ");
  while(err == ERR_NONE)
  {
    err = demoNfcDepBlockingTxRx( &gDevProto.nfcDepDev, ndefPing, sizeof(ndefPing), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &actLen, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    //(".");
    platformDelay(50);
  }
  //("\r\n Device removed.\r\n");

}


/*!
 *****************************************************************************
 * \brief Exchange APDUs
 *
 * Example how to exchange a set of predefined APDUs with PICC. The NDEF
 * application will be selected and then CC will be selected and read.
 *
 *****************************************************************************
 */
void demoSendAPDUs( SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  uint16_t   rxLen;
  ReturnCode err;

  /* Exchange APDU: NDEF Tag Application Select command */
  err = demoIsoDepBlockingTxRx(&gDevProto.isoDepDev, ndefSelectApp, sizeof(ndefSelectApp), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &rxLen, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

  if( (err == ERR_NONE) && gRxBuf.rxBuf[0] == 0x90 && gRxBuf.rxBuf[1] == 0x00)
  {
    //(" Select NDEF App successfully \r\n");

    /* Exchange APDU: Select Capability Container File */
    err = demoIsoDepBlockingTxRx(&gDevProto.isoDepDev, ccSelectFile, sizeof(ccSelectFile), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &rxLen, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;

    /* Exchange APDU: Read Capability Container File  */
    err = demoIsoDepBlockingTxRx(&gDevProto.isoDepDev, readBynary, sizeof(readBynary), gRxBuf.rxBuf, sizeof(gRxBuf.rxBuf), &rxLen, mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  }
}

/*!
 *****************************************************************************
 * \brief ISO-DEP Blocking Transceive
 *
 * Helper function to send data in a blocking manner via the rfalIsoDep module
 *
 * \warning A protocol transceive handles long timeouts (several seconds),
 * transmission errors and retransmissions which may lead to a long period of
 * time where the MCU/CPU is blocked in this method.
 * This is a demo implementation, for a non-blocking usage example please
 * refer to the Examples available with RFAL
 *
 *
 * \param[in]  isoDepDev  : device details retrived during activation
 * \param[in]  txBuf      : data to be transmitted
 * \param[in]  txBufSize  : size of the data to be transmited
 * \param[out] rxBuf      : buffer to place receive data
 * \param[in]  rxBufSize  : size of the reception buffer
 * \param[out] rxActLen   : number of data bytes received

 *
 *  \return ERR_PARAM     : Invalid parameters
 *  \return ERR_TIMEOUT   : Timeout error
 *  \return ERR_FRAMING   : Framing error detected
 *  \return ERR_PROTO     : Protocol error detected
 *  \return ERR_NONE      : No error, activation successful
 *
 *****************************************************************************
 */
ReturnCode demoIsoDepBlockingTxRx( rfalIsoDepDevice *isoDepDev, const uint8_t *txBuf, uint16_t txBufSize, uint8_t *rxBuf, uint16_t rxBufSize, uint16_t *rxActLen, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode               err;
  rfalIsoDepApduTxRxParam  isoDepTxRx;

  /* Initialize the ISO-DEP protocol transceive context */
  isoDepTxRx.txBuf        = &gTxBuf.isoDepTxBuf;
  isoDepTxRx.txBufLen     = txBufSize;
  isoDepTxRx.DID          = isoDepDev->info.DID;
  isoDepTxRx.FWT          = isoDepDev->info.FWT;
  isoDepTxRx.dFWT         = isoDepDev->info.dFWT;
  isoDepTxRx.FSx          = isoDepDev->info.FSx;
  isoDepTxRx.ourFSx       = RFAL_ISODEP_FSX_KEEP;
  isoDepTxRx.rxBuf        = &gRxBuf.isoDepRxBuf;
  isoDepTxRx.rxLen        = rxActLen;
  isoDepTxRx.tmpBuf       = &tmpBuf;


  /* Copy data to send */
  ST_MEMMOVE( gTxBuf.isoDepTxBuf.apdu, txBuf, MIN( txBufSize, RFAL_ISODEP_DEFAULT_FSC ) );

  /* Perform the ISO-DEP Transceive in a blocking way */
  rfalIsoDepStartApduTransceive( isoDepTxRx );
  do {
    rfalWorker( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    err = rfalIsoDepGetApduTransceiveStatus( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  } while(err == ERR_BUSY);

  //(" ISO-DEP TxRx %s: - Tx: %s Rx: %s \r\n", (err != ERR_NONE) ? "FAIL": "OK", hex2Str((uint8_t*)txBuf, txBufSize), (err != ERR_NONE) ? "": hex2Str( isoDepTxRx.rxBuf->apdu, *rxActLen));

  if( err != ERR_NONE )
  {
    return err;
  }

  /* Copy received data */
  ST_MEMMOVE( rxBuf, isoDepTxRx.rxBuf->apdu, MIN(*rxActLen, rxBufSize) );
  return ERR_NONE;
}


/*!
 *****************************************************************************
 * \brief NFC-DEP Blocking Transceive
 *
 * Helper function to send data in a blocking manner via the rfalNfcDep module
 *
 * \warning A protocol transceive handles long timeouts (several seconds),
 * transmission errors and retransmissions which may lead to a long period of
 * time where the MCU/CPU is blocked in this method.
 * This is a demo implementation, for a non-blocking usage example please
 * refer to the Examples available with RFAL
 *
 * \param[in]  nfcDepDev  : device details retrived during activation
 * \param[in]  txBuf      : data to be transmitted
 * \param[in]  txBufSize  : size of the data to be transmited
 * \param[out] rxBuf      : buffer to place receive data
 * \param[in]  rxBufSize  : size of the reception buffer
 * \param[out] rxActLen   : number of data bytes received

 *
 *  \return ERR_PARAM     : Invalid parameters
 *  \return ERR_TIMEOUT   : Timeout error
 *  \return ERR_FRAMING   : Framing error detected
 *  \return ERR_PROTO     : Protocol error detected
 *  \return ERR_NONE      : No error, activation successful
 *
 *****************************************************************************
 */
ReturnCode demoNfcDepBlockingTxRx( rfalNfcDepDevice *nfcDepDev, const uint8_t *txBuf, uint16_t txBufSize, uint8_t *rxBuf, uint16_t rxBufSize, uint16_t *rxActLen, SPI* mspiChannel, ST25R3911* mST25, DigitalOut* gpio_cs, InterruptIn* IRQ, DigitalOut* fieldLED_01, DigitalOut* fieldLED_02, DigitalOut* fieldLED_03, DigitalOut* fieldLED_04, DigitalOut* fieldLED_05, DigitalOut* fieldLED_06 )
{
  ReturnCode             err;
  bool                   isChaining;
  rfalNfcDepTxRxParam    rfalNfcDepTxRx;


  /* Initialize the NFC-DEP protocol transceive context */
  rfalNfcDepTxRx.txBuf        = &gTxBuf.nfcDepTxBuf;
  rfalNfcDepTxRx.txBufLen     = txBufSize;
  rfalNfcDepTxRx.rxBuf        = &gRxBuf.nfcDepRxBuf;
  rfalNfcDepTxRx.rxLen        = rxActLen;
  rfalNfcDepTxRx.DID          = RFAL_NFCDEP_DID_NO;
  rfalNfcDepTxRx.FSx          = rfalNfcDepLR2FS( rfalNfcDepPP2LR( nfcDepDev->activation.Target.ATR_RES.PPt ) );
  rfalNfcDepTxRx.FWT          = nfcDepDev->info.FWT;
  rfalNfcDepTxRx.dFWT         = nfcDepDev->info.dFWT;
  rfalNfcDepTxRx.isRxChaining = &isChaining;
  rfalNfcDepTxRx.isTxChaining = false;

  /* Copy data to send */
  ST_MEMMOVE( gTxBuf.nfcDepTxBuf.inf, txBuf, MIN( txBufSize, RFAL_NFCDEP_FRAME_SIZE_MAX_LEN ) );

  /* Perform the NFC-DEP Transceive in a blocking way */
  rfalNfcDepStartTransceive( &rfalNfcDepTxRx );
  do {
    rfalWorker( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
    err = rfalNfcDepGetTransceiveStatus( mspiChannel, mST25, gpio_cs, IRQ, fieldLED_01, fieldLED_02, fieldLED_03, fieldLED_04, fieldLED_05, fieldLED_06 ) ;
  } while(err == ERR_BUSY);

  /* //(" NFC-DEP TxRx %s: - Tx: %s Rx: %s \r\n", (err != ERR_NONE) ? "FAIL": "OK", hex2Str( (uint8_t*)rfalNfcDepTxRx.txBuf, txBufSize), (err != ERR_NONE) ? "": hex2Str( rfalNfcDepTxRx.rxBuf->inf, *rxActLen)); */

  if( err != ERR_NONE )
  {
    return err;
  }

  /* Copy received data */
  ST_MEMMOVE( rxBuf, gRxBuf.nfcDepRxBuf.inf, MIN(*rxActLen, rxBufSize) );
  return ERR_NONE;
}




/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
