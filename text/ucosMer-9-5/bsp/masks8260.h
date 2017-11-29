
/*------------------------------------------------------------------------*
*  FILE:  MASK8260.H                                                     
*                                                                        
*  DESCRIPTION:  Bit masks for MPC8260 registers. Sequence of registers  
*                follows device memory map exactly.                      
*                                                                         
*  HISTORY:  2 FEB 99 JAY                                         
*------------------------------------------------------------------------*/

/*------------------------------------------*
* CPM Command Register (CPCR)               *
*-------------------------------------------*
* NOTE: This register is cleared by reset.  *
*       See MPC8260 User's Manual.          *
*-------------------------------------------*/

#define CPCR_RST                 0x80000000  /* Software Reset Command */
#define CPCR_FLG                 0x00010000  /* Command Semaphore Flag */

/*-----------------------------------------------*/
/* Definitions for FCC CPCR Subblock/Page codes. */
/*-----------------------------------------------*/

#define  FCC1_PAGE_SUBBLOCK    0x12000000    /* page 4, code = 16 */
#define  FCC2_PAGE_SUBBLOCK    0x16200000    /* page 5, code = 17 */
#define  FCC3_PAGE_SUBBLOCK    0x1A400000    /* page 6, code = 18 */

/*-----------------------------*/
/* Opcode definitions for FCCs */
/*-----------------------------*/

#define CPCR_INIT_TX_RX_PARAMS   0x00000000   /* Opcode 0 */
#define CPCR_INIT_RX_PARAMS      0x00000001   /* Opcode 1 */
#define CPCR_INIT_TX_PARAMS      0x00000002   /* Opcode 2 */
#define CPCR_ENTER_HUNT_MODE     0x00000003   /* Opcode 3 */
#define CPCR_STOP_TX             0x00000004   /* Opcode 4 */
#define CPCR_GRACEFUL_STOP_TX    0x00000005   /* Opcode 5 */
#define CPCR_RESTART_TX          0x00000006   /* Opcode 6 */
#define CPCR_CLOSE_RX_BD         0x00000007   /* Opcode 7 */
#define CPCR_SET_GRP_ADDR        0x00000008   /* Opcode 8 */
#define CPCR_ATM_XMIT_CMD        0x0000000A   /* Opcode 10 */

/*-----------------------------------------------------*/
/* General Definitions for FCC CPCR Command Operations */
/*-----------------------------------------------------*/

#define READY_TO_RX_CMD          0x00000000

/*-------------------------*/
/* General FCC Definitions */
/*-------------------------*/

#define  DISABLE_TX_RX   0xFFFFFFCF  /* Clear the ENT/ENR bits in the GFMR
                                        Disables the transmit & Receive
                                        port                              */

#define  GFMR_ENT    0x00000010  /* ENT bit for the GFMR register */
#define  GFMR_ENR    0x00000020  /* ENR bit for the GFMR register */
#define  GFMR_INT_LB 0x40000000  /* Internal Loopback mask */

#define  ALL_ONES    0xFFFF

#define FDSR_HDLC    0x7E7E    /* FCC Data Synch. Reg. */


/*----------------------------*/
/* FCCE Register in HDLC Mode */
/*----------------------------*/

#define  HDLC_FCCE_IDL     0x00000100  /* Idle sequence status changed */
#define  HDLC_FCCE_FLG     0x00000200  /* Flag Status */
#define  HDLC_FCCE_RXB     0x00010000  /* Complete frame not received */
#define  HDLC_FCCE_TXB     0x00020000  /* Last byte of the buffer xmited */
#define  HDLC_FCCE_BSY     0x00040000  /* frame discarded-no buffer */
#define  HDLC_FCCE_RXF     0x00080000  /* frame received */
#define  HDLC_FCCE_TXE     0x00100000  /* Error like CTS lost or overrun */
#define  HDLC_FCCE_GRA     0x00800000  /* Graceful stop complete */

/*----------------------------*/
/* FCCM Register in HDLC Mode */
/*----------------------------*/

#define  HDLC_FCCM_IDL     0x00000100  /* Idle sequence status changed */
#define  HDLC_FCCM_FLG     0x00000200  /* Flag Status */
#define  HDLC_FCCM_RXB     0x00010000  /* Complete frame not received */
#define  HDLC_FCCM_TXB     0x00020000  /* Last byte of the buffer xmited */
#define  HDLC_FCCM_BSY     0x00040000  /* frame discarded-no buffer */
#define  HDLC_FCCM_RXF     0x00080000  /* frame received */
#define  HDLC_FCCM_TXE     0x00100000  /* Error like CTS lost or overrun */
#define  HDLC_FCCM_GRA     0x00800000  /* Graceful stop complete */

/*------------------------------*/
/* FPSMR1 Register in HDLC Mode */
/*------------------------------*/

#define  HDLC_FPSMR_16BIT_CRC   0x00000000   /* CRC Selection */
#define  HDLC_FPSMR_32BIT_CRC   0x00000080   /* CRC Selection */
#define  HDLC_FPSMR_NBL         0x00008000   /* Nibble Mode */
#define  HDLC_FPSMR_TS          0x00400000   /* Time Stamp */
#define  HDLC_FPSMR_MFF         0x04000000   /* Multiple frames in FIFO */
#define  HDLC_FPSMR_FSE         0x08000000   /* Flag Sharing Enable */
#define  HDLC_FPSMR_NOF_0       0x00000000   /* No flags between frames */
#define  HDLC_FPSMR_NOF_1       0x10000000   /* 1 flag between frames */
#define  HDLC_FPSMR_NOF_2       0x20000000   /* 2 flags between frames */
#define  HDLC_FPSMR_NOF_3       0x30000000   /* 3 flags between frames */
#define  HDLC_FPSMR_NOF_4       0x40000000   /* 4 flags between frames */
#define  HDLC_FPSMR_NOF_5       0x50000000   /* 5 flags between frames */
#define  HDLC_FPSMR_NOF_6       0x60000000   /* 6 flags between frames */
#define  HDLC_FPSMR_NOF_7       0x70000000   /* 7 flags between frames */
#define  HDLC_FPSMR_NOF_8       0x80000000   /* 8 flags between frames */
#define  HDLC_FPSMR_NOF_9       0x90000000   /* 9 flags between frames */
#define  HDLC_FPSMR_NOF_10      0xA0000000   /* 10 flags between frames */
#define  HDLC_FPSMR_NOF_11      0xB0000000   /* 11 flags between frames */
#define  HDLC_FPSMR_NOF_12      0xC0000000   /* 12 flags between frames */
#define  HDLC_FPSMR_NOF_13      0xD0000000   /* 13 flags between frames */
#define  HDLC_FPSMR_NOF_14      0xE0000000   /* 14 flags between frames */
#define  HDLC_FPSMR_NOF_15      0xF0000000   /* 15 flags between frames */
