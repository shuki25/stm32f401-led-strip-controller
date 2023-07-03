/*
* ir_codes.h
* Created on: 2023-07-02
* Author: Joshua Butler, MD, MHI
*
* This file contains IR remote codes for the IR remote control
*
*/


#ifndef __IR_CODES_H
#define __IR_CODES_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#if REMOTE_17KEYS    
#define IR_UP		0x00FF18E7
#define IR_DOWN		0x00FF4AB5
#define IR_LEFT		0x00FF10EF
#define IR_RIGHT	0x00FF5AA5
#define IR_OK 		0x00FF38C7
#define IR_NUM_1	0x00FFA25D
#define IR_NUM_2 	0x00FF629D
#define IR_NUM_3 	0x00FFE21D
#define IR_NUM_4 	0x00FF22DD
#define IR_NUM_5 	0x00FF02FD
#define IR_NUM_6 	0x00FFC23D
#define IR_NUM_7 	0x00FFE01F
#define IR_NUM_8 	0x00FFA857
#define IR_NUM_9 	0x00FF906F
#define IR_NUM_0 	0x00FF9867
#define IR_STAR 	0x00FF6897
#define IR_POUND 	0x00FFB04F
#elif REMOTE_44KEYS
#define	IR_BPlus  0x00FF3AC5 
#define	IR_BMinus 0x00FFBA45 
#define	IR_PLAY   0x00FF827D 
#define	IR_PWR 	  0x00FF02FD 
#define	IR_R 	  0x00FF1AE5 
#define	IR_G 	  0x00FF9A65 
#define	IR_B  	  0x00FFA25D 
#define	IR_W 	  0x00FF22DD 
#define	IR_B1	  0x00FF2AD5 
#define	IR_B2	  0x00FFAA55 
#define	IR_B3	  0x00FF926D 
#define	IR_B4	  0x00FF12ED 
#define	IR_B5	  0x00FF0AF5 
#define	IR_B6	  0x00FF8A75 
#define	IR_B7	  0x00FFB24D 
#define	IR_B8	  0x00FF32CD 
#define	IR_B9	  0x00FF38C7 
#define	IR_B10	  0x00FFB847 
#define	IR_B11	  0x00FF7887 
#define	IR_B12	  0x00FFF807 
#define	IR_B13	  0x00FF18E7 
#define	IR_B14	  0x00FF9867 
#define	IR_B15	  0x00FF58A7 
#define	IR_B16	  0x00FFD827 
#define	IR_UPR 	  0x00FF28D7 
#define	IR_UPG 	  0x00FFA857 
#define	IR_UPB 	  0x00FF6897 
#define	IR_QUICK  0x00FFE817 
#define	IR_DOWNR  0x00FF08F7 
#define	IR_DOWNG  0x00FF8877 
#define	IR_DOWNB  0x00FF48B7 
#define	IR_SLOW   0x00FFC837 
#define	IR_DIY1   0x00FF30CF 
#define	IR_DIY2   0x00FFB04F 
#define	IR_DIY3   0x00FF708F 
#define	IR_AUTO   0x00FFF00F 
#define	IR_DIY4   0x00FF10EF 
#define	IR_DIY5   0x00FF906F 
#define	IR_DIY6   0x00FF50AF 
#define	IR_FLASH  0x00FFD02F 
#define	IR_JUMP3  0x00FF20DF 
#define	IR_JUMP7  0x00FFA05F 
#define	IR_FADE3  0x00FF609F 
#define	IR_FADE7  0x00FFE01F 

#endif
    
    
#ifdef __cplusplus
}
#endif

#endif /* __IR_CODES_H */
