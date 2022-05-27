#ifndef _DWM_MSG_HEADER_H_
#define _DWM_MSG_HEADER_H_

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************************************************************/
/*                                                MSG_IDX                                                           */
/********************************************************************************************************************/
#define SRC_IDX 0
#define DST_IDX 1
#define TYPE_IDX 2
#define PIR_IDX 3
#define LEN_IDX 4
#define MSG_IDX 5

/********************************************************************************************************************/
/*                                           Initialization Phase                                                   */
/********************************************************************************************************************/
#define BROADCAST 255
#define UP 'W'
#define DOWN 'S'
#define LEFT 'A'
#define RIGHT 'D'

/********************************************************************************************************************/
/*                                                MSG_TYPES                                                         */
/********************************************************************************************************************/
#define TYPE_DATA 0
#define TYPE_LAST 1
#define TYPE_ACK_1 2
#define TYPE_ACK_2 3
#define TYPE_RTS 4
#define TYPE_CTS 5

#define TYPE_DROP 10
#define TYPE_ACTIVATE 11
#define TYPE_MOVE 12

//#define MAX_FILE_SIZE 15000
#define MAX_FILE_SIZE 150000

#ifdef __cplusplus
}
#endif

#endif /* _DWM_MSG_HEADER_H_ */
