/*
 * uart_receive.h
 *
 *  Created on: Mar 30, 2020
 *      Author: brvi
 */

#ifndef UART_RECEIVE_TASK_H_
#define UART_RECEIVE_TASK_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/


#define LINE_LENGTH		(40u)

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
/* Task_Status Receive */
void task_uart_receive(void *param);



#endif /* UART_RECEIVE_TASK_H_ */
