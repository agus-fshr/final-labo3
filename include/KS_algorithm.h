/*
 * KS_algorithm.h
 *
 *  Created on: Apr 21, 2024
 *      Author: mpaur
 */

#ifndef INC_KS_ALGORITHM_H_
#define INC_KS_ALGORITHM_H_

/********************** CPP guard ********************************************/

/********************** End of CPP guard *************************************/

/********************** inclusions *******************************************/
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/********************** macros ***********************************************/
//Note Frecuency
#define DOn       261 //.63
#define DOSOn     277 //.18
#define REn       293 //.66
#define RESOSn    311 //.13
#define MIn       329 //.63
#define FAn       349 //.23
#define SOLn      392 //
#define SOLSOSn   415 //.3
#define LAn       440 //
#define LASOSn    466 //
#define SIn       492 //.88
#define DOn1      261 //.63
#define DOSOn1    277 //.18
#define REn1      293 //.66
#define RESOSn1   311 //.13
#define MIn1      329 //.63
#define FAn1      349 //.23
#define SOLn1     392 //
#define SOLSOSn1  415 //.3
#define LAn1      440 //
#define LASOSn1   466 //
#define SIn1      492 //.88
#define DO8n     523 //.25

/********************** typedef **********************************************/

/********************** external data declaration ****************************/

/********************** external functions declaration ***********************/
uint32_t  KS_algorithm (uint32_t NewData);

#endif /* INC_KS_ALGORITHM_H_ */

/********************** end of file ******************************************/