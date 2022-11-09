#ifndef FC_UTILITY_H
#define FC_UTILITY_H

#include "stm32f4xx_hal.h"

#include <stdint.h>


/*! Typedef Declarations */

typedef enum {

  FCSUCCESS = 0U,
  FCFAIL = !SUCCESS,

} FCErrorStatus;

typedef enum {

  TRUE = 0U,
  FALSE = !TRUE

}FCStatus;


#endif // FC_UTILITY_H
