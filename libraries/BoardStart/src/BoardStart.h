#pragma once

// Las cosas que hay que hacer antes de que un sketch pueda confiar en el hardware.
// Son tres trabajos --el reloj, el ADC y el bus I2C--, cada uno en su header, y
// éste los incluye a los tres, para quien quiera el arranque entero sin elegir.
//
// Los tres fallan sin ningún mensaje --el síntoma es una placa que no funciona-- y
// los tres se resuelven en pocas líneas si se sabe cuáles. Cada header explica
// las suyas.

#include "BoardClock.h"
#include "BoardAdc.h"
#include "I2CBus.h"
