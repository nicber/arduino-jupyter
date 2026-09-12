#pragma once

// Las cosas que hay que hacer antes de que un sketch pueda confiar en el hardware.
// Eran tres trabajos en un solo header de casi quinientas lineas; ahora cada uno
// tiene el suyo y esto los incluye a los tres, para quien quiera el arranque entero
// sin elegir.
//
// Los tres fallan de manera muda --el sintoma es una placa que «no anda»-- y los
// tres se arreglan en tres lineas si uno sabe cuales. Cada header cuenta las suyas.

#include "BoardClock.h"
#include "BoardAdc.h"
#include "I2CBus.h"
