#include <16F873A.h>

#ifdef DEBUG
   #device ICD=TRUE
#endif

#device adc=10
#use delay(clock=20000000)
#ifdef DEBUG
   #use rs232 (DEBUGGER)
#endif
#fuses HS,PUT,NOWDT
#include <mcp4921.c>
