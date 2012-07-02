#include <16F873A.h>

//#include <16F737.h>
#device ICD=TRUE
#device adc=10
//#device adc=8
#use delay(clock=20000000)
#fuses HS,NOWDT
#use rs232 (debugger,STREAM=MONITOR)
#include <mcp4921.c>

