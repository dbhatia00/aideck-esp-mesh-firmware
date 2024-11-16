#ifndef CPX_RECEIVE_H
#define CPX_RECEIVE_H

#include "cpx.h"
#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void cpxReceiveInit(void);
void sendTelemetryToCF(const TelemetryData_t *telemetryData);

// Declare the queue handle as extern to be used in spi_transport.c
extern xQueueHandle cpxRxQueue;

#endif // CPX_RECEIVE_H
