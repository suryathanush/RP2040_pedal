/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"  // printf(), sleep_ms(), getchar_timeout_us(), to_us_since_boot(), get_absolute_time()
#include "pico/bootrom.h" // reset_usb_boot()
#include <tusb.h>         // tud_cdc_connected()
#include <RF24.h>         // RF24 radio object
#include "defaultPins.h"  // board presumptive default pin numbers for CE_PIN and CSN_PIN

int PAYLOAD_SIZE = 16;
// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN, 4000000);

bool setup_radio(){
    uint8_t address[][6] = {"Wpipe", "Rpipe"};

    while (!tud_cdc_connected()) {
        sleep_ms(10);
    }
    sleep_ms(2000);

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
        printf("radio hardware is not responding!!\n");
        return false;
    }

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio.setPayloadSize(PAYLOAD_SIZE); // float datatype occupies 4 bytes
    radio.setRetries(5,15);
    radio.setAutoAck(false);
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[0]); // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[1]); // using pipe 1

    radio.printPrettyDetails(); // (larger) function that prints human readable data
}