/*
 * AP_MGM_PROTOCOL.h
 *
 *  Created on: Sep 08, 2018
 *      Author: guy tzoler
 *
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * MGM Command is 5 bytes
 *
 * Throttle command
 *		253_ADDRESS_RPML_RPMH_BCC
 *		Reply: 255 - OK
 *
 *		ADDRESS:
 *		0 .. 255 set by “Controller 2” software, parameter P15.
 *
 *		RPML + 256*RPMH:
 *		-1024 .. 1024 for PWM regulation -100% - 100%
 *		-25000 .. 25000 for rpm x 10 regulation
 *		-Inom .. Inom for current regulation, where Inom is nominal current for your esc
 *		Choice depend on parameter P69 (Driving type B) settings.
 *
 *		-- Implemented with no reversed thrust so will be working: 0 .. 1024 for PWM regulation 0% - 100%
 *
 *		BCC:
 *		BCC = ADDRESS xor RPML xor RPMH
 *
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>

//#include <GCS_MAVLink/GCS.h>

#define MGM_DATA_FRAME_SIZE		5
#define MGM_THR_CMD				253
#define MGM_CMD_MIN_VAL			0
#define MGM_CMD_MAX_VAL			1023

class AP_MGM_Protocol {
public:
    AP_MGM_Protocol();

    /* Do not allow copies */
    AP_MGM_Protocol(const AP_MGM_Protocol &other) = delete;
    AP_MGM_Protocol &operator=(const AP_MGM_Protocol&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:

    AP_HAL::UARTDriver *port;
    
    void init(void);
    void send_command(uint8_t data[MGM_DATA_FRAME_SIZE]);
    void update_mgm_bitmask(uint32_t new_bitmask);

    uint32_t last_mgm_update_time;
    uint32_t mgm_time_frame_micros;
    uint32_t last_used_bitmask;
    uint32_t last_mgm_report_time;

    AP_Int32 bitmask;
    bool initialised;
};
