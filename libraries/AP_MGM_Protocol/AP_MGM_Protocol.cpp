#include "AP_MGM_Protocol.h"

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MGM_Protocol::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of mgm protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_MGM_Protocol, bitmask, 0),

    AP_GROUPEND
};

// constructor
AP_MGM_Protocol::AP_MGM_Protocol(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MGM_Protocol::init(void)
{

    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MGM,0);

    if (port == nullptr && uint32_t(bitmask.get()) > 0) {
            // if no dedicated port is assigned but some channels are assigned can share the stream with volz since both are TTL at 115200
            port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);
    }

    update_mgm_bitmask(bitmask);
}

void AP_MGM_Protocol::update()
{
    if (!initialised) {
        initialised = true;
        init();
    }
    
    if (port == nullptr) {
        return;
    }

    uint32_t now = AP_HAL::micros();
    if (now - last_mgm_update_time < mgm_time_frame_micros || port->txspace() < MGM_DATA_FRAME_SIZE) {
        if(port->txspace() < MGM_DATA_FRAME_SIZE){
            gcs().send_text(MAV_SEVERITY_CRITICAL, "MGM: failed sending command due to low txspace");
        }
        return;
    }

    last_mgm_update_time = now;

    uint8_t i;
    uint16_t value;

    // loop for all 16 channels
    for (i=0; i<NUM_SERVO_CHANNELS; i++) {
        // check if current channel is needed for MGM protocol
        if (last_used_bitmask & (1U<<i)) {
            SRV_Channel *ch = SRV_Channels::srv_channel(i);
            if (ch == nullptr) {
                continue;
            }
            
            // check if current channel PWM is within range
            if (ch->get_output_pwm() < ch->get_output_min()) {
                value = MGM_CMD_MIN_VAL;
            } else {
                value = ch->get_output_pwm() - ch->get_output_min();
            }

            // scale the PWM value to MGM value
            value = value * MGM_CMD_MAX_VAL / (ch->get_output_max() - ch->get_output_min());

            // make sure value stays in range
            if (value > MGM_CMD_MAX_VAL) {
                value = MGM_CMD_MAX_VAL;
            }

            // prepare MGM protocol data.
            uint8_t data[MGM_DATA_FRAME_SIZE];

            data[0] = MGM_THR_CMD;
            data[1] = i + 1;		// send address as 1 based index so ch1 will have address 1, ch2 will have address 2 ....
            data[2] = LOWBYTE(value);
            data[3] = 0x03 & HIGHBYTE(value);

            send_command(data);

            if (now - last_mgm_report_time > 1000000){
                last_mgm_report_time = now;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "MGM: Sent: %1d %2d %3d", data[1], data[2], data[3]);
            }
        }
    }
}

// calculate BCC for MGM serial protocol and send the data.
void AP_MGM_Protocol::send_command(uint8_t data[MGM_DATA_FRAME_SIZE])
{
    uint8_t bcc;

    bcc = data[1] ^ data[2] ^ data[3];

	data[4] = bcc;

    port->write(data, MGM_DATA_FRAME_SIZE);
    port->flush();
}

void AP_MGM_Protocol::update_mgm_bitmask(uint32_t new_bitmask)
{
    uint8_t count = 0;
    last_used_bitmask = new_bitmask;

    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (new_bitmask & (1U<<i)) {
            count++;
        }
    }

    // have a safety margin of 20% to allow for not having full uart
    // utilisation. We really don't want to start filling the uart
    // buffer or we'll end up with servo lag
    const float safety = 1.3;

    // each channel take about 425.347us to transmit so total time will be ~ number of channels * 450us
    // rounded to 450 to make sure we don't go over the baud rate.
    uint32_t channels_micros = count * 450 * safety;

    // limit the minimum to 20000 will result a max refresh frequency of 50hz.
    if (channels_micros < 20000) {
        channels_micros = 20000;
    }

    mgm_time_frame_micros = channels_micros;
}
