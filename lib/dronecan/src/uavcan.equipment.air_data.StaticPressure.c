

#define CANARD_DSDLC_INTERNAL
#include <uavcan.equipment.air_data.StaticPressure.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_equipment_air_data_StaticPressure_encode(struct uavcan_equipment_air_data_StaticPressure* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE);
    _uavcan_equipment_air_data_StaticPressure_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool uavcan_equipment_air_data_StaticPressure_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_air_data_StaticPressure* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_uavcan_equipment_air_data_StaticPressure_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_air_data_StaticPressure sample_uavcan_equipment_air_data_StaticPressure_msg(void) {

    struct uavcan_equipment_air_data_StaticPressure msg;






    msg.static_pressure = random_float_val();







    msg.static_pressure_variance = random_float16_val();





    return msg;

}
#endif
