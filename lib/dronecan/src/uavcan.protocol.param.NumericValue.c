

#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.param.NumericValue.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_param_NumericValue_encode(struct uavcan_protocol_param_NumericValue* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE);
    _uavcan_protocol_param_NumericValue_encode(buffer, &bit_ofs, msg, 
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
bool uavcan_protocol_param_NumericValue_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_param_NumericValue* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_uavcan_protocol_param_NumericValue_decode(transfer, &bit_ofs, msg,
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
struct uavcan_protocol_param_NumericValue sample_uavcan_protocol_param_NumericValue_msg(void) {

    struct uavcan_protocol_param_NumericValue msg;


    msg.union_tag = random_range_unsigned_val(0, 2);

    switch(msg.union_tag) {




        case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY: {



            msg.empty = sample_uavcan_protocol_param_Empty_msg();


            break;

        }



        case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE: {




            msg.integer_value = (int64_t)random_bitlen_signed_val(64);



            break;

        }



        case UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE: {




            msg.real_value = random_float_val();



            break;

        }




    }

    return msg;

}
#endif
