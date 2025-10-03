#include <dronecan.h>

/*
    Kick off the CAN driver, Canard, do some parameter management
*/
void DroneCAN::init(CanardOnTransferReception onTransferReceived,
                    CanardShouldAcceptTransfer shouldAcceptTransfer,
                    const std::vector<parameter> &param_list,
                    const char *name)
{
    // start our CAN driver
    CANInit(CAN_1000KBPS, 2);

    strncpy(this->node_name, name, sizeof(this->node_name));

    IWatchdog.reload();

    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    if (this->node_id > 0)
    {
        canardSetLocalNodeID(&this->canard, node_id);
    }
    else
    {
        Serial.println("Waiting for DNA node allocation");
    }

    // initialise the internal LED
    pinMode(19, OUTPUT);

    IWatchdog.reload();

    // put our user params into memory
    this->set_parameters(param_list);

    // get the parameters in the EEPROM
    this->read_parameter_memory();

    while(canardGetLocalNodeID(&this->canard) == CANARD_BROADCAST_NODE_ID)
    {
        this->cycle();
        IWatchdog.reload();
        digitalWrite(19, this->led_state);
        this->led_state = !this->led_state;
    }
}

/*
    Gets the node id our DNA requests on init
*/
uint8_t DroneCAN::get_preferred_node_id()
{
    float ret = this->getParameter("NODEID");
    if (ret > 0 || ret < 127)
    {
        return (uint8_t)ret;
    }
    else
    {
        Serial.println("No NODEID in storage, setting..");
        this->setParameter("NODEID", PREFERRED_NODE_ID);
        return PREFERRED_NODE_ID;
    }
}

/*
    Processes any DroneCAN actions required. Call as quickly as practical !
*/
void DroneCAN::cycle()
{
    const uint32_t now = millis();

    if (now - this->looptime > 1000)
    {
        this->looptime = millis();
        this->process1HzTasks(this->micros64());
        digitalWrite(19, this->led_state);
        this->led_state = !this->led_state;
    }

    this->processRx();
    this->processTx();
    this->request_DNA();
}

/*
    For compatibility
*/
uint64_t DroneCAN::micros64()
{
    return (uint64_t)micros();
}

/*
    Returns the unique STM CPU ID required for DNA
*/
void DroneCAN::getUniqueID(uint8_t uniqueId[16])
{
    memset(uniqueId, 0, 16);

    uint32_t cpuid0 = HAL_GetUIDw0();
    uint32_t cpuid1 = HAL_GetUIDw1();
    uint32_t cpuid2 = HAL_GetUIDw2();

    uniqueId[0] = (uint8_t)(cpuid0 >> 24);
    uniqueId[1] = (uint8_t)(cpuid0 >> 16);
    uniqueId[2] = (uint8_t)(cpuid0 >> 8);
    uniqueId[3] = (uint8_t)(cpuid0);
    uniqueId[4] = (uint8_t)(cpuid1 >> 24);
    uniqueId[5] = (uint8_t)(cpuid1 >> 16);
    uniqueId[6] = (uint8_t)(cpuid1 >> 8);
    uniqueId[7] = (uint8_t)(cpuid1);
    uniqueId[8] = (uint8_t)(cpuid2 >> 24);
    uniqueId[9] = (uint8_t)(cpuid2 >> 16);
    uniqueId[10] = (uint8_t)(cpuid2 >> 8);
    uniqueId[11] = (uint8_t)(cpuid2);
    uniqueId[12] = 0;
    uniqueId[13] = 0;
    uniqueId[14] = 0;
    uniqueId[15] = 0;
}

/*
    Responds to a request for node info from CAN devices
*/
void DroneCAN::handle_GetNodeInfo(CanardRxTransfer *transfer)
{
    Serial.print("GetNodeInfo request from");
    Serial.println(transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = uptime;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = this->version_major;
    pkt.software_version.minor = this->version_minor;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = this->hardware_version_major;
    pkt.hardware_version.minor = this->hardware_version_minor;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char *)pkt.name.data, this->node_name, sizeof(pkt.name.data));
    pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
    handle parameter GetSet request
*/
void DroneCAN::handle_param_GetSet(CanardRxTransfer *transfer)
{
    // Decode the incoming request
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req))
    {
        return; // malformed
    }

    IWatchdog.reload();

    // Figure out which parameter they meant
    size_t idx = SIZE_MAX;

    if ((int)req.name.len > 0)
    {
        // Name‐based lookup
        Serial.print("Name based lookup");
        for (size_t i = 0; i < parameters.size(); i++)
        {
            auto &p = parameters[i];
            if (req.name.len == strlen(p.name) &&
                memcmp(req.name.data, p.name, req.name.len) == 0)
            {
                idx = i;
                Serial.println(idx);
                break;
            }
        }
    }
    // If that failed, try index‐based lookup
    if (idx == SIZE_MAX && req.index < parameters.size())
    {
        idx = req.index;
        Serial.print("Parameter index lookup");
        Serial.println(idx);
    }

    IWatchdog.reload();

    // If it’s a _set_ request, apply the new value
    if (idx != SIZE_MAX && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
    {
        auto &p = parameters[idx];
        switch (p.type)
        {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            p.value = req.value.integer_value;
            EEPROM.put(idx * sizeof(float), p.value);
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            p.value = req.value.real_value;
            EEPROM.put(idx * sizeof(float), p.value);
            break;
        default:
            // unsupported type
            break;
        }
    }

    IWatchdog.reload();

    // Now build the GetSet _response_, always sending one back
    struct uavcan_protocol_param_GetSetResponse rsp;
    memset(&rsp, 0, sizeof(rsp));

    if (idx != SIZE_MAX)
    {
        auto &p = parameters[idx];
        // tag + value
        rsp.value.union_tag = p.type;
        if (p.type == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
        {
            rsp.value.integer_value = p.value;
        }
        else
        {
            rsp.value.real_value = p.value;
        }

        // copy name (must pad/zero any unused bytes)
        size_t namelen = strlen(p.name);
        rsp.name.len = namelen;
        memset(rsp.name.data, 0, sizeof(rsp.name.data));
        memcpy(rsp.name.data, p.name, namelen);
    }
    // else idx==SIZE_MAX: leave rsp.name.len=0 / value empty

    // Encode & send
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t len = uavcan_protocol_param_GetSetResponse_encode(&rsp, buffer);
    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           len);
}

/*
    handle parameter executeopcode request
*/
void DroneCAN::handle_param_ExecuteOpcode(CanardRxTransfer *transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req))
    {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
    {
        // Reset all parameters to defaults
        for (size_t i = 0; i < parameters.size(); i++)
        {
            parameters[i].value = parameters[i].min_value; // Or some default value
        }
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
    {
        // Save all the changed parameters to permanent storage
        for (size_t i = 0; i < parameters.size(); i++)
        {
            EEPROM.put(i * 4, parameters[i].value);
        }
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
    Read the EEPROM parameter storage and set the current parameter list to the read values
*/
void DroneCAN::read_parameter_memory()
{
    float p_val = 0.0;

    for (size_t i = 0; i < parameters.size(); i++)
    {
        EEPROM.get(i * 4, p_val);
        parameters[i].value = p_val;
    }
}

/*
    Get a parameter from storage by name
    Only handles float return values
    returns __FLT_MIN__ if no parameter found with the provided name
*/
float DroneCAN::getParameter(const char *name)
{
    for (size_t i = 0; i < parameters.size(); i++)
    {
        auto &p = parameters[i];
        if (strlen(name) == strlen(p.name) &&
            memcmp(name, p.name, strlen(name)) == 0)
        {
            return parameters[i].value;
        }
    }
    return __FLT_MIN__;
}

/*
    Set a parameter from storage by name
    Values get stored as floats
    returns -1 if storage failed, 0 if good
*/
int DroneCAN::setParameter(const char *name, float value)
{
    for (size_t i = 0; i < parameters.size(); i++)
    {
        auto &p = parameters[i];
        if (strlen(name) == strlen(p.name) &&
            memcmp(name, p.name, strlen(name)) == 0)
        {
            parameters[i].value = value;
            return 0;
        }
    }
    return -1;
}

/*
    handle a DNA allocation packet
*/
int DroneCAN::handle_DNA_Allocation(CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
    {
        // already allocated
        return 0;
    }

    Serial.println("We got a node ID message back");

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        Serial.println("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return 0;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0)
    {
        Serial.println("DNA failed this time");
        DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return 0;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data))

    {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
        Serial.println("second stage Node ID allocation");
    }
    else
    {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(&canard, msg.node_id);
        Serial.print("Node ID allocated: ");
        Serial.println(msg.node_id);
    }
    return 0;
}

/*
    ask for a dynamic node allocation
*/
void DroneCAN::request_DNA()
{

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
    {
        return;
    }

    // we're still waiting for a DNA allocation of our node ID
    if (millis() < DNA.send_next_node_id_allocation_request_at_ms)
    {
        return;
    }

    const uint32_t now = millis();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    uint8_t pref_node_id = (uint8_t)(this->get_preferred_node_id() << 1U);
    
    Serial.print("Requesting ID ");
    Serial.println(pref_node_id / 2); // not sure why this is over 2 .. something to do with the bit shifting but this is what it actually sets
    
    allocation_request[0] = pref_node_id;

    if (DNA.node_id_allocation_unique_id_offset == 0)
    {
        allocation_request[0] |= 1; // First part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);

    if (uid_size > MaxLenOfUniqueIDInRequest)
    {
        uid_size = MaxLenOfUniqueIDInRequest;
    }
    if (uid_size + DNA.node_id_allocation_unique_id_offset > 16)
    {
        uid_size = 16 - DNA.node_id_allocation_unique_id_offset;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t)(uid_size + 1));
    if (bcast_res < 0)
    {
        Serial.print("Could not broadcast ID allocation req; error");
        Serial.println(bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
    handle a BeginFirmwareUpdate request from a management tool like DroneCAN GUI tool or MissionPlanner

    There are multiple ways to handle firmware update over DroneCAN:

    1) on BeginFirmwareUpdate reboot to the bootloader, and implement
        the firmware upudate process in the bootloader. This is good on
        boards with smaller amounts of flash

    2) if you have enough flash for 2 copies of your firmware then you
        can use an A/B scheme, where the new firmware is saved to the
        inactive flash region and a tag is used to indicate which
        firmware to boot next time

    3) you could write the firmware to secondary storage (such as a
        microSD) and the bootloader would flash it on next boot

    In this example firmware we will write it to a file
    newfirmware.bin, which is option 3

    Note that you cannot rely on the form of the filename. The client
    may hash the filename before sending
 */
void DroneCAN::handle_begin_firmware_update(CanardRxTransfer *transfer)
{
    Serial.println("Update request received");

    auto *comms = (struct app_bootloader_comms *)0x20000000;

    if (comms->magic != APP_BOOTLOADER_COMMS_MAGIC)
    {
        memset(comms, 0, sizeof(*comms));
    }
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;

    uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req))
    {
        return;
    }

    comms->server_node_id = req.source_node_id;
    if (comms->server_node_id == 0)
    {
        comms->server_node_id = transfer->source_node_id;
    }
    memcpy(comms->path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);
    comms->my_node_id = canardGetLocalNodeID(&canard);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply{};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint16_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer);
    static uint8_t transfer_id;
    CanardTxTransfer transfer_object = {
        .transfer_type = CanardTransferTypeResponse,
        .data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
        .data_type_id = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
        .inout_transfer_id = &transfer->transfer_id,
        .priority = transfer->priority,
        .payload = &buffer[0],
        .payload_len = total_size,
    };
    const auto res = canardRequestOrRespondObj(&canard,
                                               transfer->source_node_id,
                                               &transfer_object);

    uint8_t count = 50;
    while (count--)
    {
        processTx();
        delay(1);
    }

    NVIC_SystemReset();
}

/*
    send a read for a firmware update. This asks the client (firmware
    server) for a piece of the new firmware
*/
void DroneCAN::send_firmware_read(void)
{
    uint32_t now = millis();
    if (now - fwupdate.last_read_ms < 750)
    {
        // the server may still be responding
        return;
    }
    fwupdate.last_read_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];

    struct uavcan_protocol_file_ReadRequest pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.path.path.len = strlen((const char *)fwupdate.path);
    pkt.offset = fwupdate.offset;
    memcpy(pkt.path.path.data, fwupdate.path, pkt.path.path.len);

    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           fwupdate.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
                           &fwupdate.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
    handle response to send_firmware_read()
*/
void DroneCAN::handle_file_read_response(CanardRxTransfer *transfer)
{
    if ((transfer->transfer_id + 1) % 32 != fwupdate.transfer_id ||
        transfer->source_node_id != fwupdate.node_id)
    {
        /* not for us */
        Serial.println("Firmware update: not for us");
        return;
    }
    struct uavcan_protocol_file_ReadResponse pkt;
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &pkt))
    {
        /* bad packet */
        Serial.println("Firmware update: bad packet\n");
        return;
    }
    if (pkt.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK)
    {
        /* read failed */
        fwupdate.node_id = 0;
        Serial.println("Firmware update read failure\n");
        return;
    }

    // write(fwupdate.fd, pkt.data.data, pkt.data.len);
    // if (pkt.data.len < 256)
    // {
    //     /* firmware updare done */
    //     close(fwupdate.fd);
    //     Serial.println("Firmwate update complete\n");
    //     fwupdate.node_id = 0;
    //     return;
    // }
    fwupdate.offset += pkt.data.len;

    /* trigger a new read */
    fwupdate.last_read_ms = 0;
}

/*
    send the 1Hz NodeStatus message. This is what allows a node to show
    up in the DroneCAN GUI tool and in the flight controller logs
*/
void DroneCAN::send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = uptime++;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 0;

    /*
      when doing a firmware update put the size in kbytes in VSSC so
      the user can see how far it has reached
     */
    if (fwupdate.node_id != 0)
    {
        node_status.vendor_specific_status_code = fwupdate.offset / 1024;
        node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    }

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
    Handle re-occurring slow pace tasks
*/
void DroneCAN::process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
    Send any packets currently waiting with Canard
*/
void DroneCAN::processTx()
{
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        CANSend(txf);
        canardPopTxQueue(&canard); // fuck it we ball
    }
}

/*
    Look at our mailbox
*/
void DroneCAN::processRx()
{
    const uint64_t timestamp = micros();
    if (CANMsgAvail())
    {
        CANReceive(&CAN_rx_msg);
        int ret = canardHandleRxFrame(&canard, &CAN_rx_msg, timestamp);
        if (ret < 0)
        {
            // Serial.print("Canard RX fail");
            // Serial.println(ret);
        }
    }
}

/*
    Send a debug message over CAN
*/
void DroneCAN::debug(const char *msg, uint8_t level)
{
    uavcan_protocol_debug_LogMessage pkt{};
    pkt.level.value = level;
    pkt.text.len = strlen(msg);
    strncpy((char *)pkt.text.data, msg, pkt.text.len);

    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer);
    static uint8_t transfer_id;
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
    Bare minimum callback function for DroneCAN library requirements
*/
void DroneCANonTransferReceived(DroneCAN &dronecan, CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            dronecan.handle_DNA_Allocation(transfer);
            break;
        }
        }
    }
    // switch on data type ID to pass to the right handler function
    else if (transfer->transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            dronecan.handle_GetNodeInfo(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        {

            uavcan_protocol_RestartNodeResponse pkt{};
            pkt.ok = true;
            uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE];
            uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&pkt, buffer);
            static uint8_t transfer_id;
            canardBroadcast(ins,
                            UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_SIGNATURE,
                            UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_ID,
                            &transfer_id,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            buffer,
                            len);

            Serial.println("Reset..");
            delay(200);
            // yeeeeeet
            NVIC_SystemReset();
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            dronecan.handle_param_GetSet(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            dronecan.handle_param_ExecuteOpcode(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        {
            dronecan.handle_begin_firmware_update(transfer);
            break;
        }
        }
    }
}

/*
    Bare minimum message signing required for DroneCAN library
*/
bool DroneCANshoudlAcceptTransfer(const CanardInstance *ins,
                                  uint64_t *out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        // Check if we want to handle a specific service request
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_FILE_READ_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_ID;
            return true;
        }
        }
    }

    if (transfer_type == CanardTransferTypeResponse)
    {
        // Check if we want to handle a specific service response
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_FILE_READ_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        }
    }

    if (transfer_type == CanardTransferTypeBroadcast)
    {
        // Check if we want to handle a specific broadcast packet
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE;
            return true;
        }
        }
    }
    return false;
}
