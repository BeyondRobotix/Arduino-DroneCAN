#ifndef ARDU_DRONECAN
#define ARDU_DRONECAN

#include <dronecan_msgs.h>
#include <Arduino.h>
#ifdef CANL431
    #include <canL431.h>
#endif
#ifdef CANH7
    #include <canH743.h>
#endif
#ifdef ARDUINO_NUCLEO_H723ZG
    #include <canH723.h>
#endif
#include <storage.h>
#include <vector>
#include <IWatchdog.h>
#include <app.h>
#include <simple_dronecanmessages.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a
#define PREFERRED_NODE_ID 100

class DroneCAN
{
private:
    uint8_t memory_pool[1024];
    struct uavcan_protocol_NodeStatus node_status;
    CanardCANFrame CAN_TX_msg;
    CanardCANFrame CAN_rx_msg;
    CanardCANFrame rx_frame;
    uint32_t looptime;
    bool led_state = false;
    uint64_t uptime = 0;
    std::vector<size_t> sorted_indices;                   // built on first use
    int node_id = 0;
    char node_name[80];

    struct firmware_update
    {
        char path[256];
        uint8_t node_id;
        uint8_t transfer_id;
        uint32_t last_read_ms;
        int fd;
        uint32_t offset;
    } fwupdate;

    struct dynamic_node_allocation
    {
        uint32_t send_next_node_id_allocation_request_at_ms;
        uint32_t node_id_allocation_unique_id_offset;
    } DNA;

    struct app_bootloader_comms
    {
        uint32_t magic;
        uint32_t ip;
        uint32_t netmask;
        uint32_t gateway;
        uint32_t reserved;
        uint8_t server_node_id;
        uint8_t my_node_id;
        uint8_t path[201];
    };

    static void getUniqueID(uint8_t id[16]);
    uint8_t get_preferred_node_id();

    void read_parameter_memory();
    void request_DNA();
    void send_NodeStatus(void);
    void process1HzTasks(uint64_t timestamp_usec);
    void processTx();
    void processRx();
    static uint64_t micros64();

    // Helper function to set parameter by index with validation and persistence
    void setParameterByIndex(size_t idx, float value);

    // Helper function to find parameter index by name
    size_t getParameterIndex(const char *name, size_t name_len);

public:
    struct parameter
    {
        const char *name;
        enum uavcan_protocol_param_Value_type_t type;
        float value;
        float min_value;
        float max_value;
    };

    // Shorter type aliases for parameter definitions
    static constexpr uavcan_protocol_param_Value_type_t INT = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
    static constexpr uavcan_protocol_param_Value_type_t REAL = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
    static constexpr uavcan_protocol_param_Value_type_t FLOAT = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
    static constexpr uavcan_protocol_param_Value_type_t BOOL = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
    static constexpr uavcan_protocol_param_Value_type_t STRING = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;

    std::vector<parameter> parameters;

    // copy a parameter list into the object
    void set_parameters(const std::vector<parameter> &param_list)
    {
        parameters = param_list;
    }

    void init(CanardOnTransferReception onTransferReceived, CanardShouldAcceptTransfer shouldAcceptTransfer, const std::vector<parameter> &param_list, const char *name);
    void init(const std::vector<parameter> &param_list, const char *name);

    /*
        Bare init: brings up CAN + canard without touching parameter storage.
        Used by environments that don't have writable parameter flash (e.g. bootloader)
        or that want to avoid the std::vector parameter machinery entirely.

        preferred_node_id != 0  -> set as local node ID, skip DNA on first cycle()
        preferred_node_id == 0  -> DNA runs during cycle() as usual
    */
    void init(CanardOnTransferReception onTransferReceived,
              CanardShouldAcceptTransfer shouldAcceptTransfer,
              const char *name,
              uint8_t preferred_node_id);

    /*
        Optional hook: called from handle_file_read_response() with each chunk
        of firmware data before fwupdate.offset is advanced. If unset, the
        chunk is discarded (current upstream behavior). Set by the bootloader
        to persist chunks to flash.
    */
    typedef void (*firmware_write_fn)(uint32_t offset, const uint8_t *data, uint16_t len);
    void set_firmware_write_callback(firmware_write_fn cb) { firmware_write_cb = cb; }

private:
    firmware_write_fn firmware_write_cb = nullptr;

public:

    CanardInstance canard;
    int version_major = 0;
    int version_minor = 0;
    int hardware_version_major = 0;
    int hardware_version_minor = 0;

    // used in callbacks
    void handle_GetNodeInfo(CanardRxTransfer *transfer);
    void handle_param_GetSet(CanardRxTransfer *transfer);
    void handle_param_ExecuteOpcode(CanardRxTransfer *transfer);
    void handle_begin_firmware_update(CanardRxTransfer *transfer);
    void handle_file_read_response(CanardRxTransfer *transfer);
    int handle_DNA_Allocation(CanardRxTransfer *transfer);

    /*
        Drive the firmware-update FileRead client. Caller (bootloader) sets
        fwupdate.{node_id,path,offset} via begin_firmware_download(), then
        calls this in its main loop; it self-rate-limits to 750 ms between
        requests.
    */
    void send_firmware_read();

    /*
        Bootloader-side entry point: arm the FileRead client for an image
        held by `server_node_id` at `remote_path`. Idempotent.
    */
    void begin_firmware_download(uint8_t server_node_id, const char *remote_path);

    /* Inspect current download state. */
    bool firmware_download_active() const { return fwupdate.node_id != 0; }
    uint32_t firmware_download_offset() const { return fwupdate.offset; }
    void firmware_download_finish() { fwupdate.node_id = 0; }

    /*
        Override the health / mode / VSSC reported by send_NodeStatus().
        When `override_active` is true, send_NodeStatus uses these values
        instead of its defaults and instead of the firmware-update path.
        Set by callers that need to publish a non-default state (bootloader
        flagging a flash failure, app reporting degraded sensors, etc.).
    */
    void set_node_status_override(uint8_t health, uint8_t mode, uint16_t vssc)
    {
        node_status_override_health = health;
        node_status_override_mode   = mode;
        node_status_override_vssc   = vssc;
        node_status_override_active = true;
    }
    void clear_node_status_override() { node_status_override_active = false; }

    /*
        Pin toggled at 1 Hz inside cycle() as an "alive" heartbeat. Defaults to
        pin 19 (MicroNode's on-board LED) for backwards compatibility. Set to
        -1 to disable -- needed by callers that want to drive the LED with
        their own state machine (e.g. bootloader update progress / error
        patterns).
    */
    void set_cycle_led_pin(int pin) { cycle_led_pin = pin; }

private:
    bool     node_status_override_active = false;
    uint8_t  node_status_override_health = 0;
    uint8_t  node_status_override_mode   = 0;
    uint16_t node_status_override_vssc   = 0;
    int      cycle_led_pin = 19;

public:

    // user methods
    void cycle();
    void debug(const char *msg, uint8_t level);
    float getParameter(const char *name);
    int setParameter(const char *name, float value);
};

void DroneCANonTransferReceived(DroneCAN &dronecan, CanardInstance *ins, CanardRxTransfer *transfer);
bool DroneCANshouldAcceptTransfer(const CanardInstance *ins,
                                  uint64_t *out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id);

#endif // ARDU_DRONECAN