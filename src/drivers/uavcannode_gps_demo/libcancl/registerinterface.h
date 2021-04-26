
#ifndef UAVCAN_REGISTER_INTERFACE_H_
#define UAVCAN_REGISTER_INTERFACE_H_

#include <canard.h>

#define NUNAVUT_ASSERT
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/_register/Value_1_0.h"

#include "time.h"

//No of pre allocated register entries
#ifndef UAVCAN_REGISTER_COUNT
# define UAVCAN_REGISTER_COUNT 5
#endif

#define UAVCAN_REGISTER_ERROR_SERIALIZATION 1
#define UAVCAN_REGISTER_ERROR_OUT_OF_MEMORY 2

typedef int32_t (*register_access_set_callback)(uavcan_register_Value_1_0 *value);
typedef uavcan_register_Value_1_0(*register_access_get_callback)(void);

typedef struct {
	/// uavcan.register.Name.1.0 name
	const char *name;
	register_access_set_callback cb_set;
	register_access_get_callback cb_get;
} uavcan_register_interface_entry;


int32_t uavcan_register_interface_init(CanardInstance *ins, uavcan_node_GetInfo_Response_1_0 *info);

int32_t uavcan_register_interface_add_entry(const char *name, register_access_set_callback cb_set,
		register_access_get_callback cb_get);

// Handler for all PortID registration related messages
int32_t uavcan_register_interface_process(CanardInstance *ins, CanardTransfer *transfer);

// Handler for node.GetInfo which yields a response
int32_t uavcan_register_interface_get_info_response(CanardInstance *ins, CanardTransfer *request);

// Handler for register access interface
int32_t uavcan_register_interface_access_response(CanardInstance *ins, CanardTransfer *request);

// Handler for register list interface
int32_t uavcan_register_interface_list_response(CanardInstance *ins, CanardTransfer *request);

#endif //UAVCAN_REGISTER_INTERFACE_H_
