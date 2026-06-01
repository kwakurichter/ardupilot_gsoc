#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_SwarmMesh_config.h"

#if AP_SWARMMESH_ENABLED
#define LOG_STRUCTURE_FROM_SWARMMESH    // TODO: define log struct
#else
#define LOG_STRUCTURE_FROM_SWARMMESH
#endif