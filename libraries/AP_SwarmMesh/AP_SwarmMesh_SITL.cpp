/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_SwarmMesh_SITL.h"

#if AP_SWARMMESH_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_SwarmMesh_SITL::AP_SwarmMesh_SITL(AP_SwarmMesh &frontend) :
    AP_SwarmMesh_Backend(frontend),
    sitl(AP::sitl())
{
}

// return true if we have received a packet recently
bool AP_SwarmMesh_SITL::healthy()
{
    return (AP_HAL::millis() - last_update_ms) < 3000;
}

// update: placeholder — will simulate peer packets in a future step
void AP_SwarmMesh_SITL::update(void)
{
    last_update_ms = AP_HAL::millis();
}

#endif // AP_SWARMMESH_SITL_ENABLED
