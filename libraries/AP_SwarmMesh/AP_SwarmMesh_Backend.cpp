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

#include "AP_SwarmMesh_Backend.h"

#if AP_SWARMMESH_ENABLED

// debug
#include <stdio.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_SwarmMesh_Backend::AP_SwarmMesh_Backend(AP_SwarmMesh &frontend) :
    _frontend(frontend)
{
    const AP_SerialManager &serialmanager = AP::serialmanager();
    uart = serialmanager.find_serial(AP_SerialManager::SerialProtocol_SwarmMesh, 0);
    if (uart == nullptr) {
        return;
    }

    uart->begin(serialmanager.find_baudrate(AP_SerialManager::SerialProtocol_SwarmMesh, 0));
}

// sysid accessor
uint8_t AP_SwarmMesh_Backend::frontend_sysid() const
{
    return (uint8_t)_frontend.sysid;
}

#endif  // AP_SWARMMESH_ENABLED
