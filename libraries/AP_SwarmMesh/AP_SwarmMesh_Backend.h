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
#pragma once

#include "AP_SwarmMesh.h"

#if AP_SWARMMESH_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_SwarmMesh_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_SwarmMesh_Backend(AP_SwarmMesh &frontend);

    // return true if radio is basically healthy (we are receiving data)
    virtual bool healthy() = 0;

    // update
    virtual void update() = 0;

protected:

    // references
    AP_SwarmMesh &_frontend;

    AP_HAL::UARTDriver *uart;
};

#endif  // AP_SWARMMESH_ENABLED
