/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"

#include "config/feature.h"

#include "common/maths.h"
#include "common/utils.h"

#include "fc/config.h"

#include "flight/pid.h"
#include "flight/wind_estimator.h"

#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "sensors/pitotmeter.h"

#include "pitotmeter.h"
#include "pitotmeter_virtual.h"

#if defined(USE_WIND_ESTIMATOR) && defined(USE_PITOT_VIRTUAL) 

static void virtualPitotStart(pitotDev_t *pitot)
{
    UNUSED(pitot);
}

static void virtualPitotRead(pitotDev_t *pitot)
{
    UNUSED(pitot);
}

static void virtualPitotCalculate(pitotDev_t *pitot, float *pressure, float *temperature)
{
    UNUSED(pitot);
    float airSpeed = pidProfile()->fixedWingReferenceAirspeed; //float cm/s
    DEBUG_SET(DEBUG_VIRTUAL_PITOT, 0, 0);
    DEBUG_SET(DEBUG_VIRTUAL_PITOT, 1, 0);
    DEBUG_SET(DEBUG_VIRTUAL_PITOT, 2, 0);
    DEBUG_SET(DEBUG_VIRTUAL_PITOT, 3, gpsSol.groundSpeed); 
    if (isEstimatedWindSpeedValid()) {
        uint16_t windHeading = 0; //centidegrees
        float windSpeed = getEstimatedHorizontalWindSpeed(&windHeading); //cm/s
        DEBUG_SET(DEBUG_VIRTUAL_PITOT, 0, CENTIDEGREES_TO_DECIDEGREES(windHeading)); //deci because overflow
        DEBUG_SET(DEBUG_VIRTUAL_PITOT, 1, windSpeed);
        float horizontalWindSpeed = windSpeed * cos_approx(CENTIDEGREES_TO_RADIANS(windHeading - posControl.actualState.yaw)); //yaw int32_t centidegrees
        DEBUG_SET(DEBUG_VIRTUAL_PITOT, 2, horizontalWindSpeed);
        airSpeed = posControl.actualState.velXY - horizontalWindSpeed; //cm/s //gpsSol.groundSpeed
    }
    if (pressure)
        //*pressure = sq(airSpeed / 100) * AIR_DENSITY_SEA_LEVEL_15C / 2 + P0;
        *pressure = sq(airSpeed) * AIR_DENSITY_SEA_LEVEL_15C / 20000f + P0;
    if (temperature)
        *temperature = 288.15f;     // Temperature at standard sea level (288.15 K)
}

bool virtualPitotDetect(pitotDev_t *pitot)
{
    pitot->delay = 10000;
    pitot->start = virtualPitotStart;
    pitot->get = virtualPitotRead;
    pitot->calculate = virtualPitotCalculate;
    return feature(FEATURE_GPS);
}
#endif
