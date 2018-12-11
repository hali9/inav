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

#include "config/feature.h"

#include "common/maths.h"
#include "common/utils.h"

#include "flight/wind_estimator.h"

#include "io/gps.h"

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
    int16_t speed = gpsSol.groundSpeed; //cm/s
    float airSpeed = speed;
    if (isEstimatedWindSpeedValid()) {
        uint16_t windHeading = 0; //centidegrees
        float windSpeed = getEstimatedHorizontalWindSpeed(&windHeading); //cm/s
        int32_t heading = posControl.actualState.yaw; //centidegrees
        float horizontalWindSpeed = windSpeed * cos_approx(DEGREES_TO_RADIANS(windHeading - heading));
        float airSpeed -= horizontalWindSpeed;
    }    
    if (pressure)
        *pressure = sq(airSpeed) * AIR_DENSITY_SEA_LEVEL_15C / 2 + * P0;
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
