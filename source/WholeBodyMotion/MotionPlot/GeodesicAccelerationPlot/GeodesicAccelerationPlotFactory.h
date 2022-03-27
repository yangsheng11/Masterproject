/*
This file is part of MMM.

MMM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MMM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MMM.  If not, see <http://www.gnu.org/licenses/>.
*
* @package    MMM
* @author     Andre Meixner
* @copyright  2021 High Performance Humanoid Technologies (H2T), Karlsruhe, Germany
*
*/

#ifndef __MMM_GeodesicAccelerationPlotFACTORY_H_
#define __MMM_GeodesicAccelerationPlotFACTORY_H_

#include <string>

#include <MMM/MotionPlot/SensorPlotFactory.h>

namespace MMM
{

class GeodesicAccelerationPlotFactory : public SensorPlotFactory
{
public:
    GeodesicAccelerationPlotFactory();

    virtual ~GeodesicAccelerationPlotFactory();

    std::string getName();

    std::string getUnit();

    std::string getValueName();

    SensorPlotList createSensorPlots(MotionPtr motion, MotionRecordingPtr motions);

    bool hasSensorPlot(MotionPtr motion, MotionRecordingPtr motions);

    static SensorPlotFactoryPtr createInstance(void*);

private:
    static SubClassRegistry registry;

};

}

#endif
