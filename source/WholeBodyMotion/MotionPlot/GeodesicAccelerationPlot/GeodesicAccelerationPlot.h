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

#ifndef __MMM_GeodesicAccelerationPlot_H_
#define __MMM_GeodesicAccelerationPlot_H_

#include <MMM/MotionPlot/SensorPlot.h>

#include <Inventor/nodes/SoSeparator.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <MMM/Motion/Sensor/BasicKinematicSensor.h>

namespace MMM
{

class GeodesicAccelerationPlot : public SensorPlot
{
public:
    GeodesicAccelerationPlot(BasicKinematicSensorPtr sensor, MotionPtr motion);

    std::vector<std::string> getNames();

    std::tuple<QVector<double>, QVector<double> > getPlot(std::string name);

    std::string getName();

    static constexpr const char* NAME = "GeodesicAccelerationPlot";

    static constexpr const char* RobotNodeSetName = "RightLeg-7dof"; // probably has to be adapted here! Currently not supported in GUI! Change this line if needed

private:
    BasicKinematicSensorPtr sensor;
    MotionPtr motion;
    ModelPtr model;
    VirtualRobot::RobotNodeSetPtr rns;
};

}

#endif
