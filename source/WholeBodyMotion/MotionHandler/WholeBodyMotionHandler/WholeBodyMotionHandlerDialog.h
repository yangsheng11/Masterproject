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

#pragma once

#include <QDialog>
#include <QSettings>
#include <Inventor/nodes/SoSeparator.h>

#ifndef Q_MOC_RUN
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <MMM/Motion/Motion.h>
#endif

namespace Ui {
class WholeBodyMotionHandlerDialog;
}

class WholeBodyMotionHandlerDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WholeBodyMotionHandlerDialog(QWidget* parent);
    ~WholeBodyMotionHandlerDialog();

    void jumpTo(float timestep);

    void openMotions(MMM::MotionRecordingPtr motions, const std::map<std::string, VirtualRobot::RobotPtr> &currentRobots);

signals:
    void addVisualisation(SoSeparator* sep);

private:
    Ui::WholeBodyMotionHandlerDialog* ui;
    MMM::MotionRecordingPtr motions;
    std::map<std::string, VirtualRobot::RobotPtr> currentRobots;
};
