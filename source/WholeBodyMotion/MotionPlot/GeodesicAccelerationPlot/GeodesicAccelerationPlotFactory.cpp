#include "GeodesicAccelerationPlotFactory.h"
#include "GeodesicAccelerationPlot.h"

#include <boost/extension/extension.hpp>

#include <MMM/Motion/Sensor/BasicKinematicSensor.h>
#include <VirtualRobot/Robot.h>

using namespace MMM;

// register this factory
SensorPlotFactory::SubClassRegistry GeodesicAccelerationPlotFactory::registry(GeodesicAccelerationPlot::NAME, &GeodesicAccelerationPlotFactory::createInstance);

GeodesicAccelerationPlotFactory::GeodesicAccelerationPlotFactory() : SensorPlotFactory() {}

GeodesicAccelerationPlotFactory::~GeodesicAccelerationPlotFactory() {}

std::string GeodesicAccelerationPlotFactory::getName()
{
    return GeodesicAccelerationPlot::NAME;
}

std::string GeodesicAccelerationPlotFactory::getUnit() {
    return "Marker position";
}

std::string GeodesicAccelerationPlotFactory::getValueName() {
    return "Marker position";
}

SensorPlotList GeodesicAccelerationPlotFactory::createSensorPlots(MotionPtr motion, MotionRecordingPtr /*motions*/) {
    SensorPlotList sensorPlots;
    for (auto sensor : motion->getSensorsByType<BasicKinematicSensor>()) {
        sensorPlots.push_back(SensorPlotPtr(new GeodesicAccelerationPlot(sensor, motion)));
    }
    return sensorPlots;
}

bool GeodesicAccelerationPlotFactory::hasSensorPlot(MotionPtr motion, MotionRecordingPtr /*motions*/) {
    return motion->hasSensor(BasicKinematicSensor::TYPE) && motion->getModel() && motion->getModel()->hasRobotNodeSet(GeodesicAccelerationPlot::RobotNodeSetName);
}

SensorPlotFactoryPtr GeodesicAccelerationPlotFactory::createInstance(void *)
{
    return SensorPlotFactoryPtr(new GeodesicAccelerationPlotFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL SensorPlotFactoryPtr getFactory() {
    return SensorPlotFactoryPtr(new GeodesicAccelerationPlotFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL std::string getVersion() {
    return SensorPlotFactory::VERSION;
}
