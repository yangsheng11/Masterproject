#include "WholeBodyMotionHandlerFactory.h"
#include "WholeBodyMotionHandler.h"

#include <boost/extension/extension.hpp>

using namespace MMM;

// register this factory
MotionHandlerFactory::SubClassRegistry WholeBodyMotionHandlerFactory::registry(WholeBodyMotionHandler::NAME, &WholeBodyMotionHandlerFactory::createInstance);

WholeBodyMotionHandlerFactory::WholeBodyMotionHandlerFactory() : MotionHandlerFactory() {}

WholeBodyMotionHandlerFactory::~WholeBodyMotionHandlerFactory() = default;

std::string WholeBodyMotionHandlerFactory::getName() {
    return WholeBodyMotionHandler::NAME;
}

MotionHandlerPtr WholeBodyMotionHandlerFactory::createMotionHandler(QWidget* widget) {
    return MotionHandlerPtr(new WholeBodyMotionHandler(widget));
}

MotionHandlerFactoryPtr WholeBodyMotionHandlerFactory::createInstance(void *) {
    return MotionHandlerFactoryPtr(new WholeBodyMotionHandlerFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL MotionHandlerFactoryPtr getFactory() {
    return MotionHandlerFactoryPtr(new WholeBodyMotionHandlerFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL std::string getVersion() {
    return MotionHandlerFactory::VERSION;
}
