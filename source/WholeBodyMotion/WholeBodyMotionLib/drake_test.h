#ifdef DRAKE_AVAILABLE

#include <drake/examples/pendulum/pendulum_plant.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/trajectory_optimization/direct_collocation.h>

namespace drake {
namespace examples {
namespace pendulum {

//
// Main function for demo.
//
void doMain() {
    auto pendulum = std::make_unique<PendulumPlant<double>>();
      pendulum->set_name("pendulum");

      auto context = pendulum->CreateDefaultContext();

      const int kNumTimeSamples = 21;
      const double kMinimumTimeStep = 0.2;
      const double kMaximumTimeStep = 0.5;
      systems::trajectory_optimization::DirectCollocation dircol(
          pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
          kMaximumTimeStep);

      //... just some random code from https://github.com/RobotLocomotion/drake/blob/master/examples/pendulum/trajectory_optimization_simulation.cc
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake

#endif

