#ifndef BVP_PLANT_H
#define BVP_PLANT_H
#include "drake/systems/framework/leaf_system.h"
#include "Tools.h"
#include <VirtualRobot/XML/RobotIO.h>
using namespace drake;
template <typename T>
class bvpplant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(bvpplant)

  /// Constructs a default plant.
  bvpplant(MMM::GeodesicMotionTools::geodesic_ode_dynamic_model* dynamic_model):
      systems::LeafSystem<T>(systems::SystemTypeTag<bvpplant>{}), dynamic_model(dynamic_model) {
      this->DeclareVectorInputPort("tau", systems::BasicVector<T>(dynamic_model->getnDoF()));
      this->DeclareVectorOutputPort("state", &bvpplant::CopyStateOut,{this->all_state_ticket()});
      this->DeclareContinuousState(dynamic_model->getnDoF()*2);
  };

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit bvpplant(const bvpplant<U>& other): bvpplant(other.get_dynamic_model()) {}

  ~bvpplant() = default;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_state_output_port() const{
      DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
      return systems::LeafSystem<T>::get_output_port(0);
  };

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  VectorX<T> get_tau(const systems::Context<T>& context) const {
      VectorX<T> a;
      a.resize(dynamic_model->getnDoF());
      for(auto i=0;i<dynamic_model->getnDoF();i++){
          a(i)=this->get_input_port().Eval(context)(i);
      }
      return a;
  }


  MMM::GeodesicMotionTools::geodesic_ode_dynamic_model* get_dynamic_model() const {
      return dynamic_model;
  }

 private:
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T> *output) const{
      output->set_value(context.get_continuous_state_vector().CopyToVector());
  };

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
          systems::ContinuousState<T>* derivatives) const {
      VectorX<T> state = context.get_continuous_state_vector().CopyToVector();

      Eigen::VectorXd jointangles(dynamic_model->getnDoF());
      Eigen::VectorXd jointvelocity(dynamic_model->getnDoF());

      for(auto i=0;i<state.size()/2;i++){
          jointangles(i)=drake::ExtractDoubleOrThrow(state(i));
          jointvelocity(i)=drake::ExtractDoubleOrThrow(state(i+state.size()/2));
      }
      Eigen::MatrixXd inertia;
      Eigen::Tensor<double,3> inertiaderi;
      dynamic_model->computeInertiaAndDerivative(jointangles, inertia, inertiaderi);
      Eigen::VectorXd acceleration=dynamic_model->compute_geodesic_acceleration(jointvelocity,inertia,inertiaderi);

      auto tau = get_tau(context);
      VectorX<T> xDt(dynamic_model->getnDoF()*2);
      for(auto i=0;i<dynamic_model->getnDoF();i++){
          xDt(i)=state(i+dynamic_model->getnDoF());
          xDt(i+dynamic_model->getnDoF())=tau(i)+acceleration(i);
      }
      derivatives->SetFromVector(xDt);
  };

  MMM::GeodesicMotionTools::geodesic_ode_dynamic_model* dynamic_model;



};
#endif // BVP_PLANT_H
