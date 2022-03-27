#ifndef BVP_SOLVER_H
#define BVP_SOLVER_H
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <drake/solvers/solve.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/geometry/drake_visualizer.h"
#include <gflags/gflags.h>
#include <drake/systems/primitives/linear_system.h>
#include "BVP_Plant.h"
MMM::TrajectoryResult solve_geodesic_bvp_A(const Eigen::VectorXd &init_joint_angles,const Eigen::VectorXd& final_joint_angles,VirtualRobot::DynamicsPtr dynamics_model,const double& T,const double& timestep=0.01){
    MMM::TrajectoryResult bvpresult;
    auto n=init_joint_angles.size();
    auto pendulum = std::make_unique<bvpplant<double>>(new MMM::GeodesicMotionTools::geodesic_ode_dynamic_model(dynamics_model));
    auto context = pendulum->CreateDefaultContext();
    const int kNumTimeSamples = T/timestep+1;
    const double kMinimumTimeStep = timestep;
    const double kMaximumTimeStep = timestep;
    drake::systems::trajectory_optimization::DirectCollocation dircol(pendulum.get(),*context, kNumTimeSamples, kMinimumTimeStep,kMaximumTimeStep);
    dircol.AddEqualTimeIntervalsConstraints();
    const drake::solvers::VectorXDecisionVariable& u = dircol.input();
    VectorX<double> initial_state,final_state;
    initial_state.resize(n),final_state.resize(n);
    initial_state.setZero(),final_state.setZero();
    for(auto i=0;i<init_joint_angles.size();i++){
        initial_state(i)=init_joint_angles(i);
        final_state(i)=final_joint_angles(i);
    }
    drake::solvers::VectorXDecisionVariable dini,dfina;
    dini.resize(n,1),dfina.resize(n,1);
    for(auto i=0;i<n;i++){
        dini(i)=dircol.initial_state()(i);
        dfina(i)=dircol.final_state()(i);
    }
    dircol.AddLinearConstraint(dini ==initial_state);
    dircol.AddLinearConstraint(dfina == final_state);
    dircol.AddRunningCost(u.transpose() * u);
    const double timespan_init = T;
    std::vector<Eigen::MatrixXd> v;
    v.push_back(initial_state);
    v.push_back(final_state);
    auto traj_init_x = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold({0, timespan_init}, v);
    dircol.SetInitialTrajectory(drake::trajectories::PiecewisePolynomial<double>(), traj_init_x);
    const auto resultw = drake::solvers::Solve(dircol);
    //const drake::trajectories::PiecewisePolynomial<double> pp_traj =dircol.ReconstructInputTrajectory(resultw);
    const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =dircol.ReconstructStateTrajectory(resultw);
    bvpresult.time.resize(T/timestep+1),bvpresult.joint_angles.resize(T/timestep+1,n),bvpresult.joint_velocities.resize(T/timestep+1,n);
    for(auto i=0;i<=T/timestep;i++){
        bvpresult.time[i]=(double)i*timestep;
        for(auto j=0;j<n;j++){
            bvpresult.joint_angles(i,j)=pp_xtraj.value((double)i*timestep)(j);
            bvpresult.joint_velocities(i,j)=pp_xtraj.value((double)i*timestep)(n+j);
        }
    }
    return bvpresult;
}
MMM::TrajectoryResult euclidean_motion_solver(const Eigen::VectorXd &init_joint_angles,const Eigen::VectorXd& final_joint_angles,const double& T){
    MMM::TrajectoryResult euresult;
    auto n=init_joint_angles.size();
    euresult.time.resize(T*100+1),euresult.joint_angles.resize(T*100+1,n),euresult.joint_velocities.resize(T*100+1,n);
    for(auto i=0;i<=T*100;i++){
        euresult.time[i]=(double)i/100;
        for(auto j=0;j<n;j++){
            euresult.joint_angles(i,j)=init_joint_angles(j)+((double)i/100)*((final_joint_angles(j)-init_joint_angles(j))/T);
            euresult.joint_velocities(i,j)=(final_joint_angles(j)-init_joint_angles(j))/T;
        }
    }
    return euresult;
}
#endif // BVP_SOLVER_H
