/*
 * The following method contains ALTERED code, the original source is cited below
 *
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#pragma once

#include <rbdl/Dynamics.h>
#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"
#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <unsupported/Eigen/CXX11/Tensor>
//#include"Tools.h"
namespace RigidBodyDynamics {

/** \brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * \param model             rigid body model
 * \param Q                 state vector of the model
 * \param H                 a matrix where the inertia result will be stored in
 * \param H_derivative      a matrix where the inertia derivative will be stored in
 * \param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * \note This function only evaluates the entries of H that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling H.setZero().
 */
void CompositeRigidBodyAlgorithmDerivation (Model& model, const Eigen::VectorXd &Q, Eigen::MatrixXd &H, Eigen::Tensor<double, 3> &H_derivative, bool update_kinematics);
Eigen::Tensor<double,3> jcalc_Xj_Deri_Approx(Model& model,const Eigen::VectorXd& Q,unsigned int a);// for calculating derivative of X_J to jointangles
Eigen::Tensor<double,3> jcalc_Xj_Deri(Model& model,const Eigen::VectorXd& Q,unsigned int a);
}
