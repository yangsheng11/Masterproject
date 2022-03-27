#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include <VirtualRobot/Dynamics/Dynamics.h>
#include <functional>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include "RBDLExtension.h"
#include "finite_difference.hpp"
//#include<boost/math/differentiation/finite_difference.hpp>
#include <MMM/Motion/Motion.h>
#include <SimoxUtility/math/convert.h>
using namespace  boost::math::differentiation;
// Eigen >= 3.3 is broken with Boost < 1.71
#if EIGEN_VERSION_AT_LEAST(3,3,0)
namespace Eigen {
namespace internal{

template<typename Scalar>
struct scalar_add_op {
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_DEVICE_FUNC inline scalar_add_op(const scalar_add_op& other) : m_other(other.m_other) { }
  EIGEN_DEVICE_FUNC inline scalar_add_op(const Scalar& other) : m_other(other) { }
  EIGEN_DEVICE_FUNC inline Scalar operator() (const Scalar& a) const { return a + m_other; }
  template <typename Packet>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return internal::padd(a, pset1<Packet>(m_other)); }
  const Scalar m_other;
};
template<typename Scalar>
struct functor_traits<scalar_add_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = packet_traits<Scalar>::HasAdd }; };

} // namespace internal
}

#endif // EIGEN_VERSION_AT_LEAST(3,3,0)

#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> state_type;

namespace boost {
namespace numeric {
namespace odeint {
    template<typename B,int S1,int S2,int O, int M1, int M2>
    struct algebra_dispatcher< Eigen::Matrix<B,S1,S2,O,M1,M2> >
    {
        typedef vector_space_algebra algebra_type;
    };

    template<>
     struct vector_space_norm_inf<state_type> {
        typedef double result_type;
        result_type operator()(const state_type& p) const
        {
          return p.lpNorm<Eigen::Infinity>();
        }
    };
}}}

namespace Eigen
{
    template<typename D, int Rows, int Cols>
    Matrix<D, Rows, Cols> operator/(const Matrix<D, Rows, Cols>& v1, const Matrix<D, Rows, Cols>& v2)
    {
        return v1.array()/v2.array();
    }

    template<typename D, int Rows, int Cols>
    Matrix<D, Rows, Cols>
    abs(Matrix<D, Rows, Cols> const& m) {
        return m.cwiseAbs();
    }
}

namespace MMM
{

// this project can be used for you class structure

struct TrajectoryResult {
    Eigen::VectorXd time;
    Eigen::MatrixXd joint_angles;
    Eigen::MatrixXd joint_velocities;
};
float euclidean_distance_help(const Eigen::VectorXd &jointangles, const Eigen::VectorXd &jointangles1);
float HDPI_calculator(TrajectoryResult result1,TrajectoryResult result2);
float HDPI_calculator_quatenion(TrajectoryResult result1,TrajectoryResult result2);
Eigen::VectorXd FPDI_calculator(TrajectoryResult result1,TrajectoryResult result2,const Eigen::VectorXd& max);
float quatenion_distance_help(const Eigen::VectorXd &jointangles, const Eigen::VectorXd &jointangles1);
TrajectoryResult get_TCP_trajectory(TrajectoryResult result,VirtualRobot::RobotNodeSetPtr rns);
TrajectoryResult get_qua_trajectory(TrajectoryResult result,VirtualRobot::RobotNodeSetPtr rns);
class GeodesicMotionTools
{

public:
    class geodesic_ode {
    public:
        virtual Eigen::MatrixXd computeInertia(const Eigen::VectorXd &joint_angles) = 0;
        virtual Eigen::Tensor<double, 3> computeInertiaDerivative(const Eigen::VectorXd &joint_angles) = 0;
        virtual int getnDoF() = 0;

        virtual void computeInertiaAndDerivative(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &inertia, Eigen::Tensor<double, 3> &derivative) {
            inertia = computeInertia(joint_angles);
            derivative = computeInertiaDerivative(joint_angles);
        }

        void operator() ( const state_type &x, state_type &dxdt , const double /* t */ )
        {
            Eigen::VectorXd jointAngles = x.row(0);
            Eigen::VectorXd jointVelocities = x.row(1);
            Eigen::MatrixXd inertiaMatrix;
            Eigen::Tensor<double, 3> inertiaDerivative;
            computeInertiaAndDerivative(jointAngles, inertiaMatrix, inertiaDerivative);

            dxdt = Eigen::MatrixXd(2, jointAngles.rows());
            dxdt.row(0) = jointVelocities;
            Eigen::VectorXd jointAcceleration = compute_geodesic_acceleration(jointVelocities, inertiaMatrix, inertiaDerivative);
            dxdt.row(1) = jointAcceleration;
        }

        Eigen::VectorXd compute_geodesic_acceleration(const Eigen::VectorXd &joint_velocities, const Eigen::MatrixXd &inertiaMatrix, const Eigen::Tensor<double, 3> &inertiaMatrixDerivative) {
            Eigen::VectorXd acceleration(joint_velocities.rows());
            //calculate acceleration in ODE formular
            GeodesicMotionTools GeodesicMotion;
            //Eigen::array<long,3> shuffling({1,0,2});

            Eigen::VectorXd c_wert1(joint_velocities.rows()),c_wert2(joint_velocities.rows());
            Eigen::MatrixXd vel_matrix(1,joint_velocities.rows());
            for(auto i=0;i<joint_velocities.size();i++){
                vel_matrix(0,i)=joint_velocities[i];
            }
            Eigen::Tensor<double,3> tensor1=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(inertiaMatrixDerivative,vel_matrix,2),vel_matrix,3);
            for(auto i=0;i<tensor1.dimension(0);i++){
                c_wert1[i]=tensor1(i,0,0);
            }
            Eigen::Tensor<double,3> tesor1=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(inertiaMatrixDerivative,vel_matrix,1),vel_matrix,2);
            for(auto i=0;i<tesor1.dimension(2);i++){
                c_wert2[i]=tesor1(0,0,i)/2;
            }
            acceleration=-inertiaMatrix.inverse()*(c_wert1-c_wert2);

            return acceleration;
        }
        Eigen::VectorXd compute_jointangles_curvelenth(const TrajectoryResult &result,float &s){
            Eigen::VectorXd jointangles;
            jointangles.setZero();
            size_t i_i=0,i_f=0;
            for(auto i=0;i<result.time.size();i++){
                if(s==result.time[i]/result.time[result.time.size()-1]){
                    i_i=i;
                    jointangles=result.joint_angles.row(i_i);
                }
                else if(s>result.time[i]/result.time[result.time.size()-1] && s<result.time[i+1]/result.time[result.time.size()-1]){
                    i_i=i,i_f=i+1;
                    jointangles=result.joint_angles.row(i_i)+(result.joint_angles.row(i_f)-result.joint_angles.row(i_i))*((s-result.time[i_i]/result.time[result.time.size()-1])/0.01);
                    break;
                }
            }
            return jointangles;
        }
        float compute_curvelenth_time(const double& T,const double& initial_metricspeed, const double& final_metricspeed, double t){
            float s=0;
            double c1,c2,c3,c4=0;
            c3=initial_metricspeed;
            c2=(6-4*initial_metricspeed*T-2*final_metricspeed*T)/(T*T);
            c1=(12-6*initial_metricspeed*T-6*final_metricspeed*T)/(T*T*T);
            s=-c1*pow(t,3)/6+c2*pow(t,2)/2+c3*t+c4;
            return s;
        }
        Eigen::Tensor<double,4> compute_inertia_deri_second(const Eigen::VectorXd &jointangles, const Eigen::Tensor<double,3> &inertiaderivative){
            Eigen::Tensor<double,4> inertia_second(inertiaderivative.dimension(0),inertiaderivative.dimension(1),inertiaderivative.dimension(2),jointangles.size());
            inertia_second.setZero();
            Eigen::VectorXd jointcopy;
            Eigen::Tensor<double,3> derivativecopy;
            double e=std::numeric_limits<double>::epsilon();
            double theta=2 * sqrt(e),x;
            for(auto i=0;i<jointangles.size();i++){
                jointcopy=jointangles;
                x=jointangles[i];
                theta = detail::make_xph_representable(x, theta);
                jointcopy[i] +=theta;
                derivativecopy= computeInertiaDerivative(jointcopy);
                for(auto j=0;j<inertiaderivative.dimension(0);j++){
                    for(auto k=0;k<inertiaderivative.dimension(1);k++){
                        for(auto l=0;l<inertiaderivative.dimension(2);l++){
                            inertia_second(j,k,l,i)=(derivativecopy(j,k,l)-inertiaderivative(j,k,l))/theta;
                        }
                    }
                }
            }
            return inertia_second;
        }
        Eigen::MatrixXd compute_f2_angles_deri(const Eigen::VectorXd &jointangles, const Eigen::VectorXd & jointvelocity){
            Eigen::MatrixXd f2_q,inertiamatrix,a1(jointangles.size(),1),c1(jointangles.size(),jointangles.size()),c2(jointangles.size(),jointangles.size());
            inertiamatrix=computeInertia(jointangles),a1.setZero(),c1.setZero(),c2.setZero();
            Eigen::Tensor<double,3> inertiaderivative=computeInertiaDerivative(jointangles),inertiaderivative_inverse,tensor1,tensor2,tensor3;
            GeodesicMotionTools Geodesicmotion;
            inertiaderivative_inverse=-Geodesicmotion.tensorMatrixProduct(Geodesicmotion.tensorMatrixProduct(inertiaderivative,inertiamatrix.inverse(),1),inertiamatrix.inverse().transpose(),2);
            tensor1=Geodesicmotion.tensorMatrixProduct(Geodesicmotion.tensorMatrixProduct(inertiaderivative,jointvelocity.transpose(),1),jointvelocity.transpose(),2);
            tensor2=Geodesicmotion.tensorMatrixProduct(Geodesicmotion.tensorMatrixProduct(inertiaderivative,jointvelocity.transpose(),2),jointvelocity.transpose(),3);
            for(auto i=0;i<jointangles.size();i++){
                a1(i,0)=tensor1(0,0,i)/2-tensor2(i,0,0);
            }
            tensor3=Geodesicmotion.tensorMatrixProduct(inertiaderivative_inverse,a1.transpose(),2);
            for(auto i=0;i<jointangles.size();i++){
                for(auto j=0;j<jointangles.size();j++){
                    c1(i,j)=tensor3(i,0,j);
                }
            }
            Eigen::Tensor<double,4> inertia_second,tensor4,tensor5;
            inertia_second=compute_inertia_deri_second(jointangles,inertiaderivative);
            tensor4=Geodesicmotion.tensorMatrixProduct_vier(Geodesicmotion.tensorMatrixProduct_vier(inertia_second,jointvelocity.transpose(),1),jointvelocity.transpose(),2);
            tensor5=Geodesicmotion.tensorMatrixProduct_vier(Geodesicmotion.tensorMatrixProduct_vier(inertia_second,jointvelocity.transpose(),2),jointvelocity.transpose(),3);
            for(auto i=0;i<jointangles.size();i++){
                for(auto j=0;j<jointangles.size();j++){
                    c2(i,j)=tensor4(0,0,i,j)/2-tensor5(i,0,0,j);
                }
            }
            f2_q=c1+inertiamatrix.inverse()*c2;
            return f2_q;
        }
        Eigen::MatrixXd compute_f2_q_finite(const Eigen::VectorXd &jointangles,const Eigen::VectorXd &jointvelocity){
            Eigen::MatrixXd f2_q(jointangles.size(),jointangles.size()),inertia,inertiacopy;
            Eigen::Tensor<double,3> inertiaderi,inertiadericopy;
            Eigen::VectorXd jointcopy,f2copy,f2ini;
            f2_q.setZero();
            inertia=computeInertia(jointangles);
            inertiaderi=computeInertiaDerivative(jointangles);
            f2ini=compute_geodesic_acceleration(jointvelocity,inertia,inertiaderi);
            double e=std::numeric_limits<double>::epsilon();
            double theta=2 * sqrt(e),x;
            for(auto i=0;i<jointangles.size();i++){
                jointcopy=jointangles;
                x=jointangles[i];
                theta = detail::make_xph_representable(x, theta);
                jointcopy[i] +=theta;
                inertiacopy=computeInertia(jointcopy);
                inertiadericopy=computeInertiaDerivative(jointcopy);
                f2copy=compute_geodesic_acceleration(jointvelocity,inertiacopy,inertiadericopy);
                for(auto j=0;j<jointangles.size();j++){
                    f2_q(j,i)=(f2copy(j)-f2ini(j))/theta;
                }
            }
            return f2_q;
        }
        Eigen::MatrixXd compute_f2_velocity_finite(const Eigen::VectorXd &jointangles,const Eigen::VectorXd &jointvelocity){
            Eigen::MatrixXd f2_qdot(jointangles.size(),jointangles.size()),inertia;
            Eigen::Tensor<double,3> inertiaderi;
            Eigen::VectorXd jointcopy,f2copy,f2ini;
            f2_qdot.setZero();
            inertia=computeInertia(jointangles);
            inertiaderi=computeInertiaDerivative(jointangles);
            f2ini=compute_geodesic_acceleration(jointvelocity,inertia,inertiaderi);
            double e=std::numeric_limits<double>::epsilon();
            double theta=2 * sqrt(e),x;
            for(auto i=0;i<jointvelocity.size();i++){
                jointcopy=jointvelocity;
                x=jointvelocity[i];
                theta = detail::make_xph_representable(x, theta);
                jointcopy[i] +=theta;
                f2copy=compute_geodesic_acceleration(jointcopy,inertia,inertiaderi);
                for(auto j=0;j<jointvelocity.size();j++){
                    f2_qdot(j,i)=(f2copy(j)-f2ini(j))/theta;
                }
            }
            return f2_qdot;
        }

    };

    class geodesic_ode_dynamic_model : public geodesic_ode {
    public:
        enum DerivationMethod { FirstOrderApprox, NOrderApprox, CompositeRigidBodyAlgorithmDerivation };

        geodesic_ode_dynamic_model(VirtualRobot::DynamicsPtr dynamics_model, DerivationMethod method = CompositeRigidBodyAlgorithmDerivation, bool updateKinematics = true)
            : dynamics_model(dynamics_model), method(method), updateKinematics(updateKinematics)
        {
        }

        Eigen::MatrixXd computeInertia(const Eigen::VectorXd &joint_angles) override {
            return dynamics_model->getInertiaMatrix(joint_angles, updateKinematics);
        }

        Eigen::Tensor<double, 3> computeInertiaDerivative(const Eigen::VectorXd &joint_angles) override {
            return computeInertiaDerivative(joint_angles, method);
        }

        Eigen::Tensor<double, 3> computeInertiaDerivative(const Eigen::VectorXd &joint_angles, DerivationMethod method) {
            Eigen::MatrixXd inertiaMatrix;
            Eigen::Tensor<double, 3> inertiaDerivative;
            computeInertiaAndDerivative(joint_angles, inertiaMatrix, inertiaDerivative, method);
            return inertiaDerivative;
        }

        void computeInertiaAndDerivative(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &inertiaMatrix, Eigen::Tensor<double, 3> &inertiaDerivative) override {
            computeInertiaAndDerivative(joint_angles, inertiaMatrix, inertiaDerivative, method);
        }
        void finite_approx(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &inertiaMatrix, Eigen::Tensor<double, 3> &inertiaDerivative,DerivationMethod method){
            switch (method) {
            case FirstOrderApprox:{
                Eigen::VectorXd jointcopy;
                Eigen::MatrixXd matrixcopy;
                double e=std::numeric_limits<double>::epsilon();
                double theta=2 * sqrt(e),x;
                for(auto i=0;i<joint_angles.size();i++){
                    jointcopy=joint_angles;
                    x=joint_angles[i];
                    theta = detail::make_xph_representable(x, theta);
                    jointcopy[i] +=theta;
                    matrixcopy= computeInertia(jointcopy);
                    for(auto j=0;j<inertiaMatrix.rows();j++){
                        for(auto k=0;k<inertiaMatrix.cols();k++){
                            inertiaDerivative(j,k,i)=(matrixcopy(j,k)-inertiaMatrix(j,k))/theta;
                        }
                    }
                }
                break;
            }
            case NOrderApprox:{
                Eigen::VectorXd jointcopy,jointcopy1,jointcopy2,jointcopy3,jointcopy4,jointcopy5;
                Eigen::MatrixXd matrixcopy,matrixcopy1,matrixcopy2,matrixcopy3,matrixcopy4,matrixcopy5;
                double e=std::numeric_limits<double>::epsilon();
                double theta=std::pow(e/168, (double)1 / (double)7);
                double x;
                for(auto i=0;i<joint_angles.size();i++){
                    x=joint_angles[i];
                    theta = detail::make_xph_representable(x, theta);
                    jointcopy=joint_angles,jointcopy1=joint_angles,jointcopy2=joint_angles,jointcopy3=joint_angles,jointcopy4=joint_angles,jointcopy5=joint_angles;
                    jointcopy[i] +=theta,jointcopy1[i] -=theta,jointcopy2[i] +=2*theta,jointcopy3[i] -=2*theta,jointcopy4[i] +=3*theta,jointcopy5[i] -=3*theta;
                    matrixcopy= computeInertia(jointcopy),matrixcopy1= computeInertia(jointcopy1),matrixcopy2= computeInertia(jointcopy2),
                            matrixcopy3= computeInertia(jointcopy3),matrixcopy4= computeInertia(jointcopy4),matrixcopy5= computeInertia(jointcopy5);
                    for(auto j=0;j<inertiaMatrix.rows();j++){
                        for(auto k=0;k<inertiaMatrix.cols();k++){
                            inertiaDerivative(j,k,i)=(matrixcopy4(j,k)-matrixcopy5(j,k)+9*(matrixcopy3(j,k)-matrixcopy2(j,k))+45*(matrixcopy(j,k)-matrixcopy1(j,k)))/(60*theta);
                        }
                    }
                }
                break;
            }
            default:
                break;
            }

        }

        void computeInertiaAndDerivative(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &inertiaMatrix, Eigen::Tensor<double, 3> &inertiaDerivative, DerivationMethod method) {
            int nDoF = getnDoF();
            inertiaDerivative = Eigen::Tensor<double, 3>(nDoF, nDoF, nDoF);
            inertiaDerivative.setZero();

            switch (method) {
            case FirstOrderApprox: {
                if (inertiaMatrix.rows() == 0 || inertiaMatrix.cols() == 0)
                    inertiaMatrix = computeInertia(joint_angles);
                //should use Boost library
                //idea comes from finite differencial equation in Boost library, should add some headers.
                finite_approx(joint_angles,inertiaMatrix,inertiaDerivative,method);
                break;
            }
            case NOrderApprox: {
                if (inertiaMatrix.rows() == 0 || inertiaMatrix.cols() == 0)
                    inertiaMatrix = computeInertia(joint_angles);

                //is 6th order approx,comes from finite diffenrencial function in Boost
                finite_approx(joint_angles,inertiaMatrix,inertiaDerivative,method);
                break;
            }
            case CompositeRigidBodyAlgorithmDerivation: {
                auto model = dynamics_model->getModel();
                // currently resets inertia matrix
                inertiaMatrix = Eigen::MatrixXd(nDoF, nDoF);
                // compute inertia and derivative simultaneously
                RigidBodyDynamics::CompositeRigidBodyAlgorithmDerivation(*model.get(), joint_angles, inertiaMatrix, inertiaDerivative, updateKinematics);
                break;
            }
            default:
                break;
            }
        }

        virtual int getnDoF() override {
            return dynamics_model->getnDoF();
        }

    private:
        VirtualRobot::DynamicsPtr dynamics_model;
        DerivationMethod method;
        bool updateKinematics;
    };

    class geodesic_ode_2dof_model : public geodesic_ode {
    private:
        float link_length;
        float link_mass;
        float link_inertia;

    public:
        geodesic_ode_2dof_model(float link_length, float link_mass,float link_inertia) : link_length(link_length), link_mass(link_mass),link_inertia(link_inertia)
        {
        }

        Eigen::MatrixXd computeInertia(const Eigen::VectorXd &joint_angles) override {
            Eigen::Matrix2d inertia;
            inertia(0,0)=2*link_inertia+link_mass*pow(link_length/2,2)+2*link_mass*link_length*(link_length/2)*cos(joint_angles[1])+link_mass*pow(link_length/2,2)+link_mass*pow(link_length,2);
            inertia(1,1)=link_mass*pow(link_length/2,2)+link_inertia;
            inertia(0,1)=inertia(1,0)=link_mass*pow(link_length/2,2)+link_mass*link_length*(link_length/2)*cos(joint_angles[1])+link_inertia;
            return inertia;
        }

        Eigen::Tensor<double, 3> computeInertiaDerivative(const Eigen::VectorXd &joint_angles) override {
            Eigen::Tensor<double, 3> derivative(2,2,2);
            derivative(0,0,0)=derivative(1,0,0)=derivative(0,1,0)=derivative(1,1,0)=derivative(1,1,1)=0;
            derivative(0,0,1)=-2*link_mass*link_length*(link_length/2)*sin(joint_angles[1]);
            derivative(1,0,1)=derivative(0,1,1)=-link_mass*link_length*(link_length/2)*sin(joint_angles[1]);
            return derivative;
        }

        virtual int getnDoF() override {
            return 2;
        }
    };

    struct push_back_state_and_time
    {
        std::vector< state_type >& m_states;
        std::vector< double >& m_times;

        push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
        : m_states( states ), m_times( times ) { }

        void operator()(const state_type &x , double t )
        {
            m_states.push_back( x );
            m_times.push_back( t );
        }
    };
    struct cauchy_omica
    {
        geodesic_ode& mygeodesic;
        std::vector<state_type> myx_vec;
        cauchy_omica(geodesic_ode &geodesic,std::vector<state_type> &x_vec):mygeodesic(geodesic),myx_vec(x_vec){}
        void operator()(const state_type &x,state_type &dxdt,const double &t){
            Eigen::VectorXd omica_state=x.row(0),omica_change=x.row(1);
            dxdt = Eigen::MatrixXd(2, omica_state.rows());
            dxdt.row(0)=x.row(1);
            dxdt.row(1)=mygeodesic.compute_f2_q_finite(myx_vec[t/0.01].row(0),myx_vec[t/0.01].row(1))*omica_state+mygeodesic.compute_f2_velocity_finite(myx_vec[t/0.01].row(0),myx_vec[t/0.01].row(1))*omica_change;
        }
    };
   static float euclidean_distance(const Eigen::VectorXd &jointangles, const Eigen::VectorXd &jointangles1){
        float a,sum=0;
        for(auto i=0;i<jointangles.size();i++){
            sum+=pow(jointangles(i)-jointangles1(i),2);
        }
        a=pow(sum,0.5);
        return a;
    }
    template< typename T, typename std::enable_if<std::is_base_of<geodesic_ode, T>::value>::type* = nullptr>
    static TrajectoryResult solve_geodesic_ivp(const Eigen::VectorXd &init_joint_angles, const Eigen::VectorXd &init_joint_velocities, const T &ode, double dt = 0.01, double start_time = 0, double end_time = 1.0, bool logging = false) {
        Eigen::MatrixXd x(2, init_joint_angles.rows());
        x.row(0) = init_joint_angles;
        x.row(1) = init_joint_velocities;
        TrajectoryResult result;
        std::vector<state_type> x_vec;
        std::vector<double> times;
        /*typedef boost::numeric::odeint::runge_kutta_dopri5<state_type,double,state_type,double,boost::numeric::odeint::vector_space_algebra> error_stepper_type;
        typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
        controlled_stepper_type controlled_stepper;
        size_t steps = boost::numeric::odeint::integrate_adaptive(controlled_stepper, ode, x, start_time, end_time, dt, push_back_state_and_time(x_vec, times) );*/
        using namespace boost::numeric::odeint;
        runge_kutta4< state_type > stepper;
        size_t steps=integrate_const( stepper , ode , x , start_time,end_time , 0.01, push_back_state_and_time(x_vec, times));
        if (logging)
        {
            for( size_t i=0; i<=steps; i++ )
            {
                std::cout << times[i] << '\t' << x_vec[i].row(0) << '\t' << x_vec[i].row(1) << '\n';
            }
        }
        //std::cout<<times.size()<<" "<<x_vec.size()<<std::endl;
        for(size_t i=0;i<times.size();i++){
            result.time.resize(times.size());
            result.time[i]=times[i];
            //std::cout<<result.time[i]<<std::endl;
        }
        for(size_t i=0;i<x_vec.size();i++){
            result.joint_angles.resize(x_vec.size(),x_vec[0].cols());
            result.joint_velocities.resize(x_vec.size(),x_vec[0].cols());
            result.joint_angles.block(i,0,1,x_vec[i].cols())=x_vec[i].row(0);
            result.joint_velocities.block(i,0,1,x_vec[i].cols())=x_vec[i].row(1);
        }
        return result;
    }
    template<typename T>
    static TrajectoryResult solve_geodesic_bvp(const Eigen::VectorXd &init_joint_angles,const Eigen::VectorXd &final_jointangles,const Eigen::VectorXd &guess_joint_velocities, T &ode, double dt = 0.01, double start_time = 0, double end_time = 1.0){
        float difference;
        size_t count=0;
        Eigen::VectorXd cn;
        cn=guess_joint_velocities;
        std::vector<state_type> x_vec,x_vec1;
        std::vector<double> times,times1;
        Eigen::MatrixXd x(2,init_joint_angles.size());
        Eigen::VectorXd guess_final_jointangles,omicastate,omicavelocity,final_omica,divide;
        TrajectoryResult result;
        using namespace boost::numeric::odeint;
        runge_kutta4<state_type> stepper,stepper1;
        omicastate.resize(init_joint_angles.size()),omicavelocity.resize(init_joint_angles.size());
        omicastate.setZero(),omicavelocity.setOnes();
        Eigen::MatrixXd x1(2,init_joint_angles.size());
        divide.resize(init_joint_angles.size());
        divide.setZero();
        do {
            count+=1;
            x_vec.clear(),x_vec1.clear(),times.clear(),times1.clear();
            x.row(0)=init_joint_angles;
            x.row(1)=cn;
            integrate_const(stepper,ode,x,start_time,end_time,0.01,push_back_state_and_time(x_vec,times));
            guess_final_jointangles=x_vec[x_vec.size()-1].row(0);
            cauchy_omica omica(ode,x_vec);
            x1.row(0)=omicastate;
            x1.row(1)=omicavelocity;
            integrate_const(stepper1,omica,x1,start_time,end_time,0.01,push_back_state_and_time(x_vec1,times1));
            final_omica=x_vec1[x_vec1.size()-1].row(0);
            difference=euclidean_distance(guess_final_jointangles,final_jointangles);
            for(auto i=0;i<init_joint_angles.size();i++){
                divide(i)=(guess_final_jointangles(i)-final_jointangles(i))/final_omica(i);
            }
            cn=cn-divide;
            std::cout<<"this is time i:"<<count<<std::endl;
            std::cout<<"this is difference :"<<difference<<std::endl;
        }while(difference>0.01 && count<10);
        for(size_t i=0;i<times.size();i++){
            result.time.resize(times.size());
            result.time[i]=times[i];
        }
        for(size_t i=0;i<x_vec.size();i++){
            result.joint_angles.resize(x_vec.size(),x_vec[0].cols());
            result.joint_velocities.resize(x_vec.size(),x_vec[0].cols());
            result.joint_angles.block(i,0,1,x_vec[i].cols())=x_vec[i].row(0);
            result.joint_velocities.block(i,0,1,x_vec[i].cols())=x_vec[i].row(1);
        }
        return result;
    }
    template<typename T>
    using  MatrixType = Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic>;

    template<typename Scalar,int rank, typename sizeType>
    auto Tensor_to_Matrix(const Eigen::Tensor<Scalar,rank> &tensor,const sizeType rows,const sizeType cols)
    {
        return Eigen::Map<const MatrixType<Scalar>> (tensor.data(), rows,cols);
    }

    template<typename Scalar, typename... Dims>
    auto Matrix_to_Tensor(const MatrixType<Scalar> &matrix, Dims... dims)
    {
        constexpr int rank = sizeof... (Dims);
        return Eigen::TensorMap<Eigen::Tensor<const Scalar, rank>>(matrix.data(), {dims...});
    }

    Eigen::Tensor<double, 3> tensorMatrixProduct(const Eigen::Tensor<double, 3> &tensor, const Eigen::MatrixXd &matrix, int mode) {
        Eigen::Tensor<double, 3> product_result;
        // includes the tensor vector product
        std::map<int, Eigen::array<long,3>> map1;
        map1[0]={0,1,2};
        map1[1]={1,0,2};
        map1[2]={2,1,0};
        Eigen::array<long,3> shuffling(map1[mode-1]); // transpose Tensor for product
        Eigen::Tensor<double,3> perm_ten=tensor.shuffle(shuffling);
        Eigen::array<long,2> shape={perm_ten.dimension(0),perm_ten.dimension(1)*perm_ten.dimension(2)};
        Eigen::Tensor<double,2> reshape=perm_ten.reshape(shape);
        Eigen::MatrixXd mat_mult=matrix*Tensor_to_Matrix(reshape,reshape.dimension(0),reshape.dimension(1));
        Eigen::array<long,3> shape2 = {matrix.rows(), perm_ten.dimension(1), perm_ten.dimension(2)};
        product_result=Matrix_to_Tensor(mat_mult,mat_mult.rows(),mat_mult.cols()).reshape(shape2).shuffle(map1[mode-1]);// shuffle back
        return product_result;
    }
    Eigen::Tensor<double,4> tensorMatrixProduct_vier(const Eigen::Tensor<double,4> &tensor,const Eigen::MatrixXd &matrix,int mode){
        Eigen::Tensor<double,4> product;
        std::map<int,Eigen::array<long,4>> map1;
        map1[0]={0,1,2,3};
        map1[1]={1,0,2,3};
        map1[2]={2,1,0,3};
        map1[3]={3,1,2,0};
        Eigen::array<long,4> shuffling(map1[mode-1]);
        Eigen::Tensor<double,4> perm_ten=tensor.shuffle(shuffling);
        Eigen::array<long,2> shape={perm_ten.dimension(0),perm_ten.dimension(1)*perm_ten.dimension(2)*perm_ten.dimension(3)};
        Eigen::Tensor<double,2> reshape=perm_ten.reshape(shape);
        Eigen::MatrixXd mat_mult=matrix*Tensor_to_Matrix(reshape,reshape.dimension(0),reshape.dimension(1));
        Eigen::array<long,4> shape2={matrix.rows(),perm_ten.dimension(1),perm_ten.dimension(2),perm_ten.dimension(3)};
        product=Matrix_to_Tensor(mat_mult,mat_mult.rows(),mat_mult.cols()).reshape(shape2).shuffle(map1[mode-1]);
        return product;
    }

};

}
