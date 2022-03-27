#include "RBDLExtension.h"
#include <variant>
#include"Tools.h"
using namespace MMM;
namespace RigidBodyDynamics {
Eigen::Tensor<double,3> jcalc_Xj_Deri(Model& model,const Eigen::VectorXd& Q,unsigned int a){
    Eigen::Tensor<double,3> Xj_deri(6,6,Q.size());
    Xj_deri.setZero();
    double s,c,angle_i;
    angle_i=Q(a-1);
    s=sin(angle_i),c=cos(angle_i);
    Eigen::Vector3d axis(model.mJoints[a].mJointAxes[0][0],model.mJoints[a].mJointAxes[0][1],model.mJoints[a].mJointAxes[0][2]);
    Eigen::Vector3d axis_Tran(model.mJoints[a].mJointAxes[0][3],model.mJoints[a].mJointAxes[0][4],model.mJoints[a].mJointAxes[0][5]);
    if (model.mJoints[a].mDoFCount == 1
          && model.mJoints[a].mJointType != JointTypeCustom){
        if (model.mJoints[a].mJointType == JointTypeRevolute){
            Xj_deri(0,0,a-1)=Xj_deri(3,3,a-1)=pow(axis(0),2)*s-s;
            Xj_deri(0,1,a-1)=Xj_deri(3,4,a-1)=axis(0)*axis(1)*s+axis(2)*c;
            Xj_deri(0,2,a-1)=Xj_deri(3,5,a-1)=axis(0)*axis(2)*s-axis(1)*c;
            Xj_deri(1,0,a-1)=Xj_deri(4,3,a-1)=axis(0)*axis(1)*s-axis(2)*c;
            Xj_deri(1,1,a-1)=Xj_deri(4,4,a-1)=axis(1)*axis(1)*s-s;
            Xj_deri(1,2,a-1)=Xj_deri(4,5,a-1)=axis(1)*axis(2)*s+axis(0)*c;
            Xj_deri(2,0,a-1)=Xj_deri(5,3,a-1)=axis(0)*axis(2)*s+axis(1)*c;
            Xj_deri(2,1,a-1)=Xj_deri(5,4,a-1)=axis(2)*axis(1)*s-axis(0)*c;
            Xj_deri(2,2,a-1)=Xj_deri(5,5,a-1)=axis(2)*axis(2)*s-s;
        }
        else if (model.mJoints[a].mJointType == JointTypePrismatic){
            Xj_deri(0,0,a-1)=Xj_deri(1,1,a-1)=Xj_deri(2,2,a-1)=Xj_deri(3,3,a-1)=Xj_deri(4,4,a-1)=Xj_deri(5,5,a-1)=1;
            Xj_deri(3,1,a-1)=axis_Tran(2);
            Xj_deri(4,0,a-1)=-axis_Tran(2);
            Xj_deri(3,2,a-1)=-axis_Tran(1);
            Xj_deri(5,0,a-1)=axis_Tran(1);
            Xj_deri(4,2,a-1)=axis_Tran(0);
            Xj_deri(5,1,a-1)=-axis_Tran(0);
        }
    }
    return Xj_deri;
}
Eigen::Tensor<double,3> jcalc_Xj_Deri_Approx(Model& model,const Eigen::VectorXd& Q,unsigned int a){
    Eigen::Tensor<double,3> Xj_deri(6,6,Q.size());
    Xj_deri.setZero();
    Eigen::VectorXd jointcopy,jointcopy1,jointcopy2,jointcopy3,jointcopy4,jointcopy5;
    Eigen::MatrixXd matrixcopy,matrixcopy1,matrixcopy2,matrixcopy3,matrixcopy4,matrixcopy5;
    double e=std::numeric_limits<double>::epsilon();
    double theta=std::pow(e/168, (double)1 / (double)7);//derivation of jointangles
    double x;
    for(unsigned int i=0;i<Q.size();i++){
        x=Q[i];
        theta = detail::make_xph_representable(x, theta);// from Boost library
        jointcopy=Q,jointcopy1=Q,jointcopy2=Q,jointcopy3=Q,jointcopy4=Q,jointcopy5=Q;
        jointcopy[i] +=theta,jointcopy1[i] -=theta,jointcopy2[i] +=2*theta,jointcopy3[i] -=2*theta,jointcopy4[i] +=3*theta,jointcopy5[i] -=3*theta;
        matrixcopy=jcalc_XJ(model,a,jointcopy).toMatrix(),matrixcopy1=jcalc_XJ(model,a,jointcopy1).toMatrix(),matrixcopy2=jcalc_XJ(model,a,jointcopy2).toMatrix(),
               matrixcopy3=jcalc_XJ(model,a,jointcopy3).toMatrix(), matrixcopy4=jcalc_XJ(model,a,jointcopy4).toMatrix(),matrixcopy5=jcalc_XJ(model,a,jointcopy5).toMatrix();
        for(auto j=0;j<6;j++){
            for(auto k=0;k<6;k++){
                Xj_deri(j,k,i)=(matrixcopy4(j,k)-matrixcopy5(j,k)+9*(matrixcopy3(j,k)-matrixcopy2(j,k))+45*(matrixcopy(j,k)-matrixcopy1(j,k)))/(60*theta);
            }
        }
    }
    return Xj_deri;

}
void CompositeRigidBodyAlgorithmDerivation(Model &model, const Eigen::VectorXd &Q, Eigen::MatrixXd &H, Eigen::Tensor<double, 3> &H_derivative, bool update_kinematics) {

    assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

    std::vector<Eigen::Tensor<double,3>> deltaIc(model.mBodies.size()-2);
    std::vector<Eigen::Tensor<double,3>> xjq(model.mBodies.size()-2);
    Eigen::Tensor<double,3> X_j_qTran(6,6,Q.size());
    Eigen::Tensor<double,3> X_j_q(6,6,Q.size());
    Eigen::array<long,3> shuffling{1,0,2};
    X_j_qTran.setZero();
    X_j_q.setZero();
    GeodesicMotionTools GeodesicMotion;
    Eigen::Tensor<double,3> crossT(1,1,Q.size());
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
        if (update_kinematics) {
            jcalc_X_lambda_S (model, i, Q);
            model.X_J[i]=jcalc_XJ(model,i,Q);
        }
        model.Ic[i] = model.I[i];
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
        unsigned int dof_index_i = model.mJoints[i].q_index;
        if (model.lambda[i] != 0) {
            model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.Ic[i]);
            //calculate X_J derivative using finite diffenrencial method
            X_j_q=jcalc_Xj_Deri(model,Q,i);
            xjq[dof_index_i-1]=X_j_q;
            X_j_qTran=jcalc_Xj_Deri(model,Q,i).shuffle(shuffling);
            if(i==model.mBodies.size()-1){
                deltaIc[dof_index_i-1]=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(X_j_qTran,model.X_T[i].toMatrix().transpose(),1),(model.Ic[i].toMatrix()*model.X_J[i].toMatrix()*model.X_T[i].toMatrix()).transpose(),2)
                        +GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(X_j_q,model.X_T[i].toMatrix().transpose()*model.X_J[i].toMatrix().transpose()*model.Ic[i].toMatrix(),1),model.X_T[i].toMatrix().transpose(),2);
                for(auto k=0;k<Q.size();k++){
                    H_derivative(dof_index_i,dof_index_i,k)=0;
                }
            }
            else {
                deltaIc[dof_index_i-1]=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(X_j_qTran,model.X_T[i].toMatrix().transpose(),1),(model.Ic[i].toMatrix()*model.X_J[i].toMatrix()*model.X_T[i].toMatrix()).transpose(),2)
                        +GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(X_j_q,model.X_T[i].toMatrix().transpose()*model.X_J[i].toMatrix().transpose()*model.Ic[i].toMatrix(),1),model.X_T[i].toMatrix().transpose(),2)
                        +GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(deltaIc[dof_index_i],model.X_T[i].toMatrix().transpose()*model.X_J[i].toMatrix().transpose(),1)
                                                 ,model.X_T[i].toMatrix().transpose()*model.X_J[i].toMatrix().transpose(),2);

            }
        }
        /*if(dof_index_i==0){
            crossT=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(deltaIc[0],model.S[1].transpose(),1),model.S[1].transpose(),2);
            for(auto k=0;k<Q.size();k++){
                H_derivative(0,0,k)=crossT(0,0,k);
            }
        }*/

      if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom) {
            Math::SpatialVector F = model.Ic[i] * model.S[i];
            H(dof_index_i, dof_index_i) = model.S[i].dot(F);

            unsigned int j = i;
            unsigned int dof_index_j = dof_index_i;

            while (model.lambda[j] != 0) {
                F = model.X_lambda[j].applyTranspose(F);
                j = model.lambda[j];
                dof_index_j = model.mJoints[j].q_index;

                if (model.mJoints[j].mJointType != JointTypeCustom) {
                    if (model.mJoints[j].mDoFCount == 1) {
                        H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
                        H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);

                        // ...

                    } else if (model.mJoints[j].mDoFCount == 3) {
                        throw std::runtime_error("MultiDoF Derivative not yet supported");

                        //                        Eigen::Vector3d H_temp2 =
                        //                          (F.transpose() * model.multdof3_S[j]).transpose();
                        //                        LOG << F.transpose() << std::endl
                        //                          << model.multdof3_S[j] << std::endl;
                        //                        LOG << H_temp2.transpose() << std::endl;

                        //                        H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
                        //                        H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;

                    }
                } else if (model.mJoints[j].mJointType == JointTypeCustom){
                    throw std::runtime_error("Custom joint type not yet supported");

                    //                    unsigned int k      = model.mJoints[j].custom_joint_index;
                    //                    unsigned int dof    = model.mCustomJoints[k]->mDoFCount;
                    //                    Eigen::VectorXd H_temp2    =
                    //                    (F.transpose() * model.mCustomJoints[k]->S).transpose();

                    //                    LOG << F.transpose() << std::endl << model.mCustomJoints[j]->S << std::endl;

                    //                    LOG << H_temp2.transpose() << std::endl;

                    //                    H.block(dof_index_i,dof_index_j,1,dof) = H_temp2.transpose();
                    //                    H.block(dof_index_j,dof_index_i,dof,1) = H_temp2;

                }
            }
        } else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom) {
            throw std::runtime_error("MultiDoF Derivative not yet supported");

            //            Eigen::Matrix<double, 6, 3> F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
            //            H.block<3,3>(dof_index_i, dof_index_i) = model.multdof3_S[i].transpose() * F_63;

            //            unsigned int j = i;
            //            unsigned int dof_index_j = dof_index_i;

            //            while (model.lambda[j] != 0) {
            //            F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
            //            j = model.lambda[j];
            //            dof_index_j = model.mJoints[j].q_index;

            //            if(model.mJoints[j].mJointType != JointTypeCustom){
            //              if (model.mJoints[j].mDoFCount == 1) {
            //                Eigen::Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

            //                H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
            //                H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
            //              } else if (model.mJoints[j].mDoFCount == 3) {
            //                Eigen::Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

            //                H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
            //                H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
            //              }
            //            } else if (model.mJoints[j].mJointType == JointTypeCustom){
            //              unsigned int k = model.mJoints[j].custom_joint_index;
            //              unsigned int dof = model.mCustomJoints[k]->mDoFCount;

            //              Eigen::MatrixXd H_temp2 = F_63.transpose() * (model.mCustomJoints[k]->S);

            //              H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
            //              H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
            //            }
            //            }

        } else if (model.mJoints[i].mJointType == JointTypeCustom) {
            throw std::runtime_error("Custom joint type not yet supported");

            //            unsigned int kI = model.mJoints[i].custom_joint_index;
            //            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            //            Eigen::MatrixXd F_Nd = model.Ic[i].toMatrix()
            //            * model.mCustomJoints[kI]->S;

            //            H.block(dof_index_i, dof_index_i,dofI,dofI)
            //            = model.mCustomJoints[kI]->S.transpose() * F_Nd;

            //            unsigned int j = i;
            //            unsigned int dof_index_j = dof_index_i;

            //            while (model.lambda[j] != 0) {
            //            F_Nd = model.X_lambda[j].toMatrixTranspose() * (F_Nd);
            //            j = model.lambda[j];
            //            dof_index_j = model.mJoints[j].q_index;

            //            if(model.mJoints[j].mJointType != JointTypeCustom){
            //              if (model.mJoints[j].mDoFCount == 1) {
            //                Eigen::MatrixXd H_temp2 = F_Nd.transpose() * (model.S[j]);
            //                H.block(   dof_index_i,  dof_index_j,
            //                    H_temp2.rows(),H_temp2.cols()) = H_temp2;
            //                H.block(dof_index_j,dof_index_i,
            //                    H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
            //              } else if (model.mJoints[j].mDoFCount == 3) {
            //                Eigen::MatrixXd H_temp2 = F_Nd.transpose() * (model.multdof3_S[j]);
            //                H.block(dof_index_i,   dof_index_j,
            //                    H_temp2.rows(),H_temp2.cols()) = H_temp2;
            //                H.block(dof_index_j,   dof_index_i,
            //                    H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
            //              }
            //            } else if (model.mJoints[j].mJointType == JointTypeCustom){
            //              unsigned int k   = model.mJoints[j].custom_joint_index;
            //              unsigned int dof = model.mCustomJoints[k]->mDoFCount;

            //              Eigen::MatrixXd H_temp2 = F_Nd.transpose() * (model.mCustomJoints[k]->S);

            //              H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
            //              H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
            //            }
            //            }
            //            }
        }
    }  //for-loop
    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--){
        unsigned int dof_index_i = model.mJoints[i].q_index;
        unsigned int j=i,k,dof_index_k;
               if(i==model.mBodies.size()-1){

                   while(model.lambda[j]!=0){

                       k=model.lambda[j];
                       dof_index_k=model.mJoints[k].q_index;
                       crossT=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(xjq[j-2],model.S[i].transpose()*model.Ic[i].toMatrix(),1),model.S[k].transpose()*model.X_T[j].toMatrix().transpose(),2);
                       for(auto t=0;t<Q.size();t++){

                           H_derivative(dof_index_i,dof_index_k,t)=crossT(0,0,t);
                           H_derivative(dof_index_k,dof_index_i,t)=crossT(0,0,t);
                       }
                       j=k;
                   }
               }
               else {
                   crossT=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(deltaIc[dof_index_i],model.S[i].transpose(),1),model.S[i].transpose(),2);
                   for(auto k=0;k<Q.size();k++){
                       H_derivative(dof_index_i,dof_index_i,k)=crossT(0,0,k);
                   }
                  while(model.lambda[j]!=0){
                       k=model.lambda[j];
                       dof_index_k=model.mJoints[k].q_index;
                       crossT=GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(deltaIc[dof_index_i],model.S[i].transpose(),1),model.S[k].transpose()*model.X_lambda[j].toMatrix().transpose(),2)
                           +GeodesicMotion.tensorMatrixProduct(GeodesicMotion.tensorMatrixProduct(xjq[j-2],model.S[i].transpose()*model.Ic[i].toMatrix(),1),model.S[k].transpose()*model.X_T[j].toMatrix().transpose(),2);
                       for(auto t=0;t<Q.size();t++){
                          H_derivative(dof_index_i,dof_index_k,t)=crossT(0,0,t);
                          H_derivative(dof_index_k,dof_index_i,t)=crossT(0,0,t);
                       }
                       j=k;
                  }
               }
    }
}  //function

}
