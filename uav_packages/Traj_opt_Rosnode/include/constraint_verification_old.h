#ifndef CONSTRAINT_VERIFICATION_H
#define CONSTRAINT_VERIFICATION_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <traj_min_jerk.hpp>



namespace CV{

bool is_incorridor(const Eigen::MatrixXd& A,const Eigen::VectorXd& b, const Eigen::VectorXd& Point){
    Eigen::VectorXd Vec;
    Vec = A *Point - b.col(0);
    bool isin;
    bool allNegatives = true;
    for (int i = 0; i < Vec.size(); i++) {
        if (Vec[i] >= 0) {
            allNegatives = false;
            break;
        }
    }
    if (allNegatives) {
      isin = true;
    } else {
      isin = false;
    }
      return isin;
}

Eigen::VectorXi get_position_violation(const int& Eval_nbr,const Eigen::VectorXd& Time_vector,const int& Nbr_of_seg,std::vector<Eigen::MatrixXd*> A, Eigen::MatrixXd b,min_jerk::Trajectory& minJerkTraj){
  Eigen::VectorXi Pos_constraint_violation;
  Pos_constraint_violation.setZero(Nbr_of_seg);
  Eigen::VectorXd Evaluation_points;
  double time_keeper = Time_vector(0);
  
  for(int i = 0; i < Nbr_of_seg; i++){
    if(i == 0){
    Evaluation_points = Eigen::VectorXd::LinSpaced(Eval_nbr, 0, Time_vector(0));    
    }
  else{
    Evaluation_points = Eigen::VectorXd::LinSpaced(Eval_nbr, time_keeper, time_keeper + Time_vector(i));    
    time_keeper += Time_vector(i);
    }
  for(int j = 1; j < Eval_nbr- 1; j++){
    Eigen::Vector3d pos = minJerkTraj.getPos(Evaluation_points(j));
    Eigen::Vector2d P(pos(0), pos(1)); 
    bool Eval = is_incorridor(*A[i],b.col(i),P);
    if(Eval == false){
      Pos_constraint_violation[i] = 1;
      break;
    }
    else{
      continue;
    }
  }

  }
  return Pos_constraint_violation;
}

// Eigen::VectorXd push_front(const Eigen::VectorXd& eigenVec){
// 	eigenVec.conservativeResize(eigenVec.size() + 1);  // Augmentation de la taille du vecteur de 1
// 	eigenVec.tail(eigenVec.size() - 1) = eigenVec.head(eigenVec.size() - 1);  // Décalage des éléments du vecteur d'une position vers la fin
// 	eigenVec(0) = 0;  

//     return eigenVec;
// }


class Constraint_Verification{
  public:
  Eigen::VectorXi Psn_constraint;
  Eigen::VectorXi Vel_constraint;
  Eigen::VectorXi Acc_constraint;
  Eigen::VectorXi New_Index_vec;
  Eigen::VectorXi rho_a_plus;
  Eigen::VectorXi K_plus;
  Eigen::VectorXi rho_v_plus;
  bool constraint_ok;

  Constraint_Verification() {}
  
  void Verify_constraints(double Max_vel, double Max_acc,const int& Eval_nbr,const Eigen::VectorXd& Time_vector,
                        std::vector<Eigen::MatrixXd*> A, Eigen::MatrixXd b,min_jerk::Trajectory& minJerkTraj,const Eigen::VectorXd& init_Psn,const Eigen::VectorXi& Index_vec,
                        const Eigen::VectorXd& K_,const Eigen::VectorXd& rho_v_,const Eigen::VectorXd& rho_a_){

    int N = minJerkTraj.getPieceNum();
    bool Vel =  minJerkTraj.checkMaxVelRate(Max_vel);



    if(Vel == false){
      Vel_constraint = minJerkTraj.get_MaxVel_Violation(Max_vel);
    }
    else{
     Vel_constraint.setZero(N);
    }
    bool Acc = minJerkTraj.checkMaxAccRate(Max_acc);


    if(Acc == false){
     Acc_constraint = minJerkTraj.get_MaxAcc_Violation(Max_acc);
    }
    else{
     Acc_constraint.setZero(N);
    }
    
    bool P1 = is_incorridor(*A[0],b.col(0),init_Psn);

    Psn_constraint = get_position_violation(Eval_nbr,Time_vector,N,A,b,minJerkTraj);
    if(P1 == false){
      Psn_constraint[0] = 0;      
    }
    
    Eigen::VectorXi sum;
    sum.setZero(N);
    int counter = 0; 
    // Somme des vecteurs élément par élément
    for (int i = 0; i < N; i++){
        sum(i) = Psn_constraint[i] + Vel_constraint[i] + Acc_constraint[i];
        if(sum(i) > 0){
        counter = counter +1; 
        } 
    }

    if(counter == 0){
      New_Index_vec = Index_vec;
      rho_a_plus.setZero(Index_vec.size());
      K_plus.setZero(Index_vec.size());
      rho_v_plus.setZero(Index_vec.size());
      constraint_ok = true;


    }
    else{

      constraint_ok = false;
      New_Index_vec.setZero(Index_vec.size() + counter);
      New_Index_vec(0) = Index_vec(0);
      New_Index_vec(1) = Index_vec(1);
      rho_a_plus.setZero(Index_vec.size() + counter-1);
      K_plus.setZero(Index_vec.size() + counter-1);
      rho_v_plus.setZero(Index_vec.size() + counter-1);
      int old_index = 1;
      int new_index = 1;
      K_plus(0) = Psn_constraint(0);
      rho_v_plus(0) = Vel_constraint(0);
      rho_a_plus(0) = Acc_constraint(0);
     

      for(int i = 0;  i < sum.size();i++){
        
        if(sum(i) > 0) { 

          if(i >0){
          K_plus(new_index-1) = Psn_constraint(old_index-2);
          rho_v_plus(new_index-1) = Vel_constraint(old_index-2);
          rho_a_plus(new_index-1) = Acc_constraint(old_index-2);
          K_plus(new_index) = Psn_constraint(old_index-2);
          rho_v_plus(new_index) = Vel_constraint(old_index-2);
          rho_a_plus(new_index) = Acc_constraint(old_index-2);

          }
          int IX_interm =(int) (Index_vec(old_index-1) + Index_vec(old_index))/2;


          New_Index_vec(new_index) = IX_interm;
          New_Index_vec(new_index +1 ) = Index_vec(old_index);
          old_index  = old_index+1;
          new_index = new_index+2;

          K_plus(i) = Psn_constraint(i);
          rho_v_plus(i) = Vel_constraint(i);
          rho_a_plus(i) = Acc_constraint(i);


        }
        else{
          if(i >0){
          K_plus(new_index-1) = Psn_constraint(old_index-2);
          rho_v_plus(new_index-1) = Vel_constraint(old_index-2);
          rho_a_plus(new_index-1) = Acc_constraint(old_index-2);
          }
          New_Index_vec(new_index) = Index_vec(old_index);
          old_index  = old_index+1;
          new_index = new_index+1;
        }
      
      }

    }
                        }
};
}
#endif // CONSTRAINT_VERIFICATION_H
