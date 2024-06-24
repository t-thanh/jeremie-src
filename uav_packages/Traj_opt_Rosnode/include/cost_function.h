#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <traj_min_jerk.hpp>



namespace CF{
std::tuple<Eigen::Matrix2Xd, Eigen::VectorXd> split_vector(const Eigen::VectorXd& x)
{
    int n = x.size() / 3;
    Eigen::MatrixXd matrix(2, n);
    Eigen::VectorXd vector(n + 1);
    // std::cout<<  x.segment(0, n)<<std::endl;
    matrix.row(0)  = x.segment(0, n), 
    matrix.row(1) = x.segment(n, n);
    vector = x.tail(n+1);

    return std::make_tuple(matrix, vector);
}

class Cost_function{
public:
  bool verbose;
  int M;

  Eigen::Matrix3d First_point;
  Eigen::Matrix3d Last_point;

  min_jerk::JerkOpt jerkOpt;
  //min_jerk::Trajectory minJerkTraj;

  double costJ1 = 0;
  double costJ21 = 0;
  double costJ22 = 0;
  double costJ23 = 0;
//  double c = 0; to be removed
  Eigen::MatrixXd Grad_J1;
  Eigen::VectorXd Grad_J21T;
  Eigen::MatrixXd Grad_J22q;
  Eigen::VectorXd Grad_J22T;
  Eigen::MatrixXd Grad_J23q;
  Eigen::VectorXd Grad_J23T;

  Eigen::VectorXd Full_grad;

  Eigen::VectorXd K;// Coef of J1
  double rho_T;// Coef of J21
  Eigen::VectorXd rho_v;// Coef of J22
  Eigen::VectorXd rho_a;// Coef of J23

  /// NEW (for J3)
  Eigen::VectorXd min_time;
  //Eigen::VectorXd K_T;
  double CostJ3;
  Eigen::VectorXd Grad_J3T;


// Default constructor
   Cost_function() {}

// constructor for the cost function class with arguments 
void initialize(const int& m ,const Eigen::Matrix3d& iS,const Eigen::Matrix3d& fS,Eigen::VectorXd min_time_,
        const Eigen::VectorXd& K1,const Eigen::VectorXd& rho_v1,const Eigen::VectorXd& rho_a1,bool verbose_ = false){
    // K and Rho to be added later to be given as such.
    M = m;
    First_point = iS;
    Last_point = fS;
    jerkOpt.reset(iS, fS, M );

    verbose = verbose_;

    Grad_J1.setZero(2, M - 1);
    Grad_J21T.setZero(M);
    Grad_J22q.setZero(2, M - 1);
    Grad_J22T.setZero(M);
    Grad_J23q.setZero(2, M - 1);
    Grad_J23T.setZero(M);
    K = K1;
    rho_v = rho_v1;
    rho_a = rho_a1;
    rho_T = 32;

    Full_grad.setZero(3*M -2);

    min_time.setZero(M);
    min_time = min_time_;
    //K_T.fill(1.0);
    CostJ3 = 0.0;
    Grad_J3T.setZero(M);



  }

double variable_points_update(const Eigen::VectorXd& X,const std::vector<Eigen::MatrixXd*>& A, const Eigen::MatrixXd& b,double vmax,double amax){ 
    // split X to varible points and Time_vector
    // create sample based on first and last point and variable points. 
    // X is as follow x,y,t

    
    std::tuple<Eigen::Matrix2Xd, Eigen::VectorXd> result = split_vector(X);
    Eigen::MatrixXd variable_points = std::get<0>(result);
    Eigen::VectorXd Time_vector = std::get<1>(result);
    Eigen::MatrixXd sample(2,M+1);

    Eigen::MatrixXd VPP(3,M-1);
    VPP.row(0) = variable_points.row(0);
    VPP.row(1) = variable_points.row(1);
    VPP.row(2).setZero();

    sample.block(0, 1, 2, M-1) = variable_points;
    sample.col(0) = First_point.block(0,0,2,1);
    sample.col(M) =Last_point.block(0,0,2,1); 

    // calculate the gradients 
    Full_grad.setZero();

    jerkOpt.generate(VPP, Time_vector);


    double OBJ1 = 0.0;
    OBJ1 = jerkOpt.getObjective();


    Eigen::VectorXd gradt;
    gradt = jerkOpt.getGradT();

    Eigen::Matrix3Xd gradp;
    Eigen::MatrixXd Grad(2,M-1);

    gradp = jerkOpt.getGradInnerP();
    Grad.row(0) = gradp.row(0);
    Grad.row(1) = gradp.row(1);
    //
    Calc_J1(variable_points,A,b);

    Calc_J21(Time_vector);

    Calc_J22(sample,Time_vector,vmax);

    Calc_J23(sample,Time_vector,amax);

    Calc_J3(Time_vector);

    double COST = OBJ1 + costJ1 +  costJ21+costJ22 + costJ23 + CostJ3;
    Eigen::MatrixXd Grad_tot_q = Grad +Grad_J1 + Grad_J22q + Grad_J23q;
    Eigen::VectorXd Grad_tot_t = gradt + Grad_J21T + Grad_J22T +Grad_J23T + Grad_J3T;

    // Eigen::VectorXd Full_grad(3*M -2);
    Full_grad.segment(0, M-1) = Grad_tot_q.row(0);
    Full_grad.segment(M-1, M-1) = Grad_tot_q.row(1);
    Full_grad.tail(M) = Grad_tot_t;
    // std::cout<< Full_grad<< std::endl;

    if(verbose == true){
    std::cout << "cost min jerk ="  <<  OBJ1 << std::endl ; 
      std::cout << " grad q =" <<  Grad << std::endl; 
      std::cout << " grad T =" <<  gradt << std::endl;
      std::cout << "cost J1 =" <<  costJ1 << std::endl;
      std::cout << " Grad_J1  =" <<  Grad_J1 << std::endl; 
      std::cout << "cost J21 =" <<  costJ21 << std::endl;
      std::cout << "  Grad_J21T =" <<  Grad_J21T << std::endl;
      std::cout << "cost J22=" <<  costJ22 << std::endl;
      std::cout << " grad 3 T =" <<  Grad_J3T << std::endl; 
      std::cout << " Grad_J22T =" <<  Grad_J22T << std::endl; 
      std::cout << "cost J23="  <<  costJ23 << std::endl;
      std::cout << " Grad_J23q=" <<  Grad_J23q << std::endl; 
      std::cout << " Grad_J23T=" <<  Grad_J23T << std::endl; 
      std::cout << "cost J3=" <<  CostJ3 << std::endl; 
      std::cout << " Grad_J3T=" <<  Grad_J3T << std::endl; 
      std::cout<< "Full_grad = "<< Full_grad<< std::endl;

    }    
    return COST;
}

void Calc_J1(const Eigen::MatrixXd& variable_points,const std::vector<Eigen::MatrixXd*>& A, const Eigen::MatrixXd& b){
  // setting cost and gradient matrix to 0 before calculating a new one 
  Grad_J1.setZero();
  costJ1 = 0;
  for (int i = 0; i < M-1; i++) {
      Eigen::VectorXd f1 = b.col(i) - (*A[i])*variable_points.col(i);
      Eigen::VectorXd f2 = b.col(i+1) - (*A[i+1])*variable_points.col(i);
      Eigen::VectorXd logf1 = f1.array().log();
      Eigen::VectorXd logf2 = f2.array().log();
      double SUM1 = logf1.sum();
      double SUM2 = logf2.sum();
      double Cost = -K(i)*(SUM1 + SUM2);
        
      if (std::isnan(Cost)) {
        //std::cout << "costJ1 is NaN" << std::endl;
        //std::cout <<  i<< std::endl;
        //std::cout << "Tries points" << std::endl;
        //std::cout << variable_points << std::endl;
        costJ1 += 10000000000;
        Grad_J1.col(i)(0) += -1000;
        Grad_J1.col(i)(1) += -1000;
        continue;
      }

      else{
      costJ1 += Cost;
          
      Eigen::VectorXd If1 = 1/f1.array();
      Eigen::VectorXd If2 = 1/f2.array();

      Eigen::VectorXd A0i = -(*A[i]).col(0);
      Eigen::VectorXd A1i = -(*A[i]).col(1);
      Eigen::VectorXd A0ip1 = -(*A[i+1]).col(0);
      Eigen::VectorXd A1ip1 = -(*A[i+1]).col(1);

      
      Grad_J1.col(i)(0) += -K(i)*A0i.dot(If1) -K(i)*A0ip1.dot(If2);
      Grad_J1.col(i)(1) += -K(i)*A1i.dot(If1) -K(i)*A1ip1.dot(If2);

      }

    }
      // std::cout<< "Grad_j1" <<std::endl;
      // std::cout<< Grad_J1 <<std::endl;
      // std::cout<< "Points essayés" <<std::endl;
      // std::cout<< variable_points  <<std::endl;
}


void Calc_J21(const Eigen::VectorXd& Time_vector ){
      // setting cost and gradient matrix to 0 before calculating a new one 
  costJ21 = 0;
  Grad_J21T.setZero();
  for(int i = 0;i < M;i++){
    costJ21 += rho_T*Time_vector(i);
    Grad_J21T(i) = rho_T;
  }
}


void Calc_J22(const Eigen::MatrixXd& sample,const Eigen::VectorXd& Time_vector ,const double& vmax ){
    // setting cost and gradient matrix to 0 before calculating a new one 
    costJ22 = 0;
    Grad_J22q.setZero();
    Grad_J22T.setZero();
    for (int i = 1; i<= M-1;i++){
      
      Eigen::VectorXd Vect1 = sample.col(i+1) - sample.col(i-1);
      Eigen::VectorXd Vect2 = (Vect1)/(Time_vector(i)+Time_vector(i-1));
      double norm_vect_sqared =   Vect2(0)* Vect2(0) + Vect2(1)*Vect2(1);
      double G = std::pow( std::max(norm_vect_sqared - std::pow(vmax,2),0.0),3);
      costJ22 = rho_v(i-1)*G; // must be changed !!!
      // gradient computation

      if(G >= 0.0000000001){
         //d/dTi -> i-1
        Grad_J22T(i-1) += rho_v(i-1)*  3*std::pow((norm_vect_sqared - std::pow(vmax,2)),2) * (- norm_vect_sqared)/std::pow((Time_vector(i)+Time_vector(i-1)),2) * 2* (Time_vector(i)+ Time_vector(i-1));
        //d/Ti+1 -> i +1
        Grad_J22T(i) += rho_v(i-1)*  3*std::pow((norm_vect_sqared -  std::pow(vmax,2)),2) * (- norm_vect_sqared)/std::pow((Time_vector(i)+Time_vector(i-1)),2) * 2* (Time_vector(i)+ Time_vector(i-1));

        if(i == 1){
            //d/dq(i+1)x
                Grad_J22q.col(i)(0) +=rho_v(i-1)* 3*std::pow((norm_vect_sqared -  std::pow(vmax,2)),2) * 2*Vect1(0)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
            //d/dq(i+1)y
                Grad_J22q.col(i)(1) +=  rho_v(i-1)*3*std::pow((norm_vect_sqared -  std::pow(vmax,2)),2) * 2*Vect1(1)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
        }

            else if(i == M-1){
                //d/dq(i-1)x
                Grad_J22q.col(i-2)(0) += rho_v(i-1)*- 3*std::pow((norm_vect_sqared - std::pow(vmax,2)),2) * 2*Vect1(0)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
                //d/dq(i-1)y
                Grad_J22q.col(i-2)(1) += rho_v(i-1)* - 3*std::pow((norm_vect_sqared - std::pow(vmax,2)),2) * 2*Vect1(1)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
                }
            else{
                //d/dq(i+1)x
                Grad_J22q.col(i)(0) +=rho_v(i-1)* 3*std::pow((norm_vect_sqared -  std::pow(vmax,2)),2) * 2*Vect1(0)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
                //d/dq(i+1)y
                Grad_J22q.col(i)(1) +=  rho_v(i-1)*3*std::pow((norm_vect_sqared -  std::pow(vmax,2)),2) * 2*Vect1(1)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
                //d/dq(i-1)x
                Grad_J22q.col(i-2)(0) += rho_v(i-1)*- 3*std::pow((norm_vect_sqared - std::pow(vmax,2)),2) * 2*Vect1(0)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
                //d/dq(i-1)y
                Grad_J22q.col(i-2)(1) += rho_v(i-1)* - 3*std::pow((norm_vect_sqared - std::pow(vmax,2)),2) * 2*Vect1(1)/std::pow((Time_vector(i)+Time_vector(i-1)),2);
      }
      }  
    }
}

void Calc_J23(const Eigen::MatrixXd& sample,const Eigen::VectorXd& Time_vector ,const double& amax){
      // setting cost and gradient matrix to 0 before calculating a new one 
  costJ23 = 0;
  Grad_J23T.setZero();
  Grad_J23q.setZero();
    for (int i = 1; i<= M-1;i++){
        Eigen::VectorXd V1 = sample.col(i+1) - sample.col(i);
        Eigen::VectorXd V2 = sample.col(i)- sample.col(i-1);

        Eigen::VectorXd V1_prime = V1/Time_vector(i);
        Eigen::VectorXd V2_prime = V2/Time_vector(i-1);

        Eigen::VectorXd V = V1_prime - V2_prime;
        Eigen::VectorXd g = 2*V/(Time_vector(i)+Time_vector(i-1));
        double norm_squared = std::pow((g(0)),2) + std::pow((g(1)),2);
        double f = norm_squared - std::pow(amax,2.0);

        double G = rho_a(i-1)*std::pow(std::max(f,0.0),3);
        costJ23 += G;
        //gradient in T 
        //d/dTi -> d g(Ti,Ti+1)^3 / dTi   = 3 g^2 * dg/dTi 
        if(G >=0.0000000001){
            double r = (Time_vector(i-1) + Time_vector(i-1))*Time_vector(i)*Time_vector(i-1);

            Grad_J23T(i-1) += rho_a(i-1)* 3 *std::pow(f,2) * ((2 *g(0)) * (2*V1(0)*r - (std::pow(Time_vector[i],2)+2*Time_vector(i-1)*Time_vector(i))* 2* (V1(0)*Time_vector(i-1) - V2(0)*Time_vector(i))) + 
                             (2 *g(1)) * (2*V1(1)*r - (std::pow(Time_vector(i),2)+2*Time_vector(i-1)*Time_vector(i))* 2* (V1(1)*Time_vector(i-1) - V2(1)*Time_vector(i))))/std::pow(r,2);
            
            Grad_J23T(i) += rho_a(i-1)* 3 *std::pow(f,2) * ((2 * g(0)) * (2*V2(0)*r - (std::pow(Time_vector(i),2)+2*Time_vector(i-1)*Time_vector(i))* 2 * (V1(0)*Time_vector(i-1) - V2(0)*Time_vector(i))) +
                             (2 * g(1)) * (2*V2(1)*r - (std::pow(Time_vector(i),2)+2*Time_vector(i-1)*Time_vector(i))* 2 * (V1(1)*Time_vector(i-1) - V2(1)*Time_vector(i))))/std::pow(r,2);
             
            if(i == 1){
                // d/dq(i+1)x
                   Grad_J23q.col(i)(0) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* Time_vector(i-1)/r;
                //  d/dq(i+1)y
                   Grad_J23q.col(i)(1)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2* Time_vector(i-1)/r;
                //  d/dq(i)x
                   Grad_J23q.col(i-1)(0)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* (-Time_vector(i) -Time_vector(i-1))/r;
                //  d/dq(i)y
                   Grad_J23q.col(i-1)(1)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2*(-Time_vector(i) - Time_vector(i-1))/r;
            }
            else if(i == M-1){
                //  d/dq(i-1)x
                   Grad_J23q.col(i-2)(0) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* Time_vector(i)/r;
                //  d/dq(i-1)y
                   Grad_J23q.col(i-2)(1) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2* Time_vector(i)/r;
                //  d/dq(i)x
                   Grad_J23q.col(i-1)(0)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* (-Time_vector(i) -Time_vector(i-1))/r;
                //  d/dq(i)y
                   Grad_J23q.col(i-1)(1)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2*(-Time_vector(i) - Time_vector(i-1))/r;
        }
            else{
              //  d/dq(i-1)x
              Grad_J23q.col(i-2)(0) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* Time_vector(i)/r;
              //  d/dq(i-1)y
              Grad_J23q.col(i-2)(1) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2* Time_vector(i)/r;
              //  d/dq(i)x
              Grad_J23q.col(i-1)(0)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* (-Time_vector(i) -Time_vector(i-1))/r;
              //  d/dq(i)y
              Grad_J23q.col(i-1)(1)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2*(-Time_vector(i) - Time_vector(i-1))/r;
              // d/dq(i+1)x
              Grad_J23q.col(i)(0) += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(0) * 2* Time_vector(i-1)/r;
              //  d/dq(i+1)y
              Grad_J23q.col(i)(1)  += rho_a(i-1)* 3 *std::pow(f,2) * 2* g(1) * 2* Time_vector(i-1)/r;
            }
        }

    }
}

void  Calc_J3(const Eigen::VectorXd& Time_vector){
  CostJ3 = 0.0;
  Grad_J3T.setZero();
  for(int i=0;i < M; i++){
    double diff = (Time_vector(i) - min_time(i));
    if(diff > 0){
      CostJ3 += -0.005*log(diff);
      Grad_J3T(i) = -0.005/(diff);
    }
    else{
      CostJ3 += 10000;
      Grad_J3T(i) += -1000;
    }
  }
}

};
}
#endif // COST_FUNCTION_H
