#ifndef CORRIDOR_GENERATION_H
#define CORRIDOR_GENERATION_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace corridor_generation{

/**
 * @brief Do the initial sampling of the recieved trajectory.
 
 * @param[in] X Target trajectory X vector 
 * @param[in] Y Target trajectory Y vector
 * @param[in] size number of point in the trajecory
 * @param[in] dt target Trajctory sampling period
 * @param[in] Max_DT Max time difference between each sumbsample
 * @param[in] Max_dist Max distance between eanch subsamble
 * @param[in] Max_pt Max initial subsample on the trajecotry
 * @return A vector with the index of the subsampled points.
 */

Eigen::VectorXi Initial_sampling(const Eigen::VectorXd& X,const Eigen::VectorXd& Y,
                                const int& size,const double& dt,const double& Max_DT, const double& Max_dist, const int& Max_pt){
  int Point_counter = 1;
  double dist_counter = 0;
  double time_counter = 0;
  Eigen::VectorXi Index_vec;

  Index_vec.setZero(1);
  for(int i =1; i < size ; i++){
    dist_counter += sqrt( pow( (X(i)-X(i-1)) ,2) + pow( (Y(i)-Y(i-1)) ,2));
    time_counter += dt;
    
    if(dist_counter >= Max_dist || time_counter >= Max_DT){
      dist_counter = 0;
      time_counter = 0;
      Point_counter += 1;
      Index_vec.conservativeResize(Index_vec.size() + 1);
      Index_vec(Index_vec.size() - 1) = i;
    }

    if(Point_counter == Max_pt){
      break;
    }
    }

    if (Point_counter == 1){
        Index_vec.conservativeResize(Index_vec.size() + 1);
        Index_vec(Index_vec.size() - 1) = size;
    }
    return Index_vec;
                                
}

/**
 * @brief Evaluate A vector at index contained in another vector
 * @param[in] vec Vector to be evaluated
 * @param[in] idx Vector on evaluation index
 * @return A vector with the subsampled points.

 */
Eigen::VectorXd evaluateAtIndices(const Eigen::VectorXd& vec, const Eigen::VectorXi& idx) {
      Eigen::VectorXd result(idx.size());
      for (int i = 0; i < idx.size(); ++i) {
          result(i) = vec(idx(i));
          }
      return result;
}


/**
 * @brief Check the distance between the straight line linking each initial subsample and the real target trjactory 
 * If it is bigger that the threshold, it creates a new subsampling point.
  
 * @param[in] samplex Target subsapled trajectory in X
 * @param[in] sampley Target subsampled trajecotory in Y
 * @param[in] treshold Distance treshold 
 * @param[in] x Initial target traj in x
 * @param[in] y Initial target traj in y
 * @param[in] index_vec Vector of subsampled index
 * @return A Matrix with new sumbsampled x an y.
 */
Eigen::MatrixXd resample(const Eigen::VectorXd& samplex, const Eigen::VectorXd& sampley,const double& treshold,
                            const Eigen::VectorXd& x,const Eigen::VectorXd& y,Eigen::VectorXi Index_vec){
  int num_divisions = 0;                        
  Eigen::MatrixXd IX(2,0);

  for (int i = 1; i < samplex.size(); i++) {
    double x1 = samplex[i - 1];
    double x2 = samplex[i];
    double y1 = sampley[i - 1];
    double y2 = sampley[i];

    int len = Index_vec[i] - Index_vec[i - 1];
    Eigen::VectorXd yi = y.segment(Index_vec[i-1],len);
    Eigen::VectorXd xi = x.segment(Index_vec[i-1], len);
    
    Eigen::VectorXd D(len);

    for (int j = 0; j < len; ++j) {
       D[j] =  abs((x2 - x1) * (y1 - yi[j]) - (x1 - xi[j]) * (y2 - y1))/sqrt(pow((x2 - x1),2) +pow((y2 - y1),2));
    }
    double max_err = D.maxCoeff();
    int max_index;
    D.maxCoeff(&max_index);

    if (max_err > treshold) {
        IX.conservativeResize(2, num_divisions+1);
        IX(0, num_divisions) = max_index;
        IX(1, num_divisions) = i;
        num_divisions++;
        }

    }
      return IX;
}


/**
 * @brief Calculate the position of the 2 points that are at a certain distance from from one point and at a certain angle from line connecting thhe 2 input points
  
 * @param[in] a first point
 * @param[in] b second point
 * @param[in] dist distance
 * @param[in] angle in degree 
 * @return A matrix2D containig the points
 */

Eigen::Matrix2Xd points_at_distance(const Eigen::Ref<const Eigen::VectorXd>& a, const Eigen::Ref<const Eigen::VectorXd>& b, 
                                    const double dist, const double angle){
// Convert the angle from degrees to radians
    const double angle_rad = angle * M_PI / 180.0;

// Compute the vector from `a` to `b`
    const Eigen::VectorXd ab = b - a;

    // Compute the normal vector to the line connecting `a` and `b`
    Eigen::VectorXd n(2);
    n << -ab(1), ab(0);

// Normalize the normal vector
    n.normalize();

// Compute the vectors representing the two points at a distance `dist`
// and oriented at angles of `+angle` and `-angle` degrees with respect to the line
    Eigen::Matrix2Xd points(2, 2);
    points.col(0) = a + std::cos(angle_rad) * dist * n - std::sin(angle_rad) * dist * ab / ab.norm();
    points.col(1) = a - std::cos(angle_rad) * dist * n - std::sin(angle_rad) * dist * ab / ab.norm();

      return points;
  }

  Eigen::MatrixXd compute_vertices(const Eigen::VectorXd& P1, const Eigen::VectorXd& P2, double dist) {
    
    Eigen::VectorXd direction = (P2 - P1).normalized();
    Eigen::VectorXd pmin = P1 + direction * dist;
    Eigen::VectorXd pmax = P2 - direction * dist;
    Eigen::Matrix2Xd v11 = points_at_distance(P1, pmin, dist, 60);
    Eigen::Matrix2Xd v12 = points_at_distance(P1, pmin, dist, 180);
    Eigen::Matrix2Xd v21 = points_at_distance(P2, pmax, dist, 180);
    Eigen::Matrix2Xd v22 = points_at_distance(P2, pmax, dist, 60);
    Eigen::MatrixXd vertices(8, 2);
    vertices.row(7) = v11.col(0);
    vertices.row(6) = v12.col(1);
    vertices.row(5) = v21.col(0);
    vertices.row(4) = v22.col(1);
    vertices.row(3) = v22.col(0);
    vertices.row(2) = v21.col(1);
    vertices.row(1) = v12.col(0);
    vertices.row(0) = v11.col(1);
    return vertices;
  }





class Corridor{

  public:
        Eigen::VectorXi Index_vec;
        Eigen::VectorXd Time_vector;
        int M ;
        Eigen::MatrixXd sample;
        std::vector<Eigen::MatrixXd*> A;
        Eigen::MatrixXd b;
        // For plotting purposes to be removed
        Eigen::MatrixXd Vertices;

  //Fisrt deterministic subsampling based on time.
  //Nbr_inital_sample = nombre de points dans la trojectire donnée rxample 501
  //Time_sample = échantillonage temporel voulu 
  //dt période d'échatillonage initiale. 

  Corridor(const int& Nbr_inital_sample,const double& Max_DT,double dt,const Eigen::VectorXd& Traj_x,const Eigen::VectorXd& Traj_y,
            const double& distance_treshold,const double& Max_dist,const int& Max_pt,const double& FOV_dist){


          Index_vec = Initial_sampling(Traj_x,Traj_y, Nbr_inital_sample,dt,Max_DT,Max_dist,Max_pt);
          Eigen::VectorXd samplex = evaluateAtIndices(Traj_x,Index_vec);
          Eigen::VectorXd sampley = evaluateAtIndices(Traj_y,Index_vec);

          sample = Distance_condition_verification(samplex,sampley,Traj_x,Traj_y,distance_treshold);
          M = sample.cols() -1;
          Constrait_hyperplane_formualation(FOV_dist);

          Time_vector.setZero(M);
          Calculate_time(dt);
        }

        ~Corridor() {
            std::cout << "Destructor called" << std::endl;

            for (int i = 0; i < A.size(); i++) {
              delete A[i];
            }  
      }

  Eigen::MatrixXd Distance_condition_verification(Eigen::VectorXd samplex,Eigen::VectorXd sampley,
                                                    Eigen::VectorXd Traj_x,Eigen::VectorXd Traj_y,double distance_treshold){
  // Index_vec must not be given as argument so that it is updated !!
    bool u = true;
    while (u) {
      Eigen::MatrixXd xi = resample(samplex,sampley,distance_treshold,Traj_x,Traj_y,Index_vec);
      if (xi.cols() != 0) {
        int iterator = 0;
          for (int i = 0; i < xi.cols(); i++) {
          int k = xi(1,i);
          int j = k + iterator;
          int new_sample_idx = Index_vec[j - 1] + (xi(0, iterator));
          Index_vec.conservativeResize(Index_vec.size() + 1);
          Index_vec(Index_vec.size()-1) = new_sample_idx;
          std::sort(Index_vec.data(), Index_vec.data() + Index_vec.size());
          iterator += 1;
          }
      samplex = evaluateAtIndices(Traj_x,Index_vec);
      sampley = evaluateAtIndices(Traj_y,Index_vec);
    }
      else {
        u = false;
    }
  }
  Eigen::MatrixXd Full_sample(2,samplex.rows());
  Full_sample.row(0) = samplex;
  Full_sample.row(1) = sampley;
  
  return Full_sample;
}

void Constrait_hyperplane_formualation(double dist){
  A.resize(M);
  for (int i = 0; i < M; i++) {
    A[i] = new Eigen::MatrixXd(8, 2);
    A[i]->setZero();
  }

  b.resize(8, M);
  b.setZero();

  for(int i=1; i<M +1; i++){
    Eigen::MatrixXd vertices = compute_vertices(sample.col(i-1), sample.col(i), dist);
    if( i ==1 ){
        Vertices.resize(8, 2);}
    else{
      Vertices.conservativeResize(8, i*2);
    }

    Vertices.block(0, 2*(i-1), 8, 2) = vertices;
    int k = 0;

    for(int j=0; j<vertices.rows(); j++){

      if (j == 0) {
        k = 7;
      } 
      else{
        k = j-1;
      }

      Eigen::VectorXd vector = vertices.row(j) - vertices.row(k);
      double norm_vector = sqrt(vector[0]*vector[0] + vector[1]*vector[1]);
      Eigen::VectorXd normalized_vect = vector/norm_vector;

      Eigen::VectorXd outward_vect(2);
      outward_vect << normalized_vect[1], -normalized_vect[0];
      double c = outward_vect[0]*vertices.row(j)(0) + outward_vect[1]*vertices.row(j)(1);
      (*A[i-1]).row(j) = outward_vect.transpose();
      b(j,i-1) = c;
    }
  }
}

void Calculate_time(double dt){
  for(int i =1; i<=M;i++){
    double DT = (Index_vec(i) -Index_vec(i-1))*dt;
    Time_vector(i-1) = DT;
  }
}



void Resample(const Eigen::VectorXi& New_Index_vec,double dt,const Eigen::VectorXd& Traj_x,const Eigen::VectorXd& Traj_y,const double& FOV_dist){
	Index_vec.resize(New_Index_vec.size());
	Index_vec = New_Index_vec;
	Eigen::VectorXd samplex = evaluateAtIndices(Traj_x,Index_vec);
	Eigen::VectorXd sampley = evaluateAtIndices(Traj_y,Index_vec);
	sample.resize(2,samplex.rows());
	sample.row(0) = samplex;
	sample.row(1) = sampley;
        M = sample.cols() -1;
        std::cout << M <<std::endl;
        std::cout << "M" <<std::endl;
      	Constrait_hyperplane_formualation(FOV_dist);

	Time_vector.resize(M);

        Time_vector.setZero(M);

        Calculate_time(dt);

        }
        
        
        
        Eigen::VectorXi Initial_sampling(const Eigen::VectorXd& X,const Eigen::VectorXd& Y,
                                const int& size,const double& dt,const double& Max_DT, const double& Max_dist, const int& Max_pt){
  int Point_counter = 1;
  double dist_counter = 0;
  double time_counter = 0;
  Eigen::VectorXi Index_vec;

  Index_vec.setZero(1);
  for(int i =1; i < size ; i++){
    dist_counter += sqrt( pow( (X(i)-X(i-1)) ,2) + pow( (Y(i)-Y(i-1)) ,2));
    time_counter += dt;
    
    if(dist_counter >= Max_dist || time_counter >= Max_DT){
      dist_counter = 0;
      time_counter = 0;
      Point_counter += 1;

      Index_vec.conservativeResize(Index_vec.size() + 1);
      Index_vec(Index_vec.size() - 1) = i;
    }

    if(Point_counter == Max_pt){
      break;
    }
    }

    if (Point_counter == 1){
        Index_vec.conservativeResize(Index_vec.size() + 1);
        Index_vec(Index_vec.size() - 1) = size;
    }
    return Index_vec;
                                
}




};



}
#endif // CORRIDOR_GENERATION_H
