#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <thread>
#include <list>

#include <Ravelin/Quatd.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Matrix3d.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/LinAlgd.h>

#include <Moby/LCP.h>
#include <Moby/NumericalException.h>

typedef Ravelin::Vector3d Vector3;
typedef Ravelin::VectorNd VectorN;
typedef Ravelin::Matrix3d Matrix3;
typedef Ravelin::MatrixNd MatrixN;
typedef Ravelin::LinAlgd  LinAlg;
typedef Ravelin::Quatd  Quat;

using std::vector;
using std::string;
using std::endl;

std::mutex write_mutex;

// get the discretization factor
const unsigned NPARAMS = 9;
const unsigned DISC = 10;

// converts parameters to an n-ary floating point representation
void to_params(unsigned NPARAMS, unsigned DISC, unsigned long long i, vector<double>& P)
{
  P.resize(NPARAMS);
  for (unsigned j=0; j< NPARAMS; j++)
  {
    unsigned rem = i % DISC;
    i /= DISC;
    P[NPARAMS-j-1] = (double) rem/(DISC-1);
  }
}

/// Creates the cvxopt version of the problem
void determine_LCP(unsigned long long i, double mu, int& pass)
{

  vector<double> p(NPARAMS);
  // determine parameters
  to_params(NPARAMS, DISC, i, p);

  MatrixN M;
  VectorN q;
  Matrix3 workM;
  const unsigned NK = 4, NC = 1;


  // setup mass values
  double m_d = p[0] + 0.01;
  double m_e = p[1] + 0.01;
  double m_f = (double) 1.0;
  double m_A = m_d;
  double m_B = m_d + m_e;
  double m_C = m_d + m_e + m_f;
  double m_a = m_A*m_A + m_B*m_B;
  double m_b = m_A*m_A + m_C*m_C;
  double m_c = m_B*m_B + m_C*m_C;

  // setup three eigenvalues of M
  double e_a = m_a/m_c;
  double e_b = m_b/m_c;
  double e_c = (double) 1.0;

  // setup quaternion (from LaValle)
  double q_x = std::sqrt(1 - p[2])*std::sin(2*M_PI*p[3]);
  double q_y = std::sqrt(1 - p[2])*std::cos(2*M_PI*p[3]);
  double q_z = std::sqrt(p[2])*std::sin(2*M_PI*p[4]);
  double q_w = std::sqrt(p[2])*std::cos(2*M_PI*p[4]);

  // setup rotation matrix using unit quaternion
  Quat qq(q_x, q_y, q_z, q_w);
  Matrix3 R(qq);

  // setup inertia matrix
//  Matrix3 X = Matrix3::zero();
//  X(0,0) = e_a;
//  X(1,1) = e_b;
//  X(2,2) = e_c;
//  R.transpose_mult(X,workM);
//  workM.mult(R,X);

  // setup inverse inertia matrix
  Matrix3 iX = Matrix3::zero();
  iX(0,0) = 1.0/e_a;
  iX(1,1) = 1.0/e_b;
  iX(2,2) = 1.0/e_c;
  // iX_r = R'iX R
  R.mult(iX,workM);
  workM.mult_transpose(R,iX);

  Vector3 N(0,0,1);

  // 2d Vel
  // setup tangent only velocity direction
  double theta = p[5] * M_PI;
  double u = p[6] * 2.0 - 1.0;
  Vector3 v(u*std::sin(theta),u*std::cos(theta),0);

  // Set up Sliding Direction Contact Jacobian vector
  Vector3 S(v);
  S.normalize();

  // 3d Force
  // NOTE: This determines the "direction of gravity" and the "mass"
  // THis way we can keep the problem oriented to the contact frame
  double fx = p[7] * 2 - (double) 1.0;
  double fy = 0;//p[8] * 2 - (double) 1.0;
  double fz = p[8] * 2 - (double) 1.0;
  Vector3 f(fx,fy,fz);

  // setup LCP matrix (non-constant parts)
  M.set_zero(1,1);
  q.set_zero(1);

  // N inv(M) (mu S' − N')
  iX.transpose_mult(N,q);
//  std::cout << "N iX: " << std::endl << q;
  M(0,0) = q.dot(N - mu*S);

  q = q.dot(f);

  // If M is not positive definite 
  // then Painleve' paradox is active
 // write_mutex.lock();
  if(M(0,0) <= 0 && q[0] < 0){
//    std::cout << 0
//               << " " << p[0] << " " << p[1] << " " << p[2] << " " << p[3]
//               << " " << p[4] << " " << p[5] << " " << p[6] << " " << p[7]
//               << " " << p[8] << std::endl;
    pass = 0;
    return;
  } else {
//    std::cout << 1
//               << " " << p[0] << " " << p[1] << " " << p[2] << " " << p[3]
//               << " " << p[4] << " " << p[5] << " " << p[6] << " " << p[7]
//               << " " << p[8] << std::endl;
  }
 // write_mutex.unlock();

  // Return LCP of the form
  // [N inv(M) (mu S' − N')] [cn] +  [N inv(M) f + Ndot v] = alpha
  // cn >= 0, alpha >= 0, cn'N alpha = 0
    pass = 1;
    return;
}

void test_lcp(double mu){
    unsigned long long eq = 0;
    const unsigned long long N = (unsigned long long) std::pow(DISC,NPARAMS);
    for (unsigned long long i=0; i< N; i++)
    {
      // determine the convex model
      int pass = false;
      determine_LCP(i,mu,pass);
      if(pass == 1) eq++;
   
      if(i%1000000 == 0){
        write_mutex.lock();
        std::cerr << "Progress, mu: " << mu 
                  <<", iteration:" << i 
                  <<", success: " << eq << " / " << N << std::endl;
        write_mutex.unlock();
      }
    }
    write_mutex.lock();
    std::cerr << "Progress, mu: " << mu <<", success: " << eq << " / " << N << std::endl;
    std::cout << "Progress, mu: " << mu <<", success: " << eq << " / " << N << std::endl;
    write_mutex.unlock();
}

int main(int argc, char* argv[])
{
  std::vector<std::thread> t;
  for (double mu=0.1;mu<1.5;mu+=0.1){
    t.push_back(std::thread(test_lcp,mu));
  }
  for (int i=0;i<t.size();i++){
    t[i].join();
  }
}
