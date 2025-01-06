#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Coordinates.h>

#define PI 3.1415926535897932384626433832795
#define nw 480
#define J_p 1.1
#define l_p 0.009
#define l_s 0.012
#define r_l 0.02
#define J_l -1
#define l_l 0.005
#define m_l 0.077
#define mu 0.000001256637061435917

using namespace BLA;

unsigned long startTime;

const int dim = 3;
const int n = 100;  //Discretizations of the levitating magnet
const int magnets_num = 4;

//--------------Compute field from magnet in polar coords-------------------
Matrix<n> computeC(float mu0, float r, Matrix<n> rho){
  Matrix<n> result;
  for(int i = 0; i<n;++i){
    result(i) = mu0/(4*PI*sqrt(r*rho(i)));
  }
  return result;
}

Matrix<n> computeK2(float r, Matrix<n> rho, Matrix<n> z){
  Matrix<n> result;
  for(int i = 0; i<n;++i){
    result(i) = 4*r*rho(i)/((r+rho(i))*(r+rho(i))+z(i)*z(i));
    if(result(i)>1){
      result(i)=1.0;
    }
    if(result(i)<0){
      result(i)=0.0;
    }
  }
  return result;
}

Matrix<2,n> elliptic(Matrix<n> m){
  double tol = 5e-13;
  Matrix<n> a0;
  Matrix<n> b0;
  for(int i=0;i<n;i++){
    a0(i)=1;
    b0(i)=0;
  }
  Matrix<n> c0;
  for(int i=0;i<n;++i){
    b0(i)=sqrt(1-m(i));
  }
  int i1 = 0;
  double mm = 1e308; //inf
  int f = 0;

  Matrix<n> a_n;
  Matrix<n> b_n;
  Matrix<n> c_n;
  Matrix<n> w1;
  Matrix<n> s0 = m;

  while(mm>tol){       //This is a big fault of this method, this condition should be dependent on convergence of mm wrt. tol, but I found a simple for loop to be sufficient for correct results in this instance
    for(int i=0;i<n;++i){
      a_n(i) = (a0(i)+b0(i))/2;
      b_n(i) = sqrt(double(a0(i))*double(b0(i)));
      c_n(i) = (double(a0(i))-double(b0(i)))/2;
    }
    i1 = i1+1;
    float max_val = 0;
    for(int i=0;i<n;++i){
      w1(i)=pow(2,i1)*pow(c_n(i),2);
      if(w1(i) > max_val){
        max_val = w1(i);
      }
    }
    mm=max_val;
    f = f+1;
    for(int i=0;i<n;++i){
      s0(i) = s0(i)+w1(i);
      a0(i) = a_n(i);
      b0(i) = b_n(i);
      c0(i) = c_n(i);
    }
  }
  Matrix<2,n> result;
  for(int i=0;i<n;++i){
    if(m(i)==1){
      result(0,i) = 1e308;  //inf
      result(1,i) = 1.0;
    }
    else{
      result(0,i)=PI/(2*a_n(i));
      result(1,i)=result(0,i)*(1-s0(i)/2);
    }
  }
  return result;
}

Matrix<3,n> calculateB(Matrix<n> phi,Matrix<n> rho,Matrix<n> z,Matrix<n> c,Matrix<n> k2,Matrix<n> K,Matrix<n> E,float r){
  Matrix<3,n> result;
  for(int i=0;i<n;++i){
    result(0,i) = phi(i);
    result(1,i) = - (z(i) / rho(i)) * c(i) * sqrt(k2(i)) * (K(i) - (pow(rho(i),2) + pow(r,2) + pow(z(i),2)) / (pow((rho(i) - r),2) + pow(z(i),2)) * E(i));
    result(2,i) = c(i)*sqrt(k2(i))*(K(i)-(pow(rho(i),2)-pow(r,2)+pow(z(i),2))/(pow((rho(i)-r),2)+pow(z(i),2))*E(i));
  }
  return result;
}

Matrix<3,n> computeFieldCircularWirePolar(Matrix<n> phi, Matrix<n> rho, Matrix<n> z, float r,float mu0){
  Matrix<n> c = computeC(mu0,r,rho);
  Matrix<n> k2 = computeK2(r,rho,z);
  Matrix<2,n> K_E = elliptic(k2);
  Matrix<n> K;
  Matrix<n> E;
  for(int i=0;i<n;++i){
    K(i) = K_E(0,i);
    E(i) = K_E(1,i);
  }
  Matrix<3,n> B = calculateB(phi,rho,z,c,k2,K,E,r);
  for(int i=0;i<n;++i){
    if(rho(i)==0){
      B(0,i)=0.0;
      B(1,i)=0.0;
      B(2,i)=mu0*r*r/(2*(pow(sqrt(r*r+z(i)*z(i)),3)));
    }
  }
  return B;
}

//----------------Coordinate transforms--------------------
Matrix<3,n> cart2pol(Matrix<n> x,Matrix<n> y,Matrix<n> z){
  Matrix<3,n> result;
  for(int i=0;i<n;++i){
    result(0,i) = atan2(y(i),x(i));
    result(1,i) = sqrt(pow(x(i),2)+pow(y(i),2));
    result(2,i) = z(i);
  }
  return result;
}

Matrix<3,n> pol2cart(Matrix<3,n> b){
  Matrix<3,n> result;

  for(int j=0;j<n;++j){
    result(0,j) = double(b(1,j))*cos(double(b(0,j)));
    result(1,j) = double(b(1,j))*sin(double(b(0,j)));
    result(2,j) = double(b(2,j));
  }
  return result;
}

Matrix<3,n> computeFieldCircularWireCartesian(Matrix<n> x, Matrix<n> y, Matrix<n> z,float r, float mu0){
  Matrix<3,n> polar = cart2pol(x,y,z);
  Matrix<n> phi;
  Matrix<n> rho;
  Matrix<n> z_p;
  for(int i = 0;i<n;++i){
    phi(i) = polar(0,i);
    rho(i) = polar(1,i);
    z_p(i) = polar(2,i);
  }

  Matrix<3,n> B = computeFieldCircularWirePolar(phi,rho,z_p,r,mu0);
  Matrix<3,n> B_cart = pol2cart(B);
  return B_cart;
}

//----------------Compute field from base-------------------
Matrix<3,8*n> computeFieldBase(Matrix<n> x, Matrix<n> y, Matrix<n> z){
  Matrix<3,8*n> result;
  //Permanent
  Matrix<n> phi;
  Matrix<n> rho;
  Matrix<n> z_p;
  Matrix<3,4> permanentCoords = {0.028,-0.028,0.028,-0.028,0.028,0.028,-0.028,-0.028,0.0046,0.0046,0.0046,0.0046};
  
  for(int i=0;i<4;++i){
    //Matrix<3,n> polar_p = cart2pol(x-permanentCoords(i),y-permanentCoords(i),z-permanentCoords(i));
    //for(int j = 0;j<n;++j){
      //phi(j)=polar_p(0,j);
      //rho(j)=polar_p(1,j);
      //z_p(j) = polar_p(2,j);
    //}
    float r_p = 0.02;
    Matrix<3,n> b_temp = computeFieldCircularWireCartesian(x-permanentCoords(0,i),y-permanentCoords(1,i),z-permanentCoords(2,i),1,4*PI);       //REMEMBER TO CHANGE TO R AND MU
    
    for(int k = 0;k<n;++k){
      result(0,n*i+k) = double(b_temp(0,k));
      result(1,n*i+k) = double(b_temp(1,k));
      result(2,n*i+k) = double(b_temp(2,k));
    }
  }

  //Solenoids
  Matrix<3,4> solenoidsCoords = {0.02, 0, -0.02,0,0,0.02,0,-0.02,0.006,0.006,0.006,0.006};
  for(int i=0;i<4;++i){
    float r_s = 0.0093;
    Matrix<3,n> b_temp = computeFieldCircularWireCartesian(x-solenoidsCoords(0,i),y-solenoidsCoords(1,i),z-solenoidsCoords(2,i),1,4*PI);
    //Serial << b_temp << '\n';
    for(int k = 0;k<n;++k){
      result(0,n*4+n*i+k) = double(b_temp(0,k))*nw;
      result(1,n*4+n*i+k) = double(b_temp(1,k))*nw;
      result(2,n*4+n*i+k) = double(b_temp(2,k))*nw;
    }
  }
  return result;
}

//----------------Force and torque computation-------------------
Matrix<3,n> pCalc(Matrix<n> theta){  //Replace 100 with n!!!
  Matrix<3,n> result;
  for(int i=0;i<n;i++){
    result(0,i) = r_l*cos(theta(i));
    result(1,i) = r_l*sin(theta(i));
    result(2,i) = 0;
  }
  return result;
}

Matrix<3,3> rotCalc(double a, double b, double g){
  Matrix<3,3> result = {1*cos(b)*cos(g),0,0,0,cos(a)*cos(g),0,0,0,cos(a)*cos(b)};
  return result;
}

Matrix<3,n> pvec_Calc(double x, double y, double z, Matrix<3,n> p,Matrix<3,3> R){
  Matrix<3,n> result;
  for(int i =0;i<n;i++){
    Matrix<3,1> p_temp = {p(0,i),p(1,i),p(2,i)};
    Matrix<3,1> temp_res = R*p_temp;
    result(0,i) = temp_res(0)+x;
    result(1,i) = temp_res(1)+y;
    result(2,i) = temp_res(2)+z;
  }
  return result;
}

Matrix<3,n> computeTangent(Matrix<3,3> R, Matrix<n> theta){
  Matrix<3,n> result;
  for(int i=0;i<n;i++){
    Matrix<3,1> temp = {cos(theta(i)+PI/2),sin(theta(i)+PI/2),0};
    Matrix<3,1> temp_res = R*temp;
    result(0,i) = temp_res(0);
    result(1,i) = temp_res(1);
    result(2,i) = temp_res(2);
  }
  return result;
}

Matrix<3,n> computeKCross(double K, Matrix<3,n> tangent){
  Matrix<3,n> result;
  for(int i=0;i<n;i++){
    result(0,i) = K*l_l*tangent(0,i);
    result(1,i) = K*l_l*tangent(1,i);
    result(2,i) = K*l_l*tangent(2,i);
  }
  return result;
}

Matrix<3,n> computeJVec(double K, Matrix<3,n> n_vec){
  Matrix<3,n> result;
  for(int i=0;i<n;i++){
    result(0,i)=K*l_l*n_vec(0,i);
    result(1,i)=K*l_l*n_vec(1,i);
    result(2,i)=K*l_l*n_vec(2,i);
  }
  return result;
}

Matrix<3,1> cross(Matrix<3,1> a, Matrix<3,1> b){
  Matrix<3,1> result;
  result(0) = a(1)*b(2)-a(2)*b(1);
  result(1) = a(2)*b(0)-a(0)*b(2);
  result(2) = a(0)*b(1)-a(1)*b(0);
  return result;
}

double trapezoidalIntegration(Matrix<n+1> x, Matrix<n+1> y){
  double result = 0;
  double diff = (x(n)-x(0))/n;        //Generally this is a vector, but the vector theta (here x) is by design evenly-spaced, so the difference between each element is constant
  //Serial << y << '\n';
  for(int i=0;i<n;i++){
    result += diff*(y(i)+y(i+1))/2;
  }
  return result;
}

Matrix<6,8> computeForceAndTorque(Matrix<10> x){
  double K = -J_l/mu;                                       //This might not be accurate enough (very big)
  Matrix<6,8> result;
  Matrix<n> theta;  
  for(int i=0;i<n;i++){
    theta(i)=i*2*PI/n;
  }
  Matrix<3,n> p_vec = pCalc(theta);
  Matrix<3,3> R = rotCalc(x(4),x(5),0);
  Matrix<3,n> p = pvec_Calc(double(x(0)),double(x(1)),double(x(2)),p_vec,R);    //Remember to put in x!!
  Matrix<n> px;
  Matrix<n> py;
  Matrix<n> pz;
  for(int i=0;i<n;i++){
    px(i)=p(0,i);
    py(i)=p(1,i);
    pz(i)=p(2,i);
  }
  Matrix<3,8*n> B = computeFieldBase(px,py,pz);
  Matrix<3,n> tangent = computeTangent(R,theta);
  Matrix<3,n> K_cross = computeKCross(K,tangent);     //Might be inaccurate, see note on K variable
  Matrix<3,n> nvec;
  for(int i=0;i<n;i++){
    Matrix<3,1> temp = {0,0,1};
    Matrix<3,1> temp_res = R*temp;
    nvec(0,i) = temp_res(0);
    nvec(1,i) = temp_res(1);
    nvec(2,i) = temp_res(2);
  }
  Matrix<3,n> Jvec = computeJVec(K,nvec);

  Matrix<n+1> theta_trap;
  Matrix<n+1> F_m_totx;
  Matrix<n+1> F_m_toty;
  Matrix<n+1> F_m_totz;
  Matrix<n+1> T_m_totx;
  Matrix<n+1> T_m_toty;
  Matrix<n+1> T_m_totz;  
  Matrix<n+1> F_s_totx;
  Matrix<n+1> F_s_toty;
  Matrix<n+1> F_s_totz;
  Matrix<n+1> T_s_totx;
  Matrix<n+1> T_s_toty;
  Matrix<n+1> T_s_totz;
  for(int i=0;i<4;i++){
    for(int j=0;j<n;j++){
      Matrix<3,1> K_temp = {K_cross(0,j),K_cross(1,j),K_cross(2,j)};
      Matrix<3,1> J_temp = {Jvec(0,j),Jvec(1,j),Jvec(2,j)};
      Matrix<3,1> Bm_temp = {B(0,n*i+j),B(1,n*i+j),B(2,n*i+j)};
      Matrix<3,1> Bs_temp = {B(0,4*n+n*i+j),B(1,4*n+n*i+j),B(2,4*n+n*i+j)};
      Matrix<3,1> temp_res_Fm = cross(K_temp,Bm_temp);
      Matrix<3,1> temp_res_Tm = cross(J_temp,Bm_temp);
      Matrix<3,1> temp_res_Fs = cross(K_temp,Bs_temp);
      Matrix<3,1> temp_res_Ts = cross(J_temp,Bs_temp);
      F_m_totx(j)=temp_res_Fm(0);
      F_m_toty(j)=temp_res_Fm(1);
      F_m_totz(j)=temp_res_Fm(2);
      T_m_totx(j)=temp_res_Tm(0);
      T_m_toty(j)=temp_res_Tm(1);
      T_m_totz(j)=temp_res_Tm(2);
      F_s_totx(j)=temp_res_Fs(0);
      F_s_toty(j)=temp_res_Fs(1);
      F_s_totz(j)=temp_res_Fs(2);
      T_s_totx(j)=temp_res_Ts(0);
      T_s_toty(j)=temp_res_Ts(1);
      T_s_totz(j)=temp_res_Ts(2);
      theta_trap(j)=theta(j);
    }
    theta_trap(n) = 2*PI;
    F_m_totx(n)=F_m_totx(0);
    F_m_toty(n)=F_m_toty(0);
    F_m_totz(n)=F_m_totz(0);
    T_m_totx(n)=T_m_totx(0);
    T_m_toty(n)=T_m_toty(0);
    T_m_totz(n)=T_m_totz(0);
    F_s_totx(n)=F_s_totx(0);
    F_s_toty(n)=F_s_toty(0);
    F_s_totz(n)=F_s_totz(0);
    T_s_totx(n)=T_s_totx(0);
    T_s_toty(n)=T_s_toty(0);
    T_s_totz(n)=T_s_totz(0);
    double fxm = r_l*trapezoidalIntegration(theta_trap,F_m_totx);
    double fym = r_l*trapezoidalIntegration(theta_trap,F_m_toty);
    double fzm = r_l*trapezoidalIntegration(theta_trap,F_m_totz);
    double txm = r_l*trapezoidalIntegration(theta_trap,T_m_totx);
    double tym = r_l*trapezoidalIntegration(theta_trap,T_m_toty);
    double tzm = r_l*trapezoidalIntegration(theta_trap,T_m_totz);
    double fxs = r_l*trapezoidalIntegration(theta_trap,F_s_totx);
    double fys = r_l*trapezoidalIntegration(theta_trap,F_s_toty);
    double fzs = r_l*trapezoidalIntegration(theta_trap,F_s_totz);
    double txs = r_l*trapezoidalIntegration(theta_trap,T_s_totx);
    double tys = r_l*trapezoidalIntegration(theta_trap,T_s_toty);
    double tzs = r_l*trapezoidalIntegration(theta_trap,T_s_totz);
    result(0,i) = fxm;
    result(1,i) = fym;
    result(2,i) = fzm;
    result(3,i) = txm;
    result(4,i) = tym;
    result(5,i) = tzm;    
    result(0,i+4) = fxs;
    result(1,i+4) = fys;
    result(2,i+4) = fzs;
    result(3,i+4) = txs;
    result(4,i+4) = tys;
    result(5,i+4) = tzs;
  }
  return result;
}
//-----------------QR decomposition--------------------
Matrix<4> Householder(Matrix<5,4> A, Matrix<5> b){
  Matrix<5,5> I5 = {1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1};
  Matrix<4,4> I4 = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  Matrix<3,3> I3 = {1,0,0,0,1,0,0,0,1};
  Matrix<2,2> I2 = {1,0,0,1};
  
  Matrix<5> A1 = {A(0,0),A(1,0),A(2,0),A(3,0),A(4,0)};
  double len_A = sqrt(A1(0)*A1(0)+A1(1)*A1(1)+A1(2)*A1(2)+A1(3)*A1(3)+A1(4)*A1(4));
  Matrix<5> U1 = {A1(0)-len_A,A1(1),A1(2),A1(3),A1(4)};
  double len_U = sqrt(U1(0)*U1(0)+U1(1)*U1(1)+U1(2)*U1(2)+U1(3)*U1(3)+U1(4)*U1(4));
  Matrix<5> V1;
  for(int i=0;i<5;i++){
    V1(i)=sqrt(2)*U1(i)/len_U;
  }
   Matrix<5,5> Q1 = I5-V1*~V1;
   
   Matrix<5,4> R = Q1*A;

   Matrix<4> A2 = {R(1,1),R(2,1),R(3,1),R(4,1)};
   len_A = sqrt(A2(0)*A2(0)+A2(1)*A2(1)+A2(2)*A2(2)+A2(3)*A2(3));
   Matrix<4> U2 = {A2(0)-len_A,A2(1),A2(2),A2(3)};
   len_U = sqrt(U2(0)*U2(0)+U2(1)*U2(1)+U2(2)*U2(2)+U2(3)*U2(3));
   Matrix<4> V2;
   for(int i=0;i<4;i++){
    V2(i)=sqrt(2)*U2(i)/len_U;
   }
   Matrix<4,4> Q_temp2 = I4-V2*~V2;
   Matrix<5,5> Q2 = {1,0,0,0,0,0,Q_temp2(0,0),Q_temp2(0,1),Q_temp2(0,2),Q_temp2(0,3),0,Q_temp2(1,0),Q_temp2(1,1),Q_temp2(1,2),Q_temp2(1,3),0,Q_temp2(2,0),Q_temp2(2,1),Q_temp2(2,2),Q_temp2(2,3),0,Q_temp2(3,0),Q_temp2(3,1),Q_temp2(3,2),Q_temp2(3,3)};
   R = Q2*R;

  Matrix<3> A3 = {R(2,2),R(3,2),R(4,2)};
  len_A = sqrt(A3(0)*A3(0)+A3(1)*A3(1)+A3(2)*A3(2));
  Matrix<3> U3 = {A3(0)-len_A,A3(1),A3(2)};
  len_U = sqrt(U3(0)*U3(0)+U3(1)*U3(1)+U3(2)*U3(2));
  Matrix<3> V3;
  for(int i=0;i<3;i++){
    V3(i)=sqrt(2)*U3(i)/len_U;
  }
  Matrix<3,3> Q_temp3 = I3-V3*~V3;
  Matrix<5,5> Q3 = {1,0,0,0,0,0,1,0,0,0,0,0,Q_temp3(0,0),Q_temp3(0,1),Q_temp3(0,2),0,0,Q_temp3(1,0),Q_temp3(1,1),Q_temp3(1,2),0,0,Q_temp3(2,0),Q_temp3(2,1),Q_temp3(2,2)};
  R = Q3*R;
  Matrix<2> A4 = {R(3,3),R(4,3)};
  len_A = sqrt(A4(0)*A4(0)+A4(1)*A4(1));
  Matrix<2> U4 = {A4(0)-len_A,A4(1)};
  len_U = sqrt(U4(0)*U4(0)+U4(1)*U4(1));
  Matrix<2> V4;
  for(int i=0;i<2;i++){
    V4(i)=sqrt(2)*U4(i)/len_U;
  }
  Matrix<2,2> Q_temp4 = I2-V4*~V4;
  Matrix<5,5> Q4 = {1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,Q_temp4(0,0),Q_temp4(0,1),0,0,0,Q_temp4(1,0),Q_temp4(1,1)};
  R = Q4*R;
  Matrix<5,5> Q = Q4*Q3*Q2*Q1;
  

  Matrix<4> result = {0,0,0,0};
  Matrix<5> b_temp = Q*b;
  result(3) = b_temp(3)/R(3,3);
  result(2) = (b_temp(2)-R(2,3)*result(3))/(R(2,2));
  result(1) = (b_temp(1)-R(1,2)*result(2)-R(1,3)*result(3))/R(1,1);
  result(0) = (b_temp(0)-R(0,1)*result(1)-R(0,2)*result(2)-R(0,3)*result(3))/R(0,0);
  return result;
}

Matrix<5> project(Matrix<5> u, Matrix<5> a){
  Matrix<5> result = {0,0,0,0,0};
  double inner_u_a = double(u(0))*double(a(0))+double(u(1))*double(a(1))+double(u(2))*double(a(2))+double(u(3))*double(a(3))+double(u(4))*double(a(4));
  double inner_u_u = double(u(0))*double(u(0))+double(u(1))*double(u(1))+double(u(2))*double(u(2))+double(u(3))*double(u(3))+double(u(4))*double(u(4));
  double rat = inner_u_a/inner_u_u;
  for(int i=0;i<5;i++){
    result(i)=rat*u(i);
  }
  return result;
}

Matrix<5> generateE(Matrix<5> u){
  Matrix<5> result = {0,0,0,0,0};
  double abs = sqrt(double(u(0))*double(u(0))+double(u(1))*double(u(1))+double(u(2))*double(u(2))+double(u(3))*double(u(3))+double(u(4))*double(u(4)));
  for(int i=0;i<5;i++){
    result(i)=double(u(i))/abs;
  }
  return result;
}
//----------Gram-Schmidt---------
Matrix<4> QRd(Matrix<5,4> A, Matrix<5> b){

  Matrix<5> u1 = {double(A(0,0)),double(A(1,0)),double(A(2,0)),double(A(3,0)),double(A(4,0))};
  Matrix<5> e1 = generateE(u1);
  Matrix<5> u2t = {double(A(0,1)),double(A(1,1)),double(A(2,1)),double(A(3,1)),double(A(4,1))};
  Matrix<5> u2 = u2t-project(u1,u2t);
  Matrix<5> e2 = generateE(u2);
  Matrix<5> u3t = {double(A(0,2)),double(A(1,2)),double(A(2,2)),double(A(3,2)),double(A(4,2))};
  Matrix<5> u3 = u3t-project(u1,u3t)-project(u2,u3t);
  Matrix<5> e3 = generateE(u3);
  Matrix<5> u4t = {double(A(0,3)),double(A(1,3)),double(A(2,3)),double(A(3,3)),double(A(4,3))};
  Matrix<5> u4 = u4t-project(u1,u4t)-project(u2,u4t)-project(u3,u4t);
  Matrix<5> e4 = generateE(u4);

  
  Matrix<4,5> Q_T = ~e1 && ~e2 && ~e3 && ~e4;
  Matrix<4,4> R = Q_T*A;
  Matrix<4> result = {0,0,0,0};
  Matrix<4> b_temp = Q_T*b;
  result(3)=b_temp(3)/R(3,3);
  result(2)=(b_temp(2)-R(2,3)*result(3))/R(2,2);
  result(1)=(b_temp(1)-R(1,2)*result(2)-R(1,3)*result(3))/R(1,1);
  result(0)=(b_temp(0)-R(0,1)*result(1)-R(0,2)*result(2)-R(0,3)*result(3))/R(0,0);
  return result;
}
//----------------The controller-----------------------
Matrix<6> feedbackLinearization(Matrix<10> x, Matrix<5,6> g, Matrix<5> h, Matrix<4> I_m,Matrix<6,10> K){
  unsigned long internalTime1;
  unsigned long internalTime2;
  Matrix<4> result1 = {0,0,0,0};
  internalTime1 = micros(); 
  Matrix<6,8> FT = computeForceAndTorque(x);
  internalTime1 = micros()-internalTime1;
  Matrix<3,4> Fm;
  Matrix<3,4> Fs;
  Matrix<2,4> Tm;
  Matrix<2,4> Ts;
  
  for(int i=0;i<4;i++){
    Fm(0,i) = FT(0,i);
    Fm(1,i) = FT(1,i);
    Fm(2,i) = FT(2,i);
    Tm(0,i) = FT(3,i);
    Tm(1,i) = FT(4,i);
    Fs(0,i) = FT(0,i+4);
    Fs(1,i) = FT(1,i+4);
    Fs(2,i) = FT(2,i+4);
    Ts(0,i) = FT(3,i+4);
    Ts(1,i) = FT(4,i+4);
  }
  internalTime2 = micros();
  Matrix<5,4> combined_m = Fm && Tm;
  Matrix<5,4> combined_s = Fs && Ts;
  Matrix<5> test = h-combined_m*I_m-g*K*x;
  result1 = Householder(combined_s,test);
  internalTime2 = micros()-internalTime2;
  Matrix<6> result = {result1(0),result1(1),result1(2),result1(3),0,0};
  result(4) = internalTime1;
  result(5) = internalTime2;

  return result;
}

//--------------------Used for computation time analysis-------------------------
double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

//----------------------Main----------------------------
void setup() {
  Serial.begin(115200);
  Serial.print('\n');
  Serial.print('\n');
  //Matrix<3,8*n> B = computeFieldBase(x,y,z);      //This more or less works, what remains is to solve the problem of numbers with a need of more than 2^-52 accuracy??? how does matlab do it? (resulting in tiny numbers having wrong sign)
  Matrix<5,6> g = {m_l,0,0,0,0,0,0,m_l,0,0,0,0,0,0,m_l,0,0,0,0,0,0,0.0000061686,0,0,0,0,0,0,0,0.0000061686};        //These matrices might need to be defined more precisely
  Matrix<5> h = {0,0,-9.81,0,0};
  Matrix<4> I_m = {J_p/mu*l_p,J_p/mu*l_p,J_p/mu*l_p,J_p/mu*l_p};
  Serial << g << '\n';
  Matrix<6,10> K = {20699.999,0,0,0,0,320,0,0,0,0,0,36000,0,0,0,0,420,0,0,0,0,0,50000,0,0,0,0,600,0,0,0,0,0,32000,0,0,0,0,420,0,0,0,0,0,32000,0,0,0,0,420,0,0,0,0,0,0,0,0,0,0};
  //startTime = micros();
  Matrix<3,1000> times;
  for(int i=0;i<1000;i++){
    Matrix<10> x = {randomDouble(-0.05,0.05),randomDouble(-0.05,0.05),randomDouble(-0.05,0.05),randomDouble(-1,1),randomDouble(-1,1),randomDouble(-0.015,0.015),randomDouble(-0.015,0.015),randomDouble(-0.015,0.015),randomDouble(-0.25,0.25),randomDouble(-0.25,0.25)};
    startTime = micros();
    Matrix<6> u = feedbackLinearization(x,g,h,I_m,K);
    times(0,i) = micros()-startTime;
    times(1,i) = u(4);
    times(2,i) = u(5);
    startTime = micros();
  }
  for(int i=0;i<1000;i++){
    Serial.print(times(0,i));
    Serial.print(", ");
  }
  Serial.println("");
  for(int i=0;i<1000;i++){
    Serial.print(times(1,i));
    Serial.print(", ");
  }
  Serial.println("");
  for(int i=0;i<1000;i++){
    Serial.print(times(2,i));
    Serial.print(", ");
  }
}

void loop() {

}
