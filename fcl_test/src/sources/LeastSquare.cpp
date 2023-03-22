
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


void least_square(Eigen::MatrixXd A ,Eigen::VectorXd B, Eigen::VectorXd& Result_, int Number_Point, double dt)
{

    cout << "The solution using normal equations is:\n"
        << (A.transpose() * A).ldlt().solve(A.transpose() * B) << endl;
        Result_ = (A.transpose() * A).ldlt().solve(A.transpose() * B);

}


int main()
{
    int Number_Point = 2;
    double dt = 0.01;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(Number_Point, 2);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(Number_Point);
    Eigen::VectorXd Result_;

    for(int i=0; i < Number_Point ; i++)
    {
      A(i,0) = 1;
      A(i,1) = (i+1) * dt;
          
    }

    for(int i=0; i < Number_Point ; i++)
    {
      B(i) = 0.2 + 0.1 * i;
          
    }

    cout<<"A: "<<endl<<A<<endl;
    cout<<"B: "<<endl<<B<<endl;
    
    least_square(A, B, Result_, Number_Point, dt);

    std::cout << "Result_ = " << Result_ <<std::endl;

    
    return 0;


}
