#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Matrix2f m2;
    m2 << 1, 2 ,3, 4;
    std::cout << m2.data()[1] << std::endl;
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
  
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    
    std::cout << "Example of vector dot products \n";
    auto v2 = v.transpose();
    
//std::cout << w * v << std::endl; // error
    std::cout << w * v2 << std::endl; // 3*3,
    std::cout << v2 * w << std::endl; // 1
    
    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << i + j << std::endl << std::endl;
    std::cout << i * 2 << std::endl << std::endl;
    std::cout << i * j << std::endl << std::endl;
    std::cout << i * v << std::endl << std::endl; // 14 32 50
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    using namespace Eigen;
    
    Vector3f p(2, 1, 1);
    auto theta = EIGEN_PI / 4;
    Matrix3f r, t;
    r << cosf(theta), -sinf(theta), 0, sinf(theta), cosf(theta), 0,
               0, 0, 1;
    t << 1, 0, 1, 0, 1, 2, 0, 0, 1;
    auto pPrime = t * r * p;
    std::cout << pPrime << std::endl;
    
    return 0;
}
