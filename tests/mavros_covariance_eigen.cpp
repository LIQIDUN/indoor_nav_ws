// Reproduce MAVROS 1.20.1 odom.cpp dense covariance rotation (offline).
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <limits>
int main() {
    using Matrix = Eigen::Matrix<double,6,6>;
    Matrix covariance=Matrix::Zero(), rotation=Matrix::Identity();
    rotation(0,0)=0; rotation(1,1)=0;
    rotation(0,1)=1; rotation(1,0)=1; rotation(2,2)=-1;
    covariance(0,0)=.01;covariance(1,1)=.04;covariance(2,2)=1000;
    Matrix transformed=rotation*covariance*rotation.transpose();
    assert(std::abs(transformed(0,0)-.04)<1e-12);
    assert(transformed(2,2)==1000);
    covariance(0,0)=std::numeric_limits<double>::quiet_NaN();
    transformed=rotation*covariance*rotation.transpose();
    // First element of the MAVLink upper-right triangle retains unknown sentinel.
    assert(std::isnan(transformed(0,0)));
    assert(std::isnan(transformed(2,2)));
}
