#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dwarf_calibration {

  typedef Eigen::Matrix<float,6,6> Matrix6f;
  typedef Eigen::Matrix<float,6,1> Vector6f;

  struct Sample{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Isometry3f _T;
  };
  
  typedef std::pair<Sample, Sample> SamplePair;
  typedef std::vector<SamplePair> SamplePairVector;


  //!converts from Vector6<Derived1> to Isometry3<Derived1>
  //!@param v: a vector (vx, vy, vz, qx, qy, qz) representing the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized quaternion, with qw>0.
  //!@returns the Isometry corresponding to the transform described by v
  //!
  template <typename Derived1>
    inline Eigen::Transform<Derived1,3,Eigen::Isometry> v2t(const Eigen::Matrix<Derived1, 6, 1>& v)
    {
      Eigen::Transform<Derived1,3,Eigen::Isometry> T; //return an homogeneous transformation in a 3-dim space, aka 4x4
      T.template setIdentity();
      T.template translation() = v.template head<3>();
      Derived1 w = v.template block<3,1>(3,0).squaredNorm();
      if (w < (Derived1)1){
        w = sqrt(1 - w);
        Eigen::Quaternion<Derived1> q(w, v(3), v(4), v(5));
        T.template linear() = q.template toRotationMatrix();
      } else{
        Eigen::Matrix<Derived1, 3, 1> qv = v.template block<3,1>(3,0);
        qv.template normalize();
        Eigen::Quaternion<Derived1> q(0, qv(0), qv(1), qv(2));
        T.template linear() = q.template toRotationMatrix();
      }
      return T;
    }

  //!converts from Isometry3<Derived1 to Vector6<Derived1>
  //!@param t: an isometry
  //!@returns a vector (tx, ty, tz, qx, qy, qz) reptesenting the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized queternion, with qw>0.
  template <typename Derived1>
    inline Eigen::Matrix<Derived1, 6, 1> t2v(const Eigen::Transform<Derived1,3,Eigen::Isometry>& t){
    Eigen::Matrix<Derived1, 6, 1> v;
    v.template head<3>()= t.template translation();
    Eigen::Quaternion<Derived1> q(t.template linear());
    v(3) = q.template x();
    v(4) = q.template y();
    v(5) = q.template z();
    if (q.template w()<(Derived1)0)
      v.template block<3,1>(3,0) *= (Derived1)(-1);
    return v;
  }

  
  
}
