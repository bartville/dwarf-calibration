#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "dwarf-utils.h"

namespace dwarf_calibration {

  class DwarfSolver{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DwarfSolver();
    ~DwarfSolver(){
      /*nothing to destroy for this dwarf*/
    }

    // set stuff
    inline void setInitialGuess(const Eigen::Isometry3f& init_guess){
      _T = init_guess;
    }
    
    inline void setDataset(SamplePairVector* dataset){
      _dataset = dataset;
      _dataset_size = _dataset->size();
    }

    inline void setIterations(const int iterations){
      _iterations = iterations;
    }
    
    // get stuff
    Eigen::Isometry3f T(){
      return _T;
    }

    //compute
    void solve();  
    
  private:
    Eigen::Isometry3f _T;
    SamplePairVector* _dataset;
    int _dataset_size;
    int _iterations;

    Matrix6f _H;
    Vector6f _b;
    float _chi2;

    void updateSolution(const Vector6f& delta);
    Vector6f computeError(const Eigen::Isometry3f& guess,
			  const Sample& reference,
			  const Sample& measure);
    Vector6f computeDisturbedError(const int id,
				   const float& epsilon,
				   const Sample& reference,
				   const Sample& measure);
    void linearize();
    

  };

}
