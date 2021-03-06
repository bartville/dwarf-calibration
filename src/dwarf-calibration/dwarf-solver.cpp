#include "dwarf-solver.h"

namespace dwarf_calibration {

  //!constructor
  //!
  DwarfSolver::DwarfSolver(){
    _T.setIdentity();
    _dataset = 0;
    _dataset_size = 0;
    _iterations = 20;
    
    _H.setZero();
    _b.setZero();
    _chi2 = 0.f;
    std::cerr << "[DwarfSolver]: a new Dwarf is born!" << std::endl;
    
  }

  //!compute the error function for our 3d calibration problem
  //!@param guess: a guess isometry
  //!@param reference: reference, i.e. a sample of the relative trajectory #1 
  //!@param measure: measure, i.e. a sample of the relative trajectory #2 
  //!@returns the error computed as a Vector6
  //!
  Vector6f DwarfSolver::computeError(const Eigen::Isometry3f& guess,
				     const Sample& reference,
				     const Sample& measure) {
    Eigen::Isometry3f transf_estimate = guess.inverse() * reference._T * guess;
    Eigen::Isometry3f error_iso = transf_estimate.inverse() * measure._T;
    return t2v(error_iso);   
  }

  //!compute the error function applying an epsilon noise to the guess
  //!@param id: id of the coordinate to apply the noise
  //!@param epsilon: noise amount, its sign may vary, since the noise may be positive or negative
  //!@param reference: reference, i.e. a sample of the relative trajectory #1 
  //!@param measure: measure, i.e. a sample of the relative trajectory #2 
  //!@returns the error computed as a Vector6
  //!
  Vector6f DwarfSolver::computeDisturbedError(const int id,
					      const float& epsilon,
					      const Sample& reference,
					      const Sample& measure) {
    Vector6f disturb;
    disturb.setZero();
    disturb(id) = epsilon;
    Eigen::Isometry3f guess = _T;
    guess = guess * v2t(disturb);
    return computeError(guess, reference, measure);    
  }
  

  //!performs one step of gauss-newton(levemberg-marqardt) algorithm
  //!
  void DwarfSolver::linearize(){
    _H.setZero();
    _b.setZero();
    _chi2 = 0;

    const float epsilon = 1e-6;
    const float i_epsilon = 1./epsilon;
    
    SamplePairVector::iterator data_iterator;
    for(data_iterator = _dataset->begin();
	data_iterator != _dataset->end();
	++data_iterator) {

      Sample reference = (*data_iterator).first;
      Sample measure = (*data_iterator).second;

      Vector6f e = computeError(_T, reference, measure);

      Matrix6f J;
      J.setZero();
      for(int i=0; i < 6; ++i){
	J.col(i) = i_epsilon * (computeDisturbedError(i, epsilon,reference,measure) - computeDisturbedError(i,-epsilon,reference,measure));
      }

      _H += J.transpose() * J;
      _b += J.transpose() * e;
      _chi2 += e.transpose() * e; 
    }    
  }

  //!update the solution, by applying a delta correction
  //!@params delta: delta solution of the levemberg-marqardt algorithm 
  //!
  void DwarfSolver::updateSolution(const Vector6f& delta){
    _T = _T * v2t(delta); //check this
    //std::cerr << "T: " << t2v(_T).transpose() << std::endl;
  }

  //!performs N iterations of L-M algorithm, updating the solution
  //!  
  void DwarfSolver::solve(){
    if(_dataset && _dataset_size)
      std::cerr << "[DwarfSolver]: dwarf solve!" << std::endl;
    else
      throw std::runtime_error("[DwarfSolver]: this Dwarf is angry! Where are my data!?!?");
    
    for(size_t i=0; i<_iterations; ++i){
      linearize();
      std::cerr << "chi2: " << _chi2/_dataset_size << std::endl;
      Vector6f delta_X = _H.llt().solve(-_b);
      updateSolution(delta_X);
    }        
  }
  
  
}
