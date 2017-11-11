#include <dwarf-calibration/dwarf-solver.h>

using namespace dwarf_calibration;


Eigen::Isometry3f randomRelativeIso() {
  const float tmin = 0.02;
  const float tmax = 0.1;
  float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
  float y  =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
  float z  =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
  float rx =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
  float ry =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
  float rz =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/tmax));
    Eigen::Isometry3f rand_iso;
  rand_iso.setIdentity();
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
  rand_iso.linear() = m;
  rand_iso.translation() = Eigen::Vector3f(x,y,z);
  return rand_iso;
}

int main(int argv, char** argc){
  // create a fake dataset
  SamplePairVector dataset;
  Eigen::Isometry3f T;
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ());
  T.linear() = m;
  T.translation() = Eigen::Vector3f(0.1, 0.1, 0.1);
  const int data_size = 10;
  dataset.resize(data_size);
  srand (static_cast <unsigned> (time(NULL)));

  for(int i=0; i < data_size; ++i) {
    Sample ref, curr;
    ref._T = randomRelativeIso();
    //std::cerr << "sample: " << t2v(ref._T).transpose() << std::endl;
    curr._T = T.inverse()*ref._T*T;
    SamplePair data_sample;
    data_sample.first = ref;
    data_sample.second = curr;
    dataset[i] = data_sample;    
  }
  std::cerr << "[Random Dataset Generated]!" << std::endl;
  // instantiate a solver
  DwarfSolver solver;

  solver.setDataset(&dataset);
  
  //define an initial guess
  Eigen::Isometry3f initial_guess;
  initial_guess.setIdentity();
  solver.setInitialGuess(initial_guess);

  const int iterations = 30;
  solver.setIterations(iterations);
  
  solver.solve();
  
  Eigen::Isometry3f solution = solver.T();
  std::cerr << "Dwarf Solution:  " << t2v(solution).transpose() << std::endl;
  std::cerr << "Actual Solution: " << t2v(T).transpose() << std::endl;

  
  return 0;
}
