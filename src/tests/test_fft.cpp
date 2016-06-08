#include <unsupported/Eigen/FFT>
#include <utils/misc_functions.hpp>
#include <hri_costspace/human_trajectories/HRICS_spectral_analysis.hpp>

using std::cout;
using std::endl;

Eigen::VectorXd LoadTimeSequence()
{
  Eigen::MatrixXd y_tmp = move3d_load_matrix_from_csv_file("y.csv");
  cout << "y_tmp : " << y_tmp.size() << endl;

  Eigen::VectorXd y(y_tmp.rows());
  for (int i = 0; i < y.size(); i++) {
    y[i] = y_tmp(i, 0);
  }
  return y;
}

void SaveFrequency(const Eigen::VectorXcd& freq)
{
  cout << "freq : " << freq.transpose() << endl;
}

int main(int argc, char* argv[])
{
  Eigen::FFT<double> fft;
  Eigen::VectorXd timevec = LoadTimeSequence();
  Eigen::VectorXcd freqvec;

  fft.fwd(freqvec, timevec);
  SaveFrequency(freqvec);

  // manipulate freqvec
  fft.inv(timevec, freqvec);

  //

  Eigen::MatrixXd x = move3d_load_matrix_from_csv_file("time_serie.csv");
  Eigen::MatrixXd y_1 = move3d_load_matrix_from_csv_file("spectrum.csv");
  Eigen::MatrixXcd y_2 = ComputeFFT2(x);

  cout << "*** 1 *** " << endl
       << y_1 << endl;
  cout << "*** 2 *** " << endl
       << y_2.cwiseAbs() << endl;

  cout << "DIST : " << (y_1 - y_2.cwiseAbs()).norm() << endl;
}
