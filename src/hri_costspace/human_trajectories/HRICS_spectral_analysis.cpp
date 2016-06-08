#include <HRICS_spectral_analysis.hpp>
#include <utils/misc_functions.hpp>
#include <unsupported/Eigen/FFT>
#include <HRICS_dynamic_time_warping.hpp>
#include <iomanip>

using std::cout;
using std::endl;

Eigen::VectorXd ComputeFFT(const Eigen::VectorXd& signal)
{
  Eigen::FFT<double> fft;
  Eigen::VectorXcd freqvec;
  fft.fwd(freqvec, signal);
  Eigen::VectorXd spectrum_magnitude(freqvec.size());
  for (int i = 0; i < spectrum_magnitude.size(); i++) {
    spectrum_magnitude[i] = std::abs(freqvec[i]);
  }
  return spectrum_magnitude;
}

Eigen::MatrixXcd ComputeFFT2(const Eigen::MatrixXd& signal)
{
  Eigen::FFT<double> fft;
  Eigen::MatrixXcd out(signal.rows(), signal.cols());

  for (int k = 0; k < signal.rows(); k++) {
    Eigen::VectorXcd tmpOut(signal.cols());
    Eigen::VectorXd in = signal.row(k);
    fft.fwd(tmpOut, in);
    out.row(k) = tmpOut;
  }

  for (int k = 0; k < out.cols(); k++) {
    Eigen::VectorXcd tmpOut(signal.rows());
    fft.fwd(tmpOut, out.col(k));
    out.col(k) = tmpOut;
  }

  return out;
}

double ComputeSAL(const Eigen::VectorXd& spectrum, double dt)
{
  //  double f_bin = 1. / (dt * double(spectrum.size()));
  //  cout << "f_bin : " << f_bin << endl;
  // Here put 2^round(K) + 4 (coresponds to approx 20Hz)
  int K_c = 200;  // Corresponds to 20 Hz id dt = 0.01 sec and K = 1000
  double v0 = spectrum[0];
  double k_const = std::pow(1. / double(K_c - 1), 2);
  double sal = 0.;
  for (int k = 1; k < K_c; k++) {
    double delta_v = (spectrum[k] - spectrum[k - 1]) / v0;
    sal += std::sqrt(k_const + std::pow(delta_v, 2));
  }
  return -1. * sal;
}

std::vector<double> FromEigen(const Eigen::VectorXd& x)
{
  std::vector<double> v(x.size());
  for (int i = 0; i < x.size(); i++) {
    v[i] = x[i];
  }
  return v;
}

Eigen::VectorXd ComputeSpeedProfile(const Eigen::MatrixXd& signal,
                                    bool is_transform,
                                    double dt)
{
  double sqrt_dt = std::sqrt(dt);
  Eigen::VectorXd speed(signal.cols());
  for (int t = 0; t < signal.cols() - 1; t++) {
    std::vector<double> P1 = FromEigen(signal.col(t + 1));
    std::vector<double> P2 = FromEigen(signal.col(t));
    double dist;
    if (is_transform) {
      dist = move3d_tansform_distance(P1, P2);
    } else {
      dist = move3d_centers_distance(P1, P2);
    }
    speed[t] = dist / sqrt_dt;
  }
  speed[signal.cols() - 1] = speed[signal.cols() - 2];
  return speed;
}

static int nb_ith = 0;
static bool save_to_file = false;

double sal_performance(const Move3D::Trajectory& trajectory,
                       const std::vector<Move3D::Joint*>& joints)
{
  double dt = trajectory.getDeltaTime();
  Eigen::MatrixXd traj;
  Eigen::VectorXd speed_profile;
  if (joints.size() == 1) {
    traj = trajectory.getJointPoseTrajectory(joints[0]);
    if (!is_finite(traj)) {
      cout << "Error in task demo trajectory" << endl;
    }
    speed_profile = ComputeSpeedProfile(traj, true, dt);
  } else {
    traj = trajectory.getJointPoseTrajectory(joints);
    if (!is_finite(traj)) {
      cout << "Error in joint center demo trajectory" << endl;
    }
    speed_profile = ComputeSpeedProfile(traj, false, dt);
  }

  Eigen::VectorXd speed_profile_padded = Eigen::VectorXd::Zero(1000);
  speed_profile_padded.segment(0, speed_profile.size()) = speed_profile;
  Eigen::VectorXd spectrum = ComputeFFT(speed_profile_padded);

  if (save_to_file) {
    std::stringstream ss;
    ss.str("");
    ss << "spectrum";
    ss << "_" << std::setw(5) << std::setfill('0') << nb_ith << ".csv";
    move3d_save_matrix_to_csv_file(spectrum, "sal_profiles/" + ss.str());
    ss.str("");
    ss << "speed_profile";
    ss << "_" << std::setw(5) << std::setfill('0') << nb_ith << ".csv";
    move3d_save_matrix_to_csv_file(speed_profile, "sal_profiles/" + ss.str());
    nb_ith++;
  }

  return ComputeSAL(spectrum, dt);
}

std::vector<double> sal_sequences_similarities(
    const Move3D::Trajectory& t0,
    const std::vector<Move3D::Trajectory>& t_tests,
    const std::vector<Move3D::Joint*>& joints)
{
  std::vector<double> scost;
  double sal0 = sal_performance(t0, joints);
  // for all test trajectories
  scost.resize(t_tests.size());
  for (size_t i = 0; i < scost.size(); i++) {

    // If this is positive then the motion is
    // the larger the score the more smooth
    // if > then it is more smooth than the original motion
    scost[i] = 100. * ((sal_performance(t_tests[i], joints) - sal0) / sal0);
  }
  return scost;
}
