#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 80;
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

double cost_cte = 3000.0;
double cost_epsi = 3000.0;
double cost_v = 1.0;
double cost_delta = 5.0;
double cost_a = 5.0;
double cost_deltav = 700.0;
double cost_deltadot = 200.0;
double cost_adot = 10.0;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    // Make the error cost high for cte, epsi and diff from ref_v.
    for (int i = 0; i < N; i++)
    {
      fg[0] += cost_cte*CppAD::pow(vars[cte_start + i], 2);
      fg[0] += cost_epsi*CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += cost_v*CppAD::pow(vars[v_start + i] - ref_v, 2);
    }
    //  Price the  use of actuators high.
    for (int i = 0; i < N - 1; i++)
    {
      fg[0] += cost_delta*CppAD::pow(vars[delta_start + i], 2);
      fg[0] += cost_a*CppAD::pow(vars[a_start + i], 2);
      // Price the product of steering action and velocity
      fg[0] += cost_deltav*CppAD::pow(vars[delta_start + i] * vars[v_start + i], 2);
//      fg[0] += 10*CppAD::pow(CppAD::cos(vars[psi_start + i]) * vars[v_start + i], 1);
    }
    // Price the gap between sequential actuations high.
    for (int i = 0; i < N - 2; i++)
    {
      fg[0] += cost_deltadot*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += cost_adot*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> delta = vars[delta_start + t - 1];
      AD<double> a = vars[a_start + t - 1];
      //
      // Account for delay is seeing result of action to be 1 cycle behind.
      //
      if (t > 1)
      {   
        a = vars[a_start + t - 2];
        delta = vars[delta_start + t - 2];
      }
      // Diffs.
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);
      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      // TODO: Setup the rest of the model constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta * dt);
      fg[1 + v_start + t] = v1 - (v0 + a * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0/Lf * delta * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];
  //
  // Incorporate delay. Predict state after latency and pass that to the solver
  //
  static double last_delta = 0;
  static double last_a = 0;
  // Does not work as well as accounting for it in constraint above so set to 0.
  double delay = 0.0;
  double x_new = x + v*cos(psi)*delay;
  double y_new = y + v*sin(psi)*delay;
  double psi_new = psi + v / Lf * last_delta * delay;
  double v_new = v + last_a * delay;
  double cte_new = cte + v * sin(epsi) * delay;
  double epsi_new = epsi + v / Lf * last_delta * delay;

  x = x_new;
  y = y_new;
  psi = psi_new;
  v = v_new;
  cte = cte_new;
  epsi = epsi_new;

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  cost_cte = costfactor[0];
  cost_epsi = costfactor[1];
  cost_v = costfactor[2];
  cost_delta = costfactor[3];
  cost_a = costfactor[4];
  cost_deltav = costfactor[5];
  cost_deltadot = costfactor[6];
  cost_adot = costfactor[7];
  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options
  std::string options;
  options += "Integer print_level  1\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  options += "Numeric max_cpu_time        0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  //
  // Check some of the solution values
  //
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  traj_x.clear();
  traj_y.clear();
  last_delta = solution.x[delta_start];
  last_a = solution.x[a_start];
  for(int i = 0; i < N-1; i++)
  {
    traj_x.push_back(solution.x[x_start + i + 1]);
    traj_y.push_back(solution.x[y_start + i + 1]);
  }
  auto cost = solution.obj_value;
  std::cout << "Cost \t" << cost;
  std::cout << " X \t" << solution.x[x_start + 1];
  std::cout << " Y \t" << solution.x[y_start + 1];
  std::cout << " PSI \t" << solution.x[psi_start + 1];
  std::cout << " V \t" << solution.x[v_start + 1];
  std::cout << " CTE \t" << solution.x[cte_start + 1];
  std::cout << " EPSI \t" << solution.x[epsi_start + 1];
  std::cout << " delta \t" << solution.x[delta_start];
  std::cout << " a \t" << solution.x[a_start];
  std::cout << std::endl;
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}
