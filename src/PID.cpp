#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {  
  _p_error = 0.0;
  _i_error = 0.0;
  _d_error = 0.0;

  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}

void PID::UpdateError(double cte) {
    // 1st update the differential error
    _d_error = cte - _p_error;
    _i_error = _i_error + cte;
    _p_error = cte;

}

double PID::TotalError() {
    return - _Kp * _p_error - _Ki * _i_error - _Kd * _d_error;
}

//def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
//    x_trajectory = []
//    y_trajectory = []
//    prev_cte = robot.y
//    int_cte = 0
//    for i in range(n):
//        cte = robot.y
//        diff_cte = cte - prev_cte
//        prev_cte = cte
//        int_cte += cte
//        steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
//        robot.move(steer, speed)
//        x_trajectory.append(robot.x)
//        y_trajectory.append(robot.y)
//    return x_trajectory, y_trajectory


//def twiddle(tol=0.2):
//    p = [0, 0, 0]
//    dp = [1, 1, 1]
//    robot = make_robot()
//    x_trajectory, y_trajectory, best_err = run(robot, p)

//    it = 0
//    while sum(dp) > tol:
//        print("Iteration {}, best error = {}".format(it, best_err))
//        for i in range(len(p)):
//            p[i] += dp[i]
//            robot = make_robot()
//            x_trajectory, y_trajectory, err = run(robot, p)

//            if err < best_err:
//                best_err = err
//                dp[i] *= 1.1
//            else:
//                p[i] -= 2 * dp[i]
//                robot = make_robot()
//                x_trajectory, y_trajectory, err = run(robot, p)

//                if err < best_err:
//                    best_err = err
//                    dp[i] *= 1.1
//                else:
//                    p[i] += dp[i]
//                    dp[i] *= 0.9
//        it += 1
//    return p
