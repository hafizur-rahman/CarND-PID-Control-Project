#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    twiddle = false;
    current_step = 1;
    best_error = std::numeric_limits<double>::max();
    param_index = 0;

    dp[0] = Kp;
    dp[1] = Kd;
}

void PID::UpdateError(double cte) {    
    i_error += cte;
    d_error = cte - p_error;

    p_error = cte;

    current_step++;

    int step_size = 1000;

    const double tolerance = 0.2;
    if (twiddle && current_step % step_size == 0) {
        UpdateCoefficient(param_index, dp[param_index % 2]);

        double total_error = -(p_error * Kp + i_error * Ki + d_error * Kd);
        // if (total_error < tolerance){
        //     twiddle = false;
        // } else {
            if (total_error < best_error) {
                std::cout << "Kp: " << Kp << ", Kd: " << Kd << ", error: " << total_error << endl;

                best_error = total_error;            
                if (current_step > step_size) dp[param_index % 2] *= 1.1;            
            } else {
                UpdateCoefficient(param_index, -2 * dp[param_index % 2]);

                total_error = -(p_error * Kp + i_error * Ki + d_error * Kd);
                if (total_error < best_error) {
                    std::cout << "Kp: " << Kp << ", Kd: " << Kd << ", error: " << total_error << endl;

                    best_error = total_error;            
                    dp[param_index % 2] *= 1.1;            
                } else {
                    UpdateCoefficient(param_index, dp[param_index % 2]);
                    
                    dp[param_index % 2] *= 0.9;
                }
            }
            
            param_index = (param_index + 1) % 2;
        //}
    }

    
}

double PID::TotalError() {
    return -(p_error * Kp + i_error * Ki + d_error * Kd);        
}

void PID::UpdateCoefficient(int index, double delta) {
    if (index % 2 == 0) {
        Kp += delta;
    } else {
        Kd += delta;
    }
}

