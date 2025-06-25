#ifndef PID_H
#define PID_H

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/**
 *  PID controller class
 */
class PIDController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    float operator() (float error);
    void reset();

    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value
    float output_prev;  //!< last pid output value

protected:
    float error_prev; //!< last tracking error value
    float integral_prev; //!< last integral component value
    int64_t timestamp_prev; //!< Last execution timestamp
};

#endif // PID_H