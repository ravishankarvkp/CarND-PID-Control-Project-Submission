#ifndef PID_H
#define PID_H

class PID {

private:

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * private training parameters
   */
  double epochCumulativeError_; //Accumulates squares of CTEs to further compute RMSE
  double previousEpochError_;
  const double errorThreshold_ = 1e-3;
  const double learnRate_ = 1e-2;

  /** Unlike i_error, this accumulates absolute values of CTE,
  * avoiding positive-negative cancelling each other.
  */
  double i_e_fabs_error_;

public:

  /*
   * public training parameters
   */
  int counter_;
  bool needsTraining_;
  double currentEpochError_;
  const int epochLength_ = 50;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /**
  * computes coefficients adjustments according to total epoch delta error and their partial derivatives.
  * And adjusts accordingly.
  */
  void backProp();

  /**
  * Resets epoch error back to zero (for next epoch)
  */
  void resetEpochError();

  /**
  * getters for Kp, Ki, Kd.
  */
  double getKp() const;
  double getKi() const;
  double getKd() const;
};

#endif /* PID_H */
