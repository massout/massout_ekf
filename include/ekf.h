#pragma once

#include <stddef.h>

#include "matrix.h"

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
class ExtendedKalmanFilter {
   public:
    typedef Matrix<T, StateDim, 1> StateVector;
    typedef Matrix<T, StateDim, StateDim> StateMatrix;
    typedef Matrix<T, MeasurementDim, 1> MeasurementVector;
    typedef Matrix<T, MeasurementDim, StateDim> MeasurementMatrix;
    typedef Matrix<T, StateDim, ControlDim> ControlMatrix;
    typedef Matrix<T, ControlDim, 1> ControlVector;
    typedef Matrix<T, StateDim, MeasurementDim> GainMatrix;
    typedef Matrix<T, MeasurementDim, MeasurementDim> MeasurementCovariance;

    typedef void (*ProcessModel)(const StateVector&, const ControlVector&,
                                 StateVector&);
    typedef void (*ProcessJacobian)(const StateVector&, const ControlVector&,
                                    StateMatrix&);
    typedef void (*MeasurementModel)(const StateVector&, MeasurementVector&);
    typedef void (*MeasurementJacobian)(const StateVector&, MeasurementMatrix&);

    ExtendedKalmanFilter();

    void setState(const StateVector& state);
    const StateVector& state() const;

    void setCovariance(const StateMatrix& covariance);
    const StateMatrix& covariance() const;

    void setProcessNoise(const StateMatrix& noise);
    const StateMatrix& processNoise() const;

    void setMeasurementNoise(const MeasurementCovariance& noise);
    const MeasurementCovariance& measurementNoise() const;

    void setTransitionMatrix(const StateMatrix& matrix);
    const StateMatrix& transitionMatrix() const;

    void setControlMatrix(const ControlMatrix& matrix);
    const ControlMatrix& controlMatrix() const;

    void setMeasurementMatrix(const MeasurementMatrix& matrix);
    const MeasurementMatrix& measurementMatrix() const;

    const GainMatrix& kalmanGain() const;

    void setProcessModel(ProcessModel model);
    void setProcessJacobian(ProcessJacobian jacobian);
    void setMeasurementModel(MeasurementModel model);
    void setMeasurementJacobian(MeasurementJacobian jacobian);

    void predict();
    void predict(const ControlVector& control);
    bool update(const MeasurementVector& measurement);

   private:
    StateVector state_;
    StateMatrix covariance_;
    StateMatrix process_noise_;
    MeasurementMatrix measurement_matrix_;
    MeasurementCovariance measurement_noise_;
    StateMatrix transition_matrix_;
    ControlMatrix control_matrix_;
    GainMatrix kalman_gain_;

    ProcessModel process_model_;
    ProcessJacobian process_jacobian_;
    MeasurementModel measurement_model_;
    MeasurementJacobian measurement_jacobian_;
};
