#include "ekf.h"

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                     ControlDim>::ExtendedKalmanFilter() {
    state_.setZero();
    covariance_.setIdentity();
    process_noise_.setIdentity();
    transition_matrix_.setIdentity();
    measurement_matrix_.setZero();
    measurement_noise_.setIdentity();
    control_matrix_.setZero();
    kalman_gain_.setZero();
    process_model_ = static_cast<ProcessModel>(0);
    process_jacobian_ = static_cast<ProcessJacobian>(0);
    measurement_model_ = static_cast<MeasurementModel>(0);
    measurement_jacobian_ = static_cast<MeasurementJacobian>(0);
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::setState(
    const StateVector& state) {
    state_ = state;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::StateVector&
ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::state() const {
    return state_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setCovariance(const StateMatrix& covariance) {
    covariance_ = covariance;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::StateMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::covariance()
    const {
    return covariance_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setProcessNoise(const StateMatrix& noise) {
    process_noise_ = noise;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::StateMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::processNoise()
    const {
    return process_noise_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setMeasurementNoise(const MeasurementCovariance& noise) {
    measurement_noise_ = noise;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::MeasurementCovariance&
ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                     ControlDim>::measurementNoise() const {
    return measurement_noise_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setTransitionMatrix(const StateMatrix& matrix) {
    transition_matrix_ = matrix;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::StateMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                     ControlDim>::transitionMatrix() const {
    return transition_matrix_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setControlMatrix(const ControlMatrix& matrix) {
    control_matrix_ = matrix;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::ControlMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::controlMatrix()
    const {
    return control_matrix_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setMeasurementMatrix(const MeasurementMatrix& matrix) {
    measurement_matrix_ = matrix;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::MeasurementMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                     ControlDim>::measurementMatrix() const {
    return measurement_matrix_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
const typename ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                                    ControlDim>::GainMatrix&
ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::kalmanGain()
    const {
    return kalman_gain_;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim,
                          ControlDim>::setProcessModel(ProcessModel model) {
    process_model_ = model;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setProcessJacobian(ProcessJacobian jacobian) {
    process_jacobian_ = jacobian;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setMeasurementModel(MeasurementModel model) {
    measurement_model_ = model;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::
    setMeasurementJacobian(MeasurementJacobian jacobian) {
    measurement_jacobian_ = jacobian;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::predict() {
    ControlVector control;
    predict(control);
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
void ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::predict(
    const ControlVector& control) {
    StateVector predicted_state;

    if (process_model_) {
        process_model_(state_, control, predicted_state);
    } else {
        predicted_state = transition_matrix_ * state_;
        if (ControlDim > 0U) {
            StateVector control_effect = control_matrix_ * control;
            predicted_state += control_effect;
        }
    }

    StateMatrix transition_jacobian;
    if (process_jacobian_) {
        process_jacobian_(state_, control, transition_jacobian);
    } else {
        transition_jacobian = transition_matrix_;
    }

    StateMatrix covariance_prediction =
        transition_jacobian * covariance_ * transition_jacobian.transpose();
    covariance_ = covariance_prediction + process_noise_;
    state_ = predicted_state;
}

template <typename T, size_t StateDim, size_t MeasurementDim, size_t ControlDim>
bool ExtendedKalmanFilter<T, StateDim, MeasurementDim, ControlDim>::update(
    const MeasurementVector& measurement) {
    MeasurementVector expected_measurement;
    if (measurement_model_) {
        measurement_model_(state_, expected_measurement);
    } else {
        expected_measurement = measurement_matrix_ * state_;
    }

    MeasurementMatrix measurement_jacobian;
    if (measurement_jacobian_) {
        measurement_jacobian_(state_, measurement_jacobian);
    } else {
        measurement_jacobian = measurement_matrix_;
    }

    MeasurementVector innovation = measurement - expected_measurement;

    MeasurementCovariance innovation_covariance =
        measurement_jacobian * covariance_ * measurement_jacobian.transpose();
    innovation_covariance += measurement_noise_;

    MeasurementCovariance innovation_inverse;
    if (!innovation_covariance.inverse(innovation_inverse)) {
        return false;
    }

    kalman_gain_ =
        covariance_ * measurement_jacobian.transpose() * innovation_inverse;

    state_ = state_ + kalman_gain_ * innovation;
    StateMatrix identity = StateMatrix::Identity();
    covariance_ =
        (identity - kalman_gain_ * measurement_jacobian) * covariance_;
    return true;
}
