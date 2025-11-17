#include <cstdio>

#include "ekf.h"

typedef ExtendedKalmanFilter<float, 2, 1, 1> TestFilter;

static float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

int main() {
    TestFilter filter;

    TestFilter::StateVector state;
    state.setZero();
    filter.setState(state);

    TestFilter::StateMatrix covariance = TestFilter::StateMatrix::Identity();
    covariance *= 100.0f;
    filter.setCovariance(covariance);

    TestFilter::StateMatrix transition = TestFilter::StateMatrix::Identity();
    transition(0, 1) = 1.0f;
    filter.setTransitionMatrix(transition);

    TestFilter::StateMatrix process_noise = TestFilter::StateMatrix::Identity();
    process_noise *= 0.05f;
    filter.setProcessNoise(process_noise);

    TestFilter::MeasurementMatrix measurement_matrix;
    measurement_matrix.setZero();
    measurement_matrix(0, 0) = 1.0f;
    filter.setMeasurementMatrix(measurement_matrix);

    TestFilter::MeasurementCovariance measurement_noise =
        TestFilter::MeasurementCovariance::Identity();
    measurement_noise *= 0.25f;
    filter.setMeasurementNoise(measurement_noise);

    TestFilter::ControlMatrix control_matrix;
    control_matrix.setZero();
    filter.setControlMatrix(control_matrix);

    TestFilter::ControlVector control;
    control.setZero();

    float true_position = 0.0f;
    float true_velocity = 1.0f;

    for (int step = 0; step < 20; ++step) {
        true_position += true_velocity;

        TestFilter::MeasurementVector measurement;
        measurement.setZero();
        measurement(0, 0) = true_position;

        filter.predict(control);
        if (!filter.update(measurement)) {
            std::printf("Innovation covariance was singular at step %d\n",
                        step);
            return 1;
        }
    }

    const TestFilter::StateVector& estimate = filter.state();
    float position_error = estimate(0, 0) - true_position;
    float velocity_error = estimate(1, 0) - true_velocity;

    if (absoluteValue(position_error) > 0.1f ||
        absoluteValue(velocity_error) > 0.2f) {
        std::printf("EKF estimate error too large: position=%f velocity=%f\n",
                    position_error, velocity_error);
        return 1;
    }

    std::printf("Final estimate position=%f velocity=%f\n", estimate(0, 0),
                estimate(1, 0));
    return 0;
}