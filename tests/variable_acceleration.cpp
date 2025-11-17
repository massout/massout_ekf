#include <cstdio>

#include "ekf.h"

typedef ExtendedKalmanFilter<float, 2, 1, 1> AccelerationFilter;

static float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

int main() {
    AccelerationFilter filter;

    AccelerationFilter::StateVector state;
    state.setZero();
    filter.setState(state);

    AccelerationFilter::StateMatrix covariance =
        AccelerationFilter::StateMatrix::Identity();
    covariance *= 25.0f;
    filter.setCovariance(covariance);

    const float dt = 1.0f;

    AccelerationFilter::StateMatrix transition =
        AccelerationFilter::StateMatrix::Identity();
    transition(0, 1) = dt;
    filter.setTransitionMatrix(transition);

    AccelerationFilter::ControlMatrix control_matrix;
    control_matrix.setZero();
    control_matrix(0, 0) = 0.5f * dt * dt;
    control_matrix(1, 0) = dt;
    filter.setControlMatrix(control_matrix);

    AccelerationFilter::StateMatrix process_noise =
        AccelerationFilter::StateMatrix::Identity();
    process_noise *= 0.05f;
    filter.setProcessNoise(process_noise);

    AccelerationFilter::MeasurementMatrix measurement_matrix;
    measurement_matrix.setZero();
    measurement_matrix(0, 0) = 1.0f;
    filter.setMeasurementMatrix(measurement_matrix);

    AccelerationFilter::MeasurementCovariance measurement_noise =
        AccelerationFilter::MeasurementCovariance::Identity();
    measurement_noise *= 0.25f;
    filter.setMeasurementNoise(measurement_noise);

    AccelerationFilter::ControlVector control;
    control.setZero();

    float true_position = 0.0f;
    float true_velocity = 0.0f;

    static const float acceleration_sequence[] = {
        0.4f,  0.6f,  0.8f,  1.0f,  1.0f,  0.8f,  0.6f,  0.4f,  0.2f,  0.0f,
        -0.2f, -0.4f, -0.6f, -0.8f, -1.0f, -1.0f, -0.8f, -0.6f, -0.4f, -0.2f,
        0.0f,  0.2f,  0.4f,  0.6f,  0.8f,  1.0f,  0.8f,  0.6f,  0.4f};

    const size_t kSteps =
        sizeof(acceleration_sequence) / sizeof(acceleration_sequence[0]);

    for (size_t step = 0; step < kSteps; ++step) {
        const float acceleration = acceleration_sequence[step];
        control(0, 0) = acceleration;

        true_position += true_velocity * dt + 0.5f * acceleration * dt * dt;
        true_velocity += acceleration * dt;

        AccelerationFilter::MeasurementVector measurement;
        measurement.setZero();
        measurement(0, 0) = true_position;

        filter.predict(control);
        if (!filter.update(measurement)) {
            std::printf("Innovation covariance was singular at step %zu\n",
                        step);
            return 1;
        }
    }

    const AccelerationFilter::StateVector& estimate = filter.state();
    const float position_error = estimate(0, 0) - true_position;
    const float velocity_error = estimate(1, 0) - true_velocity;

    if (absoluteValue(position_error) > 0.5f ||
        absoluteValue(velocity_error) > 0.5f) {
        std::printf(
            "Variable acceleration test failed: position=%f velocity=%f\n",
            position_error, velocity_error);
        return 1;
    }

    std::printf(
        "Variable acceleration final estimate position=%f velocity=%f\n",
        estimate(0, 0), estimate(1, 0));
    return 0;
}