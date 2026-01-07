#include <cstdio>

#include "ekf.h"

typedef ExtendedKalmanFilter<float, 2, 1, 0> IndependentFilter;

static float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

int main() {
    IndependentFilter filter_a;
    IndependentFilter filter_b;

    IndependentFilter::StateVector state_a;
    state_a.setZero();
    state_a(0, 0) = 0.0f;
    state_a(1, 0) = 1.0f;
    filter_a.setState(state_a);

    IndependentFilter::StateVector state_b;
    state_b.setZero();
    state_b(0, 0) = 10.0f;
    state_b(1, 0) = -1.0f;
    filter_b.setState(state_b);

    IndependentFilter::StateMatrix transition = IndependentFilter::StateMatrix::Identity();
    transition(0, 1) = 1.0f;
    filter_a.setTransitionMatrix(transition);
    filter_b.setTransitionMatrix(transition);

    IndependentFilter::StateMatrix process_noise = IndependentFilter::StateMatrix::Identity();
    process_noise *= 0.05f;
    filter_a.setProcessNoise(process_noise);
    filter_b.setProcessNoise(process_noise);

    IndependentFilter::MeasurementMatrix measurement_matrix;
    measurement_matrix.setZero();
    measurement_matrix(0, 0) = 1.0f;
    filter_a.setMeasurementMatrix(measurement_matrix);
    filter_b.setMeasurementMatrix(measurement_matrix);

    IndependentFilter::MeasurementCovariance measurement_noise = IndependentFilter::MeasurementCovariance::Identity();
    measurement_noise *= 0.1f;
    filter_a.setMeasurementNoise(measurement_noise);
    filter_b.setMeasurementNoise(measurement_noise);

    IndependentFilter::StateMatrix covariance_a = IndependentFilter::StateMatrix::Identity();
    covariance_a *= 10.0f;
    filter_a.setCovariance(covariance_a);

    IndependentFilter::StateMatrix covariance_b = IndependentFilter::StateMatrix::Identity();
    covariance_b *= 5.0f;
    filter_b.setCovariance(covariance_b);

    float true_position_a = 0.0f;
    float true_velocity_a = 1.0f;
    float true_position_b = 10.0f;
    float true_velocity_b = -1.0f;

    for (int step = 0; step < 30; ++step) {
        true_position_a += true_velocity_a;
        true_position_b += true_velocity_b;

        IndependentFilter::MeasurementVector measurement_a;
        measurement_a.setZero();
        measurement_a(0, 0) = true_position_a;

        IndependentFilter::MeasurementVector measurement_b;
        measurement_b.setZero();
        measurement_b(0, 0) = true_position_b;

        filter_a.predict();
        filter_b.predict();

        if (!filter_a.update(measurement_a)) {
            std::printf("Filter A failed to update at step %d\n", step);

            return 1;
        }

        if (!filter_b.update(measurement_b)) {
            std::printf("Filter B failed to update at step %d\n", step);

            return 1;
        }
    }

    const IndependentFilter::StateVector& estimate_a = filter_a.state();
    const IndependentFilter::StateVector& estimate_b = filter_b.state();

    const float error_a_position = estimate_a(0, 0) - true_position_a;
    const float error_a_velocity = estimate_a(1, 0) - true_velocity_a;
    const float error_b_position = estimate_b(0, 0) - true_position_b;
    const float error_b_velocity = estimate_b(1, 0) - true_velocity_b;

    if (absoluteValue(error_a_position) > 0.1f || absoluteValue(error_a_velocity) > 0.2f) {
        std::printf("Filter A error too large: position=%f velocity=%f\n", error_a_position, error_a_velocity);

        return 1;
    }

    if (absoluteValue(error_b_position) > 0.1f || absoluteValue(error_b_velocity) > 0.2f) {
        std::printf("Filter B error too large: position=%f velocity=%f\n", error_b_position, error_b_velocity);

        return 1;
    }

    if (absoluteValue(estimate_a(0, 0) - estimate_b(0, 0)) < 1.0f && absoluteValue(estimate_a(1, 0) - estimate_b(1, 0)) < 1.0f) {
        std::printf("Filters converged to similar states\n");
    } else {
        std::printf("Filters maintained different states\n");
    }

    return 0;
}