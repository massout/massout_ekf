#include <cstdio>

#include "ekf.h"

using TestFilter = ExtendedKalmanFilter<float, 2, 1, 1>;

static bool floatEqual(float a, float b, float eps = 1e-6f) {
    return (a > b ? a - b : b - a) <= eps;
}

int main() {
    // Matrix accessor validation
    TestFilter::StateMatrix state_matrix;
    state_matrix.setZero();
    state_matrix.set(0, 1, 3.5f);

    if (!floatEqual(state_matrix.get(0, 1), 3.5f)) {
        std::printf("Matrix set/get failed\n");

        return 1;
    }

    TestFilter filter;

    filter.setStateValue(0, 0, 1.25f);
    if (!floatEqual(filter.getStateValue(0, 0), 1.25f)) {
        std::printf("State value access mismatch\n");

        return 1;
    }

    filter.setCovarianceValue(1, 1, 2.0f);
    if (!floatEqual(filter.getCovarianceValue(1, 1), 2.0f)) {
        std::printf("Covariance value access mismatch\n");

        return 1;
    }

    filter.setProcessNoiseValue(0, 0, 0.1f);
    if (!floatEqual(filter.getProcessNoiseValue(0, 0), 0.1f)) {
        std::printf("Process noise value access mismatch\n");

        return 1;
    }

    filter.setMeasurementNoiseValue(0, 0, 0.05f);
    if (!floatEqual(filter.getMeasurementNoiseValue(0, 0), 0.05f)) {
        std::printf("Measurement noise value access mismatch\n");

        return 1;
    }

    filter.setTransitionValue(0, 1, 1.0f);
    if (!floatEqual(filter.getTransitionValue(0, 1), 1.0f)) {
        std::printf("Transition matrix value access mismatch\n");

        return 1;
    }

    filter.setControlValue(1, 0, 0.5f);

    if (!floatEqual(filter.getControlValue(1, 0), 0.5f)) {
        std::printf("Control matrix value access mismatch\n");

        return 1;
    }

    filter.setMeasurementValue(0, 0, 1.0f);

    if (!floatEqual(filter.getMeasurementValue(0, 0), 1.0f)) {
        std::printf("Measurement matrix value access mismatch\n");
        
        return 1;
    }

    // Produce a Kalman gain entry via a trivial predict/update cycle
    TestFilter::MeasurementVector measurement;

    measurement.setZero();
    filter.setCovariance(TestFilter::StateMatrix::Identity());
    filter.setMeasurementNoise(TestFilter::MeasurementCovariance::Identity());
    filter.setMeasurementMatrix(TestFilter::MeasurementMatrix::Identity());
    filter.setProcessNoise(TestFilter::StateMatrix::Identity());
    measurement(0, 0) = 0.0f;
    filter.predict(TestFilter::ControlVector());

    if (!filter.update(measurement)) {
        std::printf("Update failed while computing Kalman gain\n");

        return 1;
    }

    (void)filter.getKalmanGainValue(0, 0);

    std::printf("Value access test passed\n");
    
    return 0;
}
