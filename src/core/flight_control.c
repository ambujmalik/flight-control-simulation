#include <stdbool.h>

typedef struct {
    float altitude;     // Feet
    float airspeed;     // Knots
    float pitch;       // Degrees
} AircraftState;

void stabilizePitch(AircraftState *state) {
    // Example: Simple pitch stabilization
    if (state->pitch > 15.0f) {
        printf("PITCH WARNING: Reducing angle.\n");
        // Apply nose-down correction
    }
}

int main() {
    AircraftState simState = { .altitude = 10000, .airspeed = 250, .pitch = 5.0 };
    while (true) {
        stabilizePitch(&simState);
        // Add real-time delay (simulated)
    }
    return 0;
}
