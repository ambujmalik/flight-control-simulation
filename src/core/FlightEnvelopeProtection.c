void envelopeProtection(AircraftState *state, ControlSurfaces *controls) {
    // Angle of attack protection
    double criticalAOA = calculateCriticalAOA(state);
    double currentAOA = calculateAOA(state);
    
    if (currentAOA > criticalAOA * 0.8) {
        // Reduce elevator authority and add nose-down command
        controls->elevatorPosition = fmin(controls->elevatorPosition, 0.2);
        if (currentAOA > criticalAOA) {
            controls->elevatorPosition -= 0.1;
        }
    }
    
    // Overspeed protection
    double maxSpeed = calculateMaxSpeed(state->altitude);
    if (state->airspeed > maxSpeed * 0.95) {
        // Command speed brakes and reduce throttle
        controls->spoilerPosition += 0.05;
        controls->throttlePosition *= 0.9;
    }
    
    // Bank angle protection
    double maxBank = calculateMaxBank(state->altitude, state->airspeed);
    if (fabs(state->roll) > maxBank) {
        double correction = (maxBank - fabs(state->roll)) / maxBank;
        controls->aileronPosition *= correction;
    }
}
