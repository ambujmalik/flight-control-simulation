void controlLaws(AircraftState *state, PilotInput *input, ControlSurfaces *controls) {
    // Normal law - full flight envelope protection
    if (state->altitude > 50) { // Above 50 feet
        // Pitch control with load factor demand
        double targetG = 1.0 + (input->stickPitch * 0.5);
        double currentG = calculateLoadFactor(state);
        double gError = targetG - currentG;
        
        // Apply elevator to achieve target G with protection
        controls->elevatorPosition = computeElevatorCommand(gError);
        
        // Bank angle protection
        double maxBank = fmin(30.0, 25.0 + state->altitude/1000.0);
        double targetRoll = input->stickRoll * maxBank;
        double rollError = targetRoll - state->roll;
        
        controls->aileronPosition = computeAileronCommand(rollError);
    }
    // Direct law - bypasses some protections when needed
    else if (state->altitude > 10) {
        // More direct control relationship
        controls->elevatorPosition = input->stickPitch * 0.8;
        controls->aileronPosition = input->stickRoll * 0.6;
    }
    // Mechanical backup - very basic control
    else {
        controls->elevatorPosition = input->stickPitch * 0.3;
        controls->aileronPosition = input->stickRoll * 0.3;
    }
}
