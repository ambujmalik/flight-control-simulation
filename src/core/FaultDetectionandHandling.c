#define MAX_FAULTS 32

typedef struct {
    uint32_t faultBits;
    uint32_t confirmedFaults;
    uint32_t activeFaults;
    time_t faultTimers[MAX_FAULTS];
} FaultHandler;

void detectFaults(AircraftState *state, FaultHandler *faults) {
    // Sensor consistency checks
    if (fabs(state->altitude - calculateBaroAltitude(state)) > 500) {
        setFault(faults, ALTITUDE_DISAGREE_FAULT);
    }
    
    // Control surface position feedback checks
    if (fabs(controls->elevatorPosition - getFeedbackPosition(ELEVATOR_FEEDBACK)) > 0.1) {
        setFault(faults, ELEVATOR_JAM_FAULT);
    }
    
    // Computer self-tests
    if (!memoryTest() || !cpuTest()) {
        setFault(faults, COMPUTER_HARDWARE_FAULT);
    }
}

void handleFaults(FaultHandler *faults, AircraftState *state) {
    // If we have critical faults, downgrade control laws
    if (faults->activeFaults & CRITICAL_FAULTS_MASK) {
        downgradeControlLaws(state);
    }
    
    // If we have multiple computer faults, isolate bad channels
    if (countFaults(faults, COMPUTER_FAULTS) > 1) {
        initiateChannelIsolation();
    }
    
    // Alert crew if needed
    if (faults->activeFaults & CREW_ALERT_MASK) {
        triggerMasterCaution();
    }
}
