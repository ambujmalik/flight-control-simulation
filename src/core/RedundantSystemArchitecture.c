#define NUM_COMPUTERS 3

typedef struct {
    AircraftState state;
    ControlSurfaces controls;
    bool healthy;
    uint32_t checksum;
} FlightComputer;

void redundantSystem(FlightComputer computers[NUM_COMPUTERS]) {
    // Voting system for outputs
    ControlSurfaces safeControls = {0};
    int healthyCount = 0;
    
    // Check all computers
    for (int i = 0; i < NUM_COMPUTERS; i++) {
        if (verifyComputerHealth(&computers[i])) {
            healthyCount++;
            // Sum controls from healthy computers
            safeControls.aileronPosition += computers[i].controls.aileronPosition;
            safeControls.elevatorPosition += computers[i].controls.elevatorPosition;
            // ... other control surfaces
        }
    }
    
    // Average the outputs from healthy computers
    if (healthyCount > 0) {
        safeControls.aileronPosition /= healthyCount;
        safeControls.elevatorPosition /= healthyCount;
        // ... other control surfaces
        
        applyControls(&safeControls);
    } else {
        // All computers failed - engage backup mechanical system
        activateBackupMechanical();
    }
}
