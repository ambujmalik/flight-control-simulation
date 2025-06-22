// Typical real-time executive loop
void controlLoop() {
    const uint32_t LOOP_PERIOD_MS = 20; // 50Hz update rate
    uint32_t lastWakeTime = getCurrentTime();
    
    while(true) {
        // Read sensors (time-critical)
        readPrimarySensors();
        readSecondarySensors();
        
        // Run control laws
        computeControlSurfaces();
        
        // Output to actuators
        driveActuators();
        
        // Non-critical background tasks
        if (hasTimeRemaining()) {
            runBuiltInTests();
            updateMaintenanceData();
        }
        
        // Strict timing control
        sleepUntil(lastWakeTime + LOOP_PERIOD_MS);
        lastWakeTime += LOOP_PERIOD_MS;
    }
}
