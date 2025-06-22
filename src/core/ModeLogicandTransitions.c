typedef enum {
    AP_OFF,
    AP_HDG_HOLD,
    AP_ALT_HOLD,
    AP_VS_HOLD,
    AP_APPROACH,
    AP_GO_AROUND,
    AP_LAND
} AutopilotMode;

void autopilotModeLogic(AircraftState *state, AutopilotMode *currentMode, 
                       AutopilotCommands *commands) {
    // Mode transition logic
    switch(*currentMode) {
        case AP_OFF:
            if (commands->engage && state->altitude > 50) {
                *currentMode = AP_HDG_HOLD;
            }
            break;
            
        case AP_HDG_HOLD:
            if (commands->altHoldEngage) {
                *currentMode = AP_ALT_HOLD;
            }
            else if (commands->approachPressed && state->altitude < 10000) {
                *currentMode = AP_APPROACH;
            }
            break;
            
        case AP_ALT_HOLD:
            if (commands->vsEngage) {
                *currentMode = AP_VS_HOLD;
            }
            else if (!commands->altHoldEngage) {
                *currentMode = AP_HDG_HOLD;
            }
            break;
            
        // ... other mode transitions
    }
    
    // Protection against invalid mode sequences
    if (state->altitude < 50 && *currentMode != AP_OFF && *currentMode != AP_LAND) {
        *currentMode = AP_OFF;
    }
}
