#pragma once

#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

enum fsmState { statePowerUp, stateNotRunning, statePulsePositive, statePulseNegative, stateInterPulse, stateComputeZ };
fsmState stateNext;
fsmState stateCurrent;

fsmState powerUp(void);
fsmState notRunning(void);
fsmState pulsePositive(void);
fsmState pulseNegative(void);
fsmState interPulse(void);
fsmState computeZ(void);
fsmState stepStateMachine(fsmState next_state);

// other functions
void print_mtxf(const Eigen::MatrixXf& X, uint8_t precision = 2);
void print_mtxf(const Eigen::MatrixXd& X, uint8_t precision = 2);