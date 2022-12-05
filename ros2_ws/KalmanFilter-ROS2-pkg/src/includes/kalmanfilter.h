#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"
#include <unistd.h>

#include "xbram.h"
#include "xkalmanfilterkernel.h"

#include "data.h"

#define N_SAMPLES 300
#define N_MEAS_VARS 3
#define N_CTRL_VARS 3
#define N_STATE_VARS 6

#define BRAM0(A) ((volatile u32 *)px_config0->MemBaseAddress)[A]
#define BRAM1(A) ((volatile u32 *)px_config1->MemBaseAddress)[A]

XBram x_bram0;
XBram_Config *px_config0;

XBram x_bram1;
XBram_Config *px_config1;

XKalmanfilterkernel kf_kernel;
XKalmanfilterkernel_Config *kf_config;

class KalmanFilter
{

public:
    void kalman_initialize();
    void kalman_estimator();
    void kalman_cleanup();

private:
    const float q = 0.05;
    const float r = 0.95;
} 
int main();
