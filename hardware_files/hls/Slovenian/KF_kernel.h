#include <ap_int.h>
#include <ap_fixed.h>
#include <stdint.h>

#include "matrix_ops.h"

//#define 	SAMPLE_FREQ 	10	// Hz
//#define 	DT 				0.1
//#define		BRAM_SIZE		2048
#define		N_SAMPLES		300
#define		N_STATE_VARS	6
#define		N_CTRL_VARS		3
#define		N_MEAS_VARS		3

typedef		float		KF_data_t;
//typedef		ap_fixed<16,8>	KF_data_t;

void KalmanFilterKernel(float din[N_CTRL_VARS+N_MEAS_VARS], float dout[N_STATE_VARS], ap_uint<32> counter, float q, float r);
