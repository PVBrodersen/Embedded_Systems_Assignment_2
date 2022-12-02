#include <iostream>

template <typename T, int L_max, int M_max, int N_max>
void matMultiply(T *mat_in_L, T *mat_in_R, T *mat_out, int L, int M, int N)
// Multiply T_in (M x N) * T_in (N x L) = T_out (M x L)
{
	if (L > L_max) L = L_max;
	if (M > M_max) M = M_max;
	if (N > N_max) N = N_max;


	matMul_loop_L:
	for (int i = 0; i < L; i++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=L_max
		matMul_loop_N:
		for (int k = 0; k < N; k++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=N_max
			mat_out[i*N+k] = 0;

			matMul_loop_M:
			for (int j = 0; j < M; j++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max

				mat_out[i*N+k] += mat_in_L[i*M+j] * mat_in_R[j*N+k];

			}
		}
	}
}

template <typename T, int M_max, int N_max>
void matAdd(T *mat_in_L, T *mat_in_R, T *mat_out, int M, int N)
// Add T_in (M x N) + T_in (M x N) = T_out (M x N)
{
	if (M > M_max) M = M_max;
	if (N > N_max) N = N_max;

	for (int i = 0; i < M; i++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max
		for (int j = 0; j < N; j++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=N_max

			mat_out[i*N+j] = mat_in_L[i*N+j] + mat_in_R[i*N+j];

		}
	}
}

template <typename T, int M_max, int N_max>
void matSubtract(T *mat_in_L, T *mat_in_R, T *mat_out, int M, int N)
// Subtract T_in (M x N) - T_in (M x N) = T_out (M x N)
{
	if (M > M_max) M = M_max;
	if (N > N_max) N = N_max;

	for (int i = 0; i < M; i++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max
		for (int j = 0; j < N; j++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=N_max

			mat_out[i*N+j] = mat_in_L[i*N+j] - mat_in_R[i*N+j];

		}
	}
}

template <typename T, int M_max, int N_max>
void matTranspose(T *mat_in, T *mat_out, int M, int N)
// Transpose matrix transpose(T (M x N)) = T (N x M)
{
	if (M > M_max) M = M_max;
	if (N > N_max) N = N_max;

	for (int i = 0; i < M; i++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max
		for (int j = 0; j < N; j++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=N_max

			mat_out[j*M+i] = mat_in[i*N+j];

		}
	}
}

template <typename T, int M_max>
void matDiagInverse(T *mat_in, T *mat_out, int M)
// Inverse of diagonal matrix inverse(T (M x M)) = T (M x M) using diagonal formula
{
	if (M > M_max) M = M_max;

	for (int i = 0; i < M; i++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max
		for (int j = 0; j < M; j++) {
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT max=M_max

			if (i==j) {
				mat_out[i*M+j] = ((T)1.)/mat_in[i*M+j];
			} else {
				mat_out[i*M+j] = ((T)0.);
			}

		}
	}
}








