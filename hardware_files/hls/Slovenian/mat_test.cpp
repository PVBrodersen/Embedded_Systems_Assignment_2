#include "KF_kernel.h"

#include <iostream>
#include <stdlib.h>

//int main1(int argc, char *argv[]) {
//
//	int L = 3;
//	int M = 4;
//	int N = 4;
//
//	float mat1[L*M];
//	float mat2[M*N];
//	float mat3[M*N];
//
//	float mato1[L*N];
//	float mato2[M*N];
//	float mato3[N*M];
//
//	std::cout << "Mat1 = \n[";
//	for (int i = 0; i < L; i++) {
//
//		for (int j = 0; j < M; j++) {
//			float val = 1+(i*0.1+j*1.1)*0.3/(i*j+0.79) + rand() % 24;
//			mat1[i*M+j] = val;
//
//			std::cout << val;
//
//			if (j < M-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < L-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Mat2 = \n[";
//	for (int i = 0; i < M; i++) {
//		for (int j = 0; j < N; j++) {
//			float val = 4+(i*0.1+j*1.1)*0.3/(i*j+0.79) + rand() % 10;
//			mat2[i*N+j] = val;
//
//			std::cout << val;
//
//			if (j < N-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < M-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Mat3 = \n[";
//	for (int i = 0; i < M; i++) {
//		for (int j = 0; j < N; j++) {
//			float val = rand() % 100 + 1;
//			mat3[i*N+j] = val;
//
//			std::cout << val;
//
//			if (j < N-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < M-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//
//	std::cout << "Calling matrix multiply on mato1 = mat1 * mat2\n";
//	matMultiply<float, 20, 20, 20>(mat1, mat2, mato1, L, M, N);
//	std::cout << "mato1 = \n[";
//	for (int i = 0; i < L; i++) {
//		for (int j = 0; j < N; j++) {
//			float val = mato1[i*N+j];
//
//			std::cout << val;
//
//			if (j < N-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < L-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Calling matrix addition on mato2 = mat2 + mat3\n";
//	matAdd<float, 20, 20>(mat2, mat3, mato2, M, N);
//	std::cout << "mato2 = \n[";
//	for (int i = 0; i < M; i++) {
//		for (int j = 0; j < N; j++) {
//			float val = mato2[i*N+j];
//
//			std::cout << val;
//
//			if (j < N-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < M-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Calling matrix subtraction on mato2 = mat2 - mat3\n";
//	matSubtract<float, 20, 20>(mat2, mat3, mato2, M, N);
//	std::cout << "mato2 = \n[";
//	for (int i = 0; i < M; i++) {
//		for (int j = 0; j < N; j++) {
//			float val = mato2[i*N+j];
//
//			std::cout << val;
//
//			if (j < N-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < M-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Calling matrix transpose on mato3 = transpose(mat3)\n";
//	matTranspose<float, 20, 20>(mat3, mato3, M, N);
//	std::cout << "mato3 = \n[";
//	for (int i = 0; i < N; i++) {
//		for (int j = 0; j < M; j++) {
//			float val = mato3[i*M+j];
//
//			std::cout << val;
//
//			if (j < M-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < N-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	std::cout << "Calling matrix inverse on mato3 = inverse(mat3)\n";
//	matInverse<float, 20, 20>(mat3, mato3, M, N);
//	std::cout << "mato3 = \n[";
//	for (int i = 0; i < N; i++) {
//		for (int j = 0; j < M; j++) {
//			float val = mato3[i*M+j];
//
//			std::cout << val;
//
//			if (j < M-1) {
//				std::cout << "\t";
//			}
//		}
//
//		if (i < N-1) {
//			std::cout << ";\n";
//		}
//	}
//	std::cout << "]\n\n";
//
//	return 0;
//
//}
