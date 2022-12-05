/******************************************************************************
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Use of the Software is limited solely to applications:
 * (a) running on a Xilinx device, or
 * (b) that interact with a Xilinx device through a bus or interconnect.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the Xilinx shall not be used
 * in advertising or otherwise to promote the sale, use or other dealings in
 * this Software without prior written authorization from Xilinx.
 *
 ******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include "kalmanfilter.h"

void kalman_initialize()
{
	char *buffer;
	init_platform();

	px_config0 = XBram_LookupConfig(XPAR_BRAM_0_DEVICE_ID);
	int x_status = XBram_CfgInitialize(&x_bram0, px_config0, px_config0->CtrlBaseAddress);

	px_config1 = XBram_LookupConfig(XPAR_BRAM_1_DEVICE_ID);
	x_status = XBram_CfgInitialize(&x_bram1, px_config1, px_config1->CtrlBaseAddress);

	kf_config = XKalmanfilterkernel_LookupConfig(XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);
	x_status = XKalmanfilterkernel_CfgInitialize(&kf_kernel, kf_config);
	x_status = XKalmanfilterkernel_Initialize(&kf_kernel, XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);

	XKalmanfilterkernel_Set_q(&kf_kernel, q);
	XKalmanfilterkernel_Set_r(&kf_kernel, r);
}

void kalman_estimator()
{
	// Put code to start kernel and wait for finish here
	for (int i = 0; i < N_SAMPLES; i++)
	{
		sleep(0.1);
		// read data and fill it into the BRAM
		for (int j = 0; j < 6; j++)
		{
			BRAM0(j) = *(u32 *)(&(din[j + i * 6]));
		}

		XKalmanfilterkernel_Start(&kf_kernel);
		while (!XKalmanfilterkernel_IsDone(&kf_kernel))
		{
			// wait
			xil_printf("Waiting \n");
		}

		buffer = (char *)malloc(100 * sizeof(char));
		for (int j = 0; j < 6; j++)
		{
			u32 val = BRAM1(j);
			if (j == 0)
				sprintf(buffer, "%f", *(float *)(&(val)));
			else
				sprintf(buffer, "%s, %f", buffer, *(float *)(&(val)));
		}

		sprintf(buffer, "%s\n", buffer);

		xil_printf(buffer);
		free(buffer);
	}
}

void kalman_cleanup()
{
	cleanup_platform();
}

int main(){
	return 0;
}