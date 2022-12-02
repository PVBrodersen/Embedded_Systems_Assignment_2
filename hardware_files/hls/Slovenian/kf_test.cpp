#include "KF_kernel.h"
#include "data.h"

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

#define INPUT_FILENAME "<project directory>/src/sensor_and_control_measurements.txt"
#define OUTPUT_FILENAME "KF_output_results1.txt"
#define DELIMITER ','


vector<vector<float>> parseFile(string filename) {
	vector <vector <float> > data;

	ifstream infile( INPUT_FILENAME );

	bool first_line = true;

	while(infile) {

		string s;
		if (!getline(infile, s)) break;

		if (first_line) {
			first_line = false;
			continue;
		}

		istringstream ss(s);
		vector<float> record;

		while(ss) {
			string s;
			if(!getline(ss,s,DELIMITER)) break;
			record.push_back(stof(s));
		}

		data.push_back(record);

	}

	if (!infile.eof()) {
		cerr << "Fooey!\n";
	}

	return data;
}

void writeDataToFile(float dout[N_SAMPLES*N_STATE_VARS], string filename) {

	ofstream f;
	f.open(filename);

	f << "x_hat_0,x_hat_1,x_hat_2,x_hat_3,x_hat_4,x_hat_5\n";

	for (int i = 0; i < N_SAMPLES; i++) {
		for (int j = 0; j < N_STATE_VARS; j++) {

			f << dout[i*N_STATE_VARS+j];

			if (j < N_STATE_VARS-1) {

				f << ",";
			}
		}

		if (i < N_SAMPLES-1) {

			f << "\n";
		}
	}
}

int main(int argc, char *argv[]) {

	float dout[N_SAMPLES*N_STATE_VARS];
	unsigned int clock = 0;

	for (int i = 0; i < N_SAMPLES; i++) {
		float din_temp[N_STATE_VARS];
		for (int j = 0; j < N_STATE_VARS; j++){
			din_temp[j]=din[j+i*N_STATE_VARS];
		}

		float dout_temp[N_STATE_VARS];
		KalmanFilterKernel(din_temp, dout_temp, clock, 0.05, 0.95);

		clock = clock + 10000000;

		for (int j = 0; j<N_STATE_VARS; j++){
			dout[N_STATE_VARS*i + j] = dout_temp[j];
		}
	}

	for (int i = 0; i < N_SAMPLES; i++) {
		for (int j = 0; j < N_STATE_VARS; j++) {
			cout << dout[i*N_STATE_VARS+j] << "\t";
		}
		cout << "\n";
	}

	writeDataToFile(dout, OUTPUT_FILENAME);
	cout << "written output file\n";

}











