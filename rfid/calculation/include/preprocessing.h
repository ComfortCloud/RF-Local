#pragma once
#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <time.h>
#include "RFIDSubFormat.h"

using namespace Eigen;
using namespace std;

typedef struct
{
	vector<double> x;
	vector<double> y;
	vector<double> yaw;
	vector<double> timestamp;
} OdomData;

// 相位计算函数
double phaseFunction(double x, double y, double z, double a, double b, double c, double phase_offset);

// 相位解缠函数
vector<double> unwrappingPhase(vector<double> phaseVec);

typedef struct
{
	vector<double> x_new;
	vector<double> y_new;
	double z_new;
} NewCoordinate;

NewCoordinate AntennaCoordinate(VectorXd x, VectorXd y, double z, VectorXd w, int readerID, int antennaId);

double fitting(double x1, double x2, double y1, double y2, double x);

tuple<vector<double>, vector<double>, vector<double>, vector<double>,vector<double>> linear(Antenna ant, OdomData odom);


Antenna DataFilter(Antenna ant);

// LM算法
typedef struct
{
	double x;
	double y;
	double z;
} EPC_xyz;

class LM
{
private:
	int size0 = 50; // 最小读取次数
	double e_min = 0.005;
	int k_max = 100;
	double v = 0.01;
	double z = 0;
	double y_delta = 0.7; // 货架距离/2，用以修正初始点

	// 常数
	MatrixXd I = MatrixXd::Identity(5, 5);
	double c = 3e8;
	double f = 920.625e6;
	double lameda = c / f;

	double a_it, b_it, c_it, phase_offset1_it, phase_offset2_it;

	double phase_offset1_new, phase_offset2_new;

public:
	vector<string> epcId;
	vector<EPC_xyz> epc_xyz;
	double a_new, b_new, c_new;
	int flag = 1;
	void MakeHessian(Reader reader, OdomData odom);
};

#endif
