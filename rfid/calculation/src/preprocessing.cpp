#include "preprocessing.h"

double phaseFunction(double x, double y, double z, double a, double b, double c, double phase_offset)
{
	double c0 = 3e8;
	double f = 920.625e6;
	double lameda = c0 / f;
	double result = (4 * M_PI * sqrt(pow(x - a, 2) + pow(y - b, 2) + pow(z - c, 2)) / lameda) - phase_offset;
	return result;
}

vector<double> unwrappingPhase(vector<double> phaseVec)
{
	vector<double> unwrappingphase;
	for (int i = 0; i < phaseVec.size(); i++)
	{
		phaseVec[i] = 2 * M_PI - phaseVec[i] * M_PI / 180;
	}
	double phase_lb = 1.5;
	double phase_ub = 0.5;
	double diff;
	unwrappingphase.push_back(phaseVec[0]);
	for (int i = 1; i < phaseVec.size(); i++)
	{
		diff = phaseVec[i] - phaseVec[i - 1];
		if (diff > phase_lb * M_PI)
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff - 2 * M_PI);
		}
		else if (diff < (-phase_lb * M_PI))
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff + 2 * M_PI);
		}
		else
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff);
		}
	}
	unwrappingphase.swap(phaseVec);
	vector<double>().swap(unwrappingphase);
	unwrappingphase.push_back(phaseVec[0]);
	for (int i = 1; i < phaseVec.size(); i++)
	{
		diff = phaseVec[i] - phaseVec[i - 1];
		if (diff > phase_ub * M_PI)
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff - M_PI);
		}
		else if (diff < (-phase_ub * M_PI))
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff + M_PI);
		}
		else
		{
			unwrappingphase.push_back(unwrappingphase[i - 1] + diff);
		}
	}
	return unwrappingphase;
}

NewCoordinate AntennaCoordinate(vector<double> x, vector<double> y, double z, vector<double> w, int readerID, int antennaId)
{
	NewCoordinate p;

	double x0[2] = {0.020, 0.020};	 //-0.0017,-0.0017,-0.0017
	double y0[2] = {0.340, 0.340};	 // 0.281,0.282,0.28
	double z0[2] = {1.4775, 0.5775}; // 1.558,1.061,0.59

	// std::cout << "--6--" << std::endl;

	Vector4d pVec;
	Matrix4d TRobot;
	Vector4d Ant2Robot;

	for (unsigned int i = 0; i < x.size(); i++)
	{
		// std::cout << "--7--" << std::endl;
		TRobot << cos(w[i]), -sin(w[i]), 0, x[i],
			sin(w[i]), cos(w[i]), 0, y[i],
			0, 0, 1, z,
			0, 0, 0, 1;
		if (readerID == 2)
		{

			Ant2Robot << x0[antennaId - 1], y0[antennaId - 1], z0[antennaId - 1], 1;
			pVec = TRobot * Ant2Robot;
			p.x_new.push_back(pVec[0]);
			p.y_new.push_back(pVec[1]);
			p.z_new = pVec[2];
			// std::cout << "--8--" << std::endl;
		}
		if (readerID == 1)
		{
			Ant2Robot << x0[antennaId - 1], -y0[antennaId - 1], z0[antennaId - 1], 1;
			pVec = TRobot * Ant2Robot;
			p.x_new.push_back(pVec[0]);
			p.y_new.push_back(pVec[1]);
			p.z_new = pVec[2];
		}
	}

	return p;
}

double fitting(double x1, double x2, double y1, double y2, double x)
{
	double y;
	y = (y2 - y1)/(x2 - x1) * (x - x1) + y1;

	return y;
}

tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> linear(Antenna ant1, Antenna ant2, OdomData odom)
{
	vector<double> LMphase1_get;
	vector<double> LMphase2_get;
	vector<double> LMx_get;
	vector<double> LMy_get;
	vector<double> LMw_get;
	vector<double> LMrssi1_get;
	vector<double> LMrssi2_get;
	double ph1_get, ph2_get, rssi1_get, rssi2_get;

	for (int k = 0; k < odom.timestamp.size(); k = k + 2)
	{
		if (ant1.timestamp.front() < odom.timestamp[k] &&
			odom.timestamp[k] < ant1.timestamp.back() &&
			ant2.timestamp.front() < odom.timestamp[k] &&
			odom.timestamp[k] < ant2.timestamp.back())
		{
			for (int i = 0; i < ant1.phase.size(); i++)
			{
				if (ant1.timestamp[i] >= odom.timestamp[k])
				{
					ph1_get = fitting(ant1.timestamp[i - 1], ant1.timestamp[i], ant1.phase[i - 1], ant1.phase[i], odom.timestamp[k]);
					rssi1_get = fitting(ant1.timestamp[i - 1], ant1.timestamp[i], ant1.rssi[i - 1], ant1.rssi[i], odom.timestamp[k]);
					break;
				}
			}

			for (int i = 0; i < ant2.phase.size(); i++)
			{
				if (ant2.timestamp[i] >= odom.timestamp[k])
				{
					ph2_get = fitting(ant2.timestamp[i - 1], ant2.timestamp[i], ant2.phase[i - 1], ant2.phase[i], odom.timestamp[k]);
					rssi2_get = fitting(ant2.timestamp[i - 1], ant2.timestamp[i], ant2.rssi[i - 1], ant2.rssi[i], odom.timestamp[k]);
					break;
				}
			}

			LMphase1_get.push_back(ph1_get);
			LMrssi1_get.push_back(rssi1_get);
			LMphase2_get.push_back(ph2_get);
			LMrssi2_get.push_back(rssi2_get);
			LMx_get.push_back(odom.x[k]);
			LMy_get.push_back(odom.y[k]);
			LMw_get.push_back(odom.yaw[k]);
		}
	}

	return make_tuple(LMphase1_get, LMphase2_get, LMx_get, LMy_get, LMw_get, LMrssi1_get, LMrssi2_get);
}

Antenna DataFilter(Antenna ant)
{
	int flag_fit = 1;
	vector<double> phaseVec = unwrappingPhase(ant.phase);
	std::cout << "succeed unwrapping phase " << std::endl; 
	vector<double> timestamp_rosVec = ant.timestamp;
	vector<double> rssiVec = ant.rssi;


	vector<double> e;
	double e_avg;
	double e_sum = 0;

	int n = phaseVec.size();
	vector<int> temp;
	int left_temp = 0;
	int right_temp = 0;

	for (int j = 1; j < phaseVec.size(); j++)
	{
		e.push_back(sqrt(pow((phaseVec[j] - phaseVec[j - 1]), 2) + pow((timestamp_rosVec[j] - timestamp_rosVec[j - 1]), 2)));
		e_sum += e[j - 1];
	}
	e_avg = e_sum / e.size();

	temp.push_back(0);
	for (int m = 0; m < e.size(); m++)
	{
		if (e[m] > 3 * e_avg)
		{
			temp.push_back(m);
		}
	}
	temp.push_back(n - 1);

	if (temp.size() == 2)
	{
		std::cout << "No filter" << std::endl;
		flag_fit = 0;
	}
	else if (temp.size() == 3)
	{
		if (temp[0] - 0 > n - 1 - (temp[1] + 1))
		{
			left_temp = 0;
			right_temp = temp[1];
		}
		else
		{
			left_temp = temp[1] + 1;
			right_temp = n - 1;
		}
	}
	else
	{
		int temp0 = temp[1] - temp[0];
		left_temp = temp[0];
		right_temp = temp[1];
		for (int k = 1; k < temp.size(); k++)
		{
			if ((temp[k] - temp[k - 1]) > temp0)
			{
				temp0 = temp[k] - temp[k - 1];
				left_temp = temp[k - 1] + 1;
				right_temp = temp[k];
			}
		}
	}

	if (flag_fit == 1)
	{
		vector<double> phaseVec_mid;
		vector<double> timestamp_rosVec_mid;
		vector<double> rssiVec_mid;
		for (int j = left_temp; j <= right_temp; j++)
		{
			phaseVec_mid.push_back(phaseVec[j]);
			timestamp_rosVec_mid.push_back(timestamp_rosVec[j]);
			rssiVec_mid.push_back(rssiVec[j]);
		}
		phaseVec.swap(phaseVec_mid);
		timestamp_rosVec.swap(timestamp_rosVec_mid);
		rssiVec.swap(rssiVec_mid);
		vector<double>().swap(phaseVec_mid);
		vector<double>().swap(timestamp_rosVec_mid);
		vector<double>().swap(rssiVec_mid);
		ant.phase = phaseVec;
		ant.timestamp = timestamp_rosVec;
		ant.rssi = rssiVec;

		std::cout << "The data has been filtered." << std::endl;
		std::cout << "Filtered data's size is " << ant.phase.size() << std::endl;
	}
	return ant;
}

void LM::MakeHessian(Reader reader, OdomData odom)
{
	for (int i = 0; i < 3; i++)
	{
		if(reader.ant[i].phase.size() > 0)
		{
			reader.ant[i] = DataFilter(reader.ant[i]);
		}
		else{
			std::cout << "ant " << i + 1 << " has no data." << endl;
		}
		
	}

	vector<int> id;
	for (int i = 0; i < 3; i++)
	{
		if (reader.ant[i].phase.empty())
		{
			std::cout << "ant " << i + 1 << " has no data." << endl;
		}
		else
		{
			id.push_back(i);
		}
	}
	
	switch (id.size())
	{
	case 1:
		flag = 0;
		std::cout << "Can't calc." << endl;

	case 2:
		if (reader.ant[id[0]].phase.size() < size0 || reader.ant[id[1]].phase.size() < size0)
		{
			flag = 0;
			std::cout << "Can't calc." << endl;
		}

	case 3:
		if (reader.ant[id[0]].phase.size() < size0 &&
			reader.ant[id[1]].phase.size() < size0 &&
			reader.ant[id[2]].phase.size() < size0)
		{
			flag = 0;
			std::cout << "Can't calc." << endl;
		}
		else
		{
			vector<int>().swap(id);
			int phaseSize[] = {reader.ant[0].phase.size(),
							   reader.ant[1].phase.size(),
							   reader.ant[2].phase.size()};
			int size_min = min_element(phaseSize, phaseSize + 3) - phaseSize;

			

			for (int j = 0; j < 3; j++)
			{
				if (size_min != j)
				{
					id.push_back(j);
				}
			}
		}


		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//添加如果有一根小于size0怎么办？？？？？？？？
		//！！！！！！！！！！！！！！！！！！！！！！！!

		

	default:
		break;
	}


	if (flag)
	{
		std::cout << "--1--" << std::endl;
		Antenna ant1 = reader.ant[id[0]];
		Antenna ant2 = reader.ant[id[1]];

		std::cout << "--1--" << std::endl;

		// linear fitting
		tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> LMdata1_get = linear(ant1, ant2, odom);
		vector<double> LMphase1 = get<0>(LMdata1_get);
		vector<double> LMphase2 = get<1>(LMdata1_get);
		vector<double> LMx = get<2>(LMdata1_get);
		vector<double> LMy = get<3>(LMdata1_get);
		vector<double> LMw = get<4>(LMdata1_get);
		vector<double> LMrssi1 = get<5>(LMdata1_get);
		vector<double> LMrssi2 = get<6>(LMdata1_get);
		//tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> LMdata2_get = linear(ant2, odom);
		//vector<double> LMphase2 = get<0>(LMdata2_get);
		//vector<double> LMx2 = get<1>(LMdata2_get);
		//vector<double> LMy2 = get<2>(LMdata2_get);
		//vector<double> LMw2 = get<3>(LMdata2_get);
		//vector<double> LMrssi2 = get<4>(LMdata2_get);

		std::cout << "--1--" << std::endl;

		double phase_diff1 = LMphase1[0];
		double phase_diff2 = LMphase2[0];

		for (unsigned int i = 0; i < LMphase1.size(); i++)
		{
			LMphase1[i] = LMphase1[i] - phase_diff1;
		}

		for (unsigned int i = 0; i < LMphase2.size(); i++)
		{
			LMphase2[i] = LMphase2[i] - phase_diff2;
		}

		// vector->eigen
		Eigen::VectorXd phase1 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LMphase1.data(), LMphase1.size());
		Eigen::VectorXd phase2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LMphase2.data(), LMphase2.size());
		
		//std::cout << "--2--" << std::endl;

		NewCoordinate position1 = AntennaCoordinate(LMx, LMy, z, LMw, reader.readerID, ant1.antennaId);
		vector<double> x1 = position1.x_new;
		vector<double> y1 = position1.y_new;
		double z1 = position1.z_new;

		NewCoordinate position2 = AntennaCoordinate(LMx, LMy, z, LMw, reader.readerID, ant2.antennaId);
		vector<double> x2 = position2.x_new;
		vector<double> y2 = position2.y_new;
		double z2 = position2.z_new;

		//std::cout << "--3--" << std::endl;

		//
		int minPosition1 = min_element(LMphase1.begin(), LMphase1.end()) - LMphase1.begin();
		int minPosition2 = min_element(LMphase2.begin(), LMphase2.end()) - LMphase2.begin();
		double a0, b0, c0;
		a0 = x1[minPosition1];
		if (reader.readerID == 2)
			b0 = y1[minPosition1] + y_delta;
		if (reader.readerID == 1)
			b0 = y1[minPosition1] - y_delta;
		if (LMrssi1[minPosition1] > LMrssi2[minPosition2])
			c0 = z1;
		else
			c0 = z2;

		//std::cout << "--4--" << std::endl;

		double phase_offset1_0 = 4 * M_PI * sqrt(pow(x1[0] - a0, 2) + pow(y1[0] - b0, 2) + pow(z1 - c0, 2)) / lameda - phase1(0);
		double phase_offset2_0 = 4 * M_PI * sqrt(pow(x2[0] - a0, 2) + pow(y2[0] - b0, 2) + pow(z2 - c0, 2)) / lameda - phase2(0);
		a_it = a0;
		b_it = b0;
		c_it = c0;
		phase_offset1_it = phase_offset1_0;
		phase_offset2_it = phase_offset2_0;

		int n1 = x1.size();
		int n2 = x2.size();
		MatrixXd J = MatrixXd::Zero(n1 + n2, 5);
		double e, e_new;

		VectorXd phase(n1 + n2);

		//std::cout << "--5--" << std::endl;

		for (int i = 0; i < n1; i++)
		{
			phase(i) = phase1(i);
		}
		for (int i = 0; i < n2; i++)
		{
			phase(i + n1) = phase2(i);
		}

		VectorXd step;
		MatrixXd H;
		VectorXd d_new;
		VectorXd phase_new(n1 + n2);
		VectorXd d;
		for (int k = 1; k <= k_max; k++)
		{
			for (int i = 0; i < n1; i++)
			{
				J(i, 0) = -4 * M_PI * (x1[i] - a_it) / (lameda * sqrt(pow(x1[i] - a_it, 2) + pow(y1[i] - b_it, 2) + pow(z1 - c_it, 2)));
				J(i, 1) = -4 * M_PI * (y1[i] - b_it) / (lameda * sqrt(pow(x1[i] - a_it, 2) + pow(y1[i] - b_it, 2) + pow(z1 - c_it, 2)));
				J(i, 2) = -4 * M_PI * (z1 - c_it) / (lameda * sqrt(pow(x1[i] - a_it, 2) + pow(y1[i] - b_it, 2) + pow(z1 - c_it, 2)));
				J(i, 3) = -1;
				J(i, 4) = 0;
			}
			for (int i = 0; i < n2; i++)
			{
				J(i + n1, 0) = -4 * M_PI * (x2[i] - a_it) / (lameda * sqrt(pow(x2[i] - a_it, 2) + pow(y2[i] - b_it, 2) + pow(z2 - c_it, 2)));
				J(i + n1, 1) = -4 * M_PI * (y2[i] - b_it) / (lameda * sqrt(pow(x2[i] - a_it, 2) + pow(y2[i] - b_it, 2) + pow(z2 - c_it, 2)));
				J(i + n1, 2) = -4 * M_PI * (z2 - c_it) / (lameda * sqrt(pow(x2[i] - a_it, 2) + pow(y2[i] - b_it, 2) + pow(z2 - c_it, 2)));
				J(i + n1, 3) = 0;
				J(i + n1, 4) = -1;
			}

			//std::cout << "--9--" << std::endl;

			if (k == 1)
			{
				VectorXd phase_res(n1 + n2);
				for (int i = 0; i < n1; i++)
				{
					phase_res(i) = phaseFunction(x1[i], y1[i], z1, a0, b0, c0, phase_offset1_0);
				}
				for (int i = 0; i < n2; i++)
				{
					phase_res(i + n1) = phaseFunction(x2[i], y2[i], z2, a0, b0, c0, phase_offset2_0);
				}

				phase_diff1 = phase_res(0);
				phase_diff2 = phase_res(n1);
				for (int i = 0; i < n1; i++)
				{
					phase_res(i) = phase_res(i) - phase_diff1;
				}
				for (int i = 0; i < n2; i++)
				{
					phase_res(i + n1) = phase_res(i + n1) - phase_diff2;
				}

				d = phase - phase_res;
				H = J.transpose() * J;
				step = (H + v * I).inverse() * J.transpose() * d;

				e = d.dot(d);
				continue;
			}

			//std::cout << "--10--" << std::endl;

			// std::cout << step << std::endl;

			H = J.transpose() * J;

			a_new = a_it + step(0);
			b_new = b_it + step(1);
			c_new = c_it + step(2);
			phase_offset1_new = phase_offset1_it + step(3);
			phase_offset2_new = phase_offset2_it + step(4);

			// std::cout << "--11--" << std::endl;

			for (int i = 0; i < n1; i++)
			{
				phase_new(i) = phaseFunction(x1[i], y1[i], z1, a_new, b_new, c_new, phase_offset1_new);
			}
			for (int i = 0; i < n2; i++)
			{
				phase_new(i + n1) = phaseFunction(x2[i], y2[i], z2, a_new, b_new, c_new, phase_offset2_new);
			}

			phase_diff1 = phase_new(0);
			phase_diff2 = phase_new(n1);
			for (int i = 0; i < n1; i++)
			{
				phase_new(i) = phase_new(i) - phase_diff1;
			}
			for (int i = 0; i < n2; i++)
			{
				phase_new(i + n1) = phase_new(i + n1) - phase_diff2;
			}
			// std::cout << "--12--" << std::endl;

			d_new = phase - phase_new;
			e_new = d_new.dot(d_new);
			step = (H + v * I).inverse() * J.transpose() * d_new;

			// std::cout << "--13--" << std::endl;

			if (e_new <= e)
			{
				if (e - e_new < e_min)
					break;
				v = v / 5;
				a_it = a_new;
				b_it = b_new;
				c_it = c_new;
				phase_offset1_it = phase_offset1_new;
				phase_offset2_it = phase_offset2_new;
				e = e_new;
				//std::cout << "Next loop at point:" << std::fixed << a_new << "  " << std::fixed << b_new << "  " << std::fixed << c_new << std::endl;
			}
			else
			{
				v = v * 5;
				// std::cout << v << std::endl;
			}
			//std::cout << e_new << std::endl;
		}
		cout << "Iteration starts at point:" << std::fixed << a0 << "  " << std::fixed << b0 << "  " << std::fixed << c0 << endl;
		cout << "Iteration ends at point:" << std::fixed << a_new << "  " << std::fixed << b_new << "  " << std::fixed << c_new << endl;
	}
}
