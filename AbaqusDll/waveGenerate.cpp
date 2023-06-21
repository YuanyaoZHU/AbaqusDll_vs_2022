#include<iostream>
#include <fstream>
#include <string>
#include<algorithm>
#include<stdlib.h>
#include<Windows.h>
#include"pch.h"
#include"waveGenerate.h"

using namespace std;

waveNumber::waveNumber(double h)
{
	this->h = h;
	this->omega = 0;
}

double waveNumber::newton(double w)
{
	this->omega = w;
	double k1;
	double k0 = 10;
	double delta_k;
	double f;
	double f1;
	double i = 0;
	//cout << "进入newton!" << endl;
	do
	{
		f = F(k0);
		f1 = F1(k0, ec);
		k1 = k0 - f / f1;
		delta_k = k1 - k0;
		k0 = k1;
		i++;
		if (i >= 100000)
		{
			cout << "newton法步数过大，请重新调整" << endl;
			system("pause");
			break;
		}
	} while (abs(delta_k) > ec);
	return k1;
}

double waveNumber::F(double k)
{
	double f;
	f = k * tanh(k * h) - pow(omega, 2) / g;
	return f;
}

double waveNumber::F1(double k, double dk)
{
	double f1;
	f1 = (F(k + dk) - F(k)) / dk;
	return f1;
}

/*-----------------------data--------------------------------------*/

data::data()
{
	gama = 3.3;
	sigma_a = 0.07;
	sigma_b = 0.09;
	pi = 3.1415926;
	temp = 0.23 + 0.0336 * gama - 0.185 * pow(1.9 + gama, -1);
	beta_j = 0.06238 * pow(temp, -1) * (1.094 - 0.01915 * log(gama));
	C2F = 1.0 / (2.0 * pi);
	F2W = 2.0 * pi;
	TwoPi = 2.0 * pi;

}

/*-------------------------------wavespectral2-----------------------------------*/
wavespectral2::wavespectral2(double Hs, double Tp)
{
	//cout << "进入wavespctral2类内部" << endl;
	this->Hs = Hs;
	this->Tp = Tp;

}

double wavespectral2::Spectral(double omega)
{

	if (omega <= TwoPi / Tp)//sigma的取值
	{
		this->sigma = sigma_a;
	}
	else if (omega > TwoPi / Tp)
	{
		this->sigma = sigma_b;
	}

	if (Tp / sqrt(Hs) <= 3.6)//gama的取值
	{
		gama = 5;
	}
	else if (Tp / sqrt(Hs) > 3.6 && Tp / sqrt(Hs) <= 5)
	{
		gama = exp(5.75 - 1.15 * Tp / sqrt(Hs));
	}
	else if (Tp / sqrt(Hs) > 5)
	{
		gama = 1;
	}

	double C0 = C2F * 5.0 / 16.0 * pow(Hs, 2) * Tp;
	double C1 = pow(omega * Tp / TwoPi, -5);
	double C2 = exp(-(5.0 / 4.0) * pow(Tp * omega / TwoPi, -4));
	double C3 = 1.0 - 0.287 * log(gama);
	double C4 = pow(gama, exp(-0.5 * (pow(((omega * Tp / TwoPi - 1) / sigma), 2))));
	this->S = C0 * C1 * C2 * C3 * C4;
	return S;
}