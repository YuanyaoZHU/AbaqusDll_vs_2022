#pragma once
#include<iostream>
#include <fstream>
#include <cstring>
#include<algorithm>
#include<stdlib.h>
#include<Windows.h>
#include"pch.h"
#include"waveGenerate.h"


class waveNumber
{
private:
	double g = 9.81;
	double h;
	double omega;
	double ec = 0.000001;//¾«¶È

public:
	waveNumber(double h);
	double newton(double w);
	double F(double k);
	double F1(double k, double dk);
};

class data
{
protected:
	double sigma_a;
	double sigma_b;
	double beta_j;
	double pi;
	double gama;
	double C2F;
	double temp;
	double F2W;
	double TwoPi;


public:
	data();
};


class wavespectral2 :public data
{
private:
	double Hs;
	double Tp;
	double sigma;
	double S;

public:
	wavespectral2(double Hs, double Tp);
	double Spectral(double omega);

};