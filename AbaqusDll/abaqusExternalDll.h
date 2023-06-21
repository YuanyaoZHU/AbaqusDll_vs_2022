#pragma once
#include<iostream>
#include<cmath>
#include<string>
#include<Windows.h>
#include<stdlib.h>
#include<stdio.h>
#include <fstream>
#include<Eigen/Dense>  
#include"pch.h"

using namespace std;
using namespace Eigen;

extern "C" _declspec(dllexport) int start_zhu();
extern "C" _declspec(dllexport) int Modelica2Dll(double time, double* disp, double* velocity, double* accel, double* dload);
extern "C" _declspec(dllexport) int abaqusdllurdfil(double* time, double* dload);
extern "C" _declspec(dllexport) int abaqusdlldisp(double *time, double* disp, double* velocity, double* accel_zhu);
extern "C" _declspec(dllexport) int setv(int* ELM_NO, double* time, double* V);
extern "C" _declspec(dllexport) int setcoord(int* ELM_NO, double* time, double* COORD);
extern "C" _declspec(dllexport) int getv(int* ELM_NO, double* time, double* VW, double* V);
extern "C" _declspec(dllexport) int getwaterv(int* NOEL, double* time, double* VW);
extern "C" _declspec(dllexport) int updatepanel();
void fluidVelocity(double VW[3], double time, double COORD[3]);
void calculateforce(double F[3],Vector3d Normal, Vector3d vw,int NO_local);