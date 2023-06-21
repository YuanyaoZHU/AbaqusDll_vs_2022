#pragma once
#include<iostream>
#include <fstream>
#include <cstring>
#include<algorithm>
#include<stdlib.h>
#include<Windows.h>
#include"pch.h"
#include"ReadOutFile.h"


class ReadOutFile
{
private:
	int m, n, k;
	
	

public:
	ReadOutFile();
	int ReadInPanels();
	int ReadInNodes();
	int ReadInElements();
	int ReadRandom();
	int ReadHydro();
	

	double** Nodes1;//节点，1维：节点编号，2维：[0-2]为位移，[3-5]为速度
	int** Node1;    //读取Nodes的编号,Node[xx][0],节点编号，Node[xx][1-4]为与该节点相连的单元编号。
	int** Element1; //单元，1维：单元编号；2维：[0]为Elements的编号，[1-2]包含的节点编号,[3-4]为包含该单元的Panel单元编号。
	double** randomValue;  //记录随机量
	double* freq;
	double* period;
	double* omega;
	double** hydroValue;
	int** Panel1; //Panel1：用于记录
	double** Panel2;
	int F_n;

	int kk();
	//int Elements[3][5];   //单元，1维：单元编号；2维：[0]为Elements的编号，[1-2]包含的节点编号,[3-4]为包含该单元的Panel单元编号。
	//int Panel[2][13];
	//double Panels[2][6];
	int randomline;
	int hydroline;
	int panelsline;
};

