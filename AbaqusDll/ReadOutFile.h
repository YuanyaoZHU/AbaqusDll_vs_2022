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
	

	double** Nodes1;//�ڵ㣬1ά���ڵ��ţ�2ά��[0-2]Ϊλ�ƣ�[3-5]Ϊ�ٶ�
	int** Node1;    //��ȡNodes�ı��,Node[xx][0],�ڵ��ţ�Node[xx][1-4]Ϊ��ýڵ������ĵ�Ԫ��š�
	int** Element1; //��Ԫ��1ά����Ԫ��ţ�2ά��[0]ΪElements�ı�ţ�[1-2]�����Ľڵ���,[3-4]Ϊ�����õ�Ԫ��Panel��Ԫ��š�
	double** randomValue;  //��¼�����
	double* freq;
	double* period;
	double* omega;
	double** hydroValue;
	int** Panel1; //Panel1�����ڼ�¼
	double** Panel2;
	int F_n;

	int kk();
	//int Elements[3][5];   //��Ԫ��1ά����Ԫ��ţ�2ά��[0]ΪElements�ı�ţ�[1-2]�����Ľڵ���,[3-4]Ϊ�����õ�Ԫ��Panel��Ԫ��š�
	//int Panel[2][13];
	//double Panels[2][6];
	int randomline;
	int hydroline;
	int panelsline;
};

