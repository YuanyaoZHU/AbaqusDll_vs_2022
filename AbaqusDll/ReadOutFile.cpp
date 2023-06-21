#include<iostream>
#include <fstream>
#include <string>
#include<algorithm>
#include<stdlib.h>
#include<Windows.h>
#include"pch.h"
#include"ReadOutFile.h"

using namespace std;

ReadOutFile::ReadOutFile()
{
	cout << "���뵽ReadOutFile�Ĺ��캯��" << endl;
	m = 20000;
	n = 30000;
	k = 10000;

	Nodes1 = new double* [m];
	for (int i = 0;i < m;i++)
	{
		Nodes1[i] = new double[6];
	}

	Node1 = new int* [m];
	for (int i = 0;i < m;i++)
	{
		Node1[i] = new int[5];
	}

	Element1 = new int* [n];
	for (int i = 0; i < n;i++)
	{
		Element1[i] = new int[5];
	}

	Panel1 = new int* [k];
	Panel2 = new double* [k];
	for (int i = 0; i < k;i++)
	{
		Panel1[i] = new int[13];
		Panel2[i] = new double[6];
	}
}


int ReadOutFile::ReadInPanels()
{
	cout << "��ȡPanels.txt�ļ���ʼ��" << endl;
	const string srcFileName = "./Panels.txt"; //����Panels.txt�ļ���

	//ͳ���ı��ж�����
	char c;
	this->panelsline = 0;
	fstream finl(srcFileName, ios::in);
	if (!finl)
	{
		cerr << "cannot open Panels.txt file!" << endl;
	}
	while (finl.get(c))
	{
		if (c == '\n')
			panelsline++; //������1
	}
	finl.close();

	//-------------------------------------

	ifstream fin(srcFileName, ios::in);
	if (!fin)
	{
		std::cerr << "open Panels.txt error!" << endl;
		fin.close();
		return 0;
	}

	for (int i = 0;i < k;i++)
	{
		for (int j = 0;j < 13;j++)
		{
			fin >> Panel1[i][j];
		}
		for (int j = 0;j < 6;j++)
		{
			fin >> Panel2[i][j];
		}
	}
	return 0;
}



int ReadOutFile::ReadInNodes()//��ȡNodes.txt�ļ�
{
	cout << "���뵽ReadInNodes2�У�" << endl;
	const string srcFileName = "./Nodes.txt";

	ifstream fin(srcFileName, ios::in);
	if (!fin)
	{
		std::cerr << "open Nodes.txt error!" << endl;
		fin.close();
		return 0;
	}
	else
	{
		cout << "Open Nodes.txt success!" << endl;
	}

	for (int i = 0;i < m;i++)
	{
		fin >> Node1[i][0];//����ڵ���
		//Node1[i][0] = Node[i][0];

		for (int k = 0;k < 3;k++)
		{
			fin >> Nodes1[i][k];//����ڵ�����
			//Nodes1[i][k]=Nodes[i][k];			
		}
		for (int j = 1;j < 5;j++)
		{
			fin >> Node1[i][j]; //������ýڵ������ĵ�Ԫ���
			//Node1[i][j] = Node[i][j];
		}
		/*
		for (int j = 0;j < 5;j++)
		{
			cout << Node[i][j]<<"    ";
		}
		for (int j = 0;j < 3;j++)
		{
			cout << Nodes[i][j] << "    ";
		}
		cout << endl;
		*/
	}


}

int ReadOutFile::ReadInElements() //��ȡElements.txt�ļ�
{
	cout << "��ʼ��ȡElements.txt�ļ���" << endl;
	const string srcFileName = "./Elements.txt";

	ifstream fin(srcFileName, ios::in);
	if (!fin)
	{
		std::cerr << "open Elements.txt error!" << endl;
		fin.close();
		return 0;
	}
	else
	{
		cout << "Open Elements.txt success!" << endl;
	}
	for (int i = 0;i < n;i++)
	{
		for (int j = 0;j < 5;j++)
		{
			fin >> Element1[i][j];//����ڵ�����
		}
	}
	return 0;
}

int ReadOutFile::kk()
{
	cout << "����ReadOutFile�ڲ���" << endl;
	return 0;
}

int ReadOutFile::ReadRandom()
{
	char c;
	this->randomline = 0;
	fstream fin("random_value.txt", ios::in);
	if (!fin)
	{
		cerr << "cannot open random_value.txt file!" << endl;
	}
	while (fin.get(c))
	{
		if (c == '\n')
			randomline++;
	}
	fin.close();

	double** I = new double* [randomline];
	double item = 0;
	for (int i = 0;i < randomline;i++)
	{
		I[i] = new double[2];
	}
	ifstream myfile("random_value.txt");
	if (!myfile.is_open())
	{
		cerr << "can not open random_value.txt file!" << endl;
	}
	for (int i = 0; i < randomline;i++)
	{
		for (int j = 0;j < 2;j++)
		{
			myfile >> I[i][j];
		}
	}

	randomValue = I;

	return 0;
}

int ReadOutFile::ReadHydro()
{
	//ͳ���ı��ж�����
	char c;
	this->hydroline = 0;
	fstream finl("Spar.3", ios::in);
	if (!finl)
	{
		cerr << "cannot open hydro parameter file!" << endl;
	}
	while (finl.get(c))
	{
		if (c == '\n')
			hydroline++; //������1
	}
	finl.close();

	//��ȡ����
	double** I = new double* [hydroline];
	F_n = 0;
	double item = 0;
	for (int i = 0; i < hydroline; i++)
	{
		I[i] = new double[7];
	}
	ifstream myfile("Spar.3");
	if (!myfile.is_open())
	{
		cout << "can not open this file!" << endl;

	}
	for (int i = 0; i < hydroline;i++)
	{
		for (int j = 0;j < 7;j++)
		{
			myfile >> I[i][j];
		}
	}
	//ͳ���ж��ٸ�Ƶ�ʣ���ЩƵ���Ƕ���
	for (int i = 0; i < hydroline; i++)
	{
		if (I[i][0] != item)
		{
			item = I[i][0];
			//cout << "item = " << item;
			F_n++;
		}
	}
	double* Tp_local = new double[F_n];
	double* freq1 = new double[F_n];
	double* omega1 = new double[F_n];
	double pi = 3.1415926;
	item = 0;
	int j = 0;
	for (int i = 0; i < hydroline;i++)
	{
		if (I[i][0] != item)
		{
			item = I[i][0];
			Tp_local[j] = item;
			freq1[j] = 1 / Tp_local[j];
			omega1[j] = 2 * pi * freq1[j];
			j++;
		}
	}
	period = Tp_local;
	freq = freq1;
	hydroValue = I;
	omega = omega1;

	myfile.close();

	return 0;
}