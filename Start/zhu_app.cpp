#include<iostream>
#include <fstream>
#include <cstring>
#include<algorithm>
#include"zhu_app.h"
using namespace std;


int input_zhu::kk()
{
	cout << "构造函数运行！" << endl;
	const string srcFileName = "./Panels.txt"; //输入Panels.txt文件名
	int panels1[13][2];
	double panels2[6][2];

	cout << "构造函数运行！" << endl;


	ifstream fin(srcFileName, ios::in);
	if (!fin)
	{
		std::cerr << "open error!" << endl;
		fin.close();
		return 0;
	}
	
	for (int j = 0;j < 2;j++)
	{
		for (int i = 0;i < 13;i++)
		{
			fin >> panels1[i][j];
		}
		for (int i = 0;i < 6;i++)
		{
			fin >> panels2[i][j];
		}
		for (int i = 0;i < 13;i++)
		{
			cout << panels1[i][j ]<< "    ";
		}
		for (int i = 0;i < 6;i++)
		{
			cout << panels2[i][j] << "    ";
		}
		cout << endl;
	}

	
    
	return 0;

}