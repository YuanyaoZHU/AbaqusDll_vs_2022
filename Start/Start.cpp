﻿// Start.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <Windows.h>
#include"Start.h"
#include"zhu_app.h"

using namespace std;


int main()
{
      
    
    HINSTANCE h = LoadLibrary(L"AbaqusDll.dll");
    typedef int (*START)();
    START start = (START)GetProcAddress(h, "start_zhu");

    string CaseIP;
    string modelicaCall = "ModelicaCall";
    string abaqusCall = "AbaqusCall";
    string abaqusCall2 = "AbaqusCall2";
    
    cout << "Please input the Case ID" << endl;

    cin >> CaseIP;
    string Event_ModelicaCall = modelicaCall + CaseIP;
    string Event_AbaqusCall = abaqusCall + CaseIP;
    string Event_AbaqusCall2 = abaqusCall2 + CaseIP;

    int len = MultiByteToWideChar(CP_UTF8, 0, Event_ModelicaCall.c_str(), -1, NULL, 0);
    wchar_t* wstr1 = new wchar_t[len];
    MultiByteToWideChar(CP_UTF8, 0, Event_ModelicaCall.c_str(), -1, wstr1, len);

    len = MultiByteToWideChar(CP_UTF8, 0, Event_AbaqusCall.c_str(), -1, NULL, 0);
    wchar_t* wstr2 = new wchar_t[len];
    MultiByteToWideChar(CP_UTF8, 0, Event_AbaqusCall.c_str(), -1, wstr2, len);

    len = MultiByteToWideChar(CP_UTF8, 0, Event_AbaqusCall2.c_str(), -1, NULL, 0);
    wchar_t* wstr3 = new wchar_t[len];
    MultiByteToWideChar(CP_UTF8, 0, Event_AbaqusCall2.c_str(), -1, wstr3, len);

    HANDLE event1 = CreateEvent(NULL, FALSE, FALSE, wstr1); //本段时将原来固定的Event名称改为可变的名称。
    HANDLE event2 = CreateEvent(NULL, FALSE, FALSE, wstr2);
    HANDLE event4 = CreateEvent(NULL, FALSE, FALSE, wstr3);

    if (event1 == NULL)
    {
        cout << "Failed create event1" << endl;
        GetLastError();
        getchar();
        return 0;
    }
    else
    {
        cout << "Create Event 'event1' Success!" << endl;
    }

    if (event2 == NULL)
    {
        cout << "Failed create event2" << endl;
        GetLastError();
        getchar();
        return 0;
    }
    else
    {
        cout << "Create Event 'event2' Success!" << endl;
    }

    if (event4 == NULL)
    {
        cout << "Failed create event4" << endl;
        GetLastError();
        getchar();
        return 0;
    }
    else
    {
        cout << "Create Event 'event4' Success!" << endl;
    }

    //HANDLE event1 = CreateEvent(NULL, FALSE, FALSE, L"Modelica调用");//创建阻塞Modelica的Event,本Event用于卡住Modelica计算进程
    //HANDLE event2 = CreateEvent(NULL, FALSE, FALSE, L"ABAQUS调用");//创建阻塞ABAQUS的Event，本Event用于卡住ABAQUS计算进程
                        //HANDLE event3 = CreateEvent(NULL, FALSE, FALSE, L"Modelica调用2");//创建阻塞Modelica的Event2,本Event用于确认卡住Modelica计算进程
    //HANDLE event4 = CreateEvent(NULL, FALSE, FALSE, L"ABAQUS调用2");//创建阻塞ABAQUS的Event2，本Event用于卡住ABAQUS计算进程

  
    start();

    

    /// <summary>
    /// 释放abaqus线程，关闭ABAQUS仿真之后，再“点击任意键继续”
    /// </summary>
    /// <returns></returns>
    //HANDLE event5 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用");
    HANDLE event5 = OpenEvent(EVENT_ALL_ACCESS, FALSE, wstr2);
    if (event5 == NULL)
    {
        cout << "Failed open event5" << endl;
        GetLastError();
        getchar();
        return 0;
    }
    else
    {
        cout << "OpenEvent 'event5' Success!" << endl;
    }

    SetEvent(event5);//发出信号，告诉ABAQUS这边已经计算完成
    cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
    HANDLE event6 = OpenEvent(EVENT_ALL_ACCESS, FALSE, wstr3);
    //HANDLE event6 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用2");
    if (event6 == NULL)
    {
        cout << "Failed open event6" << endl;
        GetLastError();
        getchar();
        return 0;
    }
    else
    {
        cout << "OpenEvent 'event6' Success!" << endl;
    }

    SetEvent(event6);//发出信号，告诉ABAQUS这边已经计算完成
    cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
    
    delete[] wstr1;
    delete[] wstr2;
    delete[] wstr3;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
