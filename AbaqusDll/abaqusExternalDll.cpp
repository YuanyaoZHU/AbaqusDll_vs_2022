#include<iostream>
#include<cmath>
#include<string>
#include<Windows.h>
#include<stdlib.h>
#include<stdio.h>
#include<fstream>
#include<Eigen/Dense>  
#include"pch.h"
#include"abaqusExternalDll.h"
#include"ReadOutFile.h"
#include"waveGenerate.h"

using namespace std;
using namespace Eigen;

#pragma data_seg("shared")
double DLOAD_A[6] = { 0 };
double DLOAD_M[6] = { 0 };
double DISP_A[6] = { 0 };
double DISP_M[6] = { 0 };
double VELOCITY_A[6] = { 0 };
double VELOCITY_M[6] = { 0 };
double ACCEL_ZHU_A[6] = { 0 };
double ACCEL_ZHU_M[6] = { 0 };
double ROTATION_A[3] = { 0 };
double ROTATION_M[3] = { 0 };
double TIME = -1;//modelica当前时间，在Modelica调用函数的尾部进行更新
double TIME2 = 0;//
int N = 0;
double DTIME = 0;
double dTIME = 0.01;
double TIME3 = 0;
double TIME4 = -1;

//以下参数用于计算ABAQUS差时参数
double DISP_T1[6] = { 0 };
double DISP_T2[6] = { 0 };
double VELOCITY_T1[6] = { 0 };
double VELOCITY_T2[6] = { 0 };
double ACCEL_ZHU_T1[6] = { 0 };
double ACCEL_ZHU_T2[6] = { 0 };
int P = 0;//用于对差额计算步的计数
int Q = -1;//与P搭配使用，P是记录总数，Q记录当前步数
bool flag_for_err = TRUE;//用于标明首次创建err.txt文件
bool flag_for_disp = TRUE;//用于标明是否在abaqus的log文件中写入信息。TRUE为写入，FALSE为不写入
double time_flag = -1;//用于标明在abaqus的log文件中是否显示#6所描述的内容
double time_flag1[10] = { 0 };//用于在输出GETV.txt文件时只输出一次。
double time_for_disp = -1;

//-----------------------------------------
//以下为波浪参数
double rho = 1025; //海水密度
double miu = 1.01e-3;//dynamic viscosity, kinematic viscosity = miu/rho
double U1[15000] = { 0 };//高斯白噪声参量1
double U2[15000] = { 0 };//高斯白噪声参量2
double HydroValue[22200][7] = { 0 };//读取的水动力参数
double freq[200] = { 0 };//频率
double omega[200] = { 0 };//角频率
int F_n = 0;
int wave_mod = 0;//波浪种类，1为利用Jonswap谱生成波浪参数，2为规则波
double ragular_wave_phase = NULL;
double waterDepth = NULL;

double Hs = NULL;
double Tp = NULL;
double wave_incide_direction = NULL;

double k_w[200] = { 0 };//波数
double S[200] = { 0 };//波浪谱
double AW[200] = { 0 };
double domega = NULL;
double f1[200];

double current[3] = { 0 };




double DISP_3TO6[3] = { 0 };

//double a1[4] = { 0 }; //ABAQUS时间步内的拟合曲线系数
//double a2[4] = { 0 };
//double a3[4] = { 0 };
//double a4[4] = { 0 };
//double a5[4] = { 0 };
//double a6[4] = { 0 };
double aa[6][7] = { 0 };//第一维为自由度，第二维为参数序号

double NODES[20000][6] = { 0 };//节点，1维：节点编号，2维：[0-2]为位移，[3-5]为速度
double NODES0[20000][6] = { 0 };
int NODE[20000][5] = { 0 };//读取Nodes的编号,NODE[xx][0]为节点编号，Node[xx][1-4]为与该节点相连的单元编号。

int ELEM[30000][5] = { 0 };//单元，1维：单元编号；2维：[0]为Elements的编号，[1-2]包含的节点编号,[3-4]为包含该单元的Panel单元编号

int PANEL1[10000][13] = { 0 };//Panel单元[xx][0]为Panel单元编号，[xx][1-4]为包含的Node的编号（节点1,3和节点2,4组成的向量叉乘为其法向量），[5-8]为包含的Element的编号，[9-12]为Node的重新编号（从小到大排列）
double PANEL2[10000][6] = { 0 };//Panel单元[0-2]为Panel的方向向量，[3-5]为Panel单元的中心点坐标
double PANEL3[10000][3] = { 0 };//Panel单元中心的速度，2023.3.14增加
double PANEL4[10000] = { 0 }; //Panel单元的实时面积

int Panelsline = 0; //用于记录读取了多少个Panel单元，在start_zhu这updatePanels函数中用到

int NET_MODE = 0; //描述网衣采用的模型，1为morison，2为screen type.

int selectElement[10] = { 0 };//用来记录需要观察的Element号码。
int selectPanel[10] = { 0 };
int selectNode[10] = { 0 };

double dw = 0; //网衣属性
double lw = 0;
double Sn = 0; //密实度
double L_screen = 0;//经过群化后的网眼长度
double A_screen = 0;//经过群花后的网眼面积


bool initial_SETV = TRUE;
bool initial_SETCOORD = TRUE;
bool initial_GETV = TRUE;
bool IF_ELEM_VELOCITY = TRUE; //用于判断在计算Panel单元水流相对速度时是否计算Panel单元自身的速度
bool IF_COUPLING = TRUE; //用于判断是否和MODELICA耦合
bool IF_PANEL_AREA = TRUE; //判断是否实时更新PANEL单元的面积
bool IF_VELOCITY_FILTER = TRUE;//是否采用速度过滤，即速度小于某一值时，认定其为0，以保证系统的稳定性

double Velocity_Filter_L = 0;
double Velocity_Filter_G = 0;

int PanelMod = 0;//用于判断采用哪种PanelMod，与Cheng et al（2020）文章中的一致，1: S1, 2: S2, 3 :S3

char WorkPathChar[100] = { 0 };
int CharLength = 0;

//std::string WorkPath = "Aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

#pragma data_seg()
#pragma comment(linker,"/SECTION:shared,RWS")


extern "C" _declspec(dllexport) int start_zhu()
{
    double dtime;
    int r;
    string workpath;
    string label;
    //----------------------------------------------------------
    //读取基本参数
    fstream in;
	in.open("abaqusInitial.txt", ios::in);
    if(!in)
    {
        std::cerr << "open abaqusInitial.txt error!" << endl;
        in.close();
        return 0;
    }
    in >> label;
    in >> dtime;
    in >> label;
    in >> rho; //海水密度
    in >> label;
    in >> miu; //海水动力黏度
    in >> label;
    in >> Hs;
    in >> label;
    in >> Tp;
    in >> label;
    in >> wave_mod;
    in >> label;
    in >> waterDepth;
    in >> label;
    in >> wave_incide_direction;
    //getline(in, workpath);
    in >> label;
    in >> current[0] >> current[1] >> current[2];
    in >> label;
    in >> workpath;


    if (wave_mod == 1)
    {
        //cout << "Jonswap波谱生成中" << endl;
        in >> label;
        in >> ragular_wave_phase;
        ragular_wave_phase = 0;//Jonswap波谱中不需要对规则波相位进行定义
    }
    else if (wave_mod == 2)
    {
        //cout << "请输入规则波的相位" << endl;
        in >> label;
        in >> ragular_wave_phase;
    }
    else
    {
        in >> label;
        in >> ragular_wave_phase;
        cerr << "波浪类型输入错误,均按无波浪处理！" << endl;
    }
    in >> label;
    in >> NET_MODE;
    in >> label;
    in >> dw;
    in >> label;
    in >> lw;
    
    in >> label;
    in >> L_screen;
    A_screen = pow(L_screen, 2);
    in >> label;
    in >> IF_ELEM_VELOCITY; //这个参数是判断在计算Panel相对水流速度时是否计算Panel单元自身速度的
    in >> label;
    in >> IF_COUPLING; //判断是否与modelica耦合，修改于2023.3.14
    in >> label;
    in >> IF_PANEL_AREA;//判断是否更新Panel单元的面积
    in >> label;
    in >> IF_VELOCITY_FILTER; //判断是否采用速度过滤器以保证系统的稳定性
    in >> label;
    in >> Velocity_Filter_L;//小于本值时Panel单元速度强制为0
    in >> label;
    in >> Velocity_Filter_G;//大于本值时Panel单元速度强制为0
    in >> label;
    in >> PanelMod; //输入PanelMod


    in >> label;
    for (int i = 0;i < 10;i++)
    {
        in >> selectElement[i];
    }
    in >> label;
    in >> selectPanel[0] >> selectPanel[1] >> selectPanel[2] >> selectPanel[3] >> selectPanel[4] >> selectPanel[5] >> selectPanel[6] >> selectPanel[7] >> selectPanel[8] >> selectPanel[9];
    in >> label;
    in >> selectNode[0] >> selectNode[1] >> selectNode[2] >> selectNode[3] >> selectNode[4] >> selectNode[5] >> selectNode[6] >> selectNode[7] >> selectNode[8] >> selectNode[9];

    CharLength = workpath.length();
    strcpy_s(WorkPathChar, CharLength + 1, workpath.c_str());
    
    string WorkPath;
    WorkPath = WorkPathChar;
    
	in.close();

    cout << "come in abaqusDll!" << endl;
    cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
    cout << "Initail Parameter as follow:" << endl;
    cout << "dtime = " << dtime << endl;
    cout << "Water Density = " << rho << endl;
    cout << "Dynamic Viscosity = " << miu << endl;
    cout << "Hs = " << Hs << endl;
    cout << "Tp = " << Tp << endl;
    cout << "wave_mod = " << wave_mod << endl;
    cout << "wave_incide_direction = " << wave_incide_direction << endl;
    cout << "ragular_wave_phase = " << ragular_wave_phase << endl;
    cout << "Current(x,y,z)=" << current[0] << "    " << current[1] << "    " << current[2] << endl;
    cout << "WorkPath = " << WorkPath<<endl ;
    cout << "Sn = " << Sn << endl;
    cout << "Net_Mode = " << NET_MODE << endl;
    cout << "dw = " << dw << endl;
    cout << "lw = " << lw << endl;
    cout << "L_screen = " << L_screen << endl;
    cout << "IF_ELEM_VELOCITY = " << IF_ELEM_VELOCITY << endl;
    cout << "IF_COUPLING = " << IF_COUPLING << endl;
    cout << "IF_PANEL_AREA = " << IF_PANEL_AREA << endl;
    cout << "IF_VELOCITY_FILTER = " << IF_VELOCITY_FILTER << endl;
    cout << "Velocity_Filter_L = " << Velocity_Filter_L << endl;
    cout << "Velocity_Filter_G = " << Velocity_Filter_G << endl;
    cout << "PanelMod = " << PanelMod;
    cout << "selectElement:" << selectElement[0] << "  " << selectElement[1] << endl;
    cout << "selectPanel:" << selectPanel[0] << "    " << selectPanel[5] << endl;
    cout << "selectNode: " << selectNode[0] << "    " << selectNode[5] << endl;
	cout << "Initialized success!" << endl;
    cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
    cout << endl;
    //cout << "please enter increment time:" << endl;
    //----------------------------------------------------------
    //cin >> dtime;
    DTIME = dtime;
    P = dTIME / DTIME;

    double pi = 3.1415926;

    //---------------------------------------------------------------
    //输入波浪参数
    /*
    cout << "请输入Hs:" << endl;
    cin >> Hs;
    cout << "请输入Tp:" << endl;
    cin >> Tp;
    cout << "请输入波浪类型：(1为Jonswap谱生成非规则波，2为规则波）" << endl;
    cin >> wave_mod;
    cout << "请输入水深：" << endl;
    cin >> waterDepth;
    cout << "请输入入射角：" << endl;
    cin >> wave_incide_direction;
    */

    //TIME = 0;

    //DISP[2] = -1000;
//----------------------------------------------------------------------
    
    ReadOutFile readOutFile;//ReadOutFile为用于处理网衣有限元模型中单元、节点读入及处理的类
    readOutFile.ReadInNodes();//读取Nodes.txt文档
    readOutFile.ReadInElements();
    readOutFile.ReadRandom();//读取随机数
    readOutFile.ReadHydro();//读取水动力参数
    readOutFile.ReadInPanels();//读取Panels.txt文档


 
    //----------------------------------------------------------------------
    //将读取的随机量赋值给共享内存数据
    if (readOutFile.randomline > 15000)
    {
        cout << "输入的随机量数量大于15000个，无法完整读取所有随机量数据！" << endl;
        cout << "random_value.txt的随机量个数为：" << readOutFile.randomline << endl;

        system("pause");
    }

    for (int i = 0; i < readOutFile.randomline;i++)
    {
        U1[i] = readOutFile.randomValue[i][0];
        U2[i] = readOutFile.randomValue[i][1];

    }

    cout << "随机量读取完成！" << endl;

    //----------------------------------------------------------------------------------
    for (int i = 0;i < readOutFile.hydroline;i++)
    {
        for (int j = 0;j < 7;j++)
        {
            HydroValue[i][j] = readOutFile.hydroValue[i][j];
        }
    }
    F_n = readOutFile.F_n;
    if (F_n > 200)//检验读取的频率个数是否超过设定的数量
    {
        cout << "水动力文件中波浪频率个数大于200，共享内存保存的频率个数需小于200！" << endl;
        system("pause");
    }
    for (int i = 0;i < F_n;i++)//为水动力文件的频率进行赋值
    {
        freq[i] = readOutFile.freq[i];
        omega[i] = 2 * pi * freq[i];
    }
    domega = omega[1] - omega[0];//domega是用来计算频率间隔长度的

    cout << "水动力文件读取完成！" << endl;
    
    waveNumber WaveNumber(waterDepth);//获得波数
    
    for (int i = 0;i < F_n;i++)
    {
        k_w[i] = WaveNumber.newton(readOutFile.omega[i]);
    }
    cout << "波数生成完毕！" << endl;


    //----------------------------------------------------------------------------------
    //给波浪谱赋值
    wavespectral2 WaveSpectral(Hs, Tp);
    for (int i = 0;i < F_n;i++)
    {
        S[i] = WaveSpectral.Spectral(readOutFile.omega[i]);

        AW[i] = sqrt(2 * S[i] * domega);

        f1[i] = sqrt(-2 * log(U1[i]));//这里把能赋值的都先赋值了
    }


    //----------------------------------------------------------------------------------
    //r = readOutFile.ReadInElements();//读取Elements.txt文档
    
    //赋值给NODE变量
    for (int i = 0;i < 20000;i++)
    {
        for (int j = 0;j < 3;j++)
        {
            NODES[i][j] = readOutFile.Nodes1[i][j];
            NODES0[i][j] = NODES[i][j];
        }
        for (int j = 0;j < 5;j++)
        {
            NODE[i][j] = readOutFile.Node1[i][j];
        }
    }

    //赋值给ELEM变量
    
    for (int i = 0;i < 30000;i++)
    {
        for (int j = 0;j < 5;j++)
        {
            ELEM[i][j] = readOutFile.Element1[i][j];
        }
    }

    //赋值给PANEL1变量
    Panelsline = readOutFile.panelsline;
    for (int i = 0; i < Panelsline; i++)
    {
        for (int j = 0;j < 13;j++)
        {
            PANEL1[i][j] = readOutFile.Panel1[i][j];
        }
        for (int j = 0;j < 6;j++)
        {
            PANEL2[i][j] = readOutFile.Panel2[i][j];
        }
    }
    
    //输出少量数据验证NODES和NODE读取的结果正确
    cout << "NODES读取结果" << endl;
    for (int i = 0;i < 10;i++)
    {
        for (int j = 0;j < 3;j++)
        {
            cout<<NODES[i][j]<<"    ";
        }
        for (int j = 0;j < 5;j++)
        {
            cout<<NODE[i][j] << "    ";
        }
        cout << endl;
    }

    cout << "ELEM读取结果" << endl;
    for (int i = 0;i < 10;i++)
    {
        for (int j = 0;j < 5;j++)
        {
            cout << ELEM[i][j] << "    ";
        }
        cout << endl;
    }

    cout << "PANEL1读取结果" << endl;
    for (int i = 0;i < 10;i++)
    {
        for (int j = 0;j < 13;j++)
        {
            cout << PANEL1[i][j] << "    ";
        }
        for (int j = 0;j < 6;j++)
        {
            cout << PANEL2[i][j] << "    ";
        }
        cout << endl;
    }

    /**/
//---------------------------------------------------------------
    //建立elementForce,计算得到的力
    string elementForce = "elementForce.txt";
    
    string fileName = WorkPath + elementForce;
    ofstream fout(fileName, ios::out);
    if (!fout)
    {
        std::cerr << "open elementForce.txt error!" << endl;
        fout.close();
        return 0;
    }
    else
    {
        cout << "Open elementForce.txt success!" << endl;
    }
    fout.close();

    string elementForce1 = "elementForce1.txt";
    
    fileName = WorkPath + elementForce1;
    ofstream fout1(fileName, ios::out);
    if (!fout1)
    {
        std::cerr << "open elementForce1.txt error!" << endl;
        fout.close();
        return 0;
    }
    else
    {
        cout << "Open elementForce1.txt success!" << endl;
    }
    fout1.close();

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    string A_screen_char = "A_screen.txt";

    fileName = WorkPath + A_screen_char;
    ofstream fout2(fileName, ios::out);
    if (!fout2)
    {
        std::cerr << "open A_screen.txt error!" << endl;
        fout.close();
        return 0;
    }
    else
    {
        cout << "Open A_screen.txt success!" << endl;
    }
    fout.close();
    //--------------------------------------------------------

    string B_screen_char = "B_screen.txt";

    fileName = WorkPath + B_screen_char;
    ofstream fout3(fileName, ios::out);
    if (!fout3)
    {
        std::cerr << "open B_screen.txt error!" << endl;
        fout.close();
        return 0;
    }
    else
    {
        cout << "Open B_screen.txt success!" << endl;
    }
    fout.close();



//-----------------------------------------------------------
	system("pause");

	return 0;
}

extern "C" _declspec(dllexport) int Modelica2Dll(double time, double *disp, double* velocity, double* accel,double *dload) //
{
    /// <summary>
    /// 本函数由Modelica调用，用于存放位移，并获得Abaqus载荷
    /// </summary>
    /// <param name="time">输入变量，本时间步的时间，double类型</param>
    /// <param name="disp">输入变量，Modelica提供的全局位移</param>
    /// <param name="dload">输出变量，由ABAQUS计算的网衣载荷</param>
    /// <returns></returns>


    double Disp[6];
    double Velocity[6];
    double Accel[6];
    double Dload[6];
    double Pi = 3.1415926;
    double Rotation[3];
    //下面的参数用于计算ABAQUS差时参数
    double U1[6];
    double U2[6];
    double V1[6];
    double V2[6];
    double A1[6];
    double A2[6];

    //-----------------------
    //用于判断Modelica与ABAQUS的时间是否为同一时间步
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    cout << "进入Modelica2Dll程序！" << endl;

    TIME_INT = int(time * 10 / dTIME);
    TIME3_INT = int(TIME3 * 10 / dTIME);

    err = abs(TIME_INT - TIME3_INT);

    if (err <= 1 )
    {
        ERR = TRUE;
    }
    else
    {
        ERR = FALSE;
    }
    //-------------------------------------------------------
    //信息输出
    fstream errtxt;
    if (flag_for_err)
    {
        errtxt.open("err.txt", ios::out);
        if (!errtxt.is_open())
        {
            cout << "err.txt is not open!" << endl;
        }

        errtxt.close();
        errtxt.open("err.txt", ios::out);
        if (!errtxt.is_open())
        {
            cout << "disp_modelica.txt is not open!" << endl;
        }
        flag_for_err = FALSE;
        errtxt.close();
    }

    errtxt.open("err.txt", ios::app);
    if (!errtxt.is_open())
    {
        cout << "err.txt is not open!" << endl;
        system("pause");
    }
    errtxt << "time = " << time << "    TIME = " << TIME << "    TIME_INT = " << TIME_INT << "    TIME3_INT = " << TIME3_INT << "    err = " << err << "    ERR = " << ERR << endl;
    errtxt.close();
    //---------------------------------
    
    //cout << "#1 判断是否进入赋值步骤！" << "    time = " << time << "    " << "TIME = " << TIME << "    " << "TIME3 = " << TIME3 << "    ERR = "<< ERR << endl;
    if (time > TIME && ERR)
    {


        for (int i = 0;i < 3;i++)
        {
            Disp[i] = disp[i];
            DISP_M[i] = Disp[i]; //* 1000;//单位转换m到mm
            DISP_T2[i] = DISP_M[i];//给T2时刻位移赋值
            if (DISP_M[i] != DISP_A[i])
            {
                DISP_A[i] = DISP_M[i];
            }
            Velocity[i] = velocity[i];
            VELOCITY_M[i] = Velocity[i]; //* 1000;
            VELOCITY_T2[i] = VELOCITY_M[i];//给T2时刻速度赋值
            if (VELOCITY_M[i] != VELOCITY_A[i])
            {
                VELOCITY_A[i] = VELOCITY_M[i];
            }
            Accel[i] = accel[i];
            ACCEL_ZHU_M[i] = Accel[i];  // *1000;
            ACCEL_ZHU_T2[i] = ACCEL_ZHU_M[i];//给T2时刻加速度赋值
            if (ACCEL_ZHU_M[i] != ACCEL_ZHU_A[i])
            {
                ACCEL_ZHU_A[i] = ACCEL_ZHU_M[i];
            }
        }
        for (int i = 3;i < 6;i++)
        {
            Disp[i] = disp[i];
            ROTATION_M[i - 3] = Disp[i] - DISP_M[i];
            DISP_M[i] = Disp[i];//单位转换m到mm
            DISP_T2[i] = DISP_M[i];//给T2时刻位移赋值
            if (DISP_A[i] != DISP_M[i])
            {
                DISP_A[i] = DISP_M[i];
            }
            if (ROTATION_A[i-3] != ROTATION_M[i-3])
            {
                ROTATION_A[i-3] = ROTATION_M[i-3];
            }
            
            Velocity[i] = velocity[i];
            VELOCITY_M[i] = Velocity[i];
            VELOCITY_T2[i] = VELOCITY_M[i];//给T2时刻速度赋值
            if (VELOCITY_A[i] != VELOCITY_M[i])
            {
                VELOCITY_A[i] = VELOCITY_M[i];
            }
            Accel[i] = accel[i];
            ACCEL_ZHU_M[i] = Accel[i];
            ACCEL_ZHU_T2[i] = ACCEL_ZHU_M[i];////给T2时刻加速度赋值
            if (ACCEL_ZHU_A[i] != ACCEL_ZHU_M[i])
            {
                ACCEL_ZHU_A[i] = ACCEL_ZHU_M[i];
            }
        }
        //-----------------------------------------------------------------------
        //求解ABAQUS差值拟合系数a[1],a[2],a[3],a[4]
        for (int i = 0;i < 6;i++)//新建局部变量方便使用（原变量名太长）
        {
            U1[i] = DISP_T1[i];
            U2[i] = DISP_T2[i];
            V1[i] = VELOCITY_T1[i];
            V2[i] = VELOCITY_T2[i];
            A1[i] = ACCEL_ZHU_T1[i];
            A2[i] = ACCEL_ZHU_T2[i];
        }
        double dt;
        dt = dTIME;

        for (int i = 0;i < 6;i++)
        {
            aa[i][0] = 120 / pow(dt, 5) * (U2[i] - U1[i]) - 12 / pow(dt, 4) * (5 * V2[i] - 11 * V1[i]) + 1 / pow(dt, 3) * (7 * A2[i] - 10 * A1[i]); //aa[i][0]为公式中的a3
            aa[i][1] = -180 / pow(dt, 4) * (U2[i] - U1[i]) + 12 / pow(dt, 3) * (7 * V2[i] - 8 * V1[i]) - 3 / pow(dt, 2) * (3 * A2[i] - 6 * A1[i]);
            aa[i][2] = 60 / pow(dt, 3) * (U2[i] - U1[i]) - 12 / pow(dt, 2) * (2 * V2[i] + 3 * V1[i]) + 3 / dt * (A2[i] - 3 * A1[i]);
            aa[i][3] = A1[i];
            aa[i][4] = V1[i];
            aa[i][5] = U1[i];
            aa[i][6] = ACCEL_ZHU_T2[i] - ACCEL_ZHU_T1[i];

            cout << "aa = " << aa[i][0] << "    " << aa[i][1] << "    " << aa[i][2] << "    " << aa[i][3] << "    " << aa[i][4] << "    " << aa[i][5] << endl;
        }
        
        /*
        Matrix3d A;//A为系数矩阵
        Matrix3d Ainverse;//A的逆矩阵，方便运算
        Vector3d a[6];//6个自由度上对应的a3,a2,a1,注意：这里a[i](1)是公式推导中的a3, a[i](2)是a2, a[i](3)是a1  !!
        Vector3d S[6];//S是6自由度上的值矩阵
        A << pow(dt, 3), pow(dt, 2), dt,
            1 / 4 * pow(dt, 4), 1 / 3 * pow(dt, 3), 1 / 2 * pow(dt, 2),
            1 / 20 * pow(dt, 5), 1 / 12 * pow(dt, 4), 1 / 6 * pow(dt, 3);
        for (int i = 0; i < 6;i++)
        {
            S[i] << A2[i] - A1[i], V2[i] - V1[i] - A1[i] * dt, U2[i] - U1[i] - V1[i] * dt - 1 / 2 * A1[i] * pow(dt, 2);

            Ainverse = A.inverse();
            a[i] = Ainverse * S[i];
            for (int j = 0;j < 3;j++)
            {
                aa[i][j] = a[i](j);
            }
            aa[i][3] = A1[i];
            aa[i][4] = V1[i];//C1 = V1
            aa[i][5] = U1[i];//C2 = U1
            cout << "aa = " <<aa[i][0] << "    " << aa[i][1] << "    " << aa[i][2] << "    " << aa[i][3] << "    " << aa[i][4] << "    " << aa[i][5] << endl;

        }
        /*
        for (int j = 0;j < 3;j++)  //这里采用a1-a6的写法是考虑到C++与FORTRAN之间的行列相反，防止发生错误的一种写法
        {
            a1[j] = a[0](j);
            a2[j] = a[1](j);
            a3[j] = a[2](j);
            a4[j] = a[3](j);
            a5[j] = a[4](j);
            a6[j] = a[5](j);
        }
        a1[3] = A1[0];
        a2[3] = A1[1];
        a3[3] = A1[2];
        a4[3] = A1[3];
        a5[3] = A1[4];
        a6[3] = A1[5];
        */
        for (int i = 0;i < 6;i++)
        {
            DISP_T1[i] = DISP_T2[i];
            VELOCITY_T1[i] = VELOCITY_T2[i];
            ACCEL_ZHU_T1[i] = ACCEL_ZHU_T2[i];
        }


        //-----------------------------------------------------------------------
        //建立disp_modelica.txt文件，并将并将更新后的DISP数据输入进去。
        fstream in;
        if (time == 0)
        {
            in.open("disp_modelica.txt", ios::out);
            if (!in.is_open())
            {
                cout << "disp_modelica.txt is not open!" << endl;
            }

            in.close();
            in.open("force_modelica.txt", ios::out);
            if (!in.is_open())
            {
                cout << "force_modelica.txt is not open!" << endl;
            }

            in.close();
        }

        in.open("disp_modelica.txt", ios::app);
        if (!in.is_open())
        {
            cout << "disp_modelica.txt is not open!" << endl;
        }
        in << time << "    " << DISP_M[0]      << "    " << DISP_M[1]      << "    " << DISP_M[2]      << "    " << DISP_M[3]      << "    " << DISP_M[4]      << "    " << DISP_M[5]
                   << "    " << VELOCITY_M[0]  << "    " << VELOCITY_M[1]  << "    " << VELOCITY_M[2]  << "    " << VELOCITY_M[3]  << "    " << VELOCITY_M[4]  << "    " << VELOCITY_M[5]
                   << "    " << ACCEL_ZHU_M[0] << "    " << ACCEL_ZHU_M[1] << "    " << ACCEL_ZHU_M[2] << "    " << ACCEL_ZHU_M[3] << "    " << ACCEL_ZHU_M[4] << "    " << ACCEL_ZHU_M[5]<<endl;
        in.close();

        TIME = time;
        TIME3 = TIME + dTIME;
        //cout <<"#2 TIME进行更新！" <<"    TIME = " << TIME << "    " << "    TIME3= " << TIME3 << endl; //这一步在耦合控制之前
       
        if (time >= dTIME)//这个判断是用来让程序在0时刻不耦合
        {
            //---------------------------------------------------------------------
            //调用线程工具，告诉ABAQUS可以运行
            HANDLE event1 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用");
            HANDLE event3 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用2");
            if (event1 == NULL)
            {
                cout << "Failed open event1" << endl;
                GetLastError();
                getchar();
                return 0;
            }
            else
            {
                cout << "OpenEvent 'event1' Success!" << endl;
            }
            if (event3 == NULL)
            {
                cout << "Failed open event3" << endl;
                GetLastError();
                getchar();
                return 0;
            }
            else
            {
                cout << "OpenEvent 'event3' Success!" << endl;
            }

            SetEvent(event1);//发出信号，告诉ABAQUS这边已经计算完成
            SetEvent(event3);//发出信号，告诉ABAQUS这边已经计算完成
            cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
            cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
            

            //---------------------------------------------------------------------
            //阻塞Modelica线程，等待ABAQUS发出进行信号
            /*
            const int EVENT_NUM = 2;

            HANDLE hObject[EVENT_NUM] = { NULL,NULL };

            hObject[0] = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica调用");
            hObject[1] = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica调用2");
            WaitForMultipleObjects(EVENT_NUM, hObject, TRUE, INFINITE);
            */
            //WaitForSingleObject(event2, INFINITE);//等待ABAQUS的DLOAD输入
            //----------------------------------------------------------------------

            //---------------------------------------------------------------------
            //阻塞Modelica线程，等待ABAQUS发出进行信号
            HANDLE event4 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica调用");
            WaitForSingleObject(event4, INFINITE);//等待ABAQUS的DLOAD输入
            if (event4 == NULL)
            {
                cout << "Failed Open event4" << endl;
                GetLastError();
                getchar();
                return 0;
            }
            else
            {
                cout << "OpenEvent 'event4' Success!" << endl;
            }
        }

        
        ////////////////////////////////////////////////////////////////////////////
        //以上内容为线程控制
    }
    TIME = time;

    /*if (TIME != TIME3)
    {
        cout << "TIME=    " << TIME << "    TIME3=    " << TIME3 << "    TIME_INT=    " << TIME_INT << "    TIME_INT3=    " << TIME3_INT << endl;
    }
    */



    //cout << "NEW    " << time << "    " << DISP_M[0] << "    " << DISP_M[1] << "    " << DISP_M[2] << "    " << DISP_M[3] << "    " << DISP_M[4] << "    " << DISP_M[5] << endl;
        //Dload[0] = 10;//物体在x轴上收到10N的力

    //-----------------------------------------------------------------
    //输入ABAQUS更新后的载荷
    for (int i = 0;i < 6;i++)
    {
        Dload[i] = DLOAD_M[i];
        dload[i] = -Dload[i];
    }
    fstream in;
    in.open("force_modelica.txt", ios::app);
    if (!in.is_open())
    {
        cout << "force_modelica.txt is not open!" << endl;
    }
    in << time << "    " << dload[0] << "    " << dload[1] << "    " << dload[2] << "    " << dload[3] << "    " << dload[4] << "    " << dload[5] << endl;
    in.close();
	return 0;
}


extern "C" _declspec(dllexport) int abaqusdllurdfil(double *time, double* dload)
{
    double Dload[6];
    //TIME3 = time[0] + DTIME;
    //cout << "#3 abaqusdllurdfil刚进来时，各全局时间展示!" << "    TIME =  " << TIME <<"    TIME3 =  " << TIME3 << "    time[0]=    " << time[0] << endl;

    for (int i = 0;i < 6;i++)
    {
        Dload[i] = dload[i];
        DLOAD_A[i] = Dload[i];
        DLOAD_M[i] = DLOAD_A[i];
    }
    //---------------------------------------
    //判断时间步是否一致
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    TIME_INT = int(time[0] * 10 / DTIME);
    TIME3_INT = int(TIME * 10 / DTIME);//int(TIME3 * 10 / DTIME); 这里URDFIL子程序是在ABAQUS本步末尾处进行调用的，此时time[0]还未更新，理论上是等这一步调用完了，才进入到DISP新一步的调用，2022.10.16

    err = abs(TIME_INT - TIME3_INT);

    if (IF_COUPLING) //当IF_COUPLING为TRUE时，才与MODELICA进行耦合
    {

        if (err <= 1)
        {
            ERR = TRUE;
            cout << "ERR1 is ture!" << endl;

        }
        else
        {
            ERR = FALSE;
        }
        //---------------------------------
        if (ERR)  //增加这个判断，当ABAQUS与Modelica时间步相同时，才执行阻塞事件
        {
            cout << "#4 UDFILE程序中ERR判断为真时，各时间状态!" << "    TIME_INT=" << TIME_INT << "    TIME3_INT=" << TIME3_INT << "    time[0]=    " << time[0] << "    TIME3=  " << TIME3 << endl;
            HANDLE event1 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica调用");
            if (event1 == NULL)
            {
                cout << "Failed open event1" << endl;
                GetLastError();
                getchar();
                return 0;
            }
            else
            {
                cout << "OpenEvent 'event1' Success!" << endl;
            }

            SetEvent(event1);//发出信号，告诉Modelica这边已经计算完成
            cout << "Send signal to tell Modelica DLOAD have been writen..." << endl;

            HANDLE event2 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用");
            WaitForSingleObject(event2, INFINITE);//等待Modelica的DISP输入
            if (event2 == NULL)
            {
                cout << "Failed Open event2" << endl;
                GetLastError();
                getchar();
                return 0;
            }
            else
            {
                cout << "OpenEvent 'event2' Success!" << endl;
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////
    //以上内容为线程控制
    return 0;
}

extern "C" _declspec(dllexport) int abaqusdlldisp(double *time, double* disp, double* velocity, double* accel_zhu)
{
    
    double Disp[6];
    double Disp1[6];
    double Velocity[6];
    double Accel[6];
    double t;
    double t1;

    //---------------------------------------
    //判断时间步是否一致
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    //TIME4 = time[1] + DTIME; //time[1]是步内时刻，time[0]是全局时刻，这里之前不知道为什么会用DISP里面的TIME(2)来对应TIME_ZHU(1)

    TIME_INT = int(time[1] * 10 / DTIME);
    TIME3_INT = int(TIME3 * 10 / DTIME);

    err = abs(TIME_INT - TIME3_INT);

    //cout << "#5 Disp刚进入后各时间！"<< "    TIME_INT=  " << TIME_INT << "    TIME3_INT=  " << TIME3_INT <<"    time[1]=  " << time[1] << "    time[0]=  " << time[0] <<"    TIME2=  "<< TIME2 <<endl;


    if (time[0] != TIME2)
    {
        Q = Q + 1;
        //cout << "#7 Q的值    Q = "<< Q <<endl;
    }
    if (Q == P)//if (err <= 1 )
    {
        ERR = TRUE;
        cout << "ERR2 is ture!" << endl;
        Q = 0;
    }
    else
    {
        ERR = FALSE;
    }

    if (time[0] != TIME2 && N==1 && ERR)//N的作用是防止程序第一次调用就卡住了，因为time[0]在首次运行时是不等于TIME2的
    {
       
        //////////////////////////////////////////////////////////////////////////
        //----------------------------------------------------------------------//
        //当全局时间更新时，本步将堵住ABAQUS的运行，直到Modelica更新完DISP，
        //这样做主要是因为按照理论上讲，DISP子程序应该在URDFIL之后运行，但是
        //在测试时，DISP程序会有几率跑到URDFIL前面运行，导致写在URDFIL中的线
        //程控制事件不能有效的在时间步更新时卡住ABAQUS，所以这里做了双保险，
        //以控制位移的有效更新。
        //
        //2022.10.14记录
        //----------------------------------------------------------------------//
        //////////////////////////////////////////////////////////////////////////
        HANDLE event2 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS调用2");
        
        if (event2 == NULL)
        {
            cout << "Failed Open ABAQUS_2" << endl;
            GetLastError();
            getchar();
            return 0;
        }
        else
        {
            cout << "OpenEvent 'ABAQUS_2' Success!" << endl;
        }
        //ResetEvent(event2);
        WaitForSingleObject(event2, INFINITE);//等待Modelica的DISP输入
        ////////////////////////////////////////////////////////////////////////////
    }
    //------------------------------------------------------------------------------
    N = 1;
    //------------------------------------------------------------------------------
    //本段程序用于计算ABAQUS在Modelica时间步中间部分的运动
    t = time[1] - TIME + dTIME;
    if (DTIME < dTIME)
    {
        t = t - DTIME;
    }
    t1 = t;

    if (time_flag != time[1])
    {
        cout << "#6 中间步的时间计算！   t=  " << t << "    time[1]=  " << time[1] << "    TIME=  " << TIME << "    dTIME=  " << dTIME << endl;

        time_flag = time[1];
    }
    
    
    for (int i = 0;i < 6;i++)
    {
        Accel[i] = aa[i][6]/dTIME*t+aa[i][3];
        // Accel[i] = double(aa[i][0] * pow(t, 3) + aa[i][1] * pow(t, 2) + aa[i][2] * t + aa[i][3]);
        //cout << " Accel["<< i <<"] = " << Accel[i] << endl;
        Velocity[i] = double(1 / 4 * aa[i][0] * t*t*t*t + 1 / 3 * aa[i][1] * t*t*t + 1 / 2 * aa[i][2] * t*t + aa[i][3] * t + aa[i][4]);
        //cout << "   Velocity[" << i << "] = " << Velocity[i] ;
        Disp[i] = double(1 / 20 * aa[i][0] * pow(t, 5) + 1 / 12 * aa[i][1] * pow(t, 4) + 1 / 6 * aa[i][2] * pow(t, 3) + 1 / 2 * aa[i][3] * pow(t, 2) + aa[i][4] * t + aa[i][5]); 
        Disp1[i] = double(1 / 20 * aa[i][0] * pow(t1, 5) + 1 / 12 * aa[i][1] * pow(t1, 4) + 1 / 6 * aa[i][2] * pow(t1, 3) + 1 / 2 * aa[i][3] * pow(t1, 2) + aa[i][4] * t1 + aa[i][5]);
    }

    if (t!=time_for_disp)//此步用于输出当前abaqus获得的位移、速度、加速度，到abaqusDisp.txt文件中
    {
        string abaqusDisp = "abaqusDisp.txt";
        string WorkPath = WorkPathChar;
        string fileName = WorkPath + abaqusDisp;
        if (flag_for_disp)//初次打开abaqusDisp.txt
        {
            fstream fout(fileName, ios::out);
            if (!fout)
            {
                std::cerr << "open abaqusDisp.txt error!" << endl;
                fout.close();
                return 0;
            }
            else
            {
                cout << "Open abaqusDisp.txt success!" << endl;
            }
            fout.close();
            flag_for_disp = FALSE;
        }

        fstream fout(fileName, ios::app);
        if (!fout)
        {
            std::cerr << "open abaqusDisp.txt error!" << endl;
            fout.close();
            return 0;
        }
        else
        {
            if (Q == 1)
            {
                fout << "-------------------------------------------------------------------------" << endl;
            }
            fout << "time0 = " << time[0] << "    time1 = "<< time[1]<<"     t = " << t << "    Disp[1-6] = " << Disp[0] << "    " << Disp[1] << "    " << Disp[2] << "    " << Disp[3] << "    " << Disp[4] << "    " << Disp[5] << endl;
            fout << "time0 = " << time[0] << "    time1 = " << time[1] << "     t = " << t << "    Velocity[1-6] = " << Velocity[0] << "    " << Velocity[1] << "    " << Velocity[2] << "    " << Velocity[3] << "    " << Velocity[4] << "    " << Velocity[5] << endl;
            fout << "time0 = " << time[0] << "    time1 = " << time[1] << "     t = " << t << "    Accel[1-6] = " << Accel[0] << "    " << Accel[1] << "    " << Accel[2] << "    " << Accel[3] << "    " << Accel[4] << "    " << Accel[5] << endl;
            fout << "aa[1-6][6]=  " << aa[0][6] << "    " << aa[1][6] << "    " << aa[2][6] << "    " << aa[3][6] << "    " << aa[4][6] << "    " << aa[5][6] << endl;
         
            fout << endl;

        }
        fout.close();
    }
    time_for_disp = t;
   
    for (int i = 0;i < 3;i++)
    {
        disp[i] = Disp[i];
        velocity[i] = Velocity[i];
        accel_zhu[i] = Accel[i];
    }
    for (int i = 3;i < 6;i++)
    {
        disp[i] = Disp[i]-aa[i][5];
        velocity[i] = Velocity[i];
        accel_zhu[i] = Accel[i];
        //if (TIME2 != time[0])
        //{
        //    DISP_3TO6[i - 3] = Disp[i];
        //}        
        //cout << " disp[" << i << "] = " << disp[i] << endl;
    }
    // 由于采用的是直接计算拟合曲线的系数，而非矩阵法，在初步计算时，计算结果不会发生奇异性，所以没必要在开始时将位置强制设置成0
    if (time[1] < 3* dTIME)//这是为了在0时刻时，防止发生奇异矩阵
    {
        for (int i = 0;i < 6;i++)
        {
            disp[i] = 0;
            velocity[i] = 0;
            accel_zhu[i] = 0;
        }
    }
    

    /*
    for (int i = 0;i < 3;i++)
    {
        Disp[i] = DISP_A[i];
        disp[i] = Disp[i];
        Velocity[i] = VELOCITY_A[i];
        velocity[i] = Velocity[i];
        Accel[i] = ACCEL_ZHU_A[i];
        accel_zhu[i] = Accel[i];
    }
    for (int i = 3;i < 6;i++)
    {
        Disp[i] = ROTATION_A[i - 3];
        disp[i] = Disp[i];
        Velocity[i] = VELOCITY_A[i];
        velocity[i] = Velocity[i];
        Accel[i] = ACCEL_ZHU_A[i];
        accel_zhu[i] = Accel[i];
    }
    
    cout << "DISP has been written!" << endl;
    cout << "DISP    " << time[0] << "    " << disp[0] << "    " << disp[1] << "    " << disp[2] << "    " << disp[3] << "    " << disp[4] << "    " << disp[5] << endl;
    cout << "Velocity" << time[0] << "    " << velocity[0] << "    " << velocity[1] << "    " << velocity[2] << "    " << velocity[3] << "    " << velocity[4] << "    " << velocity[5] << endl;
    cout<< "Accel  " <<time[0] << "    " << accel_zhu[0] << "    " << accel_zhu[1] << "    " << accel_zhu[2] << "    " << accel_zhu[3] << "    " << accel_zhu[4] << "    " << accel_zhu[5] << endl;
    */
    TIME2 = time[0];
    return 0;
}

extern "C" _declspec(dllexport) int setv(int* ELM_NO, double* time, double* V)
{
    int EN;
    EN = *ELM_NO;
    NODES0[EN - 1][3] = NODES[EN - 1][3]; //之前有错误
    NODES0[EN - 1][4] = NODES[EN - 1][4];
    NODES0[EN - 1][5] = NODES[EN - 1][5];
    NODES[EN - 1][3] = V[0];
    NODES[EN - 1][4] = V[1];
    NODES[EN - 1][5] = V[2];
    ///* 2022年10月19日注释掉，减少非必要输出，保证运行速度
    if (initial_SETV)
    {
        string SETV = "SETV.txt";
        string WorkPath = WorkPathChar;
        string fileName = WorkPath + SETV;

        //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\SETV.txt";
        ofstream fout(fileName, ios::out);
        if (!fout)
        {
            std::cerr << "open SETV.txt error!" << endl;
            fout.close();
            return 0;
        }
        else
        {
            cout << "Open SETV.txt success!" << endl;
        }
        fout.close();
        initial_SETV = FALSE;

    }

    int selectElem[10];
    for (int i = 0; i < 10; i++)
    {
        selectElem[i] = selectElement[i];
    }
    //int selectNode[3] = { 969,979,975 };

    for (int i = 0;i < 10;i++)
    {
        if (EN == selectNode[i])
        {
            string SETV = "SETV.txt";
            string WorkPath = WorkPathChar;
            string fileName;
            fileName = WorkPath + SETV;
            //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\SETV.txt";
            ofstream fout(fileName, ios::app);
            if (!fout)
            {
                std::cerr << "open SETV.txt error!" << endl;
                fout.close();
                return 0;
            }
            else
            {
                //cout << "Open SETV.txt success!" << endl;
                fout<< "time=    " << time[0] << "    NODES[" << EN << "][4-6] =    " << NODES[EN - 1][3] << "    " << NODES[EN - 1][4] << "    " << NODES[EN - 1][5] << endl;
            }
            fout.close();
            //cout << "NODES["<<i<<"][4] = " << NODES[EN - 1][3] << "    NODES[" << i << "][5] = " << NODES[EN - 1][4] << "    NODES[" << i << "][6] = " << NODES[EN - 1][5] << endl;
        }
    }
    //*/
    return 0;
}

extern "C" _declspec(dllexport) int setcoord(int* ELM_NO, double* time, double* COORD)
{
    int EN;
    EN = *ELM_NO;
    NODES0[EN - 1][0] = NODES[EN - 1][0];
    NODES0[EN - 1][1] = NODES[EN - 1][1];
    NODES0[EN - 1][2] = NODES[EN - 1][2];
    NODES[EN - 1][0] = COORD[0];
    NODES[EN - 1][1] = COORD[1];
    NODES[EN - 1][2] = COORD[2];
    ///* 2022年10月19日注释掉，减少非必要输出，保证运行速度
    if (initial_SETCOORD)
    {
        string SETCOORD = "SETCOORD.txt";
        string WorkPath = WorkPathChar;
        string fileName;
        fileName = WorkPath + SETCOORD;
        //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\SETCOORD.txt";
        ofstream fout(fileName, ios::out);
        if (!fout)
        {
            std::cerr << "open SETCOORD.txt error!" << endl;
            fout.close();
            return 0;
        }
        else
        {
            cout << "Open SETCOORD.txt success!" << endl;
        }
        fout.close();
        initial_SETCOORD = FALSE;
    }

    int selectElem[10];
    for (int i = 0; i < 10;i++)
    {
        selectElem[i] = selectElement[i];
    }
    //int selectNode[3] = { 969,979,975 };

    for (int i = 0;i < 10;i++)
    {
        if (EN == selectNode[i])
        {
            string SETCOORD = "SETCOORD.txt";
            string WorkPath = WorkPathChar;
            string fileName = WorkPath + SETCOORD;
            //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\SETCOORD.txt";
            ofstream fout(fileName, ios::app);
            if (!fout)
            {
                std::cerr << "open SETCOORD.txt error!" << endl;
                fout.close();
                return 0;
            }
            else
            {
                fout <<"time=    "<< time[0] << "    NODES[" << EN << "][1-3] =    " << NODES[EN - 1][0] << "    " << NODES[EN - 1][1] << "    " << NODES[EN - 1][2] << endl;
            }
            fout.close();
        }
    }
    //*/
    return 0;
}

extern "C" _declspec(dllexport) int getv(int* ELM_NO, double* time, double* VW, double* VN)
{
    /// <summary>
    /// 本函数用于获取水流相对该单元的法向速度
    /// </summary>
    /// <param name="ELM_NO">输入：单元编号</param>
    /// <param name="VW">输入：水流速度（矢量）</param>
    /// <param name="VN">输出：法向速度</param>
    /// <returns></returns>
    int EN;  //ELEM编号
    int NO[2]; //Element包含的 Node编号
    double V_N[2][3];//Node的速度

    Vector3d VT;//单元的切向量
    Vector3d VT_0;//
    Vector3d VT_1;//
    Vector3d VE;//单元的绝对速度
    Vector3d VR;//单元相对水流的速度
    Vector3d VP;//与单元切向量和单元相对水流速度都垂直的向量
    Vector3d NV;//法向量
    Vector3d VR_N;//相对速度沿单元法向的分量
    Vector3d VR_T;//相对速度沿单元切向的分量


    double L; //单元长度

    //找到ELEMENT包含的Node的编号
    EN = *ELM_NO;

    NO[0] = ELEM[EN - 1][1]-1;
    NO[1] = ELEM[EN - 1][2]-1;
    double t_up = time[0] - TIME + dTIME;
    double t_low = TIME3 - TIME;
    double tt = t_up / t_low;

    //将两个节点的速度分别赋值到V_N中
    V_N[0][0] = tt * (NODES[NO[0]][3] - NODES0[NO[0]][3]) + NODES0[NO[0]][3];
    V_N[0][1] = tt * (NODES[NO[0]][4] - NODES0[NO[0]][4]) + NODES0[NO[0]][4];
    V_N[0][2] = tt * (NODES[NO[0]][5] - NODES0[NO[0]][5]) + NODES0[NO[0]][5];

    V_N[1][0] = tt * (NODES[NO[1]][3] - NODES0[NO[1]][3]) + NODES0[NO[1]][3];
    V_N[1][1] = tt * (NODES[NO[1]][4] - NODES0[NO[1]][4]) + NODES0[NO[1]][4];
    V_N[1][2] = tt * (NODES[NO[1]][5] - NODES0[NO[1]][5]) + NODES0[NO[1]][5];

    //L = sqrt(pow(NODES[NO[0]][0] - NODES[NO[1]][0], 2) + pow(NODES[NO[0]][1] - NODES[NO[1]][1], 2) + pow(NODES[NO[0]][2] - NODES[NO[1]][2], 2));//计算单元长度
    
    VT_0(0) = NODES0[NO[1]][0] - NODES0[NO[0]][0];
    VT_0(1) = NODES0[NO[1]][1] - NODES0[NO[0]][1];
    VT_0(2) = NODES0[NO[1]][2] - NODES0[NO[0]][2];

    VT_1(0) = NODES[NO[1]][0] - NODES[NO[0]][0];
    VT_1(1) = NODES[NO[1]][1] - NODES[NO[0]][1];
    VT_1(2) = NODES[NO[1]][2] - NODES[NO[0]][2];

    VT(0) = tt * (VT_1(0) - VT_0(0)) + VT_0(0);
    VT(1) = tt * (VT_1(1) - VT_0(1)) + VT_0(1);
    VT(2) = tt * (VT_1(2) - VT_0(2)) + VT_0(2);
    //VT(1) = (time[0] - TIME + dTIME) / (TIME3 - TIME) * (VT_1(1) - VT_0(1)) + VT_0(1);
    //VT(2) = (time[0] - TIME + dTIME) / (TIME3 - TIME) * (VT_1(2) - VT_0(2)) + VT_0(2);

    //求两个节点之间的平均速度
    VE(0) = 0.5 * (V_N[0][0] + V_N[1][0]);
    VE(1) = 0.5 * (V_N[0][1] + V_N[1][1]);
    VE(2) = 0.5 * (V_N[0][2] + V_N[1][2]);

    //求水流相对单元的速度
    VR(0) = VW[0] - VE(0);
    VR(1) = VW[1] - VE(1);
    VR(2) = VW[2] - VE(2);
    /*
    if (EN == 1921)
    {
        cout << "VR[1921]=    " << VR(0) << "    " << VR(1) << "    " << VR(2) << endl;
    }
    if (EN == 1915)
    {
        cout << "VR[1915]=    " << VR(0) << "    " << VR(1) << "    " << VR(2) << endl;
    }
    if (EN == 1907)
    {
        cout << "VR[1878]=    " << VR(0) << "    " << VR(1) << "    " << VR(2) << endl;
    }
    if (EN == 1879)
    {
        cout << "VR[1879]=    " << VR(0) << "    " << VR(1) << "    " << VR(2) << endl;
    }
    */
    VT.normalize();

    
    //判断切向向量是否与相对速度的方向小于90°,如果大于90°，则需要将切向向量反向
    double radian_angle = atan2(VR.cross(VT).norm(), VR.transpose() * VT);
    double angle = radian_angle * 180 / 3.1415926;

    if (angle > 90)
    {
        VT = -VT;
        //cout << "#vt为负！" << endl;
    }

    VR_T = (VR.dot(VT)) * VT;

    VR_N = VR - VR_T;
    /* 采用上面更简便的方法，2022年10月19日注释
    //求法向量
    VP = TV.cross(VR);
    NV = VP.cross(TV);
    NV.normalize();

    //求沿法向量的相对速度
    double VR_NV;
    VR_NV = VR.dot(NV);
    VN1 = VR_NV * NV;
    
    if (EN == 1856)
    {
        cout << "VR_NV[1856]=    " << VR_NV << endl;
    }
    if (EN == 1855)
    {
        cout << "VR_NV[1855]=    " << VR_NV << endl;
    }
    if (EN == 1878)
    {
        cout << "VR_NV[1878]=    " << VR_NV << endl;
    }
    if (EN == 1879)
    {
        cout << "VR_NV[1879]=    " << VR_NV << endl;
    }
    
    double radian_angle = atan2(VR.cross(VN1).norm(), VR.transpose() * VN1);
    double angle = radian_angle * 180 / 3.1415926;
    
    if (angle > 90)
    {
        VN1 = -VN1;
        cout << "#vn为负！" << endl;
    }
    
    */
    //赋值输出
    VN[0] = VR_N(0);
    VN[1] = VR_N(1);
    VN[2] = VR_N(2);
    /* 2022年10月19日注释掉，减少非必要输出，保证运行速度
    if (EN == 1878)
    {
        cout <<"VN[1878]=    "<< VN[0] << "    " << VN[1] << "    " << VN[2] << "    " << endl;

    }
    if (EN == 1879)
    {
        cout << "VN[1879]=    " << VN[0] << "    " << VN[1] << "    " << VN[2] << "    " << endl;

    }
    if (EN == 1855)
    {
        cout << "VN[1855]=    " << VN[0] << "    " << VN[1] << "    " << VN[2] << "    " << endl;

    }
    if (EN == 1856)
    {
        cout << "VN[1856]=    " << VN[0] << "    " << VN[1] << "    " << VN[2] << "    " << endl;

    }
    */
    if (initial_GETV)
    {
        string GETV = "GETV.txt";
        string WorkPath = WorkPathChar;
        string fileName = WorkPath + GETV;
        //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\GETV.txt";
        ofstream fout(fileName, ios::out);
        if (!fout)
        {
            std::cerr << "open GETV.txt error!" << endl;
            fout.close();
            return 0;
        }
        else
        {
            cout << "Open GETV.txt success!" << endl;
        }
        fout.close();
        initial_GETV = FALSE;
    }
    

    int selectElem[10];

        
        for (int i = 0;i < 10;i++)
        {            
            selectElem[i] = selectElement[i];
            if (EN == selectElem[i] && time_flag1[i]!=time[0])
            {
                cout << "进入到GETV.txt输出系统！" << endl;
                string GETV = "GETV.txt";
                string WorkPath = WorkPathChar;
                string fileName = WorkPath + GETV;
                //string fileName = "E:\\ABAQUSworkspace\\ABAQUSWORKFILE\\Net4\\DirectoryName\\result\\GETV.txt";
                ofstream fout(fileName, ios::app);
                if (!fout)
                {
                    std::cerr << "open GETV.txt error!" << endl;
                    fout.close();
                    return 0;
                }
                else
                {
                    fout << "time=  " << time[0] << "    TIME=  " << TIME << "    ELEM[" << EN << "]=    " << ELEM[EN - 1][1] << "    " << ELEM[EN - 1][2] << endl;
                    //fout << "NODES[" << NO[0] << "][3-5]=    " << NODES[NO[0]][3] << "    " << NODES[NO[0]][4] << "    " << NODES[NO[0]][5] << endl;
                    //fout << "NODES[" << NO[1] << "][3-5]=    " << NODES[NO[1]][3] << "    " << NODES[NO[1]][4] << "    " << NODES[NO[1]][5] << endl;

                    //fout << "time=  " << time[0] << "    TIME=  " << TIME <<"   VE[" << EN << "][1-3] = " << VE(0) << "    " << VE(1) << "    " << VE(2) << endl;
                    fout << "time=  " << time[0] << "    TIME=  " << TIME << "   VR[" << EN << "][1-3] = " << VR_N(0) << "    " << VR_N(1) << "    " << VR_N(2) << endl;
                    fout << "time=  " << time[0] << "    VE[" << EN << "]= " << VE(0) << "    " << VE(1) << "    " << VE(2) << endl;
                    fout << "time=  " << time[0] << "    VR[" << EN << "]= " << VR(0) << "    " << VR(1) << "    " << VR(2) << endl;

                        //fout << "time=  " << time[0] << "    TIME=  " << TIME << "   VN[" << EN << "][1-3] = " << VR_N(0) << "    " << VR_N(1) << "    " << VR_N(2) << endl;
                }
                fout.close();
                time_flag1[i] = time[0];
            }
        }
    return 0;
}

extern "C" _declspec(dllexport) int getwaterv(int* NOEL, double* time, double* VW)
{
    double pi = 3.1415926;
    domega = freq[1]-freq[0];
    double time1 = time[0];
    double temp1=0;
    double temp2=0;
    double zeta=0;
    double zeta1=0;
    double k=0;
    double* vw = new double[3];
    double Vw[3];

    double COORD[3];//本COORD用于NET_MODE==1的情况下
    double COORD2[3][2];//本COORD2用于NET_MODE == 2的情况下
    if (NET_MODE == 1)
    {
        int NO1;
        int NO2;

        NO1 = ELEM[*NOEL - 1][1]-1;
        NO2 = ELEM[*NOEL - 1][2]-1;
        COORD[0] = 0.5 * (NODES[NO1][0] + NODES[NO1][0]);
        COORD[1] = 0.5 * (NODES[NO1][1] + NODES[NO1][1]);
        COORD[2] = 0.5 * (NODES[NO1][2] + NODES[NO1][2]);

        fluidVelocity(vw, time1, COORD);
        Vw[0] = vw[0] + current[0];//+current为水流速度，修改于2023.3.15
        Vw[1] = vw[1] + current[1];
        Vw[2] = vw[2] + current[2];
        VW[0] = Vw[0];
        VW[1] = Vw[1];
        VW[2] = Vw[2];
        
    }
    else if (NET_MODE == 2)
    {
        int NO1;
        int NO2;
        Vector3d Normal;
        Vector3d Ve_Water;
        //double* F1;
        //double* F2;
        double* F1 = new double[3];//所在Panel单元的力
        double* F2 = new double[3];//所在另一Panel单元的力
        double velocity_local;
        Vector3d VelocityPanel;



        NO1 = ELEM[*NOEL - 1][3]-1; //包含该ELEM的Panel单元编号
        NO2 = ELEM[*NOEL - 1][4]-1;
        if (NO1 < 0)//此步用于判断该节点是否为完整的PANEL单元，如等于0说明该节点并非PANEL单元
        {
            for (int i = 0;i < 3;i++)
            {
                F1[i] = 0;
            }
        }
        else
        {
            COORD2[0][0] = PANEL2[NO1][3];//将Panel单元中心点坐标赋值给COORD2
            COORD2[1][0] = PANEL2[NO1][4];
            COORD2[2][0] = PANEL2[NO1][5];
            COORD[0] = COORD2[0][0];
            COORD[1] = COORD2[1][0];
            COORD[2] = COORD2[2][0];
            fluidVelocity(vw, time1, COORD);
            Normal(0) = PANEL2[NO1][0];
            Normal(1) = PANEL2[NO1][1];
            Normal(2) = PANEL2[NO1][2];

            VelocityPanel(0) = PANEL3[NO1][0];
            VelocityPanel(1) = PANEL3[NO1][1];
            VelocityPanel(2) = PANEL3[NO1][2];
            velocity_local = VelocityPanel.norm();
            if (velocity_local < Velocity_Filter_L && IF_VELOCITY_FILTER)
            {
                PANEL3[NO1][0] = 0;
                PANEL3[NO1][1] = 0;
                PANEL3[NO1][2] = 0;
            }
            else if (velocity_local > Velocity_Filter_G && IF_VELOCITY_FILTER)
            {
                PANEL3[NO1][0] = PANEL3[NO1][0] / velocity_local * Velocity_Filter_G;
                PANEL3[NO1][1] = PANEL3[NO1][1] / velocity_local * Velocity_Filter_G;
                PANEL3[NO1][2] = PANEL3[NO1][2] / velocity_local * Velocity_Filter_G;
            }
            Ve_Water(0) = vw[0] - PANEL3[NO1][0] + current[0];//水质点相对速度Ve_Water = vw-vx;vw为水质点速度，vx为Panel单元速度，修改于2023.3.14
            Ve_Water(1) = vw[1] - PANEL3[NO1][1] + current[1];
            Ve_Water(2) = vw[2] - PANEL3[NO1][2] + current[2];
            calculateforce(F1, Normal, Ve_Water, NO1);
        }

        if (NO2 < 0)//此步用于判断该节点是否为完整的PANEL单元，如等于0说明该节点并非PANEL单元
        {
            for (int i = 0;i < 3;i++)
            {
                F2[i] = 0;
            }
        }
        else
        {
            COORD2[0][1] = PANEL2[NO2][3];//将Panel单元中心点坐标赋值给COORD2
            COORD2[1][1] = PANEL2[NO2][4];
            COORD2[2][1] = PANEL2[NO2][5];
            COORD[0] = COORD2[0][1];
            COORD[1] = COORD2[1][1];
            COORD[2] = COORD2[2][1];
            fluidVelocity(vw, time1, COORD);
            Normal(0) = PANEL2[NO2][0];
            Normal(1) = PANEL2[NO2][1];
            Normal(2) = PANEL2[NO2][2];

            VelocityPanel(0) = PANEL3[NO2][0];
            VelocityPanel(1) = PANEL3[NO2][1];
            VelocityPanel(2) = PANEL3[NO2][2];
            velocity_local = VelocityPanel.norm();
            if (velocity_local < Velocity_Filter_L && IF_VELOCITY_FILTER)
            {
                PANEL3[NO2][0] = 0;
                PANEL3[NO2][1] = 0;
                PANEL3[NO2][2] = 0;
            }
            else if (velocity_local > Velocity_Filter_G && IF_VELOCITY_FILTER)
            {
                PANEL3[NO1][0] = PANEL3[NO1][0] / velocity_local * Velocity_Filter_G;
                PANEL3[NO1][1] = PANEL3[NO1][1] / velocity_local * Velocity_Filter_G;
                PANEL3[NO1][2] = PANEL3[NO1][2] / velocity_local * Velocity_Filter_G;
            }

            Ve_Water(0) = vw[0] - PANEL3[NO2][0] + current[0];//水质点相对速度Ve_Water = vw-vx;vw为水质点速度，vx为Panel单元速度;修改于2023.3.14
            Ve_Water(1) = vw[1] - PANEL3[NO2][1] + current[1];
            Ve_Water(2) = vw[2] - PANEL3[NO2][2] + current[2];
            calculateforce(F2, Normal, Ve_Water, NO2);
        }


        
        for (int i = 0;i < 3;i++)
        {
            Vw[i] = (F1[i] + F2[i]) / 4;//VW此时作为输出不再是水的相对速度，而是单元的作用力
            VW[i] = Vw[i];
            
        }
        

        for (int i = 0;i < 10;i++)
        {
            if (*NOEL == selectNode[i])
            {
                //建立elementForce,计算得到的力
                string elementForce = "elementForce.txt";
                string WorkPath = WorkPathChar;
                string fileName = WorkPath + elementForce;
                ofstream fout(fileName, ios::app);
                if (!fout)
                {
                    std::cerr << "open elementForce.txt error!" << endl;
                    fout.close();
                    return 0;
                }
                else
                {
                    fout << *NOEL << "  VW[0-2] " << VW[0] << "    " << VW[1] << "    " << VW[2] << endl;

                }
                fout.close();
            }
        }
        

        delete[] F1;
        delete[] F2;
       
    }
    delete[] vw;
    /* 这部分整合成了fluidVelocity函数
    if (wave_mod == 1)
    {
        double* f2 = new double[F_n];
        double* f3 = new double[F_n];

        double* f4 = new double[F_n];
        double* f5 = new double[F_n];

        for (int i = 0;i < F_n;i++)
        {
            f2[i] = cos(omega[i] * time[0] + 2 * pi * U2[i] - k_w[i] * COORD[0]);
            f3[i] = omega[i] * cosh(k_w[i] * (COORD[2] + waterDepth)) / sinh(k_w[i] * waterDepth);
            if (isinf(f3[i]))
            {
                f3[i] = 0;
            }

            f4[i] = -sin(omega[i] * time[0] + 2 * pi * U2[i] - k_w[i] * COORD[0]);//注意《船舶与海洋工程环境载荷》上第13页描述，与本处的cos sin存在一定差别，如果用sin描述波浪，则和书上一致，如用cos描述波浪，则x方向为cos，z方向为-sin
            f5[i] = omega[i] * sinh(k_w[i] * (COORD[2] + waterDepth)) / sinh(k_w[i] * waterDepth);

            if (isinf(f5[i]))
            {
                f5[i] = 0;
            }
            temp1 = f1[i] * AW[i] * f2[i] * f3[i];
            temp2 = f1[i] * AW[i] * f4[i] * f5[i];

            zeta = zeta + temp1;
            zeta1 = zeta1 + temp2;
        }
 
        VW[0] = zeta * cos(wave_incide_direction * pi / 180);
        VW[1] = zeta * sin(wave_incide_direction * pi / 180);
        VW[2] = zeta1;

        delete[] f2;
        delete[] f3;
        delete[] f4;
        delete[] f5;
    }
    else if (wave_mod == 2)
    {
        waveNumber WaveNumber(waterDepth);
        double Omega = 2 * pi / Tp;
        k = WaveNumber.newton(Omega);
        double ff2 = cos( Omega * time[0] + pi/180 * ragular_wave_phase - k * COORD[0]);
        double ff3 = Omega * cosh(k * (COORD[2] + waterDepth)) / sinh(k * waterDepth);
        double ff4 = -sin(Omega * time[0] + pi / 180 * ragular_wave_phase - k * COORD[0]);
        double ff5 = Omega * sinh(k * (COORD[2] + waterDepth)) / sinh(k * waterDepth);
        zeta = 0.5 * Hs * ff2 * ff3;//0.5*Hs为波浪振幅
        zeta1 = 0.5 * Hs * ff4 * ff5;

        if (COORD[2] <0)
        {
            VW[0] = zeta * cos(wave_incide_direction * pi / 180);
            VW[1] = zeta * sin(wave_incide_direction * pi / 180);
            VW[2] = zeta1;
        }
        else
        {
            VW[0] = 0;
            VW[1] = 0;
            VW[2] = 0;
        }
        
        if (*NOEL == 18454 || *NOEL == 17844)
        {
            cout << "Hs=  " << Hs << "    ff2 = " << ff2 << "    ff3 = " << ff3 << "    ff5 = " << ff5 << "    k = " << k << "    cosh()= " << cosh(k * (COORD[2] + waterDepth)) << "    sinh()= " << sinh(k * (COORD[2] + waterDepth)) << "    sinh(k * waterDepth)= " << sinh(k * waterDepth) << "    K= " << k * (COORD[2] + waterDepth) << "    COORD[2]=" << COORD[2] << "    zeta=  " << zeta << "    zeta1=  " << zeta1 << endl;
            cout << "NO = "<<*NOEL <<"    VW[0-2]=" << VW[0] << "    " << VW[1] << "    " << VW[2] << endl;
        }
        
    }
    */


    return 0;
}

extern "C" _declspec(dllexport) int updatepanel()
{
    Vector3d e1;
    Vector3d e2;
    Vector3d Normal;
    Vector3d Panel_side[4];
    int Node_local[4];

    //以下变量用于计算四边形面积
    bool IF_TRUE_PANEL;
    double Panel_side_length[4];
    double S_local;
    double D_local;
    double m_local;
    double n_local;
    double sqrt_inner;
    double random;

    for (int i = 0;i < Panelsline;i++)
    {
        //首先更新法向量
        IF_TRUE_PANEL = TRUE;
        
        for (int j = 0; j < 4;j++)//panel单元的4个节点
        {
            if (PANEL1[i][j + 1] == 0)
            {
                IF_TRUE_PANEL = FALSE;
            }
            Node_local[j] = PANEL1[i][j+1]-1;
        }
        
        //为向量e1和e2赋值
        e1(0) = NODES[Node_local[2]][0] - NODES[Node_local[0]][0];
        e1(1) = NODES[Node_local[2]][1] - NODES[Node_local[0]][1];
        e1(2) = NODES[Node_local[2]][2] - NODES[Node_local[0]][2];

        e2(0) = NODES[Node_local[3]][0] - NODES[Node_local[1]][0];
        e2(1) = NODES[Node_local[3]][1] - NODES[Node_local[1]][1];
        e2(2) = NODES[Node_local[3]][2] - NODES[Node_local[1]][2];

        Normal = e1.cross(e2);
        Normal.normalize();

        PANEL2[i][0] = Normal(0);
        PANEL2[i][1] = Normal(1);
        PANEL2[i][2] = Normal(2);

        //---------------------------------
        //然后更新中心点位置

        PANEL2[i][3] = (NODES[Node_local[0]][0] + NODES[Node_local[1]][0] + NODES[Node_local[2]][0] + NODES[Node_local[3]][0]) / 4;
        PANEL2[i][4] = (NODES[Node_local[0]][1] + NODES[Node_local[1]][1] + NODES[Node_local[2]][1] + NODES[Node_local[3]][1]) / 4;
        PANEL2[i][5] = (NODES[Node_local[0]][2] + NODES[Node_local[1]][2] + NODES[Node_local[2]][2] + NODES[Node_local[3]][2]) / 4;
        

        //更新PANEL中心点的速度,更新于2023.3.14
        if (IF_ELEM_VELOCITY)//当IF_ELEM_VELOCITY为TRUE时，更新节点速度
        {
            for (int j = 0;j < 3;j++)
            {                
                PANEL3[i][j] = 0;                
            }
            for (int j = 0;j < 3;j++)
            {
                for (int k = 0;k < 4;k++)
                {
                    random = rand() / double(RAND_MAX)*0.00001;
                    PANEL3[i][j] = PANEL3[i][j] + NODES[Node_local[k]][j + 3];
                }
                PANEL3[i][j] = PANEL3[i][j] / 4;
            }
        }

        //更新PANEL单元的覆盖面积
        if (IF_PANEL_AREA)
        {
            if (IF_TRUE_PANEL)
            {
                for (int j = 0;j < 3;j++)
                {
                    Panel_side[0](j) = NODES[Node_local[1]][j] - NODES[Node_local[0]][j];
                    Panel_side[1](j) = NODES[Node_local[2]][j] - NODES[Node_local[1]][j];
                    Panel_side[2](j) = NODES[Node_local[3]][j] - NODES[Node_local[2]][j];
                    Panel_side[3](j) = NODES[Node_local[0]][j] - NODES[Node_local[3]][j];
                }
                for (int j = 0;j < 4;j++)
                {
                    Panel_side_length[j] = Panel_side[j].norm();
                }
                m_local = e1.norm();
                n_local = e2.norm();
                D_local = abs((pow(Panel_side_length[1], 2) + pow(Panel_side_length[3], 2))-(pow(Panel_side_length[0], 2)+ pow(Panel_side_length[2], 2)));//公式为D=abs{(b^2+d^2)-(a^2+c^2)}，添加注释于2023.3.18
                sqrt_inner = pow(2 * m_local * n_local, 2) - pow(D_local, 2);
                if (sqrt_inner <= 0)//对于一些网格发生变形后，可能发生折叠现象，这时有可能在平方根式下就出现了负值，这在计算中会引发错误，此时需要对其进行处理，如下：
                {
                    S_local = 0;
                                   }
                else
                {
                    //S_local = 1 / 4 * sqrt(sqrt_inner);
                    double test_s = sqrt(sqrt_inner);
                    S_local = 0.25 * test_s;
                   
                }
                
                PANEL4[i] = S_local;
                   
            }
            else //如果判断不是正常的PANEL单元，则将其面积归零
            {
                PANEL4[i] = 0;
            }
            
        }
        for (int j = 0;j < 10;j++)
        {
            if (i == selectPanel[j] - 1)
            {
                string B_screen_char = "B_screen.txt";
                string WorkPath = WorkPathChar;
                string fileName = WorkPath + B_screen_char;
                ofstream fout(fileName, ios::app);
                if (!fout)
                {
                    std::cerr << "open B_screen.txt error!" << endl;
                    fout.close();
                    return 0;
                }
                else
                {
                    fout << "  NO = " << i << "    Area = " << PANEL4[i] << endl;

                }
                fout.close();
            }
        }
        
        

        //更新完成

    }
    return 0;
}

void fluidVelocity(double VW[3], double time, double COORD[3])
{
    //double* VW = new double[3];
    double pi = 3.1415926;
    domega = freq[1] - freq[0];

    double temp1 = 0;
    double temp2 = 0;
    double zeta = 0;
    double zeta1 = 0;
    double k = 0;


    if (wave_mod == 1)
    {
        double* f2 = new double[F_n];
        double* f3 = new double[F_n];

        double* f4 = new double[F_n];
        double* f5 = new double[F_n];

        for (int i = 0;i < F_n;i++)
        {
            f2[i] = cos(omega[i] * time + 2 * pi * U2[i] - k_w[i] * COORD[0]);
            f3[i] = omega[i] * cosh(k_w[i] * (COORD[2] + waterDepth)) / sinh(k_w[i] * waterDepth);
            if (isinf(f3[i]))
            {
                f3[i] = 0;
            }

            f4[i] = -sin(omega[i] * time + 2 * pi * U2[i] - k_w[i] * COORD[0]);//注意《船舶与海洋工程环境载荷》上第13页描述，与本处的cos sin存在一定差别，如果用sin描述波浪，则和书上一致，如用cos描述波浪，则x方向为cos，z方向为-sin
            f5[i] = omega[i] * sinh(k_w[i] * (COORD[2] + waterDepth)) / sinh(k_w[i] * waterDepth);

            if (isinf(f5[i]))
            {
                f5[i] = 0;
            }
            temp1 = f1[i] * AW[i] * f2[i] * f3[i];
            temp2 = f1[i] * AW[i] * f4[i] * f5[i];

            zeta = zeta + temp1;
            zeta1 = zeta1 + temp2;
        }

        VW[0] = zeta * cos(wave_incide_direction * pi / 180);
        VW[1] = zeta * sin(wave_incide_direction * pi / 180);
        VW[2] = zeta1;

        delete[] f2;
        delete[] f3;
        delete[] f4;
        delete[] f5;
    }
    else if (wave_mod == 2)
    {
        waveNumber WaveNumber(waterDepth);
        double Omega = 2 * pi / Tp;
        k = WaveNumber.newton(Omega);
        double ff2 = cos(Omega * time + pi / 180 * ragular_wave_phase - k * COORD[0]);
        double ff3 = Omega * cosh(k * (COORD[2] + waterDepth)) / sinh(k * waterDepth);
        double ff4 = -sin(Omega * time + pi / 180 * ragular_wave_phase - k * COORD[0]);
        double ff5 = Omega * sinh(k * (COORD[2] + waterDepth)) / sinh(k * waterDepth);
        zeta = 0.5 * Hs * ff2 * ff3;//0.5*Hs为波浪振幅
        zeta1 = 0.5 * Hs * ff4 * ff5;

        if (COORD[2] < 0)
        {
            VW[0] = zeta * cos(wave_incide_direction * pi / 180);
            VW[1] = zeta * sin(wave_incide_direction * pi / 180);
            VW[2] = zeta1;
        }
        else
        {
            VW[0] = 0;
            VW[1] = 0;
            VW[2] = 0;
        }    

    }
    else if (wave_mod == 0)//当wave_mod为0时，表示无波浪，此时波浪引起的水质点速度为0
    {
        VW[0] = 0;
        VW[1] = 0;
        VW[2] = 0;
    }
    
    
    //cout << VW[0] << "    " << VW[1] << "    " << VW[2] << endl;
    //return VW;
}

void calculateforce(double F[3], Vector3d Normal, Vector3d vw, int NO_local)
{
    double pi = 3.1415926;
    double CD = 0;
    double CL = 0;

    //double F[3];
    //double* F = new double[3];
    Vector3d Fd;
    
    Vector3d Fl;

    Vector3d RotateAxis;

    Vector3d iL;//lift force unit direction vector
    double rou = 1025;
    double radian_angle = atan2(Normal.cross(vw).norm(), Normal.transpose() * vw);

    double angle = radian_angle * 180 / pi;
    if (angle > 90)
    {
        radian_angle = pi - radian_angle;
        angle = 180 - angle;
        Normal = -Normal;
    }

    if (PanelMod == 0)
    {
        Sn = 2 * dw / lw;
        CD = 0.04 + (-0.04 + Sn - 1.24 * pow(Sn, 2) + 13.7 * pow(Sn, 3)) * cos(radian_angle);
        CL = (0.57 * Sn - 3.54 * pow(Sn, 2) + 10.1 * pow(Sn, 3)) * sin(2 * radian_angle);
    }
    else if (PanelMod == 1)
    {
        Sn = dw * (2 * lw + 0.5 * dw) / pow(lw, 2);
        CD = 0.04 + (-0.04 + 0.33 * Sn + 6.54 * pow(Sn, 2) - 4.88 * pow(Sn, 3)) * cos(radian_angle);
        CL = (-0.05 * Sn + 2.3 * pow(Sn, 2) - 1.76 * pow(Sn, 3)) * sin(2 * radian_angle);
        //cout << "angle = " << angle <<"    radian_angel = " << radian_angle <<endl;
        //cout << "CD = " << CD << "    CL = " << CL << endl;
    }
    else if (PanelMod == 2)
    {
        double Cdcyl;
        double CD0;
        double CL0;
        double CT;
        double Re;
        double Cd;
        Sn = 2 * (dw / lw) - pow(dw / lw, 2);
        Re = rho * dw * vw.norm() / miu / (1 - Sn);
        Cdcyl = -78.46675 + 254.73873 * log10(Re) - 327.8864 * pow(log10(Re), 2) + 223.64577 * pow(log10(Re), 3) - 87.92234 * pow(log10(Re), 4) + 20.00769 * pow(log10(Re), 5) - 2.44894 * pow(log10(Re), 6) + 0.12479 * pow(log10(Re), 7);
        CD0 = Cdcyl * Sn * (2 - Sn) / 2 / pow(1 - Sn, 2);
        Cd = CD0;

        CT = pi * Cd / (8 + Cd);
        CL0 = (0.5 * CD0 - CT) / sqrt(2.0);
        CD = CD0 * (0.9 * cos(radian_angle) + 0.1 * cos(3 * radian_angle));
        CL = CL0 * (1.0 * sin(2 * radian_angle) + 0.1 * sin(4 * radian_angle));

    }

    
    
    if (IF_PANEL_AREA)//如果IF_PANEL_AREA为真，则实时更新PANEL单元面积，注于2023.3.18
    {
        A_screen = PANEL4[NO_local];        
    }
    
    Fd = CD * 0.5 * rou * A_screen * vw.norm() * vw;

    RotateAxis = vw.cross(Normal);
    RotateAxis.normalize();
    double dAngle = 0.5*3.1415926;
    AngleAxisd v(dAngle, RotateAxis);
    Matrix3d rotMatrix = v.matrix();

    double Fl_value;      

    Fl_value = CL*  0.5 * rou * A_screen * pow(vw.norm(),2);

    iL = RotateAxis.cross(vw);  //2023.3.22 add
    iL.normalize();             //2023.3.22 add
    Fl = iL * Fl_value;         //2023.3.22 add

    //Fl = rotMatrix * vw;    
    //Fl.normalize();
    //Fl = Fl_value * Fl;
    
    for (int i = 0;i < 3;i++)
    {
        F[i] = Fd(i)+Fl(i);
    }
    //return F;
}