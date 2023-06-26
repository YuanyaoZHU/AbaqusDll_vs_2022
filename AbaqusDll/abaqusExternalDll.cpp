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
double TIME = -1;//modelica��ǰʱ�䣬��Modelica���ú�����β�����и���
double TIME2 = 0;//
int N = 0;
double DTIME = 0;
double dTIME = 0.01;
double TIME3 = 0;
double TIME4 = -1;

//���²������ڼ���ABAQUS��ʱ����
double DISP_T1[6] = { 0 };
double DISP_T2[6] = { 0 };
double VELOCITY_T1[6] = { 0 };
double VELOCITY_T2[6] = { 0 };
double ACCEL_ZHU_T1[6] = { 0 };
double ACCEL_ZHU_T2[6] = { 0 };
int P = 0;//���ڶԲ����㲽�ļ���
int Q = -1;//��P����ʹ�ã�P�Ǽ�¼������Q��¼��ǰ����
bool flag_for_err = TRUE;//���ڱ����״δ���err.txt�ļ�
bool flag_for_disp = TRUE;//���ڱ����Ƿ���abaqus��log�ļ���д����Ϣ��TRUEΪд�룬FALSEΪ��д��
double time_flag = -1;//���ڱ�����abaqus��log�ļ����Ƿ���ʾ#6������������
double time_flag1[10] = { 0 };//���������GETV.txt�ļ�ʱֻ���һ�Ρ�
double time_for_disp = -1;

//-----------------------------------------
//����Ϊ���˲���
double rho = 1025; //��ˮ�ܶ�
double miu = 1.01e-3;//dynamic viscosity, kinematic viscosity = miu/rho
double U1[15000] = { 0 };//��˹����������1
double U2[15000] = { 0 };//��˹����������2
double HydroValue[22200][7] = { 0 };//��ȡ��ˮ��������
double freq[200] = { 0 };//Ƶ��
double omega[200] = { 0 };//��Ƶ��
int F_n = 0;
int wave_mod = 0;//�������࣬1Ϊ����Jonswap�����ɲ��˲�����2Ϊ����
double ragular_wave_phase = NULL;
double waterDepth = NULL;

double Hs = NULL;
double Tp = NULL;
double wave_incide_direction = NULL;

double k_w[200] = { 0 };//����
double S[200] = { 0 };//������
double AW[200] = { 0 };
double domega = NULL;
double f1[200];

double current[3] = { 0 };




double DISP_3TO6[3] = { 0 };

//double a1[4] = { 0 }; //ABAQUSʱ�䲽�ڵ��������ϵ��
//double a2[4] = { 0 };
//double a3[4] = { 0 };
//double a4[4] = { 0 };
//double a5[4] = { 0 };
//double a6[4] = { 0 };
double aa[6][7] = { 0 };//��һάΪ���ɶȣ��ڶ�άΪ�������

double NODES[20000][6] = { 0 };//�ڵ㣬1ά���ڵ��ţ�2ά��[0-2]Ϊλ�ƣ�[3-5]Ϊ�ٶ�
double NODES0[20000][6] = { 0 };
int NODE[20000][5] = { 0 };//��ȡNodes�ı��,NODE[xx][0]Ϊ�ڵ��ţ�Node[xx][1-4]Ϊ��ýڵ������ĵ�Ԫ��š�

int ELEM[30000][5] = { 0 };//��Ԫ��1ά����Ԫ��ţ�2ά��[0]ΪElements�ı�ţ�[1-2]�����Ľڵ���,[3-4]Ϊ�����õ�Ԫ��Panel��Ԫ���

int PANEL1[10000][13] = { 0 };//Panel��Ԫ[xx][0]ΪPanel��Ԫ��ţ�[xx][1-4]Ϊ������Node�ı�ţ��ڵ�1,3�ͽڵ�2,4��ɵ��������Ϊ�䷨��������[5-8]Ϊ������Element�ı�ţ�[9-12]ΪNode�����±�ţ���С�������У�
double PANEL2[10000][6] = { 0 };//Panel��Ԫ[0-2]ΪPanel�ķ���������[3-5]ΪPanel��Ԫ�����ĵ�����
double PANEL3[10000][3] = { 0 };//Panel��Ԫ���ĵ��ٶȣ�2023.3.14����
double PANEL4[10000] = { 0 }; //Panel��Ԫ��ʵʱ���
bool PANEL5[10000] = { 0 }; //Panel��Ԫ�Ƿ���ˮ�����Σ�trueΪ�ǣ�falseΪ��

int Panelsline = 0; //���ڼ�¼��ȡ�˶��ٸ�Panel��Ԫ����start_zhu��updatePanels�������õ�

int NET_MODE = 0; //�������²��õ�ģ�ͣ�1Ϊmorison��2Ϊscreen type.

int selectElement[10] = { 0 };//������¼��Ҫ�۲��Element���롣
int selectPanel[10] = { 0 };
int selectNode[10] = { 0 };

double dw = 0; //��������
double lw = 0;
double Sn = 0; //��ʵ��
double L_screen = 0;//����Ⱥ��������۳���
double A_screen = 0;//����Ⱥ������������


bool initial_SETV = TRUE;
bool initial_SETCOORD = TRUE;
bool initial_GETV = TRUE;
bool IF_ELEM_VELOCITY = TRUE; //�����ж��ڼ���Panel��Ԫˮ������ٶ�ʱ�Ƿ����Panel��Ԫ������ٶ�
bool IF_COUPLING = TRUE; //�����ж��Ƿ��MODELICA���
bool IF_PANEL_AREA = TRUE; //�ж��Ƿ�ʵʱ����PANEL��Ԫ�����
bool IF_VELOCITY_FILTER = TRUE;//�Ƿ�����ٶȹ��ˣ����ٶ�С��ĳһֵʱ���϶���Ϊ0���Ա�֤ϵͳ���ȶ���

double Velocity_Filter_L = 0;
double Velocity_Filter_G = 0;

int PanelMod = 0;//�����жϲ�������PanelMod����Cheng et al��2020�������е�һ�£�1: S1, 2: S2, 3 :S3

char WorkPathChar[100] = { 0 };
int CharLength = 0;

//std::string WorkPath = "Aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

#pragma data_seg()
#pragma comment(linker,"/SECTION:shared,RWS")


extern "C" _declspec(dllexport) int start_zhu()
{
    double dtime;
    string workpath;
    string label;
    //----------------------------------------------------------
    //��ȡ��������
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
    in >> rho; //��ˮ�ܶ�
    in >> label;
    in >> miu; //��ˮ������
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
        //cout << "Jonswap����������" << endl;
        in >> label;
        in >> ragular_wave_phase;
        ragular_wave_phase = 0;//Jonswap�����в���Ҫ�Թ�����λ���ж���
    }
    else if (wave_mod == 2)
    {
        //cout << "��������򲨵���λ" << endl;
        in >> label;
        in >> ragular_wave_phase;
    }
    else
    {
        in >> label;
        in >> ragular_wave_phase;
        cerr << "���������������,�����޲��˴���" << endl;
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
    in >> IF_ELEM_VELOCITY; //����������ж��ڼ���Panel���ˮ���ٶ�ʱ�Ƿ����Panel��Ԫ�����ٶȵ�
    in >> label;
    in >> IF_COUPLING; //�ж��Ƿ���modelica��ϣ��޸���2023.3.14
    in >> label;
    in >> IF_PANEL_AREA;//�ж��Ƿ����Panel��Ԫ�����
    in >> label;
    in >> IF_VELOCITY_FILTER; //�ж��Ƿ�����ٶȹ������Ա�֤ϵͳ���ȶ���
    in >> label;
    in >> Velocity_Filter_L;//С�ڱ�ֵʱPanel��Ԫ�ٶ�ǿ��Ϊ0
    in >> label;
    in >> Velocity_Filter_G;//���ڱ�ֵʱPanel��Ԫ�ٶ�ǿ��Ϊ0
    in >> label;
    in >> PanelMod; //����PanelMod


    in >> label; 
    in >> selectElement[0] >> selectElement[1] >> selectElement[2] >> selectElement[3] >> selectElement[4] >> selectElement[5] >> selectElement[6] >> selectElement[7] >> selectElement[8] >> selectElement[9];    
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
    cout << "PanelMod = " << PanelMod << endl;
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
    //���벨�˲���
    /*
    cout << "������Hs:" << endl;
    cin >> Hs;
    cout << "������Tp:" << endl;
    cin >> Tp;
    cout << "�����벨�����ͣ�(1ΪJonswap�����ɷǹ��򲨣�2Ϊ���򲨣�" << endl;
    cin >> wave_mod;
    cout << "������ˮ�" << endl;
    cin >> waterDepth;
    cout << "����������ǣ�" << endl;
    cin >> wave_incide_direction;
    */

    //TIME = 0;

    //DISP[2] = -1000;
//----------------------------------------------------------------------
    
    ReadOutFile readOutFile;//ReadOutFileΪ���ڴ�����������Ԫģ���е�Ԫ���ڵ���뼰�������
    readOutFile.ReadInNodes();//��ȡNodes.txt�ĵ�
    readOutFile.ReadInElements();
    readOutFile.ReadRandom();//��ȡ�����
    readOutFile.ReadHydro();//��ȡˮ��������
    readOutFile.ReadInPanels();//��ȡPanels.txt�ĵ�


 
    //----------------------------------------------------------------------
    //����ȡ���������ֵ�������ڴ�����
    if (readOutFile.randomline > 15000)
    {
        cout << "������������������15000�����޷�������ȡ������������ݣ�" << endl;
        cout << "random_value.txt�����������Ϊ��" << readOutFile.randomline << endl;

        system("pause");
    }

    for (int i = 0; i < readOutFile.randomline;i++)
    {
        U1[i] = readOutFile.randomValue[i][0];
        U2[i] = readOutFile.randomValue[i][1];

    }

    cout << "�������ȡ��ɣ�" << endl;

    //----------------------------------------------------------------------------------
    for (int i = 0;i < readOutFile.hydroline;i++)
    {
        for (int j = 0;j < 7;j++)
        {
            HydroValue[i][j] = readOutFile.hydroValue[i][j];
        }
    }
    F_n = readOutFile.F_n;
    if (F_n > 200)//�����ȡ��Ƶ�ʸ����Ƿ񳬹��趨������
    {
        cout << "ˮ�����ļ��в���Ƶ�ʸ�������200�������ڴ汣���Ƶ�ʸ�����С��200��" << endl;
        system("pause");
    }
    for (int i = 0;i < F_n;i++)//Ϊˮ�����ļ���Ƶ�ʽ��и�ֵ
    {
        freq[i] = readOutFile.freq[i];
        omega[i] = 2 * pi * freq[i];
    }
    domega = omega[1] - omega[0];//domega����������Ƶ�ʼ�����ȵ�

    cout << "ˮ�����ļ���ȡ��ɣ�" << endl;
    
    waveNumber WaveNumber(waterDepth);//��ò���
    
    for (int i = 0;i < F_n;i++)
    {
        k_w[i] = WaveNumber.newton(readOutFile.omega[i]);
    }
    cout << "����������ϣ�" << endl;


    //----------------------------------------------------------------------------------
    //�������׸�ֵ
    wavespectral2 WaveSpectral(Hs, Tp);
    for (int i = 0;i < F_n;i++)
    {
        S[i] = WaveSpectral.Spectral(readOutFile.omega[i]);

        AW[i] = sqrt(2 * S[i] * domega);

        f1[i] = sqrt(-2 * log(U1[i]));//������ܸ�ֵ�Ķ��ȸ�ֵ��
    }


    //----------------------------------------------------------------------------------
    //r = readOutFile.ReadInElements();//��ȡElements.txt�ĵ�
    
    //��ֵ��NODE����
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

    //��ֵ��ELEM����
    
    for (int i = 0;i < 30000;i++)
    {
        for (int j = 0;j < 5;j++)
        {
            ELEM[i][j] = readOutFile.Element1[i][j];
        }
    }

    //��ֵ��PANEL1����
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
        PANEL5[i] = readOutFile.Panel3[i];
    }
    
    //�������������֤NODES��NODE��ȡ�Ľ����ȷ
    cout << "NODES��ȡ���" << endl;
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

    cout << "ELEM��ȡ���" << endl;
    for (int i = 0;i < 10;i++)
    {
        for (int j = 0;j < 5;j++)
        {
            cout << ELEM[i][j] << "    ";
        }
        cout << endl;
    }

    cout << "PANEL1��ȡ���" << endl;
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
    //����elementForce,����õ�����
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
    /// ��������Modelica���ã����ڴ��λ�ƣ������Abaqus�غ�
    /// </summary>
    /// <param name="time">�����������ʱ�䲽��ʱ�䣬double����</param>
    /// <param name="disp">���������Modelica�ṩ��ȫ��λ��</param>
    /// <param name="dload">�����������ABAQUS����������غ�</param>
    /// <returns></returns>


    double Disp[6];
    double Velocity[6];
    double Accel[6];
    double Dload[6];
    double Pi = 3.1415926;
    double Rotation[3];
    //����Ĳ������ڼ���ABAQUS��ʱ����
    double U1[6];
    double U2[6];
    double V1[6];
    double V2[6];
    double A1[6];
    double A2[6];

    //-----------------------
    //�����ж�Modelica��ABAQUS��ʱ���Ƿ�Ϊͬһʱ�䲽
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    cout << "����Modelica2Dll����" << endl;

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
    //��Ϣ���
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
    
    //cout << "#1 �ж��Ƿ���븳ֵ���裡" << "    time = " << time << "    " << "TIME = " << TIME << "    " << "TIME3 = " << TIME3 << "    ERR = "<< ERR << endl;
    if (time > TIME && ERR)
    {


        for (int i = 0;i < 3;i++)
        {
            Disp[i] = disp[i];
            DISP_M[i] = Disp[i]; //* 1000;//��λת��m��mm
            DISP_T2[i] = DISP_M[i];//��T2ʱ��λ�Ƹ�ֵ
            if (DISP_M[i] != DISP_A[i])
            {
                DISP_A[i] = DISP_M[i];
            }
            Velocity[i] = velocity[i];
            VELOCITY_M[i] = Velocity[i]; //* 1000;
            VELOCITY_T2[i] = VELOCITY_M[i];//��T2ʱ���ٶȸ�ֵ
            if (VELOCITY_M[i] != VELOCITY_A[i])
            {
                VELOCITY_A[i] = VELOCITY_M[i];
            }
            Accel[i] = accel[i];
            ACCEL_ZHU_M[i] = Accel[i];  // *1000;
            ACCEL_ZHU_T2[i] = ACCEL_ZHU_M[i];//��T2ʱ�̼��ٶȸ�ֵ
            if (ACCEL_ZHU_M[i] != ACCEL_ZHU_A[i])
            {
                ACCEL_ZHU_A[i] = ACCEL_ZHU_M[i];
            }
        }
        for (int i = 3;i < 6;i++)
        {
            Disp[i] = disp[i];
            ROTATION_M[i - 3] = Disp[i] - DISP_M[i];
            DISP_M[i] = Disp[i];//��λת��m��mm
            DISP_T2[i] = DISP_M[i];//��T2ʱ��λ�Ƹ�ֵ
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
            VELOCITY_T2[i] = VELOCITY_M[i];//��T2ʱ���ٶȸ�ֵ
            if (VELOCITY_A[i] != VELOCITY_M[i])
            {
                VELOCITY_A[i] = VELOCITY_M[i];
            }
            Accel[i] = accel[i];
            ACCEL_ZHU_M[i] = Accel[i];
            ACCEL_ZHU_T2[i] = ACCEL_ZHU_M[i];////��T2ʱ�̼��ٶȸ�ֵ
            if (ACCEL_ZHU_A[i] != ACCEL_ZHU_M[i])
            {
                ACCEL_ZHU_A[i] = ACCEL_ZHU_M[i];
            }
        }
        //-----------------------------------------------------------------------
        //���ABAQUS��ֵ���ϵ��a[1],a[2],a[3],a[4]
        for (int i = 0;i < 6;i++)//�½��ֲ���������ʹ�ã�ԭ������̫����
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
            aa[i][0] = 120 / pow(dt, 5) * (U2[i] - U1[i]) - 12 / pow(dt, 4) * (5 * V2[i] - 11 * V1[i]) + 1 / pow(dt, 3) * (7 * A2[i] - 10 * A1[i]); //aa[i][0]Ϊ��ʽ�е�a3
            aa[i][1] = -180 / pow(dt, 4) * (U2[i] - U1[i]) + 12 / pow(dt, 3) * (7 * V2[i] - 8 * V1[i]) - 3 / pow(dt, 2) * (3 * A2[i] - 6 * A1[i]);
            aa[i][2] = 60 / pow(dt, 3) * (U2[i] - U1[i]) - 12 / pow(dt, 2) * (2 * V2[i] + 3 * V1[i]) + 3 / dt * (A2[i] - 3 * A1[i]);
            aa[i][3] = A1[i];
            aa[i][4] = V1[i];
            aa[i][5] = U1[i];
            aa[i][6] = ACCEL_ZHU_T2[i] - ACCEL_ZHU_T1[i];

            cout << "aa = " << aa[i][0] << "    " << aa[i][1] << "    " << aa[i][2] << "    " << aa[i][3] << "    " << aa[i][4] << "    " << aa[i][5] << endl;
        }
        
        /*
        Matrix3d A;//AΪϵ������
        Matrix3d Ainverse;//A������󣬷�������
        Vector3d a[6];//6�����ɶ��϶�Ӧ��a3,a2,a1,ע�⣺����a[i](1)�ǹ�ʽ�Ƶ��е�a3, a[i](2)��a2, a[i](3)��a1  !!
        Vector3d S[6];//S��6���ɶ��ϵ�ֵ����
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
        for (int j = 0;j < 3;j++)  //�������a1-a6��д���ǿ��ǵ�C++��FORTRAN֮��������෴����ֹ���������һ��д��
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
        //����disp_modelica.txt�ļ��������������º��DISP���������ȥ��
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
        //cout <<"#2 TIME���и��£�" <<"    TIME = " << TIME << "    " << "    TIME3= " << TIME3 << endl; //��һ������Ͽ���֮ǰ
       
        if (time >= dTIME)//����ж��������ó�����0ʱ�̲����
        {
            //---------------------------------------------------------------------
            //�����̹߳��ߣ�����ABAQUS��������
            HANDLE event1 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS����");
            HANDLE event3 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS����2");
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

            SetEvent(event1);//�����źţ�����ABAQUS����Ѿ��������
            SetEvent(event3);//�����źţ�����ABAQUS����Ѿ��������
            cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
            cout << "Send signal to tell ABAQUS DISP have been writen..." << endl;
            

            //---------------------------------------------------------------------
            //����Modelica�̣߳��ȴ�ABAQUS���������ź�
            /*
            const int EVENT_NUM = 2;

            HANDLE hObject[EVENT_NUM] = { NULL,NULL };

            hObject[0] = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica����");
            hObject[1] = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica����2");
            WaitForMultipleObjects(EVENT_NUM, hObject, TRUE, INFINITE);
            */
            //WaitForSingleObject(event2, INFINITE);//�ȴ�ABAQUS��DLOAD����
            //----------------------------------------------------------------------

            //---------------------------------------------------------------------
            //����Modelica�̣߳��ȴ�ABAQUS���������ź�
            HANDLE event4 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica����");
            WaitForSingleObject(event4, INFINITE);//�ȴ�ABAQUS��DLOAD����
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
        //��������Ϊ�߳̿���
    }
    TIME = time;

    /*if (TIME != TIME3)
    {
        cout << "TIME=    " << TIME << "    TIME3=    " << TIME3 << "    TIME_INT=    " << TIME_INT << "    TIME_INT3=    " << TIME3_INT << endl;
    }
    */



    //cout << "NEW    " << time << "    " << DISP_M[0] << "    " << DISP_M[1] << "    " << DISP_M[2] << "    " << DISP_M[3] << "    " << DISP_M[4] << "    " << DISP_M[5] << endl;
        //Dload[0] = 10;//������x�����յ�10N����

    //-----------------------------------------------------------------
    //����ABAQUS���º���غ�
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
    //cout << "#3 abaqusdllurdfil�ս���ʱ����ȫ��ʱ��չʾ!" << "    TIME =  " << TIME <<"    TIME3 =  " << TIME3 << "    time[0]=    " << time[0] << endl;

    for (int i = 0;i < 6;i++)
    {
        Dload[i] = dload[i];
        DLOAD_A[i] = Dload[i];
        DLOAD_M[i] = DLOAD_A[i];
    }
    //---------------------------------------
    //�ж�ʱ�䲽�Ƿ�һ��
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    TIME_INT = int(time[0] * 10 / DTIME);
    TIME3_INT = int(TIME * 10 / DTIME);//int(TIME3 * 10 / DTIME); ����URDFIL�ӳ�������ABAQUS����ĩβ�����е��õģ���ʱtime[0]��δ���£��������ǵ���һ���������ˣ��Ž��뵽DISP��һ���ĵ��ã�2022.10.16

    err = abs(TIME_INT - TIME3_INT);

    if (IF_COUPLING) //��IF_COUPLINGΪTRUEʱ������MODELICA�������
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
        if (ERR)  //��������жϣ���ABAQUS��Modelicaʱ�䲽��ͬʱ����ִ�������¼�
        {
            cout << "#4 UDFILE������ERR�ж�Ϊ��ʱ����ʱ��״̬!" << "    TIME_INT=" << TIME_INT << "    TIME3_INT=" << TIME3_INT << "    time[0]=    " << time[0] << "    TIME3=  " << TIME3 << endl;
            HANDLE event1 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"Modelica����");
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

            SetEvent(event1);//�����źţ�����Modelica����Ѿ��������
            cout << "Send signal to tell Modelica DLOAD have been writen..." << endl;

            HANDLE event2 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS����");
            WaitForSingleObject(event2, INFINITE);//�ȴ�Modelica��DISP����
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
    //��������Ϊ�߳̿���
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
    //�ж�ʱ�䲽�Ƿ�һ��
    int TIME_INT;
    int TIME3_INT;
    int err;
    bool ERR;

    //TIME4 = time[1] + DTIME; //time[1]�ǲ���ʱ�̣�time[0]��ȫ��ʱ�̣�����֮ǰ��֪��Ϊʲô����DISP�����TIME(2)����ӦTIME_ZHU(1)

    TIME_INT = int(time[1] * 10 / DTIME);
    TIME3_INT = int(TIME3 * 10 / DTIME);

    err = abs(TIME_INT - TIME3_INT);

    //cout << "#5 Disp�ս�����ʱ�䣡"<< "    TIME_INT=  " << TIME_INT << "    TIME3_INT=  " << TIME3_INT <<"    time[1]=  " << time[1] << "    time[0]=  " << time[0] <<"    TIME2=  "<< TIME2 <<endl;


    if (time[0] != TIME2)
    {
        Q = Q + 1;
        //cout << "#7 Q��ֵ    Q = "<< Q <<endl;
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

    if (time[0] != TIME2 && N==1 && ERR)//N�������Ƿ�ֹ�����һ�ε��þͿ�ס�ˣ���Ϊtime[0]���״�����ʱ�ǲ�����TIME2��
    {
       
        //////////////////////////////////////////////////////////////////////////
        //----------------------------------------------------------------------//
        //��ȫ��ʱ�����ʱ����������סABAQUS�����У�ֱ��Modelica������DISP��
        //��������Ҫ����Ϊ���������Ͻ���DISP�ӳ���Ӧ����URDFIL֮�����У�����
        //�ڲ���ʱ��DISP������м����ܵ�URDFILǰ�����У�����д��URDFIL�е���
        //�̿����¼�������Ч����ʱ�䲽����ʱ��סABAQUS��������������˫���գ�
        //�Կ���λ�Ƶ���Ч���¡�
        //
        //2022.10.14��¼
        //----------------------------------------------------------------------//
        //////////////////////////////////////////////////////////////////////////
        HANDLE event2 = OpenEvent(EVENT_ALL_ACCESS, FALSE, L"ABAQUS����2");
        
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
        WaitForSingleObject(event2, INFINITE);//�ȴ�Modelica��DISP����
        ////////////////////////////////////////////////////////////////////////////
    }
    //------------------------------------------------------------------------------
    N = 1;
    //------------------------------------------------------------------------------
    //���γ������ڼ���ABAQUS��Modelicaʱ�䲽�м䲿�ֵ��˶�
    t = time[1] - TIME + dTIME;
    if (DTIME < dTIME)
    {
        t = t - DTIME;
    }
    t1 = t;

    if (time_flag != time[1])
    {
        cout << "#6 �м䲽��ʱ����㣡   t=  " << t << "    time[1]=  " << time[1] << "    TIME=  " << TIME << "    dTIME=  " << dTIME << endl;

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

    if (t!=time_for_disp)//�˲����������ǰabaqus��õ�λ�ơ��ٶȡ����ٶȣ���abaqusDisp.txt�ļ���
    {
        string abaqusDisp = "abaqusDisp.txt";
        string WorkPath = WorkPathChar;
        string fileName = WorkPath + abaqusDisp;
        if (flag_for_disp)//���δ�abaqusDisp.txt
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
    // ���ڲ��õ���ֱ�Ӽ���������ߵ�ϵ�������Ǿ��󷨣��ڳ�������ʱ�����������ᷢ�������ԣ�����û��Ҫ�ڿ�ʼʱ��λ��ǿ�����ó�0
    if (time[1] < 3* dTIME)//����Ϊ����0ʱ��ʱ����ֹ�����������
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
    NODES0[EN - 1][3] = NODES[EN - 1][3]; //֮ǰ�д���
    NODES0[EN - 1][4] = NODES[EN - 1][4];
    NODES0[EN - 1][5] = NODES[EN - 1][5];
    NODES[EN - 1][3] = V[0];
    NODES[EN - 1][4] = V[1];
    NODES[EN - 1][5] = V[2];
    ///* 2022��10��19��ע�͵������ٷǱ�Ҫ�������֤�����ٶ�
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
    ///* 2022��10��19��ע�͵������ٷǱ�Ҫ�������֤�����ٶ�
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
    /// ���������ڻ�ȡˮ����Ըõ�Ԫ�ķ����ٶ�
    /// </summary>
    /// <param name="ELM_NO">���룺��Ԫ���</param>
    /// <param name="VW">���룺ˮ���ٶȣ�ʸ����</param>
    /// <param name="VN">����������ٶ�</param>
    /// <returns></returns>
    int EN;  //ELEM���
    int NO[2]; //Element������ Node���
    double V_N[2][3];//Node���ٶ�

    Vector3d VT;//��Ԫ��������
    Vector3d VT_0;//
    Vector3d VT_1;//
    Vector3d VE;//��Ԫ�ľ����ٶ�
    Vector3d VR;//��Ԫ���ˮ�����ٶ�
    Vector3d VP;//�뵥Ԫ�������͵�Ԫ���ˮ���ٶȶ���ֱ������
    Vector3d NV;//������
    Vector3d VR_N;//����ٶ��ص�Ԫ����ķ���
    Vector3d VR_T;//����ٶ��ص�Ԫ����ķ���


    double L; //��Ԫ����

    //�ҵ�ELEMENT������Node�ı��
    EN = *ELM_NO;

    NO[0] = ELEM[EN - 1][1]-1;
    NO[1] = ELEM[EN - 1][2]-1;
    double t_up = time[0] - TIME + dTIME;
    double t_low = TIME3 - TIME;
    double tt = t_up / t_low;

    //�������ڵ���ٶȷֱ�ֵ��V_N��
    V_N[0][0] = tt * (NODES[NO[0]][3] - NODES0[NO[0]][3]) + NODES0[NO[0]][3];
    V_N[0][1] = tt * (NODES[NO[0]][4] - NODES0[NO[0]][4]) + NODES0[NO[0]][4];
    V_N[0][2] = tt * (NODES[NO[0]][5] - NODES0[NO[0]][5]) + NODES0[NO[0]][5];

    V_N[1][0] = tt * (NODES[NO[1]][3] - NODES0[NO[1]][3]) + NODES0[NO[1]][3];
    V_N[1][1] = tt * (NODES[NO[1]][4] - NODES0[NO[1]][4]) + NODES0[NO[1]][4];
    V_N[1][2] = tt * (NODES[NO[1]][5] - NODES0[NO[1]][5]) + NODES0[NO[1]][5];

    //L = sqrt(pow(NODES[NO[0]][0] - NODES[NO[1]][0], 2) + pow(NODES[NO[0]][1] - NODES[NO[1]][1], 2) + pow(NODES[NO[0]][2] - NODES[NO[1]][2], 2));//���㵥Ԫ����
    
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

    //�������ڵ�֮���ƽ���ٶ�
    VE(0) = 0.5 * (V_N[0][0] + V_N[1][0]);
    VE(1) = 0.5 * (V_N[0][1] + V_N[1][1]);
    VE(2) = 0.5 * (V_N[0][2] + V_N[1][2]);

    //��ˮ����Ե�Ԫ���ٶ�
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

    
    //�ж����������Ƿ�������ٶȵķ���С��90��,�������90�㣬����Ҫ��������������
    double radian_angle = atan2(VR.cross(VT).norm(), VR.transpose() * VT);
    double angle = radian_angle * 180 / 3.1415926;

    if (angle > 90)
    {
        VT = -VT;
        //cout << "#vtΪ����" << endl;
    }

    VR_T = (VR.dot(VT)) * VT;

    VR_N = VR - VR_T;
    /* ������������ķ�����2022��10��19��ע��
    //������
    VP = TV.cross(VR);
    NV = VP.cross(TV);
    NV.normalize();

    //���ط�����������ٶ�
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
        cout << "#vnΪ����" << endl;
    }
    
    */
    //��ֵ���
    VN[0] = VR_N(0);
    VN[1] = VR_N(1);
    VN[2] = VR_N(2);
    /* 2022��10��19��ע�͵������ٷǱ�Ҫ�������֤�����ٶ�
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
                cout << "���뵽GETV.txt���ϵͳ��" << endl;
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
    //cout << "come into getwaterv!" << endl;
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

    double COORD[3];//��COORD����NET_MODE==1�������
    double COORD2[3][2];//��COORD2����NET_MODE == 2�������
  
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
        Vw[0] = vw[0] + current[0];//+currentΪˮ���ٶȣ��޸���2023.3.15
        Vw[1] = vw[1] + current[1];
        Vw[2] = vw[2] + current[2];
        VW[0] = Vw[0];
        VW[1] = Vw[1];
        VW[2] = Vw[2];
        
    }
    else if (NET_MODE == 2)
    {
        //cout << "come into Net_MODE 2!" << endl;
        int NO1;
        int NO2;
        Vector3d Normal;
        Vector3d Ve_Water;
        //double* F1;
        //double* F2;
        double* F1 = new double[3];//����Panel��Ԫ����
        double* F2 = new double[3];//������һPanel��Ԫ����
        double velocity_local;
        Vector3d VelocityPanel;
        
        double* current1 = new double[3]; //N01��������ˮ���ٶ�
        double* current2 = new double[3]; //NO2��������ˮ���ٶ�

        



        NO1 = ELEM[*NOEL - 1][3]-1; //������ELEM��Panel��Ԫ���
        NO2 = ELEM[*NOEL - 1][4]-1;
        if (NO1 < 0)//�˲������жϸýڵ��Ƿ�Ϊ������PANEL��Ԫ�������0˵���ýڵ㲢��PANEL��Ԫ
        {
            for (int i = 0;i < 3;i++)
            {
                F1[i] = 0;
            }
            /*if (*NOEL == selectElement[0])
            {
                cout << "selectElement:  " << selectElement[0] << "    NO1= " << NO1 << endl;
            }*/
        }
        else
        {
            COORD2[0][0] = PANEL2[NO1][3];//��Panel��Ԫ���ĵ����긳ֵ��COORD2
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

            
            if (PANEL5[NO1])//����жϵ�ǰPANEL��ԪΪ���±��棬��ˮ���ٶ���Ҫ˥��
            {
                double reduce;//����ˮ��˥���ʵ�ֵ����ʽΪ r = 1-0.46 * cd
                Ve_Water(0) = current[0];
                Ve_Water(1) = current[1];
                Ve_Water(2) = current[2];
                reduce = 1 - 0.46 * CD_calculate(Ve_Water);
                current1[0] = current[0] * reduce;
                current1[1] = current[1] * reduce;
                current1[2] = current[2] * reduce;
            }
            else
            {
                current1[0] = current[0];
                current1[1] = current[1];
                current1[2] = current[2];
            }
            
            Ve_Water(0) = vw[0] - PANEL3[NO1][0] + current1[0];//ˮ�ʵ�����ٶ�Ve_Water = vw-vx;vwΪˮ�ʵ��ٶȣ�vxΪPanel��Ԫ�ٶȣ��޸���2023.3.14
            Ve_Water(1) = vw[1] - PANEL3[NO1][1] + current1[1];
            Ve_Water(2) = vw[2] - PANEL3[NO1][2] + current1[2];
            
            calculateforce(F1, Normal, Ve_Water, NO1);
            /*if (*NOEL == selectElement[0])
            {
                cout << "E(1) VeWater(1-3)=    " << Ve_Water(0) << "    " << Ve_Water(1) << "    " << Ve_Water(2) << endl;
                cout << "curent[0-2]  " << current[0] << "    " << current[1] << "    " << current[2] << endl;
                cout << "F1 =  " << F1[0] <<"    " <<F1[1]<<"    "<< F1[2] << endl;
            }*/
            
        }

        if (NO2 < 0)//�˲������жϸýڵ��Ƿ�Ϊ������PANEL��Ԫ�������0˵���ýڵ㲢��PANEL��Ԫ
        {
            for (int i = 0;i < 3;i++)
            {
                F2[i] = 0;
            }
            /*if (*NOEL == selectElement[0])
            {
                cout << "selectElement:  " << selectElement[0] << "    NO2= " << NO2 << endl;
            }*/
        }
        else
        {
            COORD2[0][1] = PANEL2[NO2][3];//��Panel��Ԫ���ĵ����긳ֵ��COORD2
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
                PANEL3[NO2][0] = PANEL3[NO2][0] / velocity_local * Velocity_Filter_G;
                PANEL3[NO2][1] = PANEL3[NO2][1] / velocity_local * Velocity_Filter_G;
                PANEL3[NO2][2] = PANEL3[NO2][2] / velocity_local * Velocity_Filter_G;
            }

            if (PANEL5[NO2])//����жϵ�ǰPANEL��ԪΪ���±��棬��ˮ���ٶ���Ҫ˥��
            {
                double reduce;//����ˮ��˥���ʵ�ֵ����ʽΪ r = 1-0.46 * cd
                Ve_Water(0) = current[0];
                Ve_Water(1) = current[1];
                Ve_Water(2) = current[2];
                reduce = 1 - 0.46 * CD_calculate(Ve_Water);
                current2[0] = current[0] * reduce;
                current2[1] = current[1] * reduce;
                current2[2] = current[2] * reduce;
            }
            else
            {
                current2[0] = current[0];
                current2[1] = current[1];
                current2[2] = current[2];
            }

            Ve_Water(0) = vw[0] - PANEL3[NO2][0] + current2[0];//ˮ�ʵ�����ٶ�Ve_Water = vw-vx;vwΪˮ�ʵ��ٶȣ�vxΪPanel��Ԫ�ٶ�;�޸���2023.3.14
            Ve_Water(1) = vw[1] - PANEL3[NO2][1] + current2[1];
            Ve_Water(2) = vw[2] - PANEL3[NO2][2] + current2[2];
           
            calculateforce(F2, Normal, Ve_Water, NO2);
            /*if (*NOEL == selectElement[0])
            {
                cout << "E(2) VeWater(1-3)=    " << Ve_Water(0) << "    " << Ve_Water(1) << "    " << Ve_Water(2) << endl;
                cout << "curent[0-2]  " << current[0] << "    " << current[1] << "    " << current[2] << endl;
                cout << "F2 =  " << F2[0] << "    " << F2[1] << "    " << F2[2] << endl;
            }*/
            
        }


        
        for (int i = 0;i < 3;i++)
        {
            Vw[i] = (F1[i] + F2[i]) / 4;//VW��ʱ��Ϊ���������ˮ������ٶȣ����ǵ�Ԫ��������
            VW[i] = Vw[i];
            
        }
        

        for (int i = 0;i < 10;i++)
        {
            if (*NOEL == selectElement[i])
            {
                //����elementForce,����õ�����
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

        delete[] current1;
        delete[] current2;
       
    }
    delete[] vw;
    //cout << "end getwaterv fucntion!" << endl;
    /* �ⲿ�����ϳ���fluidVelocity����
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

            f4[i] = -sin(omega[i] * time[0] + 2 * pi * U2[i] - k_w[i] * COORD[0]);//ע�⡶�����뺣�󹤳̻����غɡ��ϵ�13ҳ�������뱾����cos sin����һ����������sin�������ˣ��������һ�£�����cos�������ˣ���x����Ϊcos��z����Ϊ-sin
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
        zeta = 0.5 * Hs * ff2 * ff3;//0.5*HsΪ�������
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

    //���±������ڼ����ı������
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
        //���ȸ��·�����
        IF_TRUE_PANEL = TRUE;
        
        for (int j = 0; j < 4;j++)//panel��Ԫ��4���ڵ�
        {
            if (PANEL1[i][j + 1] == 0)
            {
                IF_TRUE_PANEL = FALSE;
            }
            Node_local[j] = PANEL1[i][j+1]-1;
        }
        
        //Ϊ����e1��e2��ֵ
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
        //Ȼ��������ĵ�λ��

        PANEL2[i][3] = (NODES[Node_local[0]][0] + NODES[Node_local[1]][0] + NODES[Node_local[2]][0] + NODES[Node_local[3]][0]) / 4;
        PANEL2[i][4] = (NODES[Node_local[0]][1] + NODES[Node_local[1]][1] + NODES[Node_local[2]][1] + NODES[Node_local[3]][1]) / 4;
        PANEL2[i][5] = (NODES[Node_local[0]][2] + NODES[Node_local[1]][2] + NODES[Node_local[2]][2] + NODES[Node_local[3]][2]) / 4;
        

        //����PANEL���ĵ���ٶ�,������2023.3.14
        if (IF_ELEM_VELOCITY)//��IF_ELEM_VELOCITYΪTRUEʱ�����½ڵ��ٶ�
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

        //����PANEL��Ԫ�ĸ������
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
                D_local = abs((pow(Panel_side_length[1], 2) + pow(Panel_side_length[3], 2))-(pow(Panel_side_length[0], 2)+ pow(Panel_side_length[2], 2)));//��ʽΪD=abs{(b^2+d^2)-(a^2+c^2)}�����ע����2023.3.18
                sqrt_inner = pow(2 * m_local * n_local, 2) - pow(D_local, 2);
                if (sqrt_inner <= 0)//����һЩ���������κ󣬿��ܷ����۵�������ʱ�п�����ƽ����ʽ�¾ͳ����˸�ֵ�����ڼ����л��������󣬴�ʱ��Ҫ������д������£�
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
            else //����жϲ���������PANEL��Ԫ�������������
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
        
        

        //�������

    }
    return 0;
}

//���������ڼ���ˮָ���ٶȣ�VWΪ���������ʾˮ�ʵ��ٶȣ�timeΪ��ǰʱ�̣�COORDΪ�ڵ�����
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

            f4[i] = -sin(omega[i] * time + 2 * pi * U2[i] - k_w[i] * COORD[0]);//ע�⡶�����뺣�󹤳̻����غɡ��ϵ�13ҳ�������뱾����cos sin����һ����������sin�������ˣ��������һ�£�����cos�������ˣ���x����Ϊcos��z����Ϊ-sin
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
        zeta = 0.5 * Hs * ff2 * ff3;//0.5*HsΪ�������
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
    else if (wave_mod == 0)//��wave_modΪ0ʱ����ʾ�޲��ˣ���ʱ���������ˮ�ʵ��ٶ�Ϊ0
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
        /*if (NO_local == selectPanel[0] - 1 || NO_local == selectPanel[1] - 1)
        {
            cout << "CD0 = " << CD0 << endl;
            cout << "Sn = " << Sn << endl;
            cout << "radian_angle=  " << radian_angle << endl;
        }*/

    }

    
    
    if (IF_PANEL_AREA)//���IF_PANEL_AREAΪ�棬��ʵʱ����PANEL��Ԫ�����ע��2023.3.18
    {
        A_screen = PANEL4[NO_local]; 
        /*if (NO_local == selectPanel[0] - 1 || NO_local == selectPanel[1] - 1)
        {
            cout << "A_screen = " << A_screen << endl;
        }*/
    }
    
    Fd = CD * 0.5 * rou * A_screen * vw.norm() * vw;
    /*if (NO_local == selectPanel[0]-1 || NO_local == selectPanel[1]-1)
    {
        cout << "Fd = " << Fd(0) << "    " << Fd(1) << "    " << Fd(2) << endl;
        cout << "vw = " << vw(0) << "    " << vw(1) << "    " << vw(2) << endl;
        cout << "vw.norm =" << vw.norm() << endl;
        cout << "rou = " << rou << endl;
        cout << "CD = " << CD << endl;
    }*/

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
    /*if (NO_local == selectPanel[0]-1 || NO_local == selectPanel[1]-1)
    {
        cout << "F = " << F[0] << "    " << F[1] << "    " << F[2] << endl;
    }*/
    //return F;
}

double CD_calculate(Vector3d vw)
{
    double CD_local;
    if (PanelMod == 0)
    {
        Sn = 2 * dw / lw;
        CD_local = 0.04 + (-0.04 + Sn - 1.24 * pow(Sn, 2) + 13.7 * pow(Sn, 3)) * cos(0);        
    }
    else if (PanelMod == 1)
    {
        Sn = dw * (2 * lw + 0.5 * dw) / pow(lw, 2);
        CD_local = 0.04 + (-0.04 + 0.33 * Sn + 6.54 * pow(Sn, 2) - 4.88 * pow(Sn, 3)) * cos(0);        
    }
    else if (PanelMod == 2)
    {
        double pi = 3.1415926;
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
        CD_local = CD0 * (0.9 * cos(0) + 0.1 * cos(0));        
      }
    return CD_local;
}