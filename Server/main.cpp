﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

using namespace Aris::Core;

//#include "trajectory_generator.h"


Aris::Core::MSG parseWalk(const std::string &cmd, const map<std::string, std::string> &params)
{
	Robots::WALK_PARAM  param;

	for(auto &i:params)
	{
		if(i.first=="totalCount")
		{
			param.totalCount=std::stoi(i.second);
		}
		else if(i.first=="n")
		{
			param.n=stoi(i.second);
		}
		else if(i.first=="walkDirection")
		{
			param.walkDirection=stoi(i.second);
		}
		else if(i.first=="upDirection")
		{
			param.upDirection=stoi(i.second);
		}
		else if(i.first=="distance")
		{
			param.d=stod(i.second);
		}
		else if(i.first=="height")
		{
			param.h=stod(i.second);
		}
		else if(i.first=="alpha")
		{
			param.alpha=stod(i.second);
		}
		else if(i.first=="beta")
		{
			param.beta=stod(i.second);
		}
	}

	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}

Aris::Core::MSG parseAdjust(const std::string &cmd, const map<std::string, std::string> &params)
{
	double firstEE[18] =
	{
		-0.3,-0.75,-0.65,
		-0.45,-0.75,0,
		-0.3,-0.75,0.65,
		0.3,-0.75,-0.65,
		0.45,-0.75,0,
		0.3,-0.75,0.65,
	};

	double beginEE[18]
	{
		-0.3,-0.85,-0.65,
		-0.45,-0.85,0,
		-0.3,-0.85,0.65,
		0.3,-0.85,-0.65,
		0.45,-0.85,0,
		0.3,-0.85,0.65,
	};

    Robots::ADJUST_PARAM  param;

	std::copy_n(firstEE, 18, param.targetPee[0]);
	std::fill_n(param.targetBodyPE[0], 6, 0);
	std::copy_n(beginEE, 18, param.targetPee[1]);
	std::fill_n(param.targetBodyPE[1], 6, 0);

	param.periodNum = 2;
	param.periodCount[0]=1000;
	param.periodCount[1]=1500;

	std::strcpy(param.relativeCoordinate,"B");
	std::strcpy(param.relativeBodyCoordinate,"B");

	for(auto &i:params)
	{
		if(i.first=="all")
		{

		}
		else if(i.first=="first")
		{
			param.legNum=3;
			param.motorNum=9;

            param.legID[0]=0;
            param.legID[1]=2;
            param.legID[2]=4;

			int motors[9] = { 0,1,2,6,7,8,12,13,14 };
			std::copy_n(motors, 9, param.motorID);
		}
		else if(i.first=="second")
		{
			param.legNum=3;
			param.motorNum=9;

            param.legID[0]=1;
            param.legID[1]=3;
            param.legID[2]=5;

			int motors[9] = { 3,4,5,9,10,11,15,16,17 };
			std::copy_n(motors, 9, param.motorID);
		}
		else
		{
			std::cout<<"parse failed"<<std::endl;
			return MSG{};
		}
	}

	Aris::Core::MSG msg;

	msg.CopyStruct(param);

	std::cout<<"finished parse"<<std::endl;

	return msg;
}


struct SWING_PARAM :public Robots::GAIT_PARAM_BASE
{
    double targetHeight{0.04};//positive step height relative to start
    double targetStep[3]{0,0,0.1};//relative
    std::int32_t periodCount;
    int comID;
   // double model_forcein[18]{0};
   // double actual_forcein[18]{0};
};


Aris::Core::MSG parseSwing(const std::string &cmd, const map<std::string, std::string> &params)
{
    SWING_PARAM  param;

    double targetStep[3]; //移动目标位置
    double targetHeight; //

    for(auto &i:params)
    {
        if(i.first=="component")
        {
            if(i.second=="lf")
            {
                param.comID=0;
            }
            else if(i.second=="lm")
            {
                param.comID=1;
            }
            else if(i.second=="lr")
            {
                param.comID=2;
            }
            else if(i.second=="rf")
            {
                param.comID=3;
            }
            else if(i.second=="rm")
            {
                param.comID=4;
            }
            else if(i.second=="rr")
            {
                param.comID=5;
            }

            else
            {
                std::cout<<"component parse failed"<<std::endl;
                return MSG{};
            }
        }

        else if(i.first=="mode")
        {

            if(i.second=="p")
            {
                param.actuationMode=Aris::RT_CONTROL::OM_CYCLICVEL;

            }
            else if(i.second=="f")
            {

                param.actuationMode=Aris::RT_CONTROL::OM_CYCLICTORQ;
                std::cout<<"parsing mode to torque mode mode: "<<param.actuationMode<<std::endl;

            }
            else
            {
                std::cout<<"mode parse failed"<<std::endl;
                return MSG{};
            }

        }
        else if(i.first=="u")
        {
            targetStep[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            targetStep[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            targetStep[2]=stod(i.second);
        }
        else if(i.first=="h")
        {
            targetHeight=stod(i.second);
        }

        else
        {
            std::cout<<"last parse failed"<<std::endl;
            return MSG{};
        }
    }

    std::copy_n(targetStep, 3, param.targetStep);
    param.targetHeight=targetHeight;
    param.legNum=1;
    param.motorNum=3;
    param.legID[0]=param.comID;
    int motors[3] = { 3*param.comID, 3*param.comID+1, 3*param.comID+2 };
    std::copy_n(motors, 3, param.motorID);


    param.periodCount=4000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse swing"<<std::endl;

    return msg;

}



int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
   const SWING_PARAM *pSP = static_cast<const SWING_PARAM *>(pParam);

    double realTargetPee[18];
   // double realTargetPbody[6];

    std::copy_n(pSP->beginPee, 18, realTargetPee);


    for(int i=0;i<3;i++)
    {
        realTargetPee[pSP->comID*3+i]+=pSP->targetStep[i];
    }



    double alpha=sqrt((pSP->targetHeight)/(pSP->targetHeight-pSP->targetStep[1]));

    std::int32_t N1,N2,N;
    N=pSP->periodCount;
    N1=alpha/(alpha+1)*N;
    N2=1/(alpha+1)*N;

    double pEE[18],vEE[18],aEE[18];

    double s= PI*pSP->count/pSP->periodCount;
    double s1= PI*pSP->count/N1;
    double s2= PI*(pSP->count-N1)/N2;



    pEE[pSP->comID*3] = pSP->beginPee[pSP->comID*3] * (cos(s) + 1) / 2 + realTargetPee[pSP->comID*3] * (1 - cos(s)) / 2;//X
    vEE[pSP->comID*3]=1000*1/2*PI/N*(realTargetPee[pSP->comID*3]-pSP->beginPee[pSP->comID*3])*sin(s);
    aEE[pSP->comID*3]=1000000*1/2*pow((PI/N),2)*(realTargetPee[pSP->comID*3]-pSP->beginPee[pSP->comID*3])*cos(s);

    pEE[pSP->comID*3+2] = pSP->beginPee[pSP->comID*3+2] * (cos(s) + 1) / 2 + realTargetPee[pSP->comID*3+2] * (1 - cos(s)) / 2;//Z
    vEE[pSP->comID*3+2]=1000*1/2*PI/N*(realTargetPee[pSP->comID*3+2]-pSP->beginPee[pSP->comID*3+2])*sin(s);
    aEE[pSP->comID*3+2]=1000000*1/2*pow((PI/N),2)*(realTargetPee[pSP->comID*3+2]-pSP->beginPee[pSP->comID*3+2])*cos(s);


    if(pSP->count<=N1)
    {
        pEE[pSP->comID*3+1] = pSP->beginPee[pSP->comID*3+1] * (cos(s1) + 1) / 2 + (pSP->targetHeight+pSP->beginPee[pSP->comID*3+1]) * (1 - cos(s1)) / 2;//Y
        vEE[pSP->comID*3+1]=1000*1/2*PI/N1*( pSP->targetHeight )*sin(s1);
        aEE[pSP->comID*3+1]=1000000*1/2*pow((PI/N1),2)*( pSP->targetHeight )*cos(s1);


    }
    else
    {
        pEE[pSP->comID*3+1] = (pSP->targetHeight+pSP->beginPee[pSP->comID*3+1]) * (cos(s2) + 1) / 2 + realTargetPee[pSP->comID*3+1] * (1 - cos(s2)) / 2;//Y
        vEE[pSP->comID*3+1]=1000*1/2*PI/N2*( realTargetPee[pSP->comID*3+1]-pSP->beginPee[pSP->comID*3+1]-pSP->targetHeight )*sin(s2);
        aEE[pSP->comID*3+1]=1000000*1/2*pow((PI/N2),2)*( realTargetPee[pSP->comID*3+1]-pSP->beginPee[pSP->comID*3+1]-pSP->targetHeight )*cos(s2);

    }



    if(pSP->count==1)
    {
        rt_printf("N1 %d, N2 %d\n",N1,N2);
        rt_printf("pee %f,%f,%f\n",pEE[15],pEE[16],pEE[17]);
        rt_printf("pbody %f %f %f %f %f %f\n",pSP->beginBodyPE[0],pSP->beginBodyPE[1],pSP->beginBodyPE[2],pSP->beginBodyPE[3],pSP->beginBodyPE[4],pSP->beginBodyPE[5],pSP->beginBodyPE[6]);

    }

      pRobot->SetPee(pEE, pSP->beginBodyPE);
      pRobot->SetVee(vEE);
      pRobot->SetAee(aEE);


    //  if(pParam->count%300==0)
      //{
        //  rt_printf("motor operation mod %d\n",pParam->actuationMode);
          //rt_printf("motor position %f %f %f\n",pEE[15],pEE[16],pEE[17]);
          //rt_printf("motor velcity %f %f %f\n",vEE[0],vEE[1],vEE[2]);
          //rt_printf("motor acc  %f %f %f\n",aEE[0],aEE[1],aEE[2]);

//      }



    /*返回剩余的count数*/

    return pSP->periodCount - pSP->count - 1;


}

struct MOVES_PARAM :public Robots::GAIT_PARAM_BASE
{
    double targetPee[18]{0};
    double targetBodyPE[6]{0};
    std::int32_t periodCount;
    int comID; //移动的部件（component）序号
    bool isAbsolute{false}; //用于判断移动命令是绝对坐标还是相对坐标
};

Aris::Core::MSG parseMove(const std::string &cmd, const map<std::string, std::string> &params)
{
    MOVES_PARAM  param;

    double targetPos[3]; //移动目标位置

    for(auto &i:params)
    {
        if(i.first=="component")
        {
            if(i.second=="lf")
            {
                param.comID=0;
            }
            else if(i.second=="lm")
            {
                param.comID=1;
            }
            else if(i.second=="lr")
            {
                param.comID=2;
            }
            else if(i.second=="rf")
            {
                param.comID=3;
            }
            else if(i.second=="rm")
            {
                param.comID=4;
            }
            else if(i.second=="rr")
            {
                param.comID=5;

            }
            else if(i.second=="bd")
            {
                param.comID=6;
            }
            else
            {
                std::cout<<"parse failed"<<std::endl;
                return MSG{};
            }
        }
        //绝对坐标移动
        else if(i.first=="x")
        {
            targetPos[0]=stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="y")
        {
            targetPos[1]=stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="z")
        {
            targetPos[2]=stod(i.second);
            param.isAbsolute=true;
        }
        //相对坐标移动
        else if(i.first=="u")
        {
            targetPos[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            targetPos[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            targetPos[2]=stod(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }


    if(param.comID==6)
    {
        std::copy_n(targetPos, 3, param.targetBodyPE);
    }
    else
    {
        std::copy_n(targetPos, 3, &param.targetPee[3*param.comID]);
        param.legNum=1;
        param.motorNum=3;
        param.legID[0]=param.comID;
        int motors[3] = { 3*param.comID, 3*param.comID+1, 3*param.comID+2 };
        std::copy_n(motors, 3, param.motorID);

    }

    param.periodCount=3000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}


int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const MOVES_PARAM *pMP = static_cast<const MOVES_PARAM *>(pParam);

    double realTargetPee[18];
    double realTargetPbody[6];
    std::copy_n(pMP->beginPee, 18, realTargetPee);
    std::copy_n(pMP->beginBodyPE, 6, realTargetPbody);

    //绝对坐标
    if (pMP->isAbsolute)
    {
        if(pMP->comID==6)
        {
            std::copy_n(pMP->targetBodyPE, 6, realTargetPbody);
        }
        else
        {
            std::copy_n(&(pMP->beginPee[pMP->comID*3]), 3, &realTargetPee[pMP->comID*3]);
        }
    }
    //相对坐标
    else
    {
        if(pMP->comID==6)
        {
            for(int i=0;i<6;i++)
            {
                realTargetPbody[i]+=pMP->targetBodyPE[i];
            }
        }
        else
        {
            for(int i=0;i<18;i++)
            {
                realTargetPee[i]+=pMP->targetPee[i];
            }
        }
    }

    double s = -(PI / 2)*cos(PI * (pMP->count  + 1) / pMP->periodCount ) + PI / 2;

    /*插值当前的末端和身体位置*/
    double pEE[18], pBody[6];

    for (int i = 0; i < 18; ++i)
    {
        pEE[i] = pMP->beginPee[i] * (cos(s) + 1) / 2 + realTargetPee[i] * (1 - cos(s)) / 2;
    }
    for (int i = 0; i < 6; ++i)
    {
        pBody[i] = pMP->beginBodyPE[i] * (cos(s) + 1) / 2 + realTargetPbody[i] * (1 - cos(s)) / 2;
    }

    pRobot->SetPee(pEE, pBody);

    /*返回剩余的count数*/

    return pMP->periodCount - pMP->count - 1;

}


int main()
{

	auto rs = Robots::ROBOT_SERVER::GetInstance();
    rs->CreateRobot<Robots::ROBOT_III>();
    rs->LoadXml("/usr/Robots/CMakeDemo/Robot_III/resource/HexapodIII_Move.xml");
    rs->AddGait("wk",Robots::walk,parseWalk);
    rs->AddGait("ad",Robots::adjust,parseAdjust);
    rs->AddGait("move",move2,parseMove);
    rs->AddGait("swing",swing,parseSwing);

    Aris::Core::RegisterMsgCallback(1111,[](Aris::Core::MSG &msg)
    {

          Robots::FORCE_PARAM_BASE  data_force;


           int output_count;

           msg.PasteStruct(data_force);
           msg.PasteAt(&output_count,sizeof(int),sizeof(data_force));

           /*msg.PasteAt(fIN,sizeof(double)*18,0);
           msg.PasteAt(fIN_friction,sizeof(double)*18,sizeof(double)*18);
           msg.PasteAt(fIN_actual,sizeof(double)*18,sizeof(double)*36);
           msg.PasteAt(acc_model,sizeof(double)*18,sizeof(double)*54);
           msg.PasteAt(vel_model,sizeof(double)*18,sizeof(double)*72);
           msg.PasteAt(&output_count,sizeof(int),sizeof(double)*90);*/

       static std::ofstream file;
       char LogFile[300];
       char tmpDate[100];

       if (output_count==0)
       {
           time_t now;
           struct tm *p;
           time(&now);
           p = localtime(&now);

           strftime(tmpDate,99,"%Y_%m_%d_%H_%M_%S",p);//combine char strings
           sprintf(LogFile,"force_data_%s.txt",tmpDate);
           file.open(LogFile);
       }
       else
           file.open(LogFile,ios_base::app);



       file<<output_count<<"    ";
       for(int i=0;i<18;i++)
      {
          file<<data_force.Fin_modeled[i]<<"  ";

      }
       file<<"  ";

        for(int i=0;i<18;i++)
       {
           file<<data_force.Fin_read[i]<<"  ";

       }
        file<<"  ";

        for(int i=0;i<18;i++)
       {
           file<<data_force.Fin_write[i]<<"  ";

       }
        file<<"  ";


       for(int i=0;i<18;i++)
       {
           file<<data_force.Pee_filtered[i]<<"  ";

       }
       file<<"  ";

       for(int i=0;i<18;i++)
       {
           file<<data_force.Vee_filtered[i]<<"  ";

       }
       file<<"  ";

       for(int i=0;i<18;i++)
       {
           file<<data_force.Pee_desired[i]<<"   ";
       }
       file<<"  ";


       for(int i=0;i<18;i++)
       {
           file<<data_force.Vee_desired[i]<<"   ";

       }
       file<<"  ";



       file<<std::endl;
       output_count++;
       file.close();

       return 0;

    });

    rs->Start();
	/**/
	std::cout<<"finished"<<std::endl;


 	Aris::Core::RunMsgLoop();

	return 0;
}
