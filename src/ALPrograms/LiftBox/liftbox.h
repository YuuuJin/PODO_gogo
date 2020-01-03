/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */
#ifndef LIFTBOX_H
#define LIFTBOX_H
//#include <QCoreApplication>

#include "../../../share/Headers/commandlist.h"
#include "joint.h"
#include "taskmotion.h"
#include "ManualCAN.h"
#include "UserSharedMemory.h"
#include "BasicFiles/BasicTrajectory.h"
#include <iostream>
#include <libpcan.h>
#include <iostream>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <alchemy/task.h>

#define PODO_AL_NAME       "APPROACHBOX_AL"

using namespace std;

inline void pushData(doubles &tar, double var){
    tar.push_back(var);
    tar.pop_front();
}

/***************************** 1. Structs ************************************/
/* Shared memory */
pRBCORE_SHM     sharedData;
pUSER_SHM       userData;

/* RT task handler for control */
TaskMotion      *WBmotion;
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

enum{
    NOTHING = 0, SITDOWN, WAIT, HOLDBOX, RELEASEBOX, HANDUP, STANDUP
};
enum{
    MODE_NOTHING = 0, MODE_LIFTBOX, MODE_PUTBOX, MODE_BACK
};
enum{
    HOLD_YET = 0, HOLD_SUCCESS, HOLD_FAIL
};
enum{
    BACK_NOTHING = 0, BACK_BACK, BACK_SIT, BACK_HOLD, BACK_LIFT, BACK_WAIT, BACK_STAND, BACK_FRONT
};
/***************************** 2. Flags **************************************/
int     isTerminated;
int     __IS_WORKING;
int     __IS_GAZEBO;
/* Command */
int     Command_LIFTBOX = NOTHING;
int     Mode_LIFTBOX = MODE_NOTHING;
int     Mode_BACKMOTION = BACK_NOTHING;
/***************************** 3. Variables **********************************/
/* Basic */
int PODO_NO;
int PODO_NO_DAEMON = 0;
int PODO_NO_WALKREADY;
char __AL_NAME[30];

/* WBIK */
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;

/* Motion */
typedef struct{
    double SitHandX = 0.6;
    double SitHandY = 0.35;
    double SitHandZ = 0.45;
    double SitPelZ = 0.8;
    double ElbAngle = 15.0;

    double StandHandX = 0.5;
    double StandHandZ = 1.0;
    double StandPelZ = 0.78;
    double FinalHandX = 0.3;
    double FinalHandY = 0.245;
    double FinalHandZ = 1.0;

    double HandPitch = -50.;
    double HandYaw = 10.;
    double PelPitch = 60.;


    double HandUpZ = 0.15;
    double HoldY = 0.0003;
    double ReleaseY = 0.0004;

    double TimeSit = 3.0;
    double TimeMaxHold = 3.5;
    double TimeRelease = 2.5;
    double TimeHandUp = 1.5;
    double TimeStand = 4.0;

    double FTlimit = 75;//N


}input;
input in;

double PELpos[3];
double RHpos[3];
double LHpos[3];
doubles RHori(4);
doubles LHori(4);
doubles PELori(4);


double WaitTime = 0.;

int WaitMode = NOTHING;
int WaitCount = 0;
int HoldCount = 0;
int ReleaseCount = 0;


/***************************** 4. Functions **********************************/
/* Basic */
int HasAnyOwnership();
int CheckMotionOwned();
void RBTaskThread(void *);
void RBFlagThread(void *);
void CatchSignals(int _signal);
void ShowWBInfos();

/* Initialization */
int RBInitialize(void);
void ShutDownAllFlag();
void SaveWalkReadyPos();
void StartWBIKmotion(int _mode);

/* Motion */
void LiftBox_Supervisor();

void SitDown(int _mode);
int HoldBox();
int ReleaseBox();
void HandUp();
void StandUp(int _mode);
int GainOverrideSY();
void Set_RHand_Global2Local(vec3 _pos, quat _ori);
void Set_LHand_Global2Local(vec3 _pos, quat _ori);
void SetOriPitch(doubles &target, double _pitch);
void SetOriYaw(doubles &target, double _yaw);
void SetOriHand(doubles &target, double _pitch, double _yaw);
int CheckFTsensor();
void SetWaitTime(int mode, double time);
#endif // LIFTBOX_H
