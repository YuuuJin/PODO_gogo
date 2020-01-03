#include "liftbox.h"
// --------------------------------------------------------------------------------------------- //
void RHGO();
JointControlClass *joint;
void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    sprintf(__AL_NAME, "LiftBox");
    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == -1 )
        isTerminated = -1;


    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedData, joint);

    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);
        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
            case LIFTBOX_LIFTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_LIFTBOX received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_LIFTBOX;
                Command_LIFTBOX = SITDOWN;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_PUTBOX:
            {
                FILE_LOG(logSUCCESS) << "Command LIFTBOX_PUTBOX received..\n";

                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(-1);
                joint->SetAllMotionOwner();

                Mode_LIFTBOX = MODE_PUTBOX;
                Command_LIFTBOX = SITDOWN;

                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            case LIFTBOX_BACK_MOTION:
            {
                FILE_LOG(logSUCCESS) << "New CMD :: LIFTBOX_BACK_MOTION received..\n";

                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//back
                {
                    ShutDownAllFlag();
                    joint->RefreshToCurrentReference();
                    StartWBIKmotion(0);
                    joint->SetAllMotionOwner();
                    Mode_LIFTBOX = MODE_BACK;
                    Mode_BACKMOTION = BACK_BACK;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//sit
                {
                     Mode_BACKMOTION = BACK_SIT;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//hold
                {
                     Mode_BACKMOTION = BACK_HOLD;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 3)//stand
                {
                     Mode_BACKMOTION = BACK_STAND;
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 4)//front
                {
                     Mode_BACKMOTION = BACK_FRONT;
                }
                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
            default:
                sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
                break;
            }
    }
    cout << ">>> Process LiftBox is terminated..!!" << endl;
    return 0;
}

void RBTaskThread(void *)
{
     while(isTerminated == 0)
    {
        LiftBox_Supervisor();

        if(WB_FLAG == true)
        {
            // Global whole body model
            WBmotion->updateAll();
            if(Mode_LIFTBOX == MODE_BACK)
                WBmotion->WBIK_BACK();
            else
                WBmotion->WBIK();

            for(int i=RHY; i<=LAR; i++)
            {
                joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);
            }
            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }


        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);
        if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
            joint->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }

    }
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int RBInitialize(void)
{
    // Block program termination
    isTerminated = 0;

    char task_thread_name[30];
    char flag_thread_name[30];
    sprintf(task_thread_name, "%s_TASK", __AL_NAME);
    sprintf(flag_thread_name, "%s_FLAG", __AL_NAME);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            return false;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================


    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return false;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return false;
            }
        }
        memset(userData, 0, sizeof(USER_SHM));
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedData, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, flag_thread_name, 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(3, &aCPU);
        if(rt_task_set_affinity(&rtFlagCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Flag real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtFlagCon, &RBFlagThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Flag real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Flag real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }

    if(rt_task_create(&rtTaskCon, task_thread_name, 0, 90, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Task real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBTaskThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Task real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Task real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}
void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    // RF_or_LF: 1=LF, -1=RF, 0=PC
    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

void ShutDownAllFlag()
{
    Command_LIFTBOX = NOTHING;
    Mode_LIFTBOX = MODE_NOTHING;
    WB_FLAG = false;
}

void ShowWBInfos()
{
    printf("======================= WBInfos =======================\n");
    printf("LHPos = (%f, %f, %f)\n",WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
    printf("LHOri = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    printf("RHPos = (%f, %f, %f)\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
    printf("RHOri = (%f, %f, %f, %f)\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    printf("PelPos = (%f, %f, %f)\n",WBmotion->pCOM_2x1[0],WBmotion->pCOM_2x1[1],WBmotion->pPelZ);
    printf("PelOri = (%f, %f, %f, %f)\n",WBmotion->qPEL_4x1[0],WBmotion->qPEL_4x1[1],WBmotion->qPEL_4x1[2],WBmotion->qPEL_4x1[3]);
    printf("=======================================================\n\n");
}


void LiftBox_Supervisor()
{
    switch(Mode_LIFTBOX)
    {
    case MODE_LIFTBOX:
    {
        switch(Command_LIFTBOX)
        {
        case SITDOWN:
        {
            GainOverrideSY();
            SitDown(MODE_LIFTBOX);
            SetWaitTime(SITDOWN, in.TimeSit);
            break;
        }
        case WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == SITDOWN)
                {
                    FILE_LOG(logSUCCESS) << "SITDOWN OK";
                    Command_LIFTBOX = HOLDBOX;
                } else if(WaitMode == HANDUP)
                {
                    FILE_LOG(logSUCCESS) << "HANDUP OK";
                    Command_LIFTBOX = STANDUP;
                } else if(WaitMode == STANDUP)
                {
                    FILE_LOG(logSUCCESS) << "STANDUP OK";
                    Command_LIFTBOX = NOTHING;
                }
                WaitCount = 0;
            }
            break;
        }
        case HOLDBOX:
        {
            int StateHoldBox = HoldBox();

            if(StateHoldBox == HOLD_SUCCESS)
            {
                FILE_LOG(logSUCCESS) << "Hold Box SUCCESS";
                Command_LIFTBOX = HANDUP;
            } else if(StateHoldBox == HOLD_FAIL)
            {
                FILE_LOG(logERROR) << "Hold Box FAIL";
                Command_LIFTBOX = HANDUP;
            }
            break;
        }
        case HANDUP:
        {
            HandUp();
            SetWaitTime(HANDUP, in.TimeHandUp);
            break;
        }
        case STANDUP:
        {
            printf("standup\n");
            StandUp(MODE_LIFTBOX);
            SetWaitTime(STANDUP, in.TimeStand);
            break;
        }
        case NOTHING:
        {
            break;
        }
        default:
        {
            FILE_LOG(logERROR) << "LIFT BOX Command Error !!";
            printf("%d\n",Command_LIFTBOX);
            ShutDownAllFlag();
            break;
        }
        }

        break;
    }
    case MODE_PUTBOX:
    {
        switch(Command_LIFTBOX)
        {
        case SITDOWN:
        {
            SitDown(MODE_PUTBOX);
            SetWaitTime(SITDOWN, in.TimeSit);
            break;
        }
        case WAIT:
        {
            WaitCount++;

            if(WaitCount > WaitTime*200)
            {
                if(WaitMode == SITDOWN)
                {
                    FILE_LOG(logSUCCESS) << "SITDOWN OK";
                    Command_LIFTBOX = RELEASEBOX;
                } else if(WaitMode == STANDUP)
                {
                    FILE_LOG(logSUCCESS) << "STANDUP OK";
                    Command_LIFTBOX = NOTHING;
                }
                WaitCount = 0;
            }
            break;
        }
        case RELEASEBOX:
        {
            if(ReleaseBox() == true)
            {
                FILE_LOG(logSUCCESS) << "Release Box SUCCESS";
                Command_LIFTBOX = STANDUP;
            }
            break;
        }
        case STANDUP:
        {
            StandUp(MODE_PUTBOX);
            SetWaitTime(STANDUP, in.TimeStand);
            break;
        }
        case NOTHING:
        {
            break;
        }
        default:
        {
            FILE_LOG(logERROR) << "PUT BOX Command Error !!";
            ShutDownAllFlag();
            break;
        }
        }
        break;
    }
    case MODE_BACK:
    {
        switch(Mode_BACKMOTION)
        {
        case BACK_BACK:
        {
            //SetOriYaw(PELori, 180.0);
           // WBmotion->addPELOriInfo(PELori, 2.0);
            printf("init2rinit = %f, %f, %f\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
            WBmotion->addWSTPosInfo(-180.0, 4.0);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_SIT:
        {
            PELpos[0] = WBmotion->pCOM_2x1[0]+0.01;
            PELpos[1] = WBmotion->pCOM_2x1[1];
            PELpos[2] = -0.2;
            WBmotion->addPELPosInfo(PELpos[2], in.TimeSit);

            SetOriPitch(PELori, -50.0);
            WBmotion->addPELOriInfo(PELori, in.TimeSit);
            WBmotion->addCOMInfo(PELpos[0], PELpos[1], in.TimeSit);

            vec3 des_rpos = vec3(-0.75, -0.4, 0.2);
            vec3 des_lpos = vec3(-0.75, 0.4,  0.2);
            quat des_ori = quat(vec3(0,1,0), 0);

            Set_RHand_Global2Local(des_rpos, des_ori);
            Set_LHand_Global2Local(des_lpos, des_ori);

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeSit);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeSit);

            SetOriHand(RHori, -50.,0.);
            SetOriHand(LHori, -50.,0.);
            WBmotion->addRHOriInfo(RHori, in.TimeSit);
            WBmotion->addLHOriInfo(LHori, in.TimeSit);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        case BACK_HOLD:
        {
            int StateHoldBox = HoldBox();

            if(StateHoldBox == HOLD_SUCCESS)
            {
                FILE_LOG(logSUCCESS) << "Hold Box SUCCESS";
                Mode_BACKMOTION = BACK_LIFT;
            } else if(StateHoldBox == HOLD_FAIL)
            {
                FILE_LOG(logERROR) << "Hold Box FAIL";
                Mode_BACKMOTION = BACK_LIFT;
            }
            break;
        }
        case BACK_LIFT:
        {
            vec3 des_rpos = vec3(-0.75, RHpos[1], 0.25 + 0.2);
            vec3 des_lpos = vec3(-0.75, LHpos[1], 0.25 + 0.2);
            quat des_ori = quat(vec3(0,1,0), 0);
            Set_RHand_Global2Local(des_rpos, des_ori);
            Set_LHand_Global2Local(des_lpos, des_ori);

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeHandUp);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeHandUp);
            Mode_BACKMOTION = BACK_WAIT;
            break;
        }
        case BACK_WAIT:
        {
            static int cnt = 0;
            if(cnt > in.TimeHandUp*200)
            {
                cnt = 0;
                Mode_BACKMOTION = BACK_NOTHING;
                break;
            }

            cnt++;
            double add_boxmass = 5.0/in.TimeHandUp*200;
            WBmotion->kine_drc.m_RightHand += add_boxmass;
            WBmotion->kine_drc.m_LeftHand += add_boxmass;

            break;
        }
        case BACK_STAND:
        {
            WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+0.01, WBmotion->pCOM_2x1[1], in.TimeStand);
            WBmotion->addPELPosInfo(0.0, in.TimeStand);

            SetOriPitch(PELori, 0.);
            WBmotion->addPELOriInfo(PELori, in.TimeStand);

            RHpos[0] = LHpos[0] = 0.45;
            RHpos[2] = LHpos[2] = 0.1;//in.StandHandZ;

            SetOriHand(RHori, 0., 0.);
            SetOriHand(LHori, 0., 0.);

            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeStand);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeStand);

            WBmotion->addRHOriInfo(RHori, in.TimeStand);
            WBmotion->addLHOriInfo(LHori, in.TimeStand);
            break;
        }
        case BACK_FRONT:
        {
            WBmotion->addWSTPosInfo(0.0, 7.0);
            Mode_BACKMOTION = BACK_NOTHING;
            break;
        }
        }
        break;
    }
    case MODE_NOTHING:
    {
        break;
    }
    }

}


void SitDown(int _mode)
{
    WBmotion->addPELPosInfo(in.SitPelZ, in.TimeSit);

    SetOriPitch(PELori, in.PelPitch);
    WBmotion->addPELOriInfo(PELori, in.TimeSit);

    RHpos[0] = LHpos[0] = in.SitHandX;
    if(_mode == MODE_LIFTBOX)
    {
        RHpos[1] = -in.SitHandY;
        LHpos[1] = in.SitHandY;
    }else if(_mode == MODE_PUTBOX)
    {
        RHpos[1] = RHpos[1];
        LHpos[1] = LHpos[1];
    }else
    {
        FILE_LOG(logERROR) << "SitDown mode error!!!";
        ShutDownAllFlag();
    }
    RHpos[2] = LHpos[2] = in.SitHandZ;
    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeSit);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeSit);
//    WBmotion->addRElbPosInfo(-in.ElbAngle, in.TimeSit);
//    WBmotion->addLElbPosInfo(in.ElbAngle, in.TimeSit);
    SetOriHand(RHori, in.HandPitch, in.HandYaw);
    SetOriHand(LHori, in.HandPitch, -in.HandYaw);
    WBmotion->addRHOriInfo(RHori, in.TimeSit);
    WBmotion->addLHOriInfo(LHori, in.TimeSit);
}

void StandUp(int _mode)
{
    WBmotion->addCOMInfo(WBmotion->pCOM_2x1[0]+0.01, WBmotion->pCOM_2x1[1], in.TimeStand);
    WBmotion->addPELPosInfo(in.StandPelZ, in.TimeStand);

    SetOriPitch(PELori, 0.);
    WBmotion->addPELOriInfo(PELori, in.TimeStand);

    if(_mode == MODE_LIFTBOX)
    {
        RHpos[0] = LHpos[0] = in.StandHandX;
        RHpos[2] = LHpos[2] = in.StandHandZ;

        SetOriHand(RHori, in.HandPitch, in.HandYaw);
        SetOriHand(LHori, in.HandPitch, -in.HandYaw);
    }else if(_mode == MODE_PUTBOX)
    {
        RHpos[0] = LHpos[0] = in.FinalHandX;
        RHpos[1] = -in.FinalHandY;
        LHpos[1] = in.FinalHandY;
        RHpos[2] = LHpos[2] = in.FinalHandZ;

        SetOriPitch(RHori, -90.);
        SetOriPitch(LHori, -90.);
    }else
    {
        FILE_LOG(logERROR) << "SitDown mode error!!!";
        ShutDownAllFlag();
    }
    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeStand);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeStand);

    WBmotion->addRHOriInfo(RHori, in.TimeStand);
    WBmotion->addLHOriInfo(LHori, in.TimeStand);
}

void HandUp()
{
    RHpos[2] = RHpos[2] + in.HandUpZ;
    LHpos[2] = LHpos[2] + in.HandUpZ;
    WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], in.TimeHandUp);
    WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], in.TimeHandUp);
}

int HoldBox()
{
    if(CheckFTsensor() == true)
    {
        HoldCount = 0;
        return HOLD_SUCCESS;
    }else
    {
        if(HoldCount > in.TimeMaxHold*200)
        {
            HoldCount = 0;
            return HOLD_FAIL;
        } else
        {
            HoldCount++;

            RHpos[1] += in.HoldY;
            LHpos[1] -= in.HoldY;
            WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], 0.005);
            WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], 0.005);
            return HOLD_YET;
        }
    }
}

int ReleaseBox()
{
    if(ReleaseCount > in.TimeRelease*200)
    {
        ReleaseCount = 0;
        return true;
    } else
    {
        ReleaseCount++;

        RHpos[1] -= in.ReleaseY;
        LHpos[1] += in.ReleaseY;
        WBmotion->addRHPosInfo(RHpos[0], RHpos[1], RHpos[2], 0.005);
        WBmotion->addLHPosInfo(LHpos[0], LHpos[1], LHpos[2], 0.005);
        return false;
    }
}

void SetOriPitch(doubles &target, double _pitch)
{
    quat pelori = quat(vec3(0,1,0), _pitch*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

void SetOriYaw(doubles &target, double _yaw)
{
    quat pelori = quat(vec3(0,0,1), _yaw*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

void SetOriHand(doubles &target, double _pitch, double _yaw)
{
    quat pelori = quat(vec3(0,0,1), _yaw*D2R)*quat(vec3(0,1,0), _pitch*D2R);
    for(int i=0;i<4;i++)
    {
        target[i] = pelori[i];
    }
}

int CheckFTsensor()
{
    if(sharedData->FT[2].Fy < -in.FTlimit && sharedData->FT[3].Fy > in.FTlimit)
    {
        return true;
    }else
    {
        return false;
    }

}

void SetWaitTime(int mode, double time)
{
    Command_LIFTBOX = WAIT;
    WaitCount = 0;
    WaitMode = mode;
    WaitTime = time;
}


int GainOverrideSY(){

    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 30, 10);
    usleep(5000);


    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 30, 10);
    usleep(5000);

    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}



void Set_RHand_Global2Local(vec3 _pos, quat _ori)
{
    vec3 p_init2global;
    p_init2global[0] = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    p_init2global[1] = (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2.;
    p_init2global[2] = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2.;


    mat4 T_init2global = mat4(vec3(p_init2global),vec3(0,0,1), 0.0);

    mat4 T_global2hand = mat4(_pos,_ori);

    mat4 T_init2local = mat4(vec3(PELpos[0],PELpos[1],PELpos[2]), quat(PELori[0],PELori[1],PELori[2],PELori[3]));

    mat4 T_local2init = T_init2local.inverse();

    mat4 T_local2global = T_local2init*T_init2global;

    mat4 T_local2hand = T_local2global*T_global2hand;

    RHpos[0] = -T_local2hand.m03;
    RHpos[1] = T_local2hand.m13;
    RHpos[2] = T_local2hand.m23;
}

void Set_LHand_Global2Local(vec3 _pos, quat _ori)
{
    vec3 p_init2global;
    p_init2global[0] = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    p_init2global[1] = (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2.;
    p_init2global[2] = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2.;


    mat4 T_init2global = mat4(vec3(p_init2global),vec3(0,0,1), 0.0);

    mat4 T_global2hand = mat4(_pos,_ori);

    mat4 T_init2local = mat4(vec3(PELpos[0],PELpos[1],PELpos[2]), quat(PELori[0],PELori[1],PELori[2],PELori[3]));

    mat4 T_local2init = T_init2local.inverse();

    mat4 T_local2global = T_local2init*T_init2global;

    mat4 T_local2hand = T_local2global*T_global2hand;

    LHpos[0] = -T_local2hand.m03;
    LHpos[1] = T_local2hand.m13;
    LHpos[2] = T_local2hand.m23;
}
