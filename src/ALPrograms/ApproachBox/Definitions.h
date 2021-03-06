#ifndef DEFINITIONS_H
#define DEFINITIONS_H


#ifndef PI
#define PI  3.1415927f
#endif

#define     WIN_NUM             400
#define     TARGET_FOOT_NUM     3
#define     SHORT_FOOT_NUM      20
#define     TICK_TIME           0.005
#define     DEF_DSP_TIME        0.1
#define     DEF_SSP_TIME        0.8
#define     CoM_Height          0.87
#define     JOY_MAX             32767.0
#define     FOOT_HEIGHT         0.06;


// User Input ==============
typedef struct{
    double SINGLELOG_DSP_INIT = 1.2;
    double SINGLELOG_DSP = 0.5;
    double SINGLELOG_SSP = 2.0;
    double SINGLELOG_WIDTH = 0.1;
    double SINGLELOG_LENGTH = 0.35;
}USER_INPUT;





// walking states =========
typedef enum{
    DSP_INIT_RF = 0,
    DSP_INIT_LF,
    DSP_RF,
    SSP_RF,
    DSP_LF,
    SSP_LF,
    DSP_FINAL,
    STATE_EMPTY
}WALKING_STATE;

extern char* STATE_NAME[];
// ========================

// foot information =======
typedef enum{
    FOOTINFO_NO = 0,
    FOOTINFO_FIRST_STEP,
    FOOTINFO_LAST_STEP,
    FOOTINFO_NOTHING_LAST_STEP
}FOOTPRINT_INFO;

typedef enum{
    MOVING_NOTHING = 0,
    MOVING_RIGHT,
    MOVING_LEFT,
    MOVING_JUMP,
    MOVING_EMPTY
}MOVING_LEG;

typedef enum{
    WALKING_NORMAL = 0,
    WALKING_SINGLELOG,
    WALKING_SINGLELOG_FINAL,
    WALKING_CHANGE,
    WALKING_STEPPING_STONE
}WALKING_MODE;
// =========================


// foot & zmp struct =======
typedef struct{
    double  rfoot[3];   // x, y, z
    double  rori[3];    // yaw, roll, pitch
    double  lfoot[3];   // x, y, z
    double  lori[3];    // yaw, roll, pitch
}_footprint;

typedef struct{
    double  x;
    double  y;
}_zmp;



typedef struct{
    double  current;
    double  total;
}_state_timer;

typedef struct{
    double  x;
    double  y;
    double  z;
    double  yaw;
    double  roll;
    double  pitch;

}_foot_ref;
// ==========================


// window information =======
typedef struct{
    WALKING_STATE   state;
    _state_timer    timer;
    _zmp            zmp;
    _foot_ref       right_foot_ref;
    _foot_ref       left_foot_ref;
    _footprint      footprint;
}_window_element;
// ==========================


// footprint information ====
typedef struct{
    double  dsp_time;
    double  ssp_time;
}_time_info;

typedef struct{
    _footprint      footprint;
    _time_info      time;
    FOOTPRINT_INFO  info;
    MOVING_LEG      movingleg;
    WALKING_MODE    mode;
}_footprint_info;
// ===========================





#endif // DEFINITIONS_H
