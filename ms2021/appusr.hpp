/*
    appusr.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef appusr_hpp
#define appusr_hpp

#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Steering.h"
#include "Clock.h"
using namespace ev3api;

#include "etroboc_ext.h"

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

#include "BrainTree.h"
#include "FilteredMotor.hpp"
#include "SRLF.hpp"
#include "FilteredColorSensor.hpp"
#include "FIR.hpp"
#include "Plotter.hpp"
#include "PIDcalculator.hpp"
#include "Logger.hpp"

/* global variables */
extern FILE*        bt;
extern Clock*       clock;
extern TouchSensor* touchSensor;
extern SonarSensor* sonarSensor;
extern FilteredColorSensor* colorSensor;
extern GyroSensor*  gyroSensor;
extern SRLF*        srlf_l;
extern FilteredMotor*       leftMotor;
extern SRLF*        srlf_r;
extern FilteredMotor*       rightMotor;
extern Motor*       tailMotor;
extern Motor*       armMotor;
extern Plotter*     plotter;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//#define LOG_ON_CONSOL

/* ##__VA_ARGS__ is gcc proprietary extention.
   this is also where -std=gnu++11 option is necessary */
#ifdef LOG_ON_CONSOL
#define _log(fmt, ...) \
    syslog(LOG_NOTICE, "%08u, %s: " fmt, \
    clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define _log(fmt, ...) \
    fprintf(bt, "%08u, %s: " fmt "\n", \
    clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#endif

/* macro to covert an enumeration constant to a string */
#define STR(var) #var

/* macro for making program compatible for both left and right courses.
   the default is left course. */ 
#if defined(MAKE_RIGHT)
    static const int _COURSE = -1;
#else
    static const int _COURSE = 1;
#endif

/* these parameters are intended to be given as a compiler directive,
   e.g., -D=SPEED_NORM=50, for fine tuning                                  */
#ifndef SPEED_NORM
#define SPEED_NORM           45  /* was 50 for 2020 program                 */
#endif
#ifndef P_CONST
#define P_CONST           0.75D
#endif
#ifndef I_CONST
#define I_CONST           0.39D /*0.39*/
#endif
#ifndef D_CONST
#define D_CONST           0.08D
#endif

#ifndef SPEED_FAST
#define SPEED_FAST          85
#endif
#ifndef P_CONST_FAST
#define P_CONST_FAST        0.5D
#endif
#ifndef I_CONST_FAST
#define I_CONST_FAST        0.5D
#endif
#ifndef D_CONST_FAST
#define D_CONST_FAST        0.08D
#endif

#ifndef SPEED_SLOW
#define SPEED_SLOW         10
#endif
#ifndef P_CONST_SLOW
#define P_CONST_SLOW       0.1D
#endif
#ifndef I_CONST_SLOW
#define I_CONST_SLOW       0.0D
#endif
#ifndef D_CONST_SLOW
#define D_CONST_SLOW       0.5D
#endif
#ifndef JUMP
#define JUMP                  0
#endif
#ifndef LOG_INTERVAL
#define LOG_INTERVAL          0
#endif

#define GS_TARGET            47  /* was 47 for 2020 program                 */
#define GS_TARGET_SLOW       25
#define SONAR_ALERT_DISTANCE 10  /* in centimeters                          */
#define BLUE_DISTANCE     8040  /* 2nd blue part should be further than this   */ 

enum Color {
    Black,
    Black_2,
    Black_3,
    Jetblack,
    Blue,
    Red,
    Yellow,
    Green,
    Green_2,
    Green_3,
    Green_4,
    Green_5,
    Green_6,
    Gray,
    Gray_2,
    Gray_3,
    White,
    White_2,
    White_3,
    White_4,
};

enum CalcMode {
    More,
    Less
};


#define ARM_INITIAL_ANGLE     -58

enum BoardItem {
    LOCX, /* horizontal location    */
    LOCY, /* virtical   location    */
    DIST, /* accumulated distance   */
};

enum State {
    ST_initial,
    ST_calibrating,
    ST_running,
    ST_slalom,
    ST_garage,
    ST_ending,
    ST_end,
};

#endif /* appusr_hpp */