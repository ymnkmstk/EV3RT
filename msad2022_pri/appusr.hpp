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

#if defined(MAKE_SIM)
#include "etroboc_ext.h"
#endif

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

#include "FilteredMotor.hpp"
#include "SRLF.hpp"
#include "FilteredColorSensor.hpp"
#include "FIR.hpp"
#include "Plotter.hpp"
#include "PIDcalculator.hpp"

/* global variables */
extern FILE*        bt;
extern Clock*       ev3clock;
extern TouchSensor* touchSensor;
extern SonarSensor* sonarSensor;
extern FilteredColorSensor* colorSensor;
extern GyroSensor*  gyroSensor;
extern SRLF*        srlf_l;
extern FilteredMotor*       leftMotor;
extern SRLF*        srlf_r;
extern FilteredMotor*       rightMotor;
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
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define _log(fmt, ...) \
    printf("%08u, %s: " fmt "\n", \
    ev3clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    /* fprintf(bt, "%08u, %s: " fmt "\n", \ */
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
#define SPEED_NORM              45  /* was 50 for 2020 program                 */
#endif
#ifndef SPEED_SLOW
#define SPEED_SLOW              40
#endif
#ifndef P_CONST
#define P_CONST                 0.75D
#endif
#ifndef I_CONST
#define I_CONST                 0.39D
#endif
#ifndef D_CONST
#define D_CONST                 0.08D
#endif

#ifndef JUMP
#define JUMP                    2
#endif

#ifndef LOG_INTERVAL
#define LOG_INTERVAL            0
#endif

#define GS_TARGET               47      /* was 47 for 2020 program                 */
#define GS_TARGET_SLOW          25
#define SONAR_ALERT_DISTANCE    100     /* in millimeter                           */
#define ARM_INITIAL_ANGLE       -58
#define ARM_SHIFT_PWM           100

enum Color {
    CL_JETBLACK,
    CL_BLACK,
    CL_BLUE,
    CL_RED,
    CL_YELLOW,
    CL_GREEN,
    CL_GRAY,
    CL_WHITE,
};

enum BoardItem {
    LOCX, /* horizontal location    */
    LOCY, /* virtical   location    */
    DIST, /* accumulated distance   */
};

enum State {
    ST_INITIAL,
    ST_CALIBRATION,
    ST_RUN,
    ST_SLALOM_FIRST,
    ST_SLALOM_CHECK,
    ST_SLALOM_SECOND_A,
    ST_SLALOM_SECOND_B,
    ST_BLOCK,
    ST_ENDING,
    ST_END,
};

enum TraceSide {
    TS_NORMAL,
    TS_OPPOSITE,
};

#endif /* appusr_hpp */