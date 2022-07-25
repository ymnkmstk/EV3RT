/*
    app.cpp

    Copyright © 2022 MSAD Mode2P. All rights reserved.
*/
#include "BrainTree.h"
#include <opencv2/opencv.hpp>
using namespace cv;
/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 for compilation
*/
#include "NumCpp.hpp"
#include "Profile.hpp"
/*
    BrainTree.h, opencv.hpp and NumCpp.hpp must present
    before ev3api.h on RasPike environment.
    Note that ev3api.h is included by app.h.
*/
#include "app.h"
#include "appusr.hpp"
#include <iostream>
#include <list>
#include <numeric>
#include <math.h>
using namespace std;

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define ROI_BOUNDARY 50

/* this is to avoid linker error, undefined reference to `__sync_synchronize' */
extern "C" void __sync_synchronize() {}

/* global variables */
FILE*           bt;
Profile*        prof;
VideoCapture    cap;
Mat             frame;
Clock*          ev3clock;
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
FilteredColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
SRLF*           srlfL;
FilteredMotor*  leftMotor;
SRLF*           srlfR;
FilteredMotor*  rightMotor;
Motor*          armMotor;
Plotter*        plotter;

BrainTree::BehaviorTree* tr_calibration = nullptr;
BrainTree::BehaviorTree* tr_run         = nullptr;
BrainTree::BehaviorTree* tr_block       = nullptr;
State state = ST_INITIAL;

/*
    === NODE CLASS DEFINITION STARTS HERE ===
    A Node class serves like a LEGO block while a Behavior Tree serves as a blueprint for the LEGO object built using the LEGO blocks.
*/

/*
    usage:
    ".leaf<ResetClock>()"
    is to reset the clock and indicate the robot departure by LED color.
*/
class ResetClock : public BrainTree::Node {
public:
    Status update() override {
        ev3clock->reset();
        _log("clock reset.");
        ev3_led_set_color(LED_GREEN);
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<StopNow>()"
    is to stop the robot.
*/
class StopNow : public BrainTree::Node {
public:
    Status update() override {
        leftMotor->setPWM(0);
        rightMotor->setPWM(0);
        _log("robot stopped.");
        return Status::Success;
    }
};

/*
    usage:
    ".leaf<IsTouchOn>()"
    is to check if the touch sensor gets pressed.
*/
class IsTouchOn : public BrainTree::Node {
public:
    Status update() override {
        if (touchSensor->isPressed()) {
            _log("touch sensor pressed.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

/*
    usage:
    ".leaf<IsBackOn>()"
    is to check if the back button gets pressed.
*/
class IsBackOn : public BrainTree::Node {
public:
    Status update() override {
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            _log("back button pressed.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

/*
    usage:
    ".leaf<IsSonarOn>(distance)"
    is to determine if the robot is closer than the spedified alert distance
    to an object in front of sonar sensor.
    dist is in millimeter.
*/
class IsSonarOn : public BrainTree::Node {
public:
    IsSonarOn(int32_t d) : alertDistance(d) {}
    Status update() override {
        int32_t distance = 10 * (sonarSensor->getDistance());
        if ((distance <= alertDistance) && (distance >= 0)) {
            _log("sonar alert at %d", distance);
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int32_t alertDistance;
};

/*
    usage:
    ".leaf<IsAngleLarger>(angle)"
    is to determine if the angular location of the robot measured by the gyro sensor is larger than the spedified angular value.
    angle is in degree.
*/
class IsAngleLarger : public BrainTree::Node {
public:
    IsAngleLarger(int ang) : angle(ang) {}
    Status update() override {
        int32_t curAngle = gyroSensor->getAngle(); 
        if (curAngle >= angle){
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t angle;
};

/*
    usage:
    ".leaf<IsAngleSmaller>(angle)"
    is to determine if the angular location of the robot measured by the gyro sensor is smaller than the spedified angular value.
    angle is in degree.
*/
class IsAngleSmaller : public BrainTree::Node {
public:
    IsAngleSmaller(int ang) : angle(ang) {}
    Status update() override {
        int32_t curAngle = gyroSensor->getAngle(); 
        if (curAngle <= angle){
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t angle;
};

/*
    usage:
    ".leaf<IsDistanceEarned>(dist)"
    is to determine if the robot has accumulated for the specified distance since update() was invoked for the first time.
    dist is in millimeter.
*/
class IsDistanceEarned : public BrainTree::Node {
public:
    IsDistanceEarned(int32_t d) : deltaDistTarget(d) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
            originalDist = plotter->getDistance();
            _log("ODO=%05d, Distance accumulation started.", originalDist);
            updated = true;
        }
        int32_t deltaDist = plotter->getDistance() - originalDist;
        
        if ((deltaDist >= deltaDistTarget) || (-deltaDist <= -deltaDistTarget)) {
            if (!earned) {
                _log("ODO=%05d, Delta distance %d is earned.", plotter->getDistance(), deltaDistTarget);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t deltaDistTarget, originalDist;
    bool updated, earned;
};

/*
    usage:
    ".leaf<IsTimeEarned>(time)"
    is to determine if the robot has accumulated for the specified time since update() was invoked for the first time.
    time is in microsecond = 1/1,000,000 second.
*/
class IsTimeEarned : public BrainTree::Node {
public:
    IsTimeEarned(int32_t t) : deltaTimeTarget(t) {
        updated = false;
        earned = false;
    }
    Status update() override {
        if (!updated) {
            originalTime = ev3clock->now();
            _log("ODO=%05d, Time accumulation started.", plotter->getDistance());
             updated = true;
        }
        int32_t deltaTime = ev3clock->now() - originalTime;

        if (deltaTime >= deltaTimeTarget) {
            if (!earned) {
                 _log("ODO=%05d, Delta time %d is earned.", plotter->getDistance(), deltaTimeTarget);
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
protected:
    int32_t deltaTimeTarget, originalTime;
    bool updated, earned;
};

/*
    usage:
    ".leaf<IsColorDetected>(color)"
    is to determine if the specified color gets detected.
    For possible color that can be specified as the argument, see enum Color in "appusr.hpp".
*/
class IsColorDetected : public BrainTree::Node {
public:
    IsColorDetected(Color c) : color(c) {
        updated = false;
    }
    Status update() override {
        if (!updated) {
            _log("ODO=%05d, Color detection started.", plotter->getDistance());
            updated = true;
        }
        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);

        switch(color){
            case CL_JETBLACK:
                if (cur_rgb.r <=35 && cur_rgb.g <=35 && cur_rgb.b <=50) { 
                    _log("ODO=%05d, CL_JETBLACK detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_BLACK:
                if (cur_rgb.r <=50 && cur_rgb.g <=45 && cur_rgb.b <=60) {
                    _log("ODO=%05d, CL_BLACK detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_BLUE:
                if (cur_rgb.b - cur_rgb.r > 45 && cur_rgb.b <= 255 && cur_rgb.r <= 255) {
                    _log("ODO=%05d, CL_BLUE detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_RED:
                if (cur_rgb.r - cur_rgb.b >= 40 && cur_rgb.g < 60 && cur_rgb.r - cur_rgb.g > 30) {
                    _log("ODO=%05d, CL_RED detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_YELLOW:
                if (cur_rgb.r + cur_rgb.g - cur_rgb.b >= 130 &&  cur_rgb.r - cur_rgb.g <= 30) {
                    _log("ODO=%05d, CL_YELLOW detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_GREEN:
                if (cur_rgb.r <= 10 && cur_rgb.b <= 35 && cur_rgb.g > 43) {
                    _log("ODO=%05d, CL_GREEN detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_GRAY:
                if (cur_rgb.r <=80 && cur_rgb.g <=75 && cur_rgb.b <=105) {
                    _log("ODO=%05d, CL_GRAY detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            case CL_WHITE:
                if (cur_rgb.r >= 82 && cur_rgb.b >= 112 && cur_rgb.g >= 78) {
                    _log("ODO=%05d, CL_WHITE detected.", plotter->getDistance());
                    return Status::Success;
                }
                break;
            default:
                break;
        }
        return Status::Running;
    }
protected:
    Color color;
    bool updated;
};

/*
    usage:
    ".leaf<TraceLineCam>(speed, p, i, d, gs_min, gs_max, srew_rate, trace_side)"
    is to instruct the robot to trace the line in backward at the given speed.
    p, i, d are constants for PID control.
    gs_min, gs_max are grayscale threshold for line recognition binalization.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
    trace_side = TS_CENTER   when tracing the center of the line.
*/
class TraceLineCam : public BrainTree::Node {
public:
  TraceLineCam(int s, double p, double i, double d, int gs_min, int gs_max, double srew_rate, TraceSide trace_side) : speed(s),gsmin(gs_min),gsmax(gs_max),srewRate(srew_rate),side(trace_side) {
        updated = false;
        ltPid = new PIDcalculator(p, i, d, PERIOD_UPD_TSK, -speed, speed);
	/* initial region of interest */
	roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
	/* initial trace target */
	mx = (int)(FRAME_WIDTH/2);
	/* prepare and keep kernel for morphology */
	kernel = Mat::zeros(Size(7,7), CV_8UC1);
    }
    ~TraceLineCam() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Camera Trace run started.", plotter->getDistance());
            updated = true;
        }

	Mat img_orig, img_gray, img_bin, img_bin_mor, img_cnt_gray, scan_line;

	ER ercd = tloc_mtx(MTX1, 1000U); // test and lock the mutex
	if (ercd == E_OK) { // if successfully locked, process the frame and unlock the mutex; otherwise, return running
	  /* clone the image */
	  img_orig = frame.clone();
	  ercd = unl_mtx(MTX1);
	  assert(ercd == E_OK);
	} else {
	  _log("mutex lock failed with %d", ercd);
	  assert(ercd == E_TMOUT);
	  return Status::Running;
	}

	/* convert the image from BGR to grayscale */
	loc_cpu(); /* disable interrupts */
	cvtColor(img_orig, img_gray, COLOR_BGR2GRAY);
	unl_cpu(); /* enable interrupts */
	/* mask the upper half of the grayscale image */
	for (int i = 0; i < (int)(FRAME_HEIGHT/2); i++) {
	  for (int j = 0; j < FRAME_WIDTH; j++) {
	    img_gray.at<uchar>(i,j) = 255; /* type = CV_8U */
	  }
	}
	/* binarize the image */
	inRange(img_gray, gsmin, gsmax, img_bin);
	/* remove noise */
	morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);

	/* focus on the region of interest */
	Mat img_roi(img_bin_mor, roi);
	/* find contours in the roi with offset */
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	loc_cpu(); /* disable interrupts */
	findContours(img_roi, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(roi.x,roi.y));
	unl_cpu(); /* enable interrupts */
	/* identify the largest contour */
	if (contours.size() >= 1) {
	  int i_area_max = 0;
	  double area_max = 0.0;
	  for (int i = 0; i < contours.size(); i++) {
	    double area = contourArea(contours[i]);
	    if (area > area_max) {
	      area_max = area;
	      i_area_max = i;
	    }
	  }

	  /* calculate the bounding box around the largest contour
	     and set it as the new region of interest */ 
	  roi = boundingRect(contours[i_area_max]);
	  /* adjust the region of interest */
	  roi.x = roi.x - ROI_BOUNDARY;
	  roi.y = roi.y - ROI_BOUNDARY;
	  roi.width = roi.width + 2*ROI_BOUNDARY;
	  roi.height = roi.height + 2*ROI_BOUNDARY;
	  if (roi.x < 0) {
	    roi.x = 0;
	  }
	  if (roi.y < 0) {
	    roi.y = 0;
	  }
	  if (roi.x + roi.width > FRAME_WIDTH) {
	    roi.width = FRAME_WIDTH - roi.x;
	  }
	  if (roi.y + roi.height > FRAME_HEIGHT) {
	    roi.height = FRAME_HEIGHT - roi.y;
	  }
 
	  /* prepare for trace target calculation */
	  /*
	    Note: Mat::zeros with CV_8UC3 does NOT work and don't know why
	  */
	  Mat img_cnt(img_orig.size(), CV_8UC3, Scalar(0,0,0));
	  drawContours(img_cnt, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 1);
	  loc_cpu(); /* disable interrupts */
	  cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
	  unl_cpu(); /* enable interrupts */
	  /* scan the line really close to the image bottom to find edges */
	  scan_line = img_cnt_gray.row(img_cnt_gray.size().height -10);
	  /* convert the Mat to a NumCpp array */
	  auto scan_line_nc = nc::NdArray<nc::uint8>(scan_line.data, scan_line.rows, scan_line.cols);
	  auto edges = scan_line_nc.flatnonzero();
	  if (edges.size() >= 2) {
	    if (side != 2) {
	      mx = edges[side];
	    } else {
	      mx = (int)((edges[0]+edges[1]) / 2);
	    }
	  } else if (edges.size() == 1) {
	    mx = edges[0];
	  }
	}

	/* calculate variance of mx from the center in pixel */
	int vxp = mx - (int)(FRAME_WIDTH/2);
	/* convert the variance from pixel to milimeters
	   72 is length of the closest horizontal line on ground within the camera vision */
	float vxm = vxp * 72 / 640;
	/* calculate the rotation in degree (z-axis)
	   284 is distance from axle to the closest horizontal line on ground the camera can see */
	float theta = 180 * atan(vxm / 284) / M_PI;
	_log("mx = %d, vxm = %d, theta = %d", mx, (int)vxm, (int)theta);
	
        int8_t backward, turn, pwmL, pwmR;

        /* compute necessary amount of steering by PID control */
        turn = (-1) * _COURSE * ltPid->compute(theta, 0); /* 0 is the center */
        backward = -speed;
        /* steer EV3 by setting different speed to the motors */
        pwmL = backward - turn;
        pwmR = backward + turn;
        srlfL->setRate(srewRate);
        //leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        //rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int speed, gsmin, gsmax, mx;
    PIDcalculator* ltPid;
    double srewRate;
    TraceSide side;
    Rect roi;
    Mat kernel;
    bool updated;
};

/*
    usage:
    ".leaf<TraceLine>(speed, target, p, i, d, srew_rate, trace_side)"
    is to instruct the robot to trace the line at the given speed.
    p, i, d are constants for PID control.
    target is the brightness level for the ideal line for trace.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
    trace_side = TS_NORMAL   when in R(L) course and tracing the right(left) side of the line.
    trace_side = TS_OPPOSITE when in R(L) course and tracing the left(right) side of the line.
*/
class TraceLine : public BrainTree::Node {
public:
    TraceLine(int s, int t, double p, double i, double d, double srew_rate, TraceSide trace_side) : speed(s),target(t),srewRate(srew_rate),side(trace_side) {
        updated = false;
        ltPid = new PIDcalculator(p, i, d, PERIOD_UPD_TSK, -speed, speed);
    }
    ~TraceLine() {
        delete ltPid;
    }
    Status update() override {
        if (!updated) {
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Trace run started.", plotter->getDistance());
            updated = true;
        }

        int16_t sensor;
        int8_t forward, turn, pwmL, pwmR;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;
        /* compute necessary amount of steering by PID control */
        if (side == TS_NORMAL) {
            turn = (-1) * _COURSE * ltPid->compute(sensor, (int16_t)target);
        } else { /* side == TS_OPPOSITE */
            turn = _COURSE * ltPid->compute(sensor, (int16_t)target);
        }
        forward = speed;
        /* steer EV3 by setting different speed to the motors */
        pwmL = forward - turn;
        pwmR = forward + turn;
        srlfL->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int speed, target;
    PIDcalculator* ltPid;
    double srewRate;
    TraceSide side;
    bool updated;
};

/*
    usage:
    ".leaf<RunAsInstructed>(pwm_l, pwm_r, srew_rate)"
    is to move the robot at the instructed speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
*/
class RunAsInstructed : public BrainTree::Node {
public:
    RunAsInstructed(int pwm_l, int pwm_r, double srew_rate) : pwmL(pwm_l),pwmR(pwm_r),srewRate(srew_rate) {
        updated = false;
        if (_COURSE == -1) {
            int pwm = pwmL;
            pwmL = pwmR;
            pwmR = pwm;            
        }     
    }
    Status update() override {
        if (!updated) {
            /* The following code chunk is to properly set prevXin in SRLF */
            srlfL->setRate(0.0);
            leftMotor->setPWM(leftMotor->getPWM());
            srlfR->setRate(0.0);
            rightMotor->setPWM(rightMotor->getPWM());
            _log("ODO=%05d, Instructed run started.", plotter->getDistance());
            updated = true;
        }
        srlfL->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        srlfR->setRate(srewRate);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int pwmL, pwmR;
    double srewRate;
    bool updated;
};

/*
    usage:
    ".leaf<RotateEV3>(30, speed, srew_rate)"
    is to rotate robot 30 degrees (=clockwise) at the specified speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.
*/
class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int16_t degree, int s, double srew_rate) : deltaDegreeTarget(degree),speed(s),srewRate(srew_rate) {
        updated = false;
        assert(degree >= -180 && degree <= 180);
        if (degree > 0) {
            clockwise = 1;
        } else {
            clockwise = -1;
        }
    }
    Status update() override {
        if (!updated) {
            originalDegree = plotter->getDegree();
            srlfL->setRate(srewRate);
            srlfR->setRate(srewRate);
            /* stop the robot at start */
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            _log("ODO=%05d, Rotation started. Current degree = %d", plotter->getDistance(), originalDegree);
            updated = true;
            return Status::Running;
        }

        int16_t deltaDegree = plotter->getDegree() - originalDegree;
        if (deltaDegree > 180) {
            deltaDegree -= 360;
        } else if (deltaDegree < -180) {
            deltaDegree += 360;
        }
        if (clockwise * deltaDegree < clockwise * deltaDegreeTarget) {
            if ((srewRate != 0.0) && (clockwise * deltaDegree >= clockwise * deltaDegreeTarget - 5)) {
                /* when comes to the half-way, start decreazing the speed by tropezoidal motion */    
                leftMotor->setPWM(clockwise * 3);
                rightMotor->setPWM(-clockwise * 3);
            } else {
                leftMotor->setPWM(clockwise * speed);
                rightMotor->setPWM((-clockwise) * speed);
            }
            return Status::Running;
        } else {
            _log("ODO=%05d, Rotation ended. Current degree = %d", plotter->getDistance(), plotter->getDegree());
            return Status::Success;
        }
    }
private:
    int16_t deltaDegreeTarget, originalDegree;
    int clockwise, speed;
    bool updated;
    double srewRate;
};

/*
    usage:
    ".leaf<SetArmPosition>(target_degree, pwm)"
    is to shift the robot arm to the specified degree by the spefied power.
*/
class SetArmPosition : public BrainTree::Node {
public:
    SetArmPosition(int32_t target_degree, int pwm) : targetDegree(target_degree),pwmA(pwm) {
        updated = false;
    }
    Status update() override {
        int32_t currentDegree = armMotor->getCount();
        if (!updated) {
            _log("ODO=%05d, Arm position is moving from %d to %d.", plotter->getDistance(), currentDegree, targetDegree);
            if (currentDegree == targetDegree) {
                return Status::Success; /* do nothing */
            } else if (currentDegree < targetDegree) {
                clockwise = 1;
            } else {
                clockwise = -1;
            }
            armMotor->setPWM(clockwise * pwmA);
            updated = true;
            return Status::Running;
        }
        if (((clockwise ==  1) && (currentDegree >= targetDegree)) ||
            ((clockwise == -1) && (currentDegree <= targetDegree))) {
            armMotor->setPWM(0);
            _log("ODO=%05d, Arm position set to %d.", plotter->getDistance(), currentDegree);
            return Status::Success;
        } else {
            return Status::Running;
        }
    }
private:
    int32_t targetDegree;
    int pwmA, clockwise;
    bool updated;
};

/*
    === NODE CLASS DEFINITION ENDS HERE ===
*/


/* a cyclic handler to activate a task */
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);
    if (ercd != E_OK) {
        syslog(LOG_NOTICE, "act_tsk() returned %d", ercd);
    }
}

/* The main task */
void main_task(intptr_t unused) {
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    //bt = ev3_serial_open_file(EV3_SERIAL_BT);
    //assert(bt != NULL);
    cap = VideoCapture(0);
    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(CAP_PROP_FPS, 90);
    assert(cap.isOpened() == true);    
    /* create and initialize EV3 objects */
    ev3clock    = new Clock();
    touchSensor = new TouchSensor(PORT_1);
    // temp fix 2022/6/20 W.Taniguchi, new SonarSensor() blocks apparently
    //sonarSensor = new SonarSensor(PORT_3);
    colorSensor = new FilteredColorSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new FilteredMotor(PORT_C);
    rightMotor  = new FilteredMotor(PORT_B);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);
    /* read profile file and make the profile object ready */
    prof        = new Profile("msad2022_pri/profile.txt");
    /* determine the course L or R */
    if (prof->getValueAsStr("COURSE") == "R") {
      _COURSE = -1;
    } else {
      _COURSE = 1;
    }
 
    /* FIR parameters for a low-pass filter with normalized cut-off frequency of 0.2
        using a function of the Hamming Window */
    const int FIR_ORDER = 4; 
    const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };
    /* set filters to FilteredColorSensor */
    Filter *lpf_r = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_g = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_b = new FIR_Transposed(hn, FIR_ORDER);
    colorSensor->setRawColorFilters(lpf_r, lpf_g, lpf_b);

    leftMotor->reset();
    srlfL = new SRLF(0.0);
    leftMotor->setPWMFilter(srlfL);
    leftMotor->setPWM(0);
    rightMotor->reset();
    srlfR = new SRLF(0.0);
    rightMotor->setPWMFilter(srlfR);
    rightMotor->setPWM(0);
    armMotor->reset();

/*
    === BEHAVIOR TREE DEFINITION STARTS HERE ===
    A Behavior Tree serves as a blueprint for a LEGO object while a Node class serves as each Lego block used in the object.
*/

    /* robot starts when touch sensor is turned on */
    tr_calibration = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            // temp fix 2022/6/20 W.Taniguchi, as no touch sensor available on RasPike
            //.decorator<BrainTree::UntilSuccess>()
            //    .leaf<IsTouchOn>()
            //.end()
            .leaf<ResetClock>()
        .end()
        .build();

/*
    DEFINE ROBOT BEHAVIOR AFTER START
    FOR THE RIGHT AND LEFT COURSE SEPARATELY

    if (prof->getValueAsStr("COURSE") == "R") {
    } else {
    }
*/ 

    /* BEHAVIOR FOR THE RIGHT COURSE STARTS HERE */
    if (prof->getValueAsStr("COURSE") == "R") {
      tr_run = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::ParallelSequence>(1,2)
          .leaf<IsTimeEarned>(30000000) /* count 30 seconds */
          .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL)
        .end()
        .build();
    
      tr_block = nullptr;
    } else { /* BEHAVIOR FOR THE LEFT COURSE STARTS HERE */
      tr_run = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::ParallelSequence>(1,3)
          .leaf<IsBackOn>()
          .leaf<IsDistanceEarned>(1000)
          .composite<BrainTree::MemSequence>()
            .leaf<IsColorDetected>(CL_BLACK)
            .leaf<IsColorDetected>(CL_BLUE)
        .end()
        .leaf<TraceLineCam>(35, 0.1, 0.39, D_CONST, 0, 100, 0.0, TS_NORMAL)
        /* P=0.75, I=0.39, D=0.08 */
        .end()
        .build();

      tr_block = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
          .leaf<StopNow>()
          .leaf<IsTimeEarned>(3000000) // wait 3 seconds
          .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL)
          .leaf<StopNow>()
        .end()
        .build();
    } /* if (prof->getValueAsStr("COURSE") == "R") */

/*
    === BEHAVIOR TREE DEFINITION ENDS HERE ===
*/

    /* prepare a frame for the first frame */
    cap.read(frame);
    /* register cyclic handler to EV3RT */
    sta_cyc(CYC_UPD_TSK);

    /* indicate initialization completion by LED color */
    _log("initialization completed.");
    ev3_led_set_color(LED_ORANGE);
    state = ST_CALIBRATION;

    /* process video frames until the state gets changed */
    while (state != ST_END) {
      ER ercd = tloc_mtx(MTX1, 1000U); // test and lock the mutex
      if (ercd == E_OK) { // if successfully locked, read a frame and unlock the mutex; otherwise, do nothing
        cap.read(frame);
	ercd = unl_mtx(MTX1);
	assert(ercd == E_OK);
      } else {
	_log("mutex lock failed with %d", ercd);
	assert(ercd == E_TMOUT);
      }
      ev3clock->sleep(10000); // sleep 10 msec
    }

    /* the main task sleep until being waken up and let the registered cyclic handler to traverse the behavir trees */
    //_log("going to sleep...");
    //ER ercd = slp_tsk();
    //assert(ercd == E_OK);
    //if (ercd != E_OK) {
    //    syslog(LOG_NOTICE, "slp_tsk() returned %d", ercd);
    //}

    /* deregister cyclic handler from EV3RT */
    stp_cyc(CYC_UPD_TSK);
    /* destroy behavior tree */
    delete tr_block;
    delete tr_run;
    delete tr_calibration;
    /* destroy profile object */
    delete prof;
    /* destroy EV3 objects */
    delete lpf_b;
    delete lpf_g;
    delete lpf_r;
    delete plotter;
    delete armMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
    delete ev3clock;
    cap.release();
     _log("being terminated...");
    // temp fix 2022/6/20 W.Taniguchi, as Bluetooth not implemented yet
    //fclose(bt);
#if defined(MAKE_SIM)    
    ETRoboc_notifyCompletedToSimulator();
#endif
    ext_tsk();
}

/* periodic task to update the behavior tree */
void update_task(intptr_t unused) {
    BrainTree::Node::Status status;
    ER ercd;

    colorSensor->sense();
    plotter->plot();

/*
    === STATE MACHINE DEFINITION STARTS HERE ===
    The robot behavior is defined using HFSM (Hierarchical Finite State Machine) with two hierarchies as a whole where:
    - The upper layer is implemented as a state machine here.
    - The lower layer is implemented using Behavior Tree where each tree gets traversed within each corresponding state of the state machine.
*/
    switch (state) {
    case ST_CALIBRATION:
        if (tr_calibration != nullptr) {
            status = tr_calibration->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                switch (JUMP) { /* JUMP = 1... is for testing only */
                    case 1:
                        state = ST_BLOCK;
                        _log("State changed: ST_CALIBRATION to ST_BLOCK");
                        break;
                    default:
                        state = ST_RUN;
                        _log("State changed: ST_CALIBRATION to ST_RUN");
                        break;
                }
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_CALIBRATION to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_RUN:
        if (tr_run != nullptr) {
            status = tr_run->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_BLOCK;
                _log("State changed: ST_RUN to ST_BLOCK");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_RUN to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_BLOCK:
        if (tr_block != nullptr) {
            status = tr_block->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
            case BrainTree::Node::Status::Failure:
                state = ST_ENDING;
                _log("State changed: ST_BLOCK to ST_ENDING");
                break;
            default:
                break;
            }
        }
        break;
    case ST_ENDING:
        _log("waking up main...");
        /* wake up the main task */
        ercd = wup_tsk(MAIN_TASK);
        assert(ercd == E_OK);
        if (ercd != E_OK) {
            syslog(LOG_NOTICE, "wup_tsk() returned %d", ercd);
        }
        state = ST_END;
        _log("State changed: ST_ENDING to ST_END");
        break;    
    case ST_INITIAL:    /* do nothing */
    case ST_END:        /* do nothing */
    default:            /* do nothing */
        break;
    }
/*
    === STATE MACHINE DEFINITION ENDS HERE ===
*/

    rightMotor->drive();
    leftMotor->drive();

    //logger->outputLog(LOG_INTERVAL);
}
