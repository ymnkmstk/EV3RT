/*
  how to compile:

    g++ testTraceCam03.cpp -std=c++14 `pkg-config --cflags --libs opencv4` -I .. -o testTraceCam03
*/

/* 
  NppCpp by David Pilger
  git clone https://github.com/dpilger26/NumCpp.git
  the use of NumCpp requires -std=c++14 -I . for compilation
*/
#include "NumCpp.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <cmath>

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define ROI_BOUNDARY 50

int gs_min=0,gs_max=100,edge=0;

int main() {
  /* prepare the camera */
  VideoCapture cap(0);
  cap.set(CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  cap.set(CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  cap.set(CAP_PROP_FPS,90);

  if (!cap.isOpened()) {
    cout << "cap is not open" << endl;
  }

  /* create trackbars */
  namedWindow("testTrace1");
  createTrackbar("GS_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_min", "testTrace1", gs_min);
  createTrackbar("GS_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("GS_max", "testTrace1", gs_max);
  createTrackbar("Edge",   "testTrace1", nullptr, 2, nullptr);
  setTrackbarPos("Edge",   "testTrace1", edge);

  /* initial region of interest */
  Rect roi(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
  /* initial trace target */
  int mx = (int)(FRAME_WIDTH/2);
  
  while (true) {
    /* obtain values from the trackbars */
    gs_min = getTrackbarPos("GS_min", "testTrace1");
    gs_max = getTrackbarPos("GS_max", "testTrace1");
    edge   = getTrackbarPos("Edge",   "testTrace1");

    Mat img, img_orig, img_gray, img_bin, img_bin_mor, img_comm;
    int c;

    sleep_for(chrono::milliseconds(10));

    cap.read(img);

    /* clone the image (not necessary in this sample but necessary in Toppers) */
    img_orig = img.clone();
    /* convert the image from BGR to grayscale */
    cvtColor(img_orig, img_gray, COLOR_BGR2GRAY);
    /* mask the upper half of the grayscale image */
    for (int i = 0; i < (int)(FRAME_HEIGHT/2); i++) {
	for (int j = 0; j < FRAME_WIDTH; j++) {
	  img_gray.at<uchar>(i,j) = 255; /* type = CV_8U */
	}
    }
    /* binarize the image */
    inRange(img_gray, gs_min, gs_max, img_bin);
    /* remove noise */
    Mat kernel = Mat::zeros(Size(7,7), CV_8UC1);
    morphologyEx(img_bin, img_bin_mor, MORPH_CLOSE, kernel);

    /* focus on the region of interest */
    Mat img_roi(img_bin_mor, roi);
    /* find contours in the roi with offset */
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_roi, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(roi.x,roi.y));
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
      /* draw the largest contour on the original image */
      drawContours(img_orig, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 10);

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
      Mat img_cnt = Mat::zeros(img_orig.size(), CV_8UC3);
      drawContours(img_cnt, (vector<vector<Point>>){contours[i_area_max]}, 0, Scalar(0,255,0), 1);
      Mat img_cnt_gray;
      cvtColor(img_cnt, img_cnt_gray, COLOR_BGR2GRAY);
      /* scan the line really close to the image bottom to find edges */
      Mat scan_line = img_cnt_gray.row(img_cnt_gray.size().height -10);
      /* convert the Mat to a NumCpp array */
      auto scan_line_nc = nc::NdArray<nc::uint8>(scan_line.data, scan_line.rows, scan_line.cols);
      auto edges = scan_line_nc.flatnonzero();
      if (edges.size() >= 2) {
	if (edge == 0) {
	  mx = edges[0];
	} else if (edge == 1) {
	  mx = edges[edges.size()-1];
	} else {
	  mx = (int)((edges[0]+edges[edges.size()-1]) / 2);
	}
      } else if (edges.size() == 1) {
	mx = edges[0];
      }
    } else { /* contours.size() == 0 */
      roi = Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);
    }

    /* draw the area of interest on the original image */
    rectangle(img_orig, Point(roi.x,roi.y), Point(roi.x+roi.width,roi.y+roi.height), Scalar(255,0,0), 10);
    /* draw the trace target on the image */
    circle(img_orig, Point(mx, FRAME_HEIGHT-10), 20, Scalar(0,0,255), -1);
    /* calculate variance of mx from the center in pixel */
    int vxp = mx - (int)(FRAME_WIDTH/2);
    /* convert the variance from pixel to milimeters
       72 is length of the closest horizontal line on ground within the camera vision */
    float vxm = vxp * 72 / 640;
    /* calculate the rotation in radians (z-axis)
       284 is distance from axle to the closest horizontal line on ground the camera can see */
    float theta = atan(vxm / 284);
    cout << "mx = " << mx << ", vxm = " << vxm << ", theta = " << theta << endl;

    /* shrink the concatinated image to avoid delay in transmission */
    resize(img_orig, img_comm, Size(), 0.25, 0.25);
    /* transmit and display the image */
    imshow("testTrace2", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  destroyAllWindows();
  return 0;
}
