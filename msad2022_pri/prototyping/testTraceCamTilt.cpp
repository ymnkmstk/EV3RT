#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

int hmin=0,hmax=179,smin=0,smax=255,vmin=130,vmax=255,edge=0;

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
  createTrackbar("H_min", "testTrace1", nullptr, 179, nullptr);
  setTrackbarPos("H_min", "testTrace1", hmin);
  createTrackbar("H_max", "testTrace1", nullptr, 179, nullptr);
  setTrackbarPos("H_max", "testTrace1", hmax);
  createTrackbar("S_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("S_min", "testTrace1", smin);
  createTrackbar("S_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("S_max", "testTrace1", smax);
  createTrackbar("V_min", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("V_min", "testTrace1", vmin);
  createTrackbar("V_max", "testTrace1", nullptr, 255, nullptr);
  setTrackbarPos("V_max", "testTrace1", vmax);
  createTrackbar("Edge",  "testTrace1", nullptr, 1, nullptr);
  setTrackbarPos("Edge",  "testTrace1", edge);

  while (true) {
    /* obtain values from the trackbars */
    hmin = getTrackbarPos("H_min", "testTrace1");
    hmax = getTrackbarPos("H_max", "testTrace1");
    smin = getTrackbarPos("S_min", "testTrace1");
    smax = getTrackbarPos("S_max", "testTrace1");
    vmin = getTrackbarPos("V_min", "testTrace1");
    vmax = getTrackbarPos("V_max", "testTrace1");
    edge = getTrackbarPos("Edge",  "testTrace1");

    Mat img, img_med, img_hsv, img_bin, img_inv, img_tgt, img_edge, img_comm;
    int dx = FRAME_WIDTH/2, c;

    sleep_for(chrono::milliseconds(10));

    cap.read(img);
    /* crop the nearest/bottom part of the image */
    Mat img_bgr(img, Rect(0,FRAME_HEIGHT/2,FRAME_WIDTH,FRAME_HEIGHT/2));
    /* reduce the noise */
    medianBlur(img_bgr, img_med, 5);
    /* convert the image from BGR to HSV */
    cvtColor(img_med, img_hsv, COLOR_BGR2HSV);
    /* binarize the image */
    inRange(img_hsv, Scalar(hmin,smin,vmin), Scalar(hmax,smax,vmax), img_bin);
    /* inverse the image */
    bitwise_not(img_bin, img_inv);
    /* mask the image */
    bitwise_and(img_hsv, img_hsv, img_tgt, img_inv);
    /* label connected components in the image */
    Mat img_labeled, stats, centroids;
    int num_labels = connectedComponentsWithStats(img_inv, img_labeled, stats, centroids);

    /* ignore the largest black part (the first label) */
    Mat stats_rest, centroids_rest;
    for (int i = 1; i < num_labels; i++) {
      stats_rest.push_back(stats.row(i));
      centroids_rest.push_back(centroids.row(i));
    }
    num_labels--;

    /* prepare empty matrices */
    Mat img_lbl = Mat::zeros(img_bin.size(), CV_8UC1);
    Mat img_edge_cvt = Mat::zeros(img_bin.size(), CV_8UC3);
    
    if (num_labels >= 1) {
      /* obtain the index to the label which has the largest area,
	 corresponding to
	 index = np.argmax(stats[:,4]) in Python */
      int index, area_largest = 0;
      for (int i = 0; i < num_labels; i++) {
        if (stats_rest.at<int>(i,4) > area_largest) {
          index = i;
          area_largest = stats_rest.at<int>(i,4);
        }
      }
      /* obtain the attributes of the label */
      int x = stats_rest.at<int>(index,0);
      int y = stats_rest.at<int>(index,1);
      int w = stats_rest.at<int>(index,2);
      int h = stats_rest.at<int>(index,3);
      int s = stats_rest.at<int>(index,4);
      int mx = centroids_rest.at<double>(index,0);
      int my = centroids_rest.at<double>(index,1);
      /* draw a circle at the centroid on the target image */
      circle(img_tgt, Point(mx, my), 20, Scalar(179,0,255), -1);
      /* copy the largest component in the empty matrix */
      for (int i = 0; i < img_labeled.size().height; i++) {
	for (int j = 0; j < img_labeled.size().width; j++) {
	  if (img_labeled.at<int>(i,j) == index+1) { /* type = CV_32S */
	    img_lbl.at<uchar>(i,j) = 255; /* type = CV_8U */
	  }
	}
      }
      /* detect edges */
      Canny(img_lbl, img_edge, 255, 255);
      /* convert the inverted binary image into a regular RGB image
	 from CV_8U to CV_8UC3 */
      int from_to[] = {0,0, 0,1, 0,2};
      mixChannels(&img_edge, 1, &img_edge_cvt, 1, from_to, 3);
      /* find contours */
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours(img_edge, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
      if (contours.size() >= 2) {
	/* derive an approximated line for either edge by regression */
	Vec4f lin; /* vx, vy, x0, y0 */
	fitLine(contours[edge], lin, DIST_L2, 0, 0.01, 0.01);
	float m = FRAME_WIDTH;
	Point p1,p2;
	p1.x = (int)(lin[2] - m*lin[0]);
	p1.y = (int)(lin[3] - m*lin[1]);
	p2.x = (int)(lin[2] + m*lin[0]);
	p2.y = (int)(lin[3] + m*lin[1]);
	clipLine(Rect(0,0,img_tgt.size().width,img_tgt.size().height), p1, p2);
	/* draw the line on the target image */
	line(img_tgt, p1, p2, Scalar(179,0,255), 10, LINE_4);
	/* adjust the centroid by the line tilt */
	if (lin[1] != 0) {
	  mx = mx - p1.x + p2.x;
	}
	/* indicate the adjusted centroid by the vertical line */
	line(img_tgt, Point(mx,0), Point(mx,FRAME_HEIGHT/2), Scalar(179,0,255), 10, LINE_4);
	dx = (FRAME_WIDTH/2) - mx;
	cout << "dx = " << dx << endl;
      }
    }

    /* concatinate the images - original + edge + target */
    Mat img_v, img_vv;
    vconcat(img, img_edge_cvt, img_v);
    vconcat(img_v, img_tgt, img_vv);
    /* shrink the concatinated image to avoid delay in transmission */
    resize(img_vv, img_comm, Size(), 0.25, 0.25);
    /* transmit and display the image */
    imshow("testTrace2", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  destroyAllWindows();
  return 0;
}
