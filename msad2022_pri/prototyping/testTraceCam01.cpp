#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;
using namespace cv;
using std::this_thread::sleep_for;

int hmin=0,hmax=179,smin=0,smax=255,vmin=130,vmax=255;

int main() {
  VideoCapture cap(0);
  cap.set(CAP_PROP_FRAME_WIDTH,640);
  cap.set(CAP_PROP_FRAME_HEIGHT,480);
  cap.set(CAP_PROP_FPS,90);

  if (!cap.isOpened()) {
    cout << "cap is not open" << endl;
  }

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

  while (true) {
    hmin = getTrackbarPos("H_min", "testTrace1");
    hmax = getTrackbarPos("H_max", "testTrace1");
    smin = getTrackbarPos("S_min", "testTrace1");
    smax = getTrackbarPos("S_max", "testTrace1");
    vmin = getTrackbarPos("V_min", "testTrace1");
    vmax = getTrackbarPos("V_max", "testTrace1");

    Mat img, img_hsv, img_bin, img_inv, img_tgt, img_comm;
    int dx = 320, c;

    sleep_for(chrono::milliseconds(10));

    cap.read(img);
    // crop the nearest/bottom part of the image
    Mat img_bgr(img, Rect(0,400,640,80));
    // convert the image from BGR to HSV
    cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);
    // binarize the image
    inRange(img_hsv, Scalar(hmin,smin,vmin), Scalar(hmax,smax,vmax), img_bin);
    // inverse the image
    bitwise_not(img_bin, img_inv);
    Mat img_inv_cvt(img_inv.size(), CV_8UC3);
    // mask the image
    bitwise_and(img_hsv, img_hsv, img_tgt, img_inv);
    // label connected components in the image
    Mat img_labeled, stats, centroids;
    int num_labels = connectedComponentsWithStats(img_inv, img_labeled, stats, centroids);
    // ignore the largest black part (the first label)
    Mat stats_rest, centroids_rest;
    for (int i = 1; i < num_labels; i++) {
      stats_rest.push_back(stats.row(i));
      centroids_rest.push_back(centroids.row(i));
    }
    num_labels--;

    if (num_labels >= 1) {
      // index = np.argmax(stats[:,4]) in Python
      int index, area_largest = 0;
      for (int i = 0; i < num_labels; i++) {
        if (stats_rest.at<int>(i,4) > area_largest) {
          index = i;
          area_largest = stats_rest.at<int>(i,4);
        }
      }
      // obtain the attributes of the label
      int x = stats_rest.at<int>(index,0);
      int y = stats_rest.at<int>(index,1);
      int w = stats_rest.at<int>(index,2);
      int h = stats_rest.at<int>(index,3);
      int s = stats_rest.at<int>(index,4);
      int mx = centroids_rest.at<double>(index,0);
      int my = centroids_rest.at<double>(index,1);
      dx = 320 - mx;
      // draw a boundary box around the label
      rectangle(img_tgt, Rect(x,y,w,h), Scalar(255,0,255), 2);

      // convert the inverted binary image into a regular RGB image
      // from CV_8U to CV_8UC3
      int from_to[] = {0,0, 0,1, 0,2};
      mixChannels(&img_inv, 1, &img_inv_cvt, 1, from_to, 3); 
    }

    cout << "dx = " << dx << endl;
    // concatinate images
    Mat img_v, img_vv;
    vconcat(img, img_inv_cvt, img_v);
    vconcat(img_v, img_tgt, img_vv);
    resize(img_vv, img_comm, Size(), 0.25, 0.25);
    imshow("testTrace2", img_comm);

    c = waitKey(1);
    if ( c == 'q' || c == 'Q' ) break;
  }

  destroyAllWindows();
  return 0;
}
