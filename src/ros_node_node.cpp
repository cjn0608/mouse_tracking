#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros_node/kalman.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace std;
//using namespace cv;
const int winWidth = 800;
const int winHeight = 600;
cv::Point mousePosition = cv::Point(winWidth>>1, winHeight>>1);
//mouse call back
void mouseEvent(int event, int x, int y, int flags, void *param)
{
  if(event==CV_EVENT_MOUSEMOVE)
  {
    mousePosition=cv::Point(x,y);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ros_node_node");
  const int stateNum=4;
  const int measureNum=2;

//  double dt = 1.0/30; // Time step

  Eigen::MatrixXd A(stateNum, stateNum); // System dynamics matrix
  Eigen::MatrixXd C(measureNum, stateNum); // Output matrix
  Eigen::MatrixXd Q(stateNum, stateNum); // Process noise covariance
  Eigen::MatrixXd R(measureNum, measureNum); // Measurement noise covariance
  Eigen::MatrixXd P(stateNum, stateNum); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  C << 1, 0, 0, 0, 0, 1, 0, 0;

  // Reasonable covariance matrices
  Q << 0.05, 0, 0.05, 0, 0, 0.05, 0, 0.05, 0.05, 0, 0.05, 0, 0, 0.05, 0, 0.05;
 // Q << 0.0000000005, 0, 0.0000000005, 0, 0, 0.0000000005, 0, 0.0000000005, 0.0000000005, 0, 0.0000000005, 0, 0, 0.0000000005, 0, 0.0000000005;
  R << 0.1, 0, 0, 0.1;
 // R << 1000000, 0, 0, 1000000;
  P << 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0.1;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  kalmanfilter kf(0, A, C, Q, R, P);

  // Best guess of initial states
  Eigen::VectorXd x0(stateNum);
  x0 << 0, 0, 0, 0;
  kf.init(0,x0);
  cout << "x0: \n" << x0 << endl;
  Eigen::VectorXd y(measureNum);
  cv::Mat showImg(winWidth, winHeight,CV_8UC3);
  for(;;) {
    cv::setMouseCallback("Kalman", mouseEvent);
    showImg.setTo(0);
    y << (double)mousePosition.x, (double)mousePosition.y;
    kf.update(y);
    Eigen::VectorXd x_hat = kf.state();
    cv::Point predictPt = cv::Point(x_hat[0], x_hat[1]);
  //  circle(showImg, statePt, 5, CV_RGB(255,0,0),1);//former point
    circle(showImg, predictPt, 5, CV_RGB(0,255,0),1);//predict point
    circle(showImg, mousePosition, 5, CV_RGB(0,0,255),1);//ture point
    cv::putText(showImg, "Red: Former Point", cvPoint(10,30), cv::FONT_HERSHEY_SIMPLEX, 1 ,cv::Scalar :: all(255));
    cv::putText(showImg, "Green: Predict Point", cvPoint(10,60), cv::FONT_HERSHEY_SIMPLEX, 1 ,cv::Scalar :: all(255));
    cv::putText(showImg, "Blue: Ture Point", cvPoint(10,90), cv::FONT_HERSHEY_SIMPLEX, 1 ,cv::Scalar :: all(255));
    cv::imshow( "Kalman", showImg );
    int key = cv::waitKey(3);
    if (key == 27)
    {
      break;
    }
  }
  ros::spin();
  return 0;
}


















