#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>

#define TIME_STEP 64
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

using namespace cv;

int main() {
  wb_robot_init();

  // Initialize motors
  WbDeviceTag wheels[4];
  const char *wheel_names[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheel_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  // Initialize camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  while (wb_robot_step(TIME_STEP) != -1) {
    const unsigned char *image = wb_camera_get_image(camera);

    // Convert Webots image to OpenCV Mat
    Mat img(height, width, CV_8UC4, (void *)image);
    Mat bgr_img;
    cvtColor(img, bgr_img, COLOR_BGRA2BGR);

    // Convert to HSV
    Mat hsv;
    cvtColor(bgr_img, hsv, COLOR_BGR2HSV);

    // Threshold white color
    Scalar lower_white = Scalar(0, 0, 200);   // H:0, S:0, V:200
    Scalar upper_white = Scalar(180, 30, 255); // H:180, S:30, V:255
    Mat mask;
    inRange(hsv, lower_white, upper_white, mask);

    // Find contours
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double left_speed = 6.0;
    double right_speed = 6.0;

    if (!contours.empty()) {
      // Find largest white area
      double max_area = 0;
      std::vector<Point> largest;
      for (auto &c : contours) {
        double area = contourArea(c);
        if (area > max_area) {
          max_area = area;
          largest = c;
        }
      }

      if (!largest.empty()) {
        Moments M = moments(largest);
        if (M.m00 > 0) {
          int cx = (int)(M.m10 / M.m00);  

          if (cx < width / 3) {
            left_speed = -4;
            right_speed = 4;
          } else if (cx > 2 * width / 3) {
            left_speed = 4;
            right_speed = -4;
          } else {
            left_speed = 0;
            right_speed = 0;
            printf("White object detected and centered!\n");
          }
        }
      }
    }

    // Set wheel speeds
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
