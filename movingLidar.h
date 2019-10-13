//
// Created by 박규열 on 01/10/2019.
//

#ifndef MOVINGLIDAR_MOVINGLIDAR_H
#define MOVINGLIDAR_MOVINGLIDAR_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;




#define plf_L 0.8   //[m]
#define plf_lr 0.4 // [m]
#define Tf 0.4 //[m]




void imageshow(const string& name, Mat &original);
void binarization(Mat &input, Mat &output);
void drawLines(Mat &input, Mat &output, const vector<Point2i>& line_points, const Scalar& color = Scalar(0, 0, 255));
void ROIcut(Mat &input, Mat &output);
void topView(Mat &input, Mat &output);
void findLines(Mat &image, vector<Vec4i> &lines, double gap);
void searchLane(const Mat& frame, vector<Vec2i> &line_points, int pivot_x, int pivot_y, int gap);
void searchLanes(Mat &input, Mat &output, vector<Vec2i> &line_points_left, vector<Vec2i> &line_points_right);
void findBoundary(Mat &input, Mat &output);

void laneDetection(Mat &input, Mat &output);
void costTracker(Mat &input, Mat &output, vector<Vec2i> &route, vector<float> &angles, int pre_x, int pre_y, int direction_x, int direction_y, Mat &frame_show);
void createCostmap(Mat &input, Mat &output);

//void pathPlanning(Mat &input, Mat &output, vector<Point2i> &route, vector<float> &angle, Point2i pre_position, Point2f direction);
//void planningIntegralPath(Mat &input, Mat &output);

vector<double> PredictCar(double init_str, double V_r, double dt1, double heading);
void curveFitting(Mat &input, Mat &output, vector<Vec2i> &route, vector<double> &ans);

void curveFitting(Mat &input, Mat &output, vector<Vec2i> &route, vector<double> &ans);
double getSteerwithCurve(vector<double> &ans, int y);

void detection(Mat &input, Mat &output);

bool drawCar(Mat &input, Mat &output, Vec2i point_car, float heading, Mat &potential);




double potentialByObject(vector<double*> object, double get_x, double get_y);
double potentialByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y);
vector<double*> get_obj_info(vector<pair<double, double>> obj_move, double r);
pair<double, double> diffByObject(vector<double*> object, int now_frame, double get_x, double get_y);
pair<double, double> diffByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y);
void show_potentialField(Mat &dstImage, vector<double*> object, vector<pair<double, double>> L, vector<pair<double, double>> R);


void DynamicPotential( vector< vector<double> > &ObjCombine , vector < vector<double> >&object);


void getMovingFrame(Mat &frame, int time);



double distance(pair<double, double> car, pair<double, double > object);
double inner_product(pair<double, double> v_rel, pair<double, double> d_rel);
double TTC_master(pair<double, double> car, pair<double, double > object, pair<double, double> car_heading, pair<double, double> object_heading, double car_velocity, double object_velocity);




#endif //MOVINGLIDAR_MOVINGLIDAR_H
