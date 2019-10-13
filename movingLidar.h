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
#include <vector>


#include "preProcessing.h"


using namespace std;
using namespace cv;


#define plf_L 0.8   //[m]
#define plf_lr 0.4 // [m]
#define Tf 0.4 //[m]


void laneDetection(Mat &input, Mat &output);
void detection(Mat &input, Mat &output);
void createCostmap(Mat &input, Mat &output);
void costTracker(Mat &input, Mat &output, vector<Vec2i> &route, vector<float> &angles, int pre_x, int pre_y, int direction_x, int direction_y, Mat &frame_show);
void costTracker(Mat &dangerous, vector<Vec2i> &route, const Point2d& pre_point, const Point2d& pre_direction, Mat &frame_show,
                 const vector<double*>& object, vector<pair<double, double>> L, vector<pair<double, double>> R);
vector<double> PredictCar(double init_str, double V_r, double dt1, double heading);
void curveFitting(Mat &input, Mat &output, vector<Vec2i> &route, vector<double> &ans);
double getSteerwithCurve(vector<double> &ans, int y);
bool drawCar(Mat &input, Mat &output, Vec2i point_car, float heading, Mat &potential);


vector<double*> get_obj_info(vector<pair<double, double>> obj_move, double r);
double potentialByObject(vector<double*> object, double get_x, double get_y);
double potentialByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y);
pair<double, double> diffByObject(vector<double*> object, double get_x, double get_y);
pair<double, double> diffByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y);
void show_potentialField(Mat &dstImage, vector<double*> object, vector<pair<double, double>> L, vector<pair<double, double>> R);



void DynamicPotential( vector< vector<double> > &ObjCombine , vector < vector<double> >&object);
void DynamicPotential();


void getMovingFrame(Mat &frame, int time);
void simulation(Mat &input, Mat &output);



double distance(pair<double, double> car, pair<double, double > object);
double inner_product(pair<double, double> v_rel, pair<double, double> d_rel);
double TTC_master(pair<double, double> car, pair<double, double > object,
                  pair<double, double> car_heading, pair<double, double> object_heading, double car_velocity, double object_velocity);




#endif //MOVINGLIDAR_MOVINGLIDAR_H
