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

#include <fstream>
#include <string>


using namespace std;
using namespace cv;


#define plf_L 0.8   //[m]
#define plf_lr 0.4 // [m]
#define Tf 0.4 //[m]



class Object {
private:

public:
    Point2d position;
    Point2d direction;
    double r;

    Object(Point2d _position, Point2d _direction, double _r);

    ~Object();

};



class DAPF {
private:
    int width, height;
    vector<Object*> objects_list;
    vector<pair<double, double>> L, R, dotted;

    Object* p;

    Mat road;
    Mat dangerous;

    double car_width;
    double car_height;

    vector<Point2d> route, route_car;
    Point2d position, direction;
    Point2d pre_point, now_point;
    Point2d pre_direction, now_direction;

    double potentialByObject(double get_x, double get_y);
    double potentialByLane(double get_x, double get_y);
    Point2d diffByObject(double get_x, double get_y);
    Point2d diffByLane(double get_x, double get_y);
    // 3차식의 계수와 원하는 y 좌표를 대입하면 steer 반환
    double getSteerwithCurve(vector<double> &ans, int y);

    // 차의 위치와 헤딩을 넣으면 차량 영역이 그려진 영상을 반환
    bool drawCar(Mat &input, Mat &output, Point2d &point_car, float heading, Mat &potential, Point2d &collisionPoint);
    // 점의 방정식을 곡선으로 근사
    void curveFitting(Mat &input, Mat &output, vector<Point2d> &route, vector<double> &ans, int start, int end);

    //input : 조향각, 속력, 이동하는 데 걸리는 시간
    //init_str [rad], V_r [m/s], dt [sec], heading [rad]
    //output : 절대좌표계(UTM좌표계)에서의 변위(x,y변화량)
    vector<double> PredictCar(double init_str, double V_r, double dt1, double heading);

    // 차량과 물체의 거리 차이 |d| 를 알려주는 함수 -> 분자
    double distance(Point2d car, Point2d object);
    // v벡터와 d벡터를 외적해주는 함수 -> 분모
    double inner_product(Point2d v_rel, Point2d d_rel);
    double TTC_master(Point2d car, Point2d object, Point2d car_heading, Point2d object_heading, double car_velocity, double object_velocity);


public:
    DAPF(int width, int height);

    ~DAPF();

    // 원하는 차선 영역을 지정한다
    void initLane(vector<pair<double, double>> _L, vector<pair<double, double>> _R, vector<pair<double, double>> _dotted);

    // 원하는 정적 또는 동적 물체를 지정한다
    void pushObject(Point2d _position, Point2d _direction, double _r);

    // 차선 및 물체가 있는 영역을 그려 반환한다
    void drawDangerous();

    // 퍼텐셜 필드 및 방향을 그린다
    void showDAPF(int time);

    void setPosition(Point2d position);
    void setDirection(Point2d direction);

    // path planning, 경로 추적하여 충돌 회피를 위한 다음 위치 반환
    void costTracker();




    void simulation(Point2d startPoint, Point2d startDirection);










};



















#endif //MOVINGLIDAR_MOVINGLIDAR_H
