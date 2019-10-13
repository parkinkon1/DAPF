//
// Created by 박규열 on 14/10/2019.
//

#ifndef MOVINGLIDAR_PREPROCESSING_H
#define MOVINGLIDAR_PREPROCESSING_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;


#include "movingLidar.h"


/////////////////////////////////////////////////////////////
//////////////////// Pre-Processing /////////////////////////
/////////////////////////////////////////////////////////////

// 이미지를 보여주는 함수
void imageshow(const string& name, Mat &original) {
    Mat image_show;
    int key;
//    resize(original, image_show, Size(480, 270));
    imshow(name, original);
    key = waitKey(1);
    if (key == 27) destroyAllWindows();
}


// 컬러 이미지에 대해 Adaptive 영상 이진화
void binarization(Mat &input, Mat &output){
    int blocksize = (int)(input.rows/5) + ((int)(input.rows/5) % 2) + 1;
    cvtColor(input, output, COLOR_BGR2GRAY);
    medianBlur(output, output, 5);
    adaptiveThreshold(output, output, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blocksize, 10);
//    erode(output, output, getStructuringElement(MORPH_ELLIPSE, Size(3, 5)));
//    dilate(output, output, getStructuringElement(MORPH_ELLIPSE, Size(3, 5)));
}


// 연속된 점들을 선으로 이어 드로잉
void drawLines(Mat &input, Mat &output, const vector<Vec2i>& line_points) {
    cvtColor(input, output, COLOR_GRAY2BGR);
    for (int i = 0; i < (int)line_points.size() - 1; i++) {
        line(output, line_points[i], line_points[i+1], Scalar(0, 0, 255), 4);
    }
}


// 관심 영역 추출
void ROIcut(Mat &input, Mat &output) {
    // 입력 영상의 가로, 세로 길이
    int width = input.cols;
    int height = input.rows;
    cv::Rect myROI(0, (int)((float)height*0.6), width, (int)((float)height*0.4)); // (x,y,w,h)
    // ROI 영역으로 자르기
    output = input(myROI);
}


// top view 변환
void topView(Mat &input, Mat &output) {
    int width = input.cols;
    int height = input.rows;
    // warping 비율
    float width_ratio_top = 0.35; // 이미지 상단에서 자를 부분의 비율
    float width_ratio_bot = 1.8; // 이미지 하단에서 자를 부분의 비율
    float height_ratio = 1.4; // 밑변과 높이 간의 비율
    // warping 크기와 warping할 프레임 정의
    Size warpSize(width, (int)((float)width*height_ratio));
    Mat warpframe(warpSize, input.type());

    // Warping 전의 이미지 상의 좌표
    vector<Point2f> corners(4);
    corners[0] = Point2f((float)width*(0.5 - width_ratio_top/2), 0); // left top
    corners[1] = Point2f((float)width*(0.5 + width_ratio_top/2), 0); // right top
    corners[2] = Point2f((float)width*(0.5 - width_ratio_bot/2), height); // left bot
    corners[3] = Point2f((float)width*(0.5 + width_ratio_bot/2), height); // right bot
    // Warping 후의 좌표
    vector<Point2f> warpCorners(4);
    warpCorners[0] = Point2f(0, 0);
    warpCorners[1] = Point2f(warpframe.cols, 0);
    warpCorners[2] = Point2f(0, warpframe.rows);
    warpCorners[3] = Point2f(warpframe.cols, warpframe.rows);

    // Transformation Matrix 구하기
    Mat trans = getPerspectiveTransform(corners, warpCorners);
    // Warping
    warpPerspective(input, warpframe, trans, warpSize);
    output = warpframe;
}





/////////////////////////////////////////////////////////////
//////////////////// Lane Detection /////////////////////////
/////////////////////////////////////////////////////////////

// 영상에서 선분을 추출하여 시작점과 끝점을 lines에 저장
void findLines(Mat &image, vector<Vec4i> &lines, double gap) {
    // Hough Transform 파라미터
    float rho = 1; // distance resolution in pixels of the Hough grid
    float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
    int hough_threshold = (int)gap/2;    // minimum number of votes(intersections in Hough grid cell)
    double minLineLength = gap/2; //minimum number of pixels making up a line
    double maxLineGap = gap;   //maximum gap in pixels between connectable line segments
    // 직선 검출 (Hough Transform)
    HoughLinesP(image, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);
}


// 주어진 영상에서 gap에 대해 sliding window 방식으로 곡선 차선 검출
void searchLane(const Mat& frame, vector<Vec2i> &line_points, int pivot_x, int pivot_y, int gap) {
    // 검출된 직선들을 저장할 벡터
    vector<Vec4i> lines;
    int width = frame.cols;
    int height = frame.rows;
    // 검색할 상위 영역 프레임 추출
    int x_gap = width/8; // 위로 슬라이딩하여 검색할 영역의 좌우 크기
    int x_min = (pivot_x - x_gap < 0) ? 0 : pivot_x - x_gap;
    int x_max = (pivot_x + x_gap > width) ?  width : pivot_x + x_gap;
    int y_min = pivot_y - gap;
    int y_max = pivot_y;

    int x_offset = 3, y_offset = 3;

    // 프레임 끝까지 탐색되면 종료
    if (y_min < 0) return;
    // 검색할 프레임 자르기
    Mat search_frame = frame(Range(y_min, y_max), Range(x_min, x_max));
    // 추출한 프레임에 대해 선분 검색
    findLines(search_frame, lines, gap);

    // 선분 검출이 되면, 해당 선분의 상단점을 차선 위 점으로 추가
    int x1, y1, x2, y2, x_temp, y_temp;
    if (!lines.empty()) {
        // 인식된 점들 중 피벗과 이어진 점 추출
        for (auto & line : lines) {
            // 검출된 선분의 시작점과 끝점 좌표
            x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            // 점 두 개의 순서가 바뀌었으면 서로 교환
            if (y1 < y2) {
                x_temp = x2; y_temp = y2;
                x2 = x1; y2 = y1;
                x1 = x_temp; y1 = y_temp;
            }
            // 피벗과 가까운 선 추출
            if (abs(pivot_x - (x_min + x1)) <= x_offset && abs(pivot_y - (y_min + y1)) <= y_offset) {
                line_points.push_back(Point2i(x_min + x1, y_min + y1));
                line_points.push_back(Point2i(x_min + x2, y_min + y2));
                searchLane(frame, line_points, x_min + x2, y_min, gap);
                break;
            }
        } return; // 피벗과 이어진 선분이 없으면 종료
    } else return; // 검출된 선분이 없으면 종료
}


// 왼쪽, 오른쪽 차선 검출
void searchLanes(Mat &input, Mat &output, vector<Vec2i> &line_points_left, vector<Vec2i> &line_points_right) {
    // 인풋 이미지 크기
    int input_width = input.cols;
    int input_height = input.rows;
    // 이미지 처리를 위한 리사이징 크기
    int width = input_width/4;
    int height = input_height/4;
    // 이미지 리사이징
    Mat frame;
    resize(input, frame, Size(width, height));

    // 곡선 차선 검출해서 line_points 에 저장
    int slide_num = 6; // 최대 슬라이딩 횟수
    int gap = (int)((float)height/(float)slide_num); // 각 슬라이드의 높이
    int min_length = (int)((double)height/2);
    vector<Vec2i> dotted_line; // 직선 위 점들을 저장할 벡터
    vector<Vec4i> lines;
    vector< vector<Vec2i> > linePoints;
    Mat pivot_frame;
    int x1, y1, x2, y2, x_temp, y_temp, dotted_pivot_x, dotted_pivot_y, line_length;
    // 최대 영상 중앙까지 피벗이 될 차선의 시작점을 검출하고, 위로 차선을 검색하여 가장 긴 차선 검출
    // 피벗 차선 검출이 안될 경우 반복
    //// ------------- 왼쪽 차선 ------------------
    for (int slide = 0; slide < (int)(slide_num / 2); slide++) {
        // 초기화
        line_points_left.clear(); dotted_line.clear();
        // 해당 줄의 피벗 영상 추출
        pivot_frame = frame(Range(height - gap - gap*slide, height - gap*slide), Range(0, (int)((float)width*0.5)));
        // 피벗 영상에 대해 차선의 시작점 검출
        findLines(pivot_frame, lines, gap);
        if (lines.empty()) continue; // 검출 안되면 위로 슬라이딩하여 다시 검출
        //// 각 시작점에 대해 차선 검출, 일정 길이 이상의 차선을 검출할 때까지 반복
        for (auto &line : lines) {
            // 검출된 선분의 시작점, 끝점
            x1 = line[0]; y1 = line[1]; x2 = line[2]; y2 = line[3];
            // 점 순서가 바뀌었을 경우 조정
            if (y1 < y2) { x_temp = x2; y_temp = y2; x2 = x1; y2 = y1; x1 = x_temp; y1 = y_temp; }
            // 시작점과 끝점을 벡터에 삽입
            line_points_left.push_back(Point2i(x1, height - gap + y1 - gap*slide));
            line_points_left.push_back(Point2i(x2, height - gap + y2 - gap*slide));
            // 위 방향으로 차선 검출
            searchLane(frame, line_points_left, x2, height - gap + y2 - gap*slide, gap);
            if (line_points_left.size() == 2) continue; // 검출 안되면 다른 시작점에 대해 다시 실행
            // 검출된 선의 길이
            line_length = line_points_left[0][1] - line_points_left.back()[1];

            //// 충분히 긴 직선이 검출되면 해당 선을 추종 차선으로 선택, 검색 종료
            if (line_length >= min_length) break;
                //// 실패한 경우 dotted_line 길이 업데이트, 다른 시작점에 대해 검사
            else {
                // dotted_line이 비어있을 경우 추가
                if (dotted_line.empty()) {
                    dotted_line.insert( dotted_line.end(), line_points_left.begin(), line_points_left.end() );
                }
                    // dotted_line이 비어있지 않을 경우 더 긴 차선으로 업데이트
                else {
                    if (line_length > (dotted_line[0][1] - line_points_left.back()[1])) {
                        dotted_line.clear();
                        dotted_line.insert( dotted_line.end(), line_points_left.begin(), line_points_left.end() );
                    }
                }
                // 다른 시작점에 대해 검사
                line_points_left.clear();
                continue;
            }
        }

        //// 실선 차선을 발견한 경우 검사 종료
        if (!line_points_left.empty()) {

            break;
        }
            //// 실선 차선 발견 못한 경우, 점선 차선 사용
        else if ((dotted_line[0][1] - dotted_line.back()[1]) >= height/5) {

        }
        else continue; //// 점선 차선도 발견 못한 경우 위로 피벗 이동하여 재시작
    }

    //// ------------- 오른쪽 차선 ------------------
    for (int slide = 0; slide < (int)(slide_num / 2); slide++) {
        // 초기화
        line_points_right.clear(); dotted_line.clear();
        // 해당 줄의 피벗 영상 추출
        pivot_frame = frame(Range(height - gap - gap*slide, height - gap*slide), Range((int)((float)width*0.5), width));
        // 피벗 영상에 대해 차선의 시작점 검출
        findLines(pivot_frame, lines, gap);
        if (lines.empty()) continue; // 검출 안되면 위로 슬라이딩하여 다시 검출
        //// 각 시작점에 대해 차선 검출, 일정 길이 이상의 차선을 검출할 때까지 반복
        for (auto &line : lines) {
            // 검출된 선분의 시작점, 끝점
            x1 = line[0] + (int)((float)width*0.5); y1 = line[1]; x2 = line[2] + (int)((float)width*0.5); y2 = line[3];
            // 점 순서가 바뀌었을 경우 조정
            if (y1 < y2) { x_temp = x2; y_temp = y2; x2 = x1; y2 = y1; x1 = x_temp; y1 = y_temp; }
            // 시작점과 끝점을 벡터에 삽입
            line_points_right.push_back(Point2i(x1, height - gap + y1 - gap*slide));
            line_points_right.push_back(Point2i(x2, height - gap + y2 - gap*slide));
            // 위 방향으로 차선 검출
            searchLane(frame, line_points_right, x2, height - gap + y2 - gap*slide, gap);
            if (line_points_right.size() == 2) continue; // 검출 안되면 다른 시작점에 대해 다시 실행
            // 검출된 선의 길이
            line_length = line_points_right[0][1] - line_points_right.back()[1];

            //// 충분히 긴 직선이 검출되면 해당 선을 추종 차선으로 선택, 검색 종료
            if (line_length >= min_length) break;
                //// 실패한 경우 dotted_line 길이 업데이트, 다른 시작점에 대해 검사
            else {
                // dotted_line이 비어있을 경우 추가
                if (dotted_line.empty()) {
                    dotted_line.insert( dotted_line.end(), line_points_right.begin(), line_points_right.end() );
                }
                    // dotted_line이 비어있지 않을 경우 더 긴 차선으로 업데이트
                else {
                    if (line_length > (dotted_line[0][1] - line_points_right.back()[1])) {
                        dotted_line.clear();
                        dotted_line.insert( dotted_line.end(), line_points_right.begin(), line_points_right.end() );
                    }
                }
                // 다른 시작점에 대해 검사
                line_points_right.clear();
                continue;
            }
        }

        //// 실선 차선을 발견한 경우 검사 종료
        if (!line_points_right.empty()) {
            break;
        }
            //// 실선 차선 발견 못한 경우, 점선 차선 사용
        else if ((dotted_line[0][1] - dotted_line.back()[1]) >= height/5) {
        }
        else continue; //// 점선 차선도 발견 못한 경우 위로 피벗 이동하여 재시작
    }

    // 이미지에 인식된 차선 그리기
    cvtColor(frame, frame, COLOR_GRAY2BGR);
    // 차선이 검출된 경우
    if (!line_points_left.empty() || !line_points_right.empty()) {
        line_points_left.push_back(Point2i(0, 0));
        line_points_left.push_back(Point2i(0, height));
        line_points_right.push_back(Point2i(width, 0));
        line_points_right.push_back(Point2i(width, height));

        // 양쪽 차선을 저장하여 차선 영역 폴리곤을 그릴 벡터
        linePoints.push_back(line_points_left);
        linePoints.push_back(line_points_right);
        // 폴리곤 그리기
        fillPoly(frame, linePoints, Scalar(255, 0, 255), LINE_AA);
    }
    else {
        cout << "Lane not detected ㅜㅜ" << endl;
    }

    output = Mat::zeros(height, width, CV_8UC1);
    fillPoly(output, linePoints, Scalar(255), LINE_AA);
//    output = ~output;


}


// 흑백 이진 영상에서 오브젝트 경계를 검출
void findBoundary(Mat &input, Mat &output) {
    vector<vector<Point>> contours;
    findContours(input, contours, RETR_LIST, CHAIN_APPROX_NONE);
    output = Mat::zeros(input.rows, input.cols, CV_8UC1);
    drawContours(output, contours, -1, 255, 5);
}









#endif //MOVINGLIDAR_PREPROCESSING_H
