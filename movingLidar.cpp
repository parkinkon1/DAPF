//
// Created by 박규열 on 01/10/2019.
//

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


// 차선 영역을 검출하여, 코스트맵 반환
void laneDetection(Mat &input, Mat &output) {
    Mat frame = input.clone();
    // 인풋 이미지 크기
    int input_width = input.cols;
    int input_height = input.rows;
    // 이미지 처리를 위한 리사이징 크기
    int width = input_width/4;
    int height = input_height/4;

    // 원본 이미지 (BGR)
    imageshow("origin", frame);
    // 관심영역 추출 (BGR)
    ROIcut(frame, frame);
    // 이미지 이진화 (Binary)
    binarization(frame, frame);
    // 탑뷰 변환 (Binary)
    topView(frame, frame);


    vector<Vec2i> line_points_left, line_points_right;

    // 차선 추출 (Binary)
    searchLanes(frame, frame, line_points_left, line_points_right);
    imageshow("searchLanes", frame);
    // 코스트맵 생성 (Gray)
    createCostmap(frame, frame);
    imshow("costmap", frame);

    output = frame;

}




// 차선 영역을 검출하여, 코스트맵 반환
void detection(Mat &input, Mat &output) {
    Mat frame = input.clone();
    Mat frame_show = input.clone();
    // 인풋 이미지 크기
    int input_width = input.cols;
    int input_height = input.rows;
//    // 이미지 처리를 위한 리사이징 크기
//    int width = input_width/4;
//    int height = input_height/4;


    // 원본 이미지 (BGR)
    imshow("origin", frame);
    // 이미지 이진화 (Binary)
    binarization(frame, frame);
    imshow("binarization", frame);
    waitKey();

    // 코스트맵 생성 (Gray)
    createCostmap(frame, frame_show);
    imshow("costmap", frame);

    output = frame;

}









// 그레이스케일 이진 이미지에 대하여, 검은 영역에 대해 방사형으로 코스트맵 생성
void createCostmap(Mat &input, Mat &output) {
    Mat frame = input.clone();
    // 인풋 이미지 크기
    int input_width = input.cols;
    int input_height = input.rows;
    // 이미지 처리를 위한 리사이징 크기
    int width = input_width;
    int height = input_height;
    // 처리 속도 조절을 위한 리사이즈
    resize(frame, frame, Size(width, height));
    // 코스트 부여 단계
    int step = 50;
    frame = ~frame;
    // 마스크 크기를 키워 가며 코스트맵 생성
    Mat mask = frame.clone()/step;
    for (int i = 0; i < step; i++) {
        dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size((width * 2)/step, (width * 2)/step)));
        frame = frame + mask;
    }
    frame = ~frame;
    // 처리 속도 조절을 위한 리사이즈
    resize(frame, output, Size(input_width, input_height));

    vector<Vec2i> route, route_car;
    vector<double> next_pos;
    vector<float> angles;

    Mat frame_show = input.clone();
    cvtColor(input, frame_show, COLOR_GRAY2BGR);
    imshow("costmap", frame);
    costTracker(frame, frame, route, angles, 3*width/4, height, 0, -5, frame_show);

}


void costTracker(Mat &input, Mat &output, vector<Vec2i> &route, vector<float> &angles, int pre_x, int pre_y, int direction_x, int direction_y, Mat &frame_show) {

    Mat frame = input.clone();
    Mat potential = frame.clone();

    int width = input.cols;
    int height = input.rows;

    Point2i startPoint(3*width/4, height);

    int now_x = pre_x, now_y = pre_y;

    int gridSize = 5;
    int direction_length = 5;
    int minGap = 5;

    int max_cost = -1, cost = -1;

    float direction_norm = sqrt(powf((float)direction_x, 2) + powf((float)direction_y, 2));
    direction_x = (int)((float)direction_length * (float)direction_x / direction_norm);
    direction_y = (int)((float)direction_length * (float)direction_y / direction_norm);

    float temp_x = 1000000, temp_y = 10000000;

//    // 사방 픽셀에 대하여 높은 코스트 탐색
//    for (int x = pre_x + direction_x - gridSize; x <= pre_x + direction_x + gridSize; x++) {
//        // 예외조건 처리
//        if (x < 0 || x > width) continue;
//
//            for (int y = pre_y + direction_y - gridSize; y <= pre_y + direction_y + gridSize; y++) {
//                // 예외조건 처리
//                if ((y <= 0) || (y >= height)) continue;
////                // 방향벡터간 거리가 멀 경우 탐색하지 않음 (방향성 부여)
////                if ((x - pre_x - direction_x)*(x - pre_x - direction_x) + (y - pre_y - direction_y)*(y - pre_y - direction_y) >= minGap*minGap) continue;
//
//                // 높은 점수 픽셀 검사
//                cost = static_cast<int>(frame.at<uchar>(y, x));
//                if (cost >= max_cost) {
//                    max_cost = cost;
//                    if (pow(temp_x - (float)x, 2) + pow(temp_y - (float)y, 2) >= pow(now_x - (float)x, 2) + pow(now_y - (float)y, 2)) {
//                        now_x = x;
//                        now_y = y;
//                    }
////                    now_x = x;
////                    now_y = y;
//                    temp_x = x;
//                    temp_y = y;
//                    if (max_cost == 255) break;
//                }
//            }
//    }















    double slope = -1, angle = -1;
    if (!route.empty()) {
        slope = (double)(now_x - route.back()[0]) / (double)(route.back()[1] - now_y);
        angles.push_back(atan(slope));
    }

    route.push_back(Vec2i(now_x, now_y));



    if ((pre_x == now_x && pre_y == now_y) || max_cost <= 10) {
        cout << "route size : " << route.size() << endl;

        if (!route.empty()) {
            slope = (double)(route[(int)(2)][0] - route[0][0]) / (double)(route[0][1] - route[(int)(2)][1]);
            angle = atan(slope) * 180 / CV_PI;

            cout << "angle : " << angle << endl;

            cvtColor(frame, frame, COLOR_GRAY2BGR);
            for (int i = 0; i < (int)route.size() - 1; i++) {
//                line(frame, route[i], route[i+1], Scalar(0, 0, 255), 2);
            }
            for (int i = 0; i < (int)route.size() - 1; i++) {
                circle(frame, route[i], 5, Scalar(50, 100, 150), -1);
            }

            vector<double> ans;
            curveFitting(frame, frame, route, ans);

            imageshow("curve fitting", frame);


            vector<Vec2i> route_car;
            vector<double> next_pos;
            double steer = 0, pre_steer = 0, heading = -CV_PI/2, steer_temp;
            double x1 = 0, x2 = 0;

            int count = 0, isColl = 0;



            route_car.push_back(Vec2i(startPoint.x, startPoint.y));

            while (true) {
                pre_steer = steer;
                steer = getSteerwithCurve(ans, route_car.back()[1]);
                cout << "str : " << steer << endl;
                heading = (-CV_PI / 2) + pre_steer;

                steer_temp = (steer - pre_steer);
                cout << "dd : " << steer_temp << endl;
                if (steer_temp > 0.3491) steer_temp = 0.3491;
                else if (steer_temp < -0.3491) steer_temp = -0.3491;

                next_pos = PredictCar(steer_temp, 1.0,0.01, heading);
                cout << "before: next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                cout << "route_car : " << route_car.back() << endl;
                next_pos[0] += route_car.back()[0];
                next_pos[1] += route_car.back()[1];
                cout << "after : next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                if (next_pos[0] < 0 || next_pos[0] > width || next_pos[1] < 0 || next_pos[1] > height) break;
                route_car.push_back(Vec2i(next_pos[0],next_pos[1]));
                next_pos.clear();

                if (count % 10 == 0) {
                    isColl = drawCar(frame_show, frame_show, route_car.back(), (float)heading, potential);
                }
                count++;

                // 충돌 상황을 만나면 종료
                if (isColl) {
                    cout << "collide !!" << endl;
                    break;
                }

            }
            for (int i = 0; i < (int)route_car.size() - 1; i++) {
                line(frame_show, route_car[i], route_car[i+1], Scalar(255, 0, 255), 2);
            }
            for (int i = 0; i < (int)route_car.size(); i++) {
                circle(frame_show, route_car[i], 3, Scalar(150, 150, 200), -1);
            }

            waitKey();
            imshow("path planning", frame);
            imshow("simulation", frame_show);


            output = frame;

            return;

        }
        else {
            cout << "Lane not detected ㅜㅜ" << endl;
        }


        return;
    }
    else costTracker(input, output, route, angles, now_x, now_y, now_x - pre_x, now_y - pre_y, frame_show);

}





//input : 조향각, 속력, 이동하는 데 걸리는 시간
//init_str [rad], V_r [m/s], dt [sec], heading [rad]
//output : 절대좌표계(UTM좌표계)에서의 변위(x,y변화량)
vector<double> PredictCar(double init_str, double V_r, double dt1, double heading) {
    double del; // del : 무게중심의 조향각 [rad]
    int sign; // 조향 부호, 왼쪽조향 : -1, 오른쪽 조향 : +1
    double V_c; //V_c : 무게중심의 속력 [m/s]
    double theta; // theta : dt 이후 변한 각도 [rad]
    double Xtr; // TR 좌표계에서의 X변위 [m]
    double Ytr; // TR 좌표계에서의 Y변위 [m]
    double Xsn; // SN 좌표계에서의 X변위 [m]
    double Ysn; // SN 좌표계에서의 Y변위 [m]
    double Xxy; // XY(utm) 좌표계에서의 X변위 [m]
    double Yxy; // XY(utm) 좌표계에서의 Y변위 [m]


    cout << "prediccar_ str : " << init_str << endl;
    if (init_str < 0) sign = -1;
    else sign = 1;

    // 0.01 rad = 0.573 deg
    if (abs(init_str) < 0.001) del = 0;
    else del = sign * atan(plf_L*tan(abs(init_str)) / (plf_L - Tf * tan(abs(init_str)))); // 애커먼 - 장토 식
    //del = sign*atan(plf_lr*tan(abs(init_str))/plf_L);
    //*********************************************************************************
    if (del == 0)  V_c = V_r;
    else V_c = V_r * plf_lr * tan(abs(init_str)) / (plf_L * sin(abs(del)));
    //*********************************************************************************
    theta = sin(abs(del)) * V_c * dt1 / plf_lr;
    //*********************************************************************************
    // 직진일 때
    if (theta < 0.001) {
        Xtr = V_c * dt1;
        Ytr = 0;
    }
        //회전할 때
    else {
        Xtr = plf_lr * sin(theta) / sin(abs(del)); //좌표계 회전행렬변환
        Ytr = plf_lr * (1 - cos(theta)) / sin(abs(del)); //좌표계 회전행렬변환
    }
    //*********************************************************************************
    if (init_str >= 0) {
        Xsn = cos(del) * Xtr - sin(del) * Ytr; //좌표계 회전행렬변환
        Ysn = sin(del) * Xtr + cos(del) * Ytr; //좌표계 회전행렬변환
    }
    else {
        Xsn = cos( abs(del)) * Xtr - sin(abs(del)) * Ytr; //좌표계 회전행렬변환
        Ysn = -sin(abs(del)) * Xtr - cos(abs(del)) * Ytr; //좌표계 회전행렬변환
    }
    //*********************************************************************************
    // heading + del : Mat 상에서 현재 차량의 헤딩에 무게중심의 조향을 더하면 다음 위치에서의 헤딩이라고 가정
    Xxy = cos(-(heading + init_str)) * Xsn + sin(-(heading + init_str)) * Ysn; //좌표계 회전행렬변환
    Yxy = -sin(-(heading + init_str)) * Xsn + cos(-(heading + init_str)) * Ysn; //좌표계 회전행렬변환
    //printf("heading : %f%s\ninit_str : %f%s\nXxy : %f%s \nYxy : %f%s\n", heading * 180 / 3.141592653589793238462643383, "[deg]", init_Str * 180 / 3.141592653589793238462643383, "[deg]", Xxy, "m", Yxy, "m");
    //*********************************************************************************

    // 이 함수의 변수인 heading을 heading + del로 계속 갱신해줘야함
    // ex) _dataContainer->setValue_heading(heading + del);

    vector<double> temp;
    temp.push_back(1000 * Xxy);
    temp.push_back(1000 * Yxy);
    return temp;
}



// 점의 방정식을 곡선으로 근사
void curveFitting(Mat &input, Mat &output, vector<Vec2i> &route, vector<double> &ans) {

    if (route.size() <= 3) {
        cerr << "no Point to fitting !" << endl;
        return;
    }

    vector<pair<double, double>> v;
    for (auto & i : route) {
        v.push_back(make_pair((double)i[0], (double)i[1]));
    }
    int n = v.size();
    Mat A;


// A를 완성시키는 이중 for문
    for (int i = 0; i < n; i++) {
        vector<double> tmp;
// 3 차 방정식으로 근사한다. 이 숫자 바꿔 주면 n-1 차 방정식까지 근사 가능.
        for (int j = 3; j >= 0; j--) {
            double x = v[i].second; // x 하고 y 를 바꾸기로 함. 그래야 함수가 만들어지니까
            tmp.push_back(pow(x,j));
        }
        A.push_back(Mat(tmp).t());
    }

    cout << "A: " << endl;
    cout << A << endl;

    Mat B; // B 에는 y 좌표들을 저장한다.

    vector<double> tmp;
    for (int i = 0; i < n; i++) {
        double y = v[i].first; // x 하고 y 바꾸기로 함. 그래야 함수가 만들어지니까
        tmp.push_back(y);
    }
    B.push_back(Mat(tmp));

    cout << "B: " << endl;
    cout << B << endl;

    Mat X;
// X = invA * B; // X = A(-1)B
    X = ((A.t() * A).inv()) * A.t() * B;


    cout << "X: " << endl;
    cout << X << endl;

// X 에서 차례대로 뽑아내서 ans 에 담는다.
// 몇차 방정식으로 근사할껀지 정할때 건드려줘야 되는부분, 3차 방정식으로 근사할꺼면 4 ㅇㅇ.
    for (int i = 0; i < 4; i++) {
        ans.push_back(X.at<double>(i, 0));
    }
// 앞에서 부터 0인지 검사해서 0이면 지운다. 차수 조절을 위하여. 가운데 0은 안지워짐.
    while (!ans.empty()) {
        if (ans.at(0) == 0) ans.erase(ans.begin());
        else break;
    }
    double coef[900];
    int ans_size = ans.size(); // ans_size - 1 차 다항식이 만들어진거고, ans_size만큼 a,b,c,d,e `` 가 있다.

// 벡터보다 배열에 접근하는게 훨 빠르기 때문에 후에 계산을 위하여 배열에 넣어주었다.
    for (int i = 0; i < ans_size; i++) {
        coef[i] = ans.at(i);
    }

    for (int i = (int)input.rows; i >= 1; i -= 2) {
        double x1 = 0, x2 = 0;
        for (int j = 0; j < ans_size; j++) {
            x1 += coef[j] * pow(i, ans_size - 1 - j);
            x2 += coef[j] * pow(i - 1, ans_size - 1 - j);
        }
        output = input.clone();
        line(output, Point(x1, i ), Point(x2, i - 1), Scalar(0, 0, 255), 2);
    }


}


// 3차식의 계수와 원하는 y 좌표를 대입하면 steer 반환
double getSteerwithCurve(vector<double> &ans, int y) {

    vector<double> diff; // 미분한 값 계수 저장

    int num = ans.size();
    for (int i = 0; i < num; i++)
    {
        diff.push_back((num - (i + 1))*ans[i]); // 미분 실행
    }

    double gradient = 0.0; //기울기 initial

    for (int i = 0; i < (int)diff.size() - 1; i++)
    {
        gradient += diff[i] * (pow(y, (diff.size() - (i + 2)))); //(y,x) 점을 미분값에 대입후 합... 기울기가 나옴
    }

    double descent_deg = -atan(gradient)*180/CV_PI; // 세타값 (degree 환산)
    double descent_rad = -atan(gradient); // 세타값 (radian 환산)

    return descent_rad;

}




// 차의 위치와 헤딩을 넣으면 차량 영역이 그려진 영상을 반환
bool drawCar(Mat &input, Mat &output, Vec2i point_car, float heading, Mat &potential) {

    output = input.clone();
    int width = input.cols;
    int height = input.rows;

    float descent_rad = heading + CV_PI/2;
    float heading_deg = heading * 180 / CV_PI;

    float carCenX = point_car[0];
    float carCenY = point_car[1];
    double car_width = 40;
    double car_height = 60;

    Point2f carCen(carCenX, carCenY), pts1[4];
    Size2f carSize(car_height, car_width);

    cv::RotatedRect rotatedRectangle(carCen, carSize, heading_deg); //차량 모양 형성(중심점, 각도)
    rotatedRectangle.points(pts1);

    for (int i = 0; i < 4; i++) {
        line(output, pts1[i], pts1[(i + 1) % 4], Scalar(255, 0, 0), 2);
    }
    ////////////////////////////////////////////////////////////

    float rot_x, rot_y;
    float rot_cos = cos(heading), rot_sin = sin(heading);
    uchar *potentialData = potential.data;
    uchar *outputData = output.data;
    uchar cost;
    int maxPotential = 1;

    // 차량이 지나갈 수 없는 점이면 탐색 종료
    for (int x = (int)(carCenX - car_height/2); x <= carCenX + car_height/2; x++) {
        for (int y = (int)(carCenY - car_width/2); y <= carCenY + car_width/2; y++) {
            rot_x = rot_cos * (float)((float)x - carCenX) + rot_sin * (float)((float)y - carCenY);
            rot_y = (-rot_sin) * (float)((float)x - carCenX) + rot_cos * (float)((float)y - carCenY);
            rot_x += carCenX;
            rot_y += carCenY;

            if (rot_x <= 0 || rot_y <= 0 || rot_x >= width || rot_y >= height) {
                continue;
            }
            else if (potentialData[(int)rot_y * width + (int)rot_x] <= maxPotential) {
                cout << "High Potential !!!" << endl;
                return true;
            }

//            outputData[(int)rot_y * height * 3 + (int)rot_x * 3] = 0;
//            outputData[(int)rot_y * height * 3 + (int)rot_x * 3 + 1] = 0;
//            outputData[(int)rot_y * height * 3 + (int)rot_x * 3 + 2] = 255;

        }
    }

    return false;

//    vector<float> pointSet;
//
//    int arraySize = sizeof(pts1) / sizeof(*pts1);
//
//    for (int i = 0; i < arraySize; i++) {
//        pointSet[2 * i] = pts1[i].x;
//        pointSet[2 * i + 1] = pts1[i].y;
//    }
//
//    return isCollision(pointSet, potential);


//    Point2d arrowCen(carCenX, carCenY); //화살표 꼬리 좌표
//    Point2d arrowPoint(carCenX - 120*sin(-1*descent_rad), carCenY + 160*cos(-descent_rad)); //화살표 머리 좌표
//    arrowedLine(output, arrowCen, arrowPoint, Scalar(0), 2, 5, 0, 0.1); //물체의 벡터

}






///-------------------------------------

vector<double*> get_obj_info(vector<pair<double, double>> obj_move, double r) {
    vector<double*> v;
    for (int i = 0; i < (int)obj_move.size() - 1; i++) {
        double d[5];
        d[0] = obj_move[i].first;
        d[1] = obj_move[i].second;
        d[2] = obj_move[i + 1].first - d[0];
        d[3] = obj_move[i + 1].second - d[1];
        d[4] = r;
        v.push_back(d);
    }
    return v;
}


double potentialByObject(vector<double*> object, double get_x, double get_y) {
    double z = 0;
    for (int i = 0; i < (int)object.size(); i++) {
        double x = object[i][0];
        double y = object[i][1];
        double x_v = object[i][2];
        double y_v = object[i][3];
        double r = object[i][4];
        z += (100 - 10 * i) * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
    }
    return 1000 * z;
}


double potentialByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y) {
    double z = 0;
    for (auto & i : L) {
        double x = i.first;
        double y = i.second;
        z += 50 * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
    }

    for (auto & i : R) {
        double x = i.first;
        double y = i.second;
        z += 50 * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
    }

    return 1000 * z;
}



pair<double, double> diffByObject(vector<double*> object, int now_frame, double get_x, double get_y) {
    int i = now_frame;
    double x = object[i][0];
    double y = object[i][1];
    double x_v = object[i][2];
    double y_v = object[i][3];
    double r = object[i][4];
    double diff_x = (get_x - x) * (-1 * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2))));
    double diff_y = (get_y - y) * (-1 * exp(-0.005 * (pow(get_y - y, 2) + pow(get_x - x, 2))));
    return make_pair(diff_x, diff_y);
}



pair<double, double> diffByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y) {
    int i = get_y;
    double x = L[i].first;
    double y = L[i].second;

    double diff_x = -0.5 * (get_x - x) * log10(exp(1)) * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
    double diff_y = -0.5 * (get_y - y) * log10(exp(1)) * exp(-0.005 * (pow(get_y - y, 2) + pow(get_x - x, 2)));

    x = R[i].first;
    y = R[i].second;
    diff_x += -0.5 * (get_x - x) * log10(exp(1)) * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
    diff_y += -0.5 * (get_y - y) * log10(exp(1)) * exp(-0.005 * (pow(get_y - y, 2) + pow(get_x - x, 2)));
    return make_pair(diff_x, diff_y);
}



void show_potentialField(Mat &dstImage, vector<double*> object, vector<pair<double, double>> L, vector<pair<double, double>> R) {
//    Mat dstImage(900, 600, CV_8UC3, Scalar(255, 255, 255));
    double map[900][600];
    double big = -1;
    for (int i = 0; i < 900; i++) {
        for (int j = 0; j < 600; j++) {
            map[i][j] = potentialByObject(object, j, i);
            map[i][j] += potentialByLane(L, R, j, i);
            if (map[i][j] > big) {
                big = map[i][j];
            }
        }
    }
    for (int i = 0; i < 900; i+=20) {
        for (int j = 0; j < 600; j+=20) {
            map[i][j] /= big; // 최대 크기 1로 nomalization
            map[i][j] *= 255; // 최대 크기 255로
            circle(dstImage, Point(j, i), 10, Scalar(255 - map[i][j], 255 - map[i][j], 255 - map[i][j]), -1);
        }
    }
    imshow("dd", dstImage);
    waitKey(0);
}


///------------------------------------------------



void getMovingFrame(Mat &frame, int time) {

    Mat dstImage(900, 600, CV_8UC3, Scalar(255, 255, 255));

    vector<pair<double, double>> L;
    vector<pair<double, double>> R;
    for (int i = 0; i < 900; i++) {
        L.push_back(make_pair(100, i));
    }
    for (int i = 0; i < 900; i++) {
        R.push_back(make_pair(500, i));
    }

    cout << "generating objects..." << endl;

    vector<pair<double, double>> obj_move;
//    obj_move.push_back(make_pair(50, 100));
//    obj_move.push_back(make_pair(100, 200));
//    obj_move.push_back(make_pair(150, 300));
//    obj_move.push_back(make_pair(200, 350));
//    obj_move.push_back(make_pair(250, 400));
    obj_move.push_back(make_pair(300, 430));
    obj_move.push_back(make_pair(350, 460));
    obj_move.push_back(make_pair(400, 490));
//    obj_move.push_back(make_pair(450, 520));
//    obj_move.push_back(make_pair(500, 600));
//    obj_move.push_back(make_pair(550, 650));
//    obj_move.push_back(make_pair(600, 700));

    vector<double*> obj_info = get_obj_info(obj_move, 100);

    cout << "generating objects finished" << endl;
    cout << "generating lanes..." << endl;

    vector<pair<double, double>> points_lane_left, points_lane_right;

    for (int i = 0; i < 900; i++) {
        points_lane_left.push_back(make_pair(0, i));
        points_lane_right.push_back(make_pair(600, i));
    }


    cout << "generating lanes finished" << endl;
    cout << "generating potential field..." << endl;

    show_potentialField(dstImage, obj_info, points_lane_left, points_lane_right);

    cout << "generating potential field finished" << endl;


    for (int i = 0; i < (int)obj_move.size(); i++) {
        circle(dstImage, Point(obj_move[i].second, obj_move[i].first), 100, Scalar(0, 0, 255), 3);
    }



}




void simulation(Mat &input, Mat &output) {
    Mat frame = input.clone();
    Mat frame_show = input.clone();
    // 인풋 이미지 크기
    int input_width = input.cols;
    int input_height = input.rows;
//    // 이미지 처리를 위한 리사이징 크기
//    int width = input_width/4;
//    int height = input_height/4;


    // 원본 이미지 (BGR)
    imshow("origin", frame);
    // 이미지 이진화 (Binary)
    binarization(frame, frame);
    imshow("binarization", frame);
    waitKey();

    // 코스트맵 생성 (Gray)
    createCostmap(frame, frame_show);
    imshow("costmap", frame);

    output = frame;
}













