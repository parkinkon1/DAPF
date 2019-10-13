//
// Created by 박규열 on 01/10/2019.
//

#include "movingLidar.h"

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



// path planning : 충돌 여부 감지를 위해 이진 영상 dangerous 필요, frame_show는 BGR 이미지
void costTracker(Mat &dangerous, vector<Vec2i> &route, const Point2d& pre_point, const Point2d& pre_direction, Mat &frame_show,
                    const vector<double*>& object, vector<pair<double, double>> L, vector<pair<double, double>> R) {

    int width = dangerous.cols;
    int height = dangerous.rows;

    // 차량의 속도 (벡터 크기)
    double speed = 5;
    // 퍼텐셜 반영비율
    double alpha = 1;
    // 시작점과 도착점 정의
    Point2d startPoint((double)width * 3 / 4, (double)height);
    Point2d now_point = pre_point;
    Point2d now_direction(0, 0);

    // 퍼텐셜 힘 계산
    pair<double, double> diff_lane = make_pair(0,0), diff_object = make_pair(0,0);
    diff_lane = diffByLane(L, R, pre_point.x, pre_point.y);
    diff_object = diffByObject(object, pre_point.x, pre_point.y);

    // 방향벡터 계산
    now_direction.x = pre_direction.x + alpha * (diff_lane.first + diff_object.first);
    now_direction.y = pre_direction.y + alpha * (diff_lane.second + diff_object.second);
    // 방향벡터 정규화
    now_direction *= speed;
    now_direction /= sqrt(pow(now_direction.x , 2) + pow(now_direction.y, 2));

    // 도착점 계산
    now_point += now_direction;

    /// 경로 탐색이 더이상 불가능할 경우, 탐색 종료
    uchar *dangerous_data = dangerous.data;
    if ((int)dangerous_data[width * (int)now_point.y + (int)now_point.x] == 255) {
        cout << "Cannot make path !!" << endl;
        return;
    }

    /// 프레임을 벗어날 경우, Path Planning 종료
    if (now_point.x < 0 || now_point.y < 0 || now_point.x > width || now_point.y > height) {
        cout << "route size : " << route.size() << endl;
        // 경로가 정상적으로 생성된 경우
        if (!route.empty()) {
            double slope = (double)(route[2][0] - route[0][0]) / (double)(route[0][1] - route[2][1]);
            double angle = atan(slope) * 180 / CV_PI;
            cout << "Present steer : " << angle << endl;

            // 생성된 경로 그리기
            for (int i = 0; i < (int)route.size() - 1; i++) {
//                line(frame, route[i], route[i+1], Scalar(0, 0, 255), 2);
            }
            for (int i = 0; i < (int)route.size() - 1; i++) {
                circle(frame_show, route[i], 5, Scalar(150, 100, 150), -1);
            }

            /// 점들을 곡선으로 근사 (Curve Fitting)
            vector<double> ans;
            curveFitting(frame_show, frame_show, route, ans);

            imshow("curve fitting", frame_show);
            waitKey();


            /// 차량이 갈 수 있는 경로인지 탐색
            vector<Vec2i> route_car;
            vector<double> next_pos;
            double steer = 0, pre_steer = 0, heading, steer_temp;
            int count = 0, isColl = 0;
            // 처음 차량의 시작지점
            route_car.push_back(Vec2i(startPoint.x, startPoint.y));
            // 계획된 경로를 따라가면서, 차선 또는 물체와 부딪히는지 탐색
            while (true) {
                // curve fitting된 함수 식으로부터 현재 가야하는 헤딩 방향 계산
                pre_steer = steer;
                steer = getSteerwithCurve(ans, route_car.back()[1]);
                cout << "str : " << steer << endl;
                heading = (-CV_PI / 2) + pre_steer;
                // 조향 경계조건 설정
                steer_temp = (steer - pre_steer);
                cout << "dd : " << steer_temp << endl;
                if (steer_temp > 0.3491) steer_temp = 0.3491;
                else if (steer_temp < -0.3491) steer_temp = -0.3491;
                // 다음 차량 위치 추정 및 이동
                next_pos = PredictCar(steer_temp, 1.0,0.01, heading);
                cout << "before: next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                cout << "route_car : " << route_car.back() << endl;
                next_pos[0] += route_car.back()[0];
                next_pos[1] += route_car.back()[1];
                cout << "after : next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                if (next_pos[0] < 0 || next_pos[0] > width || next_pos[1] < 0 || next_pos[1] > height) break;
                route_car.push_back(Vec2i(next_pos[0],next_pos[1]));
                next_pos.clear();
                // 차량이 지나가는 경로 그리기
                if (count % 10 == 0) {
                    isColl = drawCar(frame_show, frame_show, route_car.back(), (float)heading, dangerous);
                } count++;
                // 충돌 상황을 만나면 종료
                if (isColl) {
                    cout << "collide !!" << endl;
                    break;
                }
            }

            // 차량이 지나간 경로 그리기
            for (int i = 0; i < (int)route_car.size() - 1; i++) {
                line(frame_show, route_car[i], route_car[i+1], Scalar(255, 0, 255), 2);
            }
            for (int i = 0; i < (int)route_car.size(); i++) {
                circle(frame_show, route_car[i], 3, Scalar(200, 150, 200), -1);
            }

            cout << "searching path finished" << endl;
            imshow("simulation", frame_show);
            waitKey();
        }
        // route에 아무것도 저장되지 않은 경우
        else { cout << "Lane not detected ㅜㅜ" << endl; }

        return;
    }
    /// 프레임을 벗어나지 않은 경우 path planning 경로 계속 탐색
    else {
        cout << "searching next position..." << endl;
        costTracker(dangerous, route, now_point, now_direction, frame_show, object, L, R);
    }

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
        cerr << "Lack of Points to fitting !" << endl;
        return;
    }

    vector<pair<double, double>> v;
    for (auto & i : route) {
        v.push_back(make_pair((double)i[0], (double)i[1]));
    }
    int n = v.size();
    Mat A;


    // A를 완성시키는 이중 for문
    // 3 차 방정식으로 근사한다. 이 숫자 바꿔 주면 n-1 차 방정식까지 근사 가능.
    for (int i = 0; i < n; i++) {
        vector<double> tmp;
        for (int j = 3; j >= 0; j--) {
            double x = v[i].second; // x 하고 y 를 바꾸기로 함. 그래야 함수가 만들어지니까
            tmp.push_back(pow(x,j));
        }
        A.push_back(Mat(tmp).t());
    }

    Mat B; // B 에는 y 좌표들을 저장한다.
    vector<double> tmp;
    for (int i = 0; i < n; i++) {
        double y = v[i].first; // x 하고 y 바꾸기로 함. 그래야 함수가 만들어지니까
        tmp.push_back(y);
    }
    B.push_back(Mat(tmp));

    // X = invA * B; // X = A(-1)B
    Mat X = ((A.t() * A).inv()) * A.t() * B;

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
    // ans_size - 1 차 다항식이 만들어진거고, ans_size만큼 a,b,c,d,e `` 가 있다.
    int ans_size = ans.size();

    // 벡터보다 배열에 접근하는게 훨 빠르기 때문에 후에 계산을 위하여 배열에 넣어주었다.
    for (int i = 0; i < ans_size; i++) {
        coef[i] = ans.at(i);
    }

    // 계산된 곡선 그리기
    output = input.clone();
    for (int i = (int)input.rows; i >= 1; i -= 2) {
        double x1 = 0, x2 = 0;
        for (int j = 0; j < ans_size; j++) {
            x1 += coef[j] * pow(i, ans_size - 1 - j);
            x2 += coef[j] * pow(i - 1, ans_size - 1 - j);
        }
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
    for (int i = 0; i < object.size(); i++) {
        double x = object[i][0];
        double y = object[i][1];
        double x_v = object[i][2];
        double y_v = object[i][3];
        double r = object[i][4];

        // z += (100 - 10 * i) * exp(-0.005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));
        double distance = pow(pow(get_x - x, 2) + pow(get_y - y, 2), (1 / 2)); // get_x, get_y 가 object하고 가까우면 값이 더해져야지
        if (distance != 0) {
            z += -1 / distance;
        }
        else{
            z += 255;
        }

    }
    if (z > 100) {
        return 100;
    }
    return z;
}


double potentialByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y) {
    double z = 0;

    int i = get_y;
    double x = L[i].first;
    double y = L[i].second;
    z += 255 * exp(-0.00005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));

    x = R[i].first;
    y = R[i].second;
    z += 255 * exp(-0.00005 * (pow(get_x - x, 2) + pow(get_y - y, 2)));

    return z;
}



pair<double, double> diffByObject(vector<double*> object, double get_x, double get_y) {
    double x = object[0][0];
    double y = object[0][1];
    double x_v = object[0][2];
    double y_v = object[0][3];
    double r = object[0][4];
    double diff_x = (get_x - x) * (1 * exp(-0.00005 * (pow(get_x - x, 2) + pow(get_y - y, 2))));
    double diff_y = (get_y - y) * (1 * exp(-0.00005 * (pow(get_y - y, 2) + pow(get_x - x, 2))));
    return make_pair(diff_x, diff_y);
}



pair<double, double> diffByLane(vector<pair<double, double>> L, vector<pair<double, double>> R, double get_x, double get_y) {
    int i = get_y;
    double x1 = L[i].first;
    double y1 = L[i].second;

    double diff_x = 1 * (get_x - x1) * log10(exp(1)) * exp(-0.00005 * (pow(get_x - x1, 2) + pow(get_y - y1, 2)));
//    double diff_y = -0.000005 * (get_y - y1) * log10(exp(1)) * exp(-0.5 * (pow(get_y - y1, 2) + pow(get_x - x1, 2)));
    double diff_y = 0;

    double x2 = R[i].first;
    double y2 = R[i].second;
    diff_x += 1 * (get_x - x2) * log10(exp(1)) * exp(-0.00005 * (pow(get_x - x2, 2) + pow(get_y - y2, 2)));
//    diff_y += -0.000005 * (get_y - y2) * log10(exp(1)) * exp(-0.5 * (pow(get_y - y2, 2) + pow(get_x - x2, 2)));
    return make_pair(diff_x, diff_y);
}



void show_potentialField(Mat &dstImage, vector<double*> object, vector<pair<double, double>> L, vector<pair<double, double>> R) {

    int width = 600;
    int height = 900;
    Mat frame_show(height, width, CV_8UC3, Scalar(255, 255, 255));

    int showing_gap = 3;

    cout << "visualization..." << endl;

    double cost = 0;
    for (int i = 0; i < height; i += showing_gap) {
        for (int j = 0; j < width; j += showing_gap) {
            cost += potentialByObject(object, j, i);
//            cost = potentialByLane(L, R, j, i);
            circle(frame_show, Point(j, i), showing_gap, Scalar(255 - cost, 255 - cost, 255 - cost), -1);
        }
    }

    cout << "visualizing vectors..." << endl;

    pair<double, double> diff_lane = make_pair(0, 0), diff_object = make_pair(0, 0);
    for (int i = 0; i < height; i += showing_gap * 10) {
        for (int j = 0; j < width; j += showing_gap * 10) {
            diff_object = diffByObject(object, j, i);
//            diff_lane = diffByLane(L, R, j, i);
            arrowedLine(frame_show, Point2d(j, i), Point2d(j + diff_object.first + diff_lane.first, i + diff_object.second + diff_lane.second), Scalar(0, 0, 255), 2, 5, 0, 0.1);
        }
    }


    cout << "visualization finished" << endl;

    imshow("Visualization", frame_show);
    waitKey(0);

    dstImage = frame_show;
}





void DynamicPotential( vector< vector<double> > &ObjCombine , vector < vector<double> >&object) {

    vector<double> ObjParticle;

    double z1 = 0, z2 = 0, z3 = 0;
    double diff_x_1, diff_x_2, diff_x_3;
    double diff_y_1, diff_y_2, diff_y_3;
    double pt1_x, pt1_y, pt2_x, pt2_y;

    for (int i = 0; i < object.size(); i++) {
        double x = object[i][0];
        double y = object[i][1];
        double x_v = object[i][2];
        double y_v = object[i][3];
        double r = object[i][4];

        pt1_x = x + x_v / 2;
        pt1_y = y + y_v / 2;
        pt2_x = x + x_v;
        pt2_y = y + y_v;

        for (int q = 0; q < 900; q++)
        {
            for (int p = 0; p < 600; p++)
            {
                z1 = 100 * exp(-( 0.005 / r )*(pow(q - x, 2) + pow(p - y, 2)));
                z2 = 150 * exp(-(0.005 / (5 * r))*(pow(q - pt1_x, 2) + pow(p - pt1_y, 2)));
                z3 = 200 * exp(-(0.005 / (10 * r))*(pow(q - pt2_x, 2) + pow(p - pt2_y, 2)));

                diff_x_1 = (q - x) * (-1 * exp(-(0.005 / r ) * (pow(q - x, 2) + pow(p - y, 2))));
                diff_y_1 = (p - y) * (-1 * exp(-(0.005 / r ) * (pow(p - y, 2) + pow(q - x, 2))));

                diff_x_2 = (q - pt1_x) * (-1 * exp(-(0.005 / (5 * r)) * (pow(q - pt1_x, 2) + pow(p - pt1_y, 2))));
                diff_y_2 = (p - pt1_y) * (-1 * exp(-(0.005 / (5 * r)) * (pow(p - pt1_y, 2) + pow(q - pt1_x, 2))));

                diff_x_3 = (q - pt2_x) * (-1 * exp(-(0.005 / (10 * r)) * (pow(q - pt2_x, 2) + pow(p - pt2_y, 2))));
                diff_y_3 = (p - pt2_y) * (-1 * exp(-(0.005 / (10 * r)) * (pow(p - pt2_y, 2) + pow(q - pt2_x, 2))));

                double z_sum = z1 + z2 + z3;
                double diff_x_sum = diff_x_1 + diff_x_2 + diff_x_3;
                double diff_y_sum = diff_y_1 + diff_y_2 + diff_y_3;

                ObjParticle.push_back(q);
                ObjParticle.push_back(p);
                ObjParticle.push_back(diff_x_sum);
                ObjParticle.push_back(diff_y_sum);
                ObjParticle.push_back(z_sum);

                ObjCombine.push_back(ObjParticle);

            }
        }
    }
}

void DynamicPotential() {

    vector< vector<double> > ObjCombine;
    vector<double> ObjParticle;

    for (int i = 0; i < 3; i++) {

        double z1, z2, z3, z4, z5;
        double diff_x_1, diff_x_2, diff_x_3, diff_x_4, diff_x_5;
        double diff_y_1, diff_y_2, diff_y_3, diff_y_4, diff_y_5;
        double pt1_x, pt1_y, pt2_x, pt2_y, pt3_x, pt3_y, pt4_x, pt4_y;


        double x = 400; // (x,y) : 오브젝트의 위치
        double y = 300;
        double x_v = -100; // (x_v, y_v) : 오브젝트의 방향 벡터
        double y_v = 100;
        double r = 100; // 오브젝트의 반경 --> 현재 반경과 함수식 계수의 비례관계가 성립되지 않음

        double q ;   // (q,p) 에 퍼텐셜 및 방향퍼텐셜을 알고 싶은 좌표를 입력하면 됨.
        double p ;

        pt1_x = x + (x_v / 4); // 물체의 예측 경로상에 퍼텐셜을 5단계로 나누어 계산한다.
        pt1_y = y + (y_v / 4);
        pt2_x = x + (x_v / 3);
        pt2_y = y + (y_v / 3);
        pt3_x = x + (x_v / 2);
        pt3_y = y + (y_v / 2);
        pt4_x = x + x_v;
        pt4_y = y + y_v;

        z1 = 100 * exp(-(0.005 / r)*(pow(q - x, 2) + pow(p - y, 2)));
        z2 = 110 * exp(-(0.005 / (1.1 * r))*(pow(q - pt1_x, 2) + pow(p - pt1_y, 2)));
        z3 = 120 * exp(-(0.005 / (1.2 * r))*(pow(q - pt2_x, 2) + pow(p - pt2_y, 2)));
        z4 = 130 * exp(-(0.005 / (1.3 * r))*(pow(q - pt3_x, 2) + pow(p - pt3_y, 2)));
        z5 = 140 * exp(-(0.005 / (1.4 * r))*(pow(q - pt4_x, 2) + pow(p - pt4_y, 2)));


        diff_x_1 = (q - x) * (-1 * exp(-(0.005 / r) * (pow(q - x, 2) + pow(p - y, 2))));
        diff_y_1 = (p - y) * (-1 * exp(-(0.005 / r) * (pow(p - y, 2) + pow(q - x, 2))));

        diff_x_2 = 1.1*(q - pt1_x) * (-1 * exp(-(0.005 / (1.1 * r)) * (pow(q - pt1_x, 2) + pow(p - pt1_y, 2))));
        diff_y_2 = 1.1*(p - pt1_y) * (-1 * exp(-(0.005 / (1.1 * r)) * (pow(p - pt1_y, 2) + pow(q - pt1_x, 2))));

        diff_x_3 = 1.2*(q - pt2_x) * (-1 * exp(-(0.005 / (1.2 * r)) * (pow(q - pt2_x, 2) + pow(p - pt2_y, 2))));
        diff_y_3 = 1.2*(p - pt2_y) * (-1 * exp(-(0.005 / (1.2 * r)) * (pow(p - pt2_y, 2) + pow(q - pt2_x, 2))));

        diff_x_4 = 1.3*(q - pt3_x) * (-1 * exp(-(0.005 / (1.3 * r)) * (pow(q - pt3_x, 2) + pow(p - pt3_y, 2))));
        diff_y_4 = 1.3*(p - pt3_y) * (-1 * exp(-(0.005 / (1.3 * r)) * (pow(p - pt3_y, 2) + pow(q - pt3_x, 2))));

        diff_x_5 = 1.4*(q - pt4_x) * (-1 * exp(-(0.005 / (1.4 * r)) * (pow(q - pt4_x, 2) + pow(p - pt4_y, 2))));
        diff_y_5 = 1.4*(p - pt4_y) * (-1 * exp(-(0.005 / (1.4 * r)) * (pow(p - pt4_y, 2) + pow(q - pt4_x, 2))));


        double z_sum = z1 + z2 + z3 + z4 + z5;
        double diff_x_sum = diff_x_1 + diff_x_2 + diff_x_3 + diff_x_4 + diff_x_5;
        double diff_y_sum = diff_y_1 + diff_y_2 + diff_y_3 + diff_y_4 + diff_y_5;

        cout << "Zs : " << z1 << " " << z2 << " " << z3 << " " << z4 << " " << z5 << endl;
        cout << "diff_xs : " << diff_x_1 << " " << diff_x_2 << " " << diff_x_3 << " " << diff_x_4 << " " << diff_x_5 << endl;
        cout << "diff_ys : " << diff_y_1 << " " << diff_y_2 << " " << diff_y_3 << " " << diff_y_4 << " " << diff_y_5 << endl;

        ObjParticle.push_back(q);
        ObjParticle.push_back(p);
        ObjParticle.push_back(diff_x_sum);
        ObjParticle.push_back(diff_y_sum);
        ObjParticle.push_back(z_sum);

        ObjCombine.push_back(ObjParticle);

    }
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
    obj_move.push_back(make_pair(50, 100));
    obj_move.push_back(make_pair(100, 200));
    obj_move.push_back(make_pair(150, 300));
    obj_move.push_back(make_pair(200, 350));
    obj_move.push_back(make_pair(250, 400));
    obj_move.push_back(make_pair(300, 430));
    obj_move.push_back(make_pair(350, 460));
    obj_move.push_back(make_pair(400, 490));
    obj_move.push_back(make_pair(450, 520));
    obj_move.push_back(make_pair(500, 600));
    obj_move.push_back(make_pair(550, 650));
    obj_move.push_back(make_pair(600, 700));

    vector<double*> obj_info = get_obj_info(obj_move, 50);


    cout << "generating objects finished" << endl;
    cout << "generating lanes..." << endl;

    vector<pair<double, double>> points_lane_left, points_lane_right;

    for (int i = 0; i < 900; i++) {
        points_lane_left.push_back(make_pair(0, i));
        points_lane_right.push_back(make_pair(600, i));
    }


    cout << "generating lanes finished" << endl;
    cout << "generating potential field..." << endl;

    show_potentialField(dstImage, obj_info, L, R);

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













    /// TTC 계산
    //차의 현재위치와 물체의 현재위치
    pair<double, double> car = make_pair(10,10);  // make_pair(5, 5);
    pair<double, double> object = make_pair(20,20);
    pair<double, double> car_heading = make_pair(1,5);
    pair<double, double> object_heading = make_pair(10,1);
    double car_velocity = 5;
    double object_velocity = 3;

    double answer = TTC_master(car, object, car_heading, object_heading, car_velocity, object_velocity);
    cout << answer << endl;


}



///----------------------------------
//차량과 물체의 거리 차이 |d| 를 알려주는 함수 -> 분자
double distance(pair<double, double> car, pair<double, double > object) {
    double x_distance = car.first > object.first ? car.first - object.first : object.first - car.first;
    double y_distance = car.second > object.second ? car.second - object.second : object.second - car.second;
    double total_distance = sqrt(x_distance * x_distance + y_distance * y_distance);
    return total_distance * total_distance;
}

//v벡터와 d벡터를 외적해주는 함수 -> 분모
double inner_product(pair<double, double> v_rel, pair<double, double> d_rel) {
    double inner = v_rel.first * d_rel.first + v_rel.second * d_rel.second;
    return inner;
}

double TTC_master(pair<double, double> car, pair<double, double > object,
                pair<double, double> car_heading, pair<double, double> object_heading, double car_velocity, double object_velocity) {
    double v_rel_x = (object_heading.first - car_heading.first) * car_velocity;
    double v_rel_y = (object_heading.second - car_heading.second) * car_velocity;
    pair<double, double> v_rel = make_pair(v_rel_x, v_rel_y);
    pair<double, double> d_rel = make_pair(object.first - car.first, object.second - object.second);

    //numerator은 분자, denominator은 분모
    double numerator = distance(car, object);
    double denominator = inner_product(v_rel, d_rel);

    double TTC = numerator / denominator;
    return TTC;
}









