//
// Created by 박규열 on 01/10/2019.
//

#include "movingLidar.h"
#include "preProcessing.h"


Object::Object(Point2d _position, Point2d _direction, double _r) {
    this->position = _position;
    this->direction = _direction;
    this->r = _r;
}

Object::~Object() {
    cout << "object extincted... position / direction / r : " << position << " " << direction << " " << r << endl;
}



double DAPF::potentialByObject(double get_x, double get_y) {
    double alpha = 10;
    double z = 0;
    double distance = 0;
    double x, y, r;

    for (auto & object : objects_list) {
        x = object->position.x;
        y = object->position.y;
        r = object->r;

        // 주행 보정을 위한 좌표점인 경우
        if (r == -1) {
            distance = sqrt(pow(get_x - x, 2) + pow(get_y - y, 2));
            z += 255 * exp(-0.01 * pow(distance, 2));
        }
            // 정적 물체인 경우
        else if (abs(object->direction.x) <= 1 && abs(object->direction.y) <= 1) {
            distance = sqrt(pow(get_x - x, 2) + pow(get_y - y, 2));
            // 물체 내부에 있는 경우
            if (distance <= r) {
                z = 255;
                break;
            }
                // 물체 외부에 있는 경우
            else {
                z += 255 * alpha / abs(distance - r);
            }
        }
            // 동적 물체인 경우
        else {
            double z1, z2, z3, z4, z5; // x편미분, y편미분값의 합
            double diff_x_1, diff_x_2, diff_x_3, diff_x_4, diff_x_5; // x편미분
            double diff_y_1, diff_y_2, diff_y_3, diff_y_4, diff_y_5;
            double pt1_x, pt1_y, pt2_x, pt2_y, pt3_x, pt3_y, pt4_x, pt4_y;

            double x_v = object->direction.x;
            double y_v = object->direction.y;

            vector<double> zs; // x편미분값, y편미분값의 합
            vector<double> diffs_x; // x편미분값
            vector<double> diffs_y; // y편미분값
            vector<double> pt_x; // 방향벡터에 의한 예측 경로상의 퍼텐셜 x좌표
            vector<double> pt_y; // 방향벡터에 의한 예측 경로상의 퍼텐셜 y좌표

            double z_sum = 0;
            double diff_x_sum = 0;
            double diff_y_sum = 0;

            distance = sqrt(pow(get_x - x, 2) + pow(get_y - y, 2)); // 차량과 물체와의 거리

            if (distance >= 0 && distance < 200) { // 차량과 물체와의 거리에 따른 방향벡터의 가중치를 부여합니다.
                x_v = x_v;
                y_v = y_v;
            }
            if (distance >= 200 && distance < 400) {
                x_v = 1.1*x_v;
                y_v = 1.1*y_v;
            }
            if (distance >= 400 && distance < 600) {
                x_v = 1.2*x_v;
                y_v = 1.2*y_v;
            }
            if (distance >= 600 && distance < 800) {
                x_v = 1.3*x_v;
                y_v = 1.3*y_v;
            }
            if (distance >= 800) {
                x_v = 1.4*x_v;
                y_v = 1.4*y_v;
            }

            int peaknum = (int)((sqrt(pow(x_v, 2) + pow(y_v, 2))) / (r)); // 예측 경로상에 추가할 퍼텐셜 함수의 개수. 방향벡터와 물체의 위치벡터 사이를 물체의 반지름으로 나눕니다.

            for (int j = 0; j <= peaknum; j++)
            {
                pt_x.push_back(x + (x_v * j / peaknum)); // pt_x 벡터에 만들어질 수 있는 퍼텐셜 함수의 x값을 저장합니다.
                pt_y.push_back(y + (y_v * j / peaknum)); // pt_y 벡터에 만들어질 수 있는 퍼텐셜 함수의 y값을 저장합니다.

            }

            zs.insert(zs.begin(), (100) * exp(-(0.005 / ((1.0) * r))*(pow(get_x - x, 2) + pow(get_y - y, 2))));

            for (int k = 1; k <= peaknum; k++)
            {
//                cout << "k : " << k << endl;
                zs.push_back((100 + 10 * k) * exp(-(0.005 / ((1.0 + 0.1*k) * r))*(pow(get_x - pt_x[k], 2) + pow(get_y - pt_y[k], 2)))); //z값은 (q,p)값에 미치는 물체의 총 퍼텐셜의 합

            }

            diffs_x.insert(diffs_x.begin(), (get_x - x) * (-1 * exp(-(0.005 / r) * (pow(get_x - x, 2) + pow(get_y - y, 2))))); // diff_x는 퍼텐셜 함수의 x편미분 값
            diffs_y.insert(diffs_y.begin(), (get_y - y) * (-1 * exp(-(0.005 / r) * (pow(get_y - y, 2) + pow(get_x - x, 2))))); // diff_y는 퍼텐셜 함수의 y편미분 값

            for (int i = 1; i <= peaknum; i++)
            {
                diffs_x.push_back((1.0 + 0.1*i)*(get_x - pt_x[i]) * (-1 * exp(-(0.005 / ((1.0 + 0.1*i) * r)) * (pow(get_x - pt_x[i], 2) + pow(get_y - pt_y[i], 2)))));
                diffs_y.push_back((1.0 + 0.1*i)*(get_y - pt_y[i]) * (-1 * exp(-(0.005 / ((1.0 + 0.1*i) * r)) * (pow(get_y - pt_y[i], 2) + pow(get_x - pt_x[i], 2)))));

            }

            for (int i = 0; i <= peaknum; i++)
            {
                z_sum += zs[i];
                diff_x_sum += diffs_x[i];
                diff_y_sum += diffs_y[i];
            }

            zs.clear();
            diffs_x.clear();
            diffs_y.clear();
            pt_x.clear();
            pt_y.clear();

            z += z_sum;
        }
    }
    return z;
}
double DAPF::potentialByLane(double get_x, double get_y) {
    double alpha = 20;
    double distance_L = 0, distance_R = 0, distance_dotted = 0;
    double z;
    double x_L = L[(int)get_y].first;
    double x_R = R[(int)get_y].first;
    double x_dotted = dotted[(int)get_y].first;
    // 왼쪽 차선 퍼텐셜 계산
    distance_L = get_x - x_L;
    if (distance_L == 0) z = 0;
    else z = 255 * alpha / abs(distance_L);
    // 오른쪽 차선 퍼텐셜 계산
    distance_R = get_x - x_R;
    if (distance_R == 0) z = 0;
    else z += 255 * alpha / abs(distance_R);
    // 점선 차선 퍼텐셜 계산
    distance_dotted = get_x - x_dotted;
    z += 255 * exp(-0.01 * pow(distance_dotted, 2));

    return z;
}
Point2d DAPF::diffByObject(double get_x, double get_y) {
    double alpha = 700;
    double distance = 0;
    double dx = 0, dy = 0;
    double diff_x = 0, diff_y = 0;
    double x, y, r;
//        cout << "ob : " <<  objects_list.capacity() << endl;
        cout << "objects_list : " << objects_list.size() << endl;
        cout << "ob : " <<  objects_list.capacity() << endl;
    if (this->objects_list.empty()) return Point2d(0, 0);
    for (auto & object : objects_list) {
        x = object->position.x;
        y = object->position.y;
        r = object->r;
        dx = get_x - x;
        dy = get_y - y;

        // 주행 보정을 위한 좌표점인 경우
        if (r == -1) {
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            if (distance <= 1) continue;
            diff_x += dx * 20 / pow(0.5 * pow(distance, 2), 3/2);
            diff_y += dy * 20 / pow(0.5 * pow(distance, 2), 3/2);
        }
        // 정적 물체인 경우
        else if (abs(object->direction.x) <= 1 && abs(object->direction.y) <= 1) {
            // 거리 계산
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            // 오브젝트 힘 계산
            if (distance <= r) {
                diff_x = 0;
                diff_y = 0;
                break;
            }
            else if (distance - r <= 1) {
                diff_x += (dx - r * (dx / distance)) * alpha;
                diff_y += (dy - r * (dy / distance)) * alpha;
            }
            else {
                diff_x += (dx - r * (dx / distance)) * alpha / pow(0.5 * pow(distance - r, 2), 3/2);
                diff_y += (dy - r * (dy / distance)) * alpha / pow(0.5 * pow(distance - r, 2), 3/2);
            }
        }
        // 동적 물체인 경우
        else {
            // 거리 계산
            distance = sqrt(pow(dx, 2) + pow(dy, 2));
            double collision_time = distance / 10;

            x += object->direction.x * collision_time;
            y += object->direction.y * collision_time;
            dx = get_x - x;
            dy = get_y - y;
            distance = sqrt(pow(dx, 2) + pow(dy, 2));

            // 오브젝트 힘 계산
            if (distance <= r) {
                diff_x = 0;
                diff_y = 0;
                break;
            }
            else if (distance - r <= 1) {
                diff_x += (dx - r * (dx / distance)) * alpha;
                diff_y += (dy - r * (dy / distance)) * alpha;
            }
            else {
                diff_x += (dx - r * (dx / distance)) * alpha / pow(0.8 * pow(distance - r, 2), 3/2);
                diff_y += (dy - r * (dy / distance)) * alpha / pow(0.8 * pow(distance - r, 2), 3/2);
            }




//            double z1, z2, z3, z4, z5; // x편미분, y편미분값의 합
//            double diff_x_1, diff_x_2, diff_x_3, diff_x_4, diff_x_5; // x편미분
//            double diff_y_1, diff_y_2, diff_y_3, diff_y_4, diff_y_5;
//            double pt1_x, pt1_y, pt2_x, pt2_y, pt3_x, pt3_y, pt4_x, pt4_y;
//
//            double x_v = object->direction.x;
//            double y_v = object->direction.y;
//
//            vector<double> zs; // x편미분값, y편미분값의 합
//            vector<double> diffs_x; // x편미분값
//            vector<double> diffs_y; // y편미분값
//            vector<double> pt_x; // 방향벡터에 의한 예측 경로상의 퍼텐셜 x좌표
//            vector<double> pt_y; // 방향벡터에 의한 예측 경로상의 퍼텐셜 y좌표
//
//            double z_sum = 0;
//            double diff_x_sum = 0;
//            double diff_y_sum = 0;
//
//            distance = sqrt(pow(get_x - x, 2) + pow(get_y - y, 2)); // 차량과 물체와의 거리
//
//            if (distance >= 0 && distance < 200) { // 차량과 물체와의 거리에 따른 방향벡터의 가중치를 부여합니다.
//                x_v = x_v;
//                y_v = y_v;
//            }
//            if (distance >= 200 && distance < 400) {
//                x_v = 1.1*x_v;
//                y_v = 1.1*y_v;
//            }
//            if (distance >= 400 && distance < 600) {
//                x_v = 1.2*x_v;
//                y_v = 1.2*y_v;
//            }
//            if (distance >= 600 && distance < 800) {
//                x_v = 1.3*x_v;
//                y_v = 1.3*y_v;
//            }
//            if (distance >= 800) {
//                x_v = 1.4*x_v;
//                y_v = 1.4*y_v;
//            }
//
//            int peaknum = (int)((sqrt(pow(x_v, 2) + pow(y_v, 2))) / (r)); // 예측 경로상에 추가할 퍼텐셜 함수의 개수. 방향벡터와 물체의 위치벡터 사이를 물체의 반지름으로 나눕니다.
//
//            for (int j = 0; j <= peaknum; j++)
//            {
//                pt_x.push_back(x + (x_v * j / peaknum)); // pt_x 벡터에 만들어질 수 있는 퍼텐셜 함수의 x값을 저장합니다.
//                pt_y.push_back(y + (y_v * j / peaknum)); // pt_y 벡터에 만들어질 수 있는 퍼텐셜 함수의 y값을 저장합니다.
//
//            }
//
//            zs.insert(zs.begin(), (100) * exp(-(0.005 / ((1.0) * r))*(pow(get_x - x, 2) + pow(get_y - y, 2))));
//
//            for (int k = 1; k <= peaknum; k++)
//            {
////                cout << "k : " << k << endl;
//                zs.push_back((100 + 10 * k) * exp(-(0.005 / ((1.0 + 0.1*k) * r))*(pow(get_x - pt_x[k], 2) + pow(get_y - pt_y[k], 2)))); //z값은 (q,p)값에 미치는 물체의 총 퍼텐셜의 합
//
//            }
//
//            diffs_x.insert(diffs_x.begin(), (get_x - x) * (-1 * exp(-(0.005 / r) * (pow(get_x - x, 2) + pow(get_y - y, 2))))); // diff_x는 퍼텐셜 함수의 x편미분 값
//            diffs_y.insert(diffs_y.begin(), (get_y - y) * (-1 * exp(-(0.005 / r) * (pow(get_y - y, 2) + pow(get_x - x, 2))))); // diff_y는 퍼텐셜 함수의 y편미분 값
//
//            for (int i = 1; i <= peaknum; i++)
//            {
//                diffs_x.push_back((1.0 + 0.1*i)*(get_x - pt_x[i]) * (-1 * exp(-(0.005 / ((1.0 + 0.1*i) * r)) * (pow(get_x - pt_x[i], 2) + pow(get_y - pt_y[i], 2)))));
//                diffs_y.push_back((1.0 + 0.1*i)*(get_y - pt_y[i]) * (-1 * exp(-(0.005 / ((1.0 + 0.1*i) * r)) * (pow(get_y - pt_y[i], 2) + pow(get_x - pt_x[i], 2)))));
//
//            }
//
//            for (int i = 0; i <= peaknum; i++)
//            {
//                z_sum += zs[i];
//                diff_x_sum += diffs_x[i];
//                diff_y_sum += diffs_y[i];
//            }
//
//            zs.clear();
//            diffs_x.clear();
//            diffs_y.clear();
//            pt_x.clear();
//            pt_y.clear();
//
//            diff_x += -diff_x_sum;
//            diff_y += -diff_y_sum;
        }
    }
    return Point2d(diff_x, diff_y);
}
Point2d DAPF::diffByLane(double get_x, double get_y) {
    double alpha = 120, alpha_dotted = 1;
    double distance_L = 0, distance_R = 0, distance_dotted = 0;
    double diff_x = 0, diff_y = 0;
    double x_L = L[(int)get_y].first;
    double x_R = R[(int)get_y].first;
    double x_dotted = dotted[(int)get_y].first;
    // 왼쪽 차선 힘 계산
    distance_L = get_x - x_L;
    if (abs(distance_L) <= 1) diff_x += -255 * alpha;
    else if (distance_L < 0) diff_x += -255 * alpha / pow(distance_L, 2);
    else diff_x += 255 * alpha / pow(distance_L, 2);
    // 오른쪽 차선 힘 계산
    distance_R = get_x - x_R;
    if (abs(distance_R) <= 1) diff_x += 255 * alpha;
    else if (distance_R < 0) diff_x += -255 * alpha / pow(distance_R, 2);
    else diff_x += 255 * alpha / pow(distance_R, 2);
    // 점선 차선 힘 계산
    distance_dotted = get_x - x_dotted;
    if (abs(distance_dotted) <= 1) diff_x += alpha_dotted * distance_dotted * 255;
    else diff_x += alpha_dotted * distance_dotted * 2 * exp(-0.001 * pow(distance_dotted, 2));

    return Point2d(diff_x, -10);
}

// 3차식의 계수와 원하는 y 좌표를 대입하면 steer 반환
double DAPF::getSteerwithCurve(vector<double> &ans, int y) {

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
bool DAPF::drawCar(Mat &input, Mat &output, Point2d &point_car, float heading, Mat &potential, Point2d &collisionPoint) {
    output = input.clone();
    float descent_rad = heading + CV_PI/2;
    float heading_deg = heading * 180 / CV_PI;

    float carCenX = point_car.x;
    float carCenY = point_car.y;

    Point2f carCen(carCenX, carCenY), pts1[4];
    Size2f carSize(car_height, car_width);

    cv::RotatedRect rotatedRectangle(carCen, carSize, heading_deg); //차량 모양 형성(중심점, 각도)
    rotatedRectangle.points(pts1);

    for (int i = 0; i < 4; i++) {
        line(output, pts1[i], pts1[(i + 1) % 4], Scalar(255, 0, 0), 2);
    }
    ////////////////////////////////////////////////////////////

    float rot_x, rot_y;
    float rot_cos = cos(-heading), rot_sin = sin(-heading);
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

            if (rot_x <= 0 || rot_y <= 0 || (int)rot_x >= width || (int)rot_y >= height) {
                continue;
            }
            else if (potentialData[(int)rot_y * width + (int)rot_x] <= maxPotential) {
                cout << "High Potential !!!" << endl;
                collisionPoint = Point2d(rot_x, rot_y);

                return true;
            }
        }
    }
    return false;
}

// 점의 방정식을 곡선으로 근사
void DAPF::curveFitting(Mat &input, Mat &output, vector<Point2d> &route, vector<double> &ans, int start, int end) {

    if (route.size() <= 3) {
        cerr << "Lack of Points to fitting !" << endl;
        return;
    }

    vector<pair<double, double>> v;
    for (int i = 0; i < 500; i++) {
        v.push_back(make_pair(route[0].x, route[0].y));
    }
    for (int i = 0; i < 500; i++) {
        v.push_back(make_pair(route[2].x, route[2].y));
    }
//    for (int i = 0; i < 5; i++) {
//        v.push_back(make_pair(route[1].x, route[1].y));
//    }
//    for (int i = 0; i < 5; i++) {
//        v.push_back(make_pair(route[2].x, route[2].y));
//    }
//    for (int i = 0; i < 5; i++) {
//        v.push_back(make_pair(route[3].x, route[3].y));
//    }
//    for (int i = 0; i < 5; i++) {
//        v.push_back(make_pair(route[4].x, route[4].y));
//    }
    for (int i = 0; i < 20; i++) {
        v.push_back(make_pair(route.back().x, route.back().y));
    }

    for (auto & i : route) {
        v.push_back(make_pair(i.x, i.y));
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
    for (int i = end; i >= start; i -= 1) {
        double x1 = 0, x2 = 0;
        for (int j = 0; j < ans_size; j++) {
            x1 += coef[j] * pow(i, ans_size - 1 - j);
            x2 += coef[j] * pow(i - 1, ans_size - 1 - j);
        }
        line(output, Point(x1, i), Point(x2, i - 1), Scalar(0, 0, 255), 2);
    }
}

//input : 조향각, 속력, 이동하는 데 걸리는 시간
//init_str [rad], V_r [m/s], dt [sec], heading [rad]
//output : 절대좌표계(UTM좌표계)에서의 변위(x,y변화량)
vector<double> DAPF::PredictCar(double init_str, double V_r, double dt1, double heading) {
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


// 차량과 물체의 거리 차이 |d| 를 알려주는 함수 -> 분자
double DAPF::distance(Point2d car, Point2d object) {
    double x_distance = car.x > object.x ? car.x - object.x : object.x - car.x;
    double y_distance = car.y > object.y ? car.y - object.y : object.y - car.y;
    double total_distance = sqrt(x_distance * x_distance + y_distance * y_distance);
    return total_distance;
}
// v벡터와 d벡터를 외적해주는 함수 -> 분모
double DAPF::inner_product(Point2d v_rel, Point2d d_rel) {
    double inner = v_rel.x * d_rel.x + v_rel.y * d_rel.y;
    return inner;
}
double DAPF::TTC_master(Point2d car, Point2d object, Point2d car_heading, Point2d object_heading, double car_velocity, double object_velocity) {
    double v_rel_x = (object_heading.x - car_heading.x) * car_velocity;
    double v_rel_y = (object_heading.y - car_heading.y) * car_velocity;
    Point2d v_rel(v_rel_x, v_rel_y);
    Point2d d_rel(object.x - car.x, object.y - object.y);

    //numerator은 분자, denominator은 분모
    double numerator = distance(car, object);
    double denominator = inner_product(v_rel, d_rel);

    double TTC = numerator / denominator;
    return TTC;
}


DAPF::DAPF(int width, int height) {
    this->width = width; this->height = height;
    this->road = Mat(height, width, CV_8UC3, Scalar(255, 255, 255));
    this->dangerous = Mat(height, width, CV_8UC1, Scalar(255));

    this->car_width = 80;
    this->car_height = 120;
}

DAPF::~DAPF() {
    cout << "DAPF deleted ~~~" << endl;
}

// 원하는 차선 영역을 지정한다
void DAPF::initLane(vector<pair<double, double>> _L, vector<pair<double, double>> _R, vector<pair<double, double>> _dotted) {
    this->L = _L; this->R = _R; this->dotted = _dotted;
}

// 원하는 정적 또는 동적 물체를 지정한다
void DAPF::pushObject(Point2d _position, Point2d _direction, double _r) {
    Object *p = new Object(_position, _direction, _r);
    objects_list.push_back(p);
}

// 차선 및 물체가 있는 영역을 그려 반환한다
void DAPF::drawDangerous() {
    // 차선 그리기
    for (int i = 0; i < (int)L.size(); i += 10) {
        circle(dangerous, Point2d(L[i].first, L[i].second), 5, Scalar(0), -1);
    }
    for (int i = 0; i < (int)R.size(); i += 10) {
        circle(dangerous, Point2d(R[i].first, R[i].second), 5, Scalar(0), -1);
    }
    for (int i = 0; i < (int)dotted.size(); i += 30) {
        circle(dangerous, Point2d(dotted[i].first, dotted[i].second), 5, Scalar(50), -1);
    }

    // 물체 그리기
    vector<Object*>::iterator iter;
    for (iter = objects_list.begin(); iter != objects_list.end(); iter++) {
        if (abs((*iter)->direction.x) >= 1 || abs((*iter)->direction.y) >= 1)
            circle(dangerous, (*iter)->position, (int)(*iter)->r, Scalar(100), -1);
        else circle(dangerous, (*iter)->position, (int)(*iter)->r, Scalar(0), -1);
    }
}

// 퍼텐셜 필드 및 방향을 그린다
void DAPF::showDAPF(int time) {
    Mat frame_show = this->road.clone();
    int showing_gap = 2;
    cout << "visualization..." << endl;
    double cost = 0;
    for (int i = 0; i < height; i += showing_gap) {
        for (int j = 0; j < width; j += showing_gap) {
            cost = potentialByObject(j, i);
            cost += potentialByLane(j, i);
            circle(frame_show, Point(j, i), showing_gap, Scalar(255 - cost, 255 - cost, 255 - cost), -1);
        }
    }
    cout << "visualizing vectors..." << endl;
    Point2d diff_lane(0, 0), diff_object(0, 0);
    for (int i = 0; i < height; i += showing_gap * 10) {
        for (int j = 0; j < width; j += showing_gap * 10) {
            diff_object = diffByObject(j, i);
            diff_lane = diffByLane(j, i);

            Point2d diff_sum = diff_object + diff_lane;
            double diff_size = sqrt(diff_sum.x * diff_sum.x + diff_sum.y * diff_sum.y);
            if (diff_size >= 30) diff_sum = (diff_sum / diff_size) * 30;
            arrowedLine(frame_show, Point2d(j, i), Point2d(j + diff_sum.x, i + diff_sum.y), Scalar(0, 0, 255), 2, 5, 0, 0.1);
        }
    }
    cout << "visualization finished" << endl;

    imshow("DAPF Visualization", frame_show);
    waitKey(time);
}

void DAPF::setPosition(Point2d position) {
    this->position = position;
    this->now_point = position;
    route.clear();
    route_car.clear();
    route.push_back(position);
    route_car.push_back(position);
}
void DAPF::setDirection(Point2d direction) {
    this->direction = direction;
    this->now_direction = direction;
}

// path planning, 경로 추적하여 충돌 회피를 위한 다음 위치 반환
void DAPF::costTracker() {
    // 차량의 속도 (벡터 크기)
    double speed = 10;
    // 퍼텐셜 반영비율
    double alpha = 1;

    pre_point = now_point;
    pre_direction = now_direction;

    // 퍼텐셜 힘 계산
    Point2d diff_lane(0,0);
    Point2d diff_object(0,0);
    Point2d diff_sum(0, 0);
    diff_lane = diffByLane(pre_point.x, pre_point.y);
    cout << "diff : objects_list size : " << objects_list.size() << endl;
    diff_object = diffByObject(pre_point.x, pre_point.y);
    diff_sum = diff_lane + diff_object;

    /// *** non-holonomic search ***
    double diff_size = sqrt(pow(diff_sum.x, 2) + pow(diff_sum.y, 2));
    double diff_ratio = diff_size / (speed / 8);
    if (diff_ratio >= 1) {
        diff_sum.x /= diff_ratio;
        diff_sum.y /= diff_ratio;
    }
    now_direction.x = pre_direction.x + alpha * diff_sum.x;
    now_direction.y = pre_direction.y + alpha * diff_sum.y;

    // 방향벡터 정규화
    now_direction /= sqrt(pow(now_direction.x , 2) + pow(now_direction.y, 2));
    now_direction *= speed;
    cout << "direction  : " << now_direction << endl;
    // 도착점 계산
    now_point += now_direction;
    cout << "now point : " << now_point << endl;

    /// 프레임을 벗어날 경우, Path Planning 종료
    uchar *dangerous_data = dangerous.data;
    if (now_point.x < 0 || now_point.y < 0 || now_point.x > width || now_point.y > height) {
        Mat frame_show;
        cvtColor(dangerous, frame_show, COLOR_GRAY2BGR);

        cout << "route size : " << route.size() << endl;
        // 경로가 정상적으로 생성된 경우
        if (!route.empty()) {
            double slope = (double)(route[2].x - route[0].x) / (double)(route[0].y - route[2].y);
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
            int degree = 4;
            curveFitting(frame_show, frame_show, route, ans, 0, height);

            imshow("curve fitting", frame_show);
            waitKey();


            /// 차량이 갈 수 있는 경로인지 탐색
            Point2d collisionPoint = Point2d(0, 0);
            vector<Point2d> curve;
            vector<double> next_pos;
            double steer = 0, pre_steer = 0, heading, steer_temp;
            int count = 0, isColl = 0, curveSlide = 0, curveSlidingGap = (int)route.size() / 5;
            // 계획된 경로를 따라가면서, 차선 또는 물체와 부딪히는지 탐색
            while (true) {
//                // curve fitting
//                curve.clear();
//                if (route_car.back().y < route[curveSlide * curveSlidingGap].y) {
//                    while (route_car.back().y < route[curveSlide * curveSlidingGap].y) {
//                        curveSlide++;
//                    }
//                }
//                for (int i = 0; i < curveSlidingGap; i++) {
//                    curve.push_back(route[curveSlide * curveSlidingGap + i]);
////                    if (curveSlide * curveSlidingGap + i > route.size()) {
////                        curve.clear();
////                        curve.assign(route.begin(), route.end());
////                        break;
////                    }
//                }
//                curveFitting(frame_show, frame_show, curve, ans, 0, height);
//                cout << "fitting " << index << " " << curve << endl;
//                imshow("curve fitting", frame_show);
//                waitKey();

                // curve fitting된 함수 식으로부터 현재 가야하는 헤딩 방향 계산
                pre_steer = steer;
                steer = getSteerwithCurve(ans, route_car.back().y);
                cout << "str : " << steer << endl;
                heading = (-CV_PI / 2) + pre_steer;
                // 조향 경계조건 설정
                steer_temp = (steer - pre_steer);
                cout << "dd : " << steer_temp << endl;
                if (steer_temp > 0.3491) steer_temp = 0.3491;
                else if (steer_temp < -0.3491) steer_temp = -0.3491;
                // 다음 차량 위치 추정 및 이동
                next_pos = PredictCar(steer_temp, 5.0,0.01, heading);
                cout << "before: next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                cout << "route_car : " << route_car.back() << endl;
                next_pos[0] += route_car.back().x;
                next_pos[1] += route_car.back().y;
                cout << "after : next_pos : " << next_pos[0] << " " << next_pos[1]<< endl;
                if (next_pos[0] < 0 || next_pos[0] > width || next_pos[1] < 0 || next_pos[1] > height) break;
                route_car.push_back(Point2d(next_pos[0],next_pos[1]));
                next_pos.clear();
                // 차량이 지나가는 경로 그리기
                if (count % 5 == 0) {
                    heading = (-CV_PI / 2) + steer;
                    isColl = drawCar(frame_show, frame_show, route_car.back(), (float)heading, dangerous, collisionPoint);
                } count++;

                // 충돌 상황을 만나면 종료
                if (isColl) {
                    cout << "collide !!" << endl;
                    circle(frame_show, collisionPoint, 10, Scalar(200, 150, 100), 5);

                    // 차량이 지나간 경로 그리기
                    for (int i = 0; i < (int)route_car.size() - 1; i++) {
                        line(frame_show, route_car[i], route_car[i+1], Scalar(255, 0, 255), 2);
                    }
                    for (int i = 0; i < (int)route_car.size(); i++) {
                        circle(frame_show, route_car[i], 3, Scalar(200, 150, 200), -1);
                    }

                    imshow("path planning", frame_show);
                    waitKey();

                    if (objects_list.size() >= 200) break;
                    objects_list.push_back(new Object(collisionPoint, Point2d(0,0), -1)),
                    setPosition(position);
                    setDirection(direction);
                    costTracker();

                    return;
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
            imshow("path planning", frame_show);
            waitKey();

            position = route_car[1];
            direction = route_car[2] - route_car[1];
            direction *= speed / sqrt(pow(direction.x , 2) + pow(direction.y, 2));
            setPosition(position);
            setDirection(direction);
            return;

        }
            // route에 아무것도 저장되지 않은 경우
        else { cout << "Lane not detected ㅜㅜ" << endl; }

        return;
    }
        /// 프레임을 벗어나지 않은 경우 path planning 경로 계속 탐색
    else {
        cout << "searching next position..." << endl;
        route.push_back(Point2d(now_point.x, now_point.y));
        costTracker();
        return;
    }

}




void DAPF::simulation(Point2d startPoint, Point2d startDirection) {
    Mat frame_show;
    Mat frame(height, width, CV_8UC3, Scalar(255, 255, 255));
    vector<Point2d> simulate;

    // 차선 그리기
    for (int i = 0; i < (int)L.size(); i += 10) {
        circle(frame, Point2d(L[i].first, L[i].second), 5, Scalar(0, 0, 0), -1);
    }
    for (int i = 0; i < (int)R.size(); i += 10) {
        circle(frame, Point2d(R[i].first, R[i].second), 5, Scalar(0, 0, 0), -1);
    }
    for (int i = 0; i < (int)dotted.size(); i += 30) {
        circle(frame, Point2d(dotted[i].first, dotted[i].second), 5, Scalar(50, 50, 50), -1);
    }

    // 물체 그리기
    vector<Object*>::iterator iter;
    for (iter = objects_list.begin(); iter != objects_list.end(); iter++) {
        if (abs((*iter)->direction.x) >= 1 || abs((*iter)->direction.y) >= 1 || (*iter)->r == -1)
            continue;
        else circle(frame, (*iter)->position, (int)(*iter)->r, Scalar(255, 10, 10), -1);
    }

    frame_show = frame.clone();
    for(iter = objects_list.begin(); iter != objects_list.end(); iter++) {
        if (abs((*iter)->direction.x) >= 1 || abs((*iter)->direction.y) >= 1) {
            circle(frame_show, (*iter)->position, (int)(*iter)->r, Scalar(100, 100, 100), -1);
            cout << "dynamic ob drawing1" << endl;
        }
    }



    // 파일 쓰기 준비
    std::ofstream out("pedestrian_dynamic.txt");
    std::string s;
    if (out.is_open()) {
        out << "position.x position.y direction.x direction.y ob_x ob_y ob_dx ob_dy\n";
    }

    double ob_x = 0, ob_y = 0, ob_dx = 0, ob_dy = 0;
    setPosition(startPoint);
    setDirection(startDirection);
    cout << "simulation start" << endl;
    while(position.x >= 0 || position.x <= width || position.y >= 0 || position.y <= height) {
        simulate.push_back(position);

        double heading_deg = atan(direction.y / (direction.x == 0 ? 1 : direction.x)) * 180 / CV_PI;

        Point2f carCen(position.x, position.y), pts1[4];
        Size2f carSize(car_height, car_width);

        cv::RotatedRect rotatedRectangle(carCen, carSize, (float)heading_deg); //차량 모양 형성(중심점, 각도)
        rotatedRectangle.points(pts1);

        for (int i = 0; i < 4; i++) {
            line(frame, pts1[i], pts1[(i + 1) % 4], Scalar(255, 0, 0), 2);
        }

        costTracker();

        // 주행 보정을 위해 추가했던 점들 삭제
        for(iter = objects_list.begin();  iter != objects_list.end(); ) {
            if ((*iter)->r == -1) {
                delete *iter;
                iter = objects_list.erase(iter);
            }
            else iter++;
        }

        // 동적 물체 이동
        frame_show = frame.clone();
        for(iter = objects_list.begin(); iter != objects_list.end(); iter++) {
            if (abs((*iter)->direction.x) >= 1 || abs((*iter)->direction.y) >= 1) {
                (*iter)->position = (*iter)->position + (*iter)->direction;
                circle(frame_show, (*iter)->position, (int)(*iter)->r, Scalar(100, 100, 100), -1);
                cout << "dynamic ob drawing" << endl;
                ob_x = (*iter)->position.x;
                ob_y = (*iter)->position.y;
                ob_dx = (*iter)->direction.x;
                ob_dy = (*iter)->direction.y;
            }
        }


        if (out.is_open()) {
            out << to_string(position.x) << " " << to_string(position.y) << " " << to_string(direction.x) << to_string(direction.y) << " "
            << to_string(ob_x) << " " << to_string(ob_y) << " " << to_string(ob_dx) << " " << to_string(ob_dy) << "\n";
        }


        imshow("simulation!!!!", frame_show);
        waitKey();



    }

    for (int i = 0; i < (int)simulate.size() - 1; i++) {
        line(frame_show, simulate[i], simulate[i+1], Scalar(0, 0, 255), 2);
    }
    for (int i = 0; i < (int)simulate.size() - 1; i++) {
        circle(frame_show, simulate[i], 5, Scalar(150, 100, 150), -1);
    }
    cout << "simulation end" << endl;
    imshow("simulation!!!!", frame_show);
    waitKey();

}

















