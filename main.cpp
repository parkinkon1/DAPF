#include "movingLidar.h"

#include <iostream>
#include <ctime>

int main() {

//    Mat input = imread("/Users/parkkyuyeol/Downloads/road4.png", IMREAD_COLOR);
//    if (input.empty()) {
//        cerr << "Image load failed" << endl;
//        return 0;
//    }
//
//    int width = input.cols;
//    int height = input.rows;
//    resize(input, input, Size(width, height));
//
//    imshow("origin", input);


    Mat input;
    getMovingFrame(input, 3);

//    detection(input, input);


//
//    VideoCapture cap("/Users/parkkyuyeol/Downloads/final_lane2.mp4");
//    if (!cap.isOpened()) {
//        cerr << "Video open failed" << endl;
//        return 0;
//    }
//
//    double fps = cap.get(CAP_PROP_FPS);
//    int delay = cvRound(1000 / fps);
//
//    Mat table = Mat::zeros(30, 200, CV_8UC3);
//    Mat table_show, frame_show;
//    clock_t start, end;
//    double result = 0, sum = 0;
//    int timecount = 0;
//
//    Mat frame;
//    while (true) {
//        cap >> frame;
//        if (frame.empty()) break;
//
//        cap.set(CAP_PROP_POS_FRAMES, cap.get(CAP_PROP_POS_FRAMES) + 3);
//
//// -----------------------------------------------------------------------------------------------
//        start = clock();
//
//        laneDetection(frame, frame);
////        planningIntegralPath(frame, frame);
//
//
//        end = clock();
//// -----------------------------------------------------------------------------------------------
//
//
//        sum += (double)(end - start); timecount++;
//        if (timecount % 5 == 0) {
//            result = sum / 5; sum = 0;
//        }
//        table_show = table.clone();
//        putText(table_show, "Time(s) : " + to_string(result/CLOCKS_PER_SEC), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
//        imshow("Execution Time", table_show);
//        waitKey(1);
//    }


    return 0;
}