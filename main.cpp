#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <iostream>
#include <dlib/opencv.h>
#include "opencv2/opencv.hpp"
#include <vector>


using namespace std;


dlib::shape_predictor sp;
dlib::frontal_face_detector detector;


vector<int> geteyes(cv::Mat binarypic, vector<double> centre)
{
    double height = binarypic.rows;
    double width = binarypic.cols;
    double centerX=height/2;
    double centerY=width/2;
    vector<double> stor = {0};
    cv::Scalar color;

    bool conIsNone = true;
    int con = 72;

    double k;
    int eyes;
    int x,y;
    double tempDouble;
    int last = 255;
    if ( centre[0]-centerY!=0 ){
        k = (centre[1]-centerX) / (centre[0]-centerY);
        for(int y=0; y<width; y++)
        {
            int x = (y-centerY)*k + centerX;
            if (x>=0 and x<height)
            {
                if (conIsNone==true) {
                    conIsNone = false;
                    con = y;
                }
                color = binarypic.at<cv::Vec2i>(x,y);
                if (color[0]==255)
                {
                    if (last==255){
                        stor.back()++;
                    } else {
                        stor.push_back(1);
                    }
                } else {
                    if (last==0){
                        stor.back()++;
                    } else {
                        stor.push_back(1);
                    }
                    last = color[0];
                }
            }
        }
    } else {
        for (int x=0;x<height;x++){
            color = binarypic.at<cv::Vec2i>(x, centre[0]);
            if (color[0]==255){
                if (last == 255){
                    stor.back()++;
                } else {
                    stor.push_back(1);
                }
            } else {
                if (last==0){
                    stor.back()++;
                } else {
                    stor.push_back(1);
                }
            }
            last = color[0];
        }
        vector<double> blacks = {};
        for (int i=0;i<stor.size();i++){
            if (i < 0 ) {
            }
            if (i%2==1){
                blacks.push_back(stor[i]);
            }
        }
        if (blacks.empty()){
            return vector<int>{(int) round(centerX), (int) round(centerY)};
        }
        eyes = 0; //55 error
        for (int i=1;i<stor.size();i+=2){
            if (*max_element(blacks.begin(),blacks.end())==stor[i]) {
                eyes = i;
                break;
            }
        }
        tempDouble = 0;
        for (int kk=0;kk<eyes;kk++){
            tempDouble = tempDouble + stor[kk];
        }
        tempDouble = tempDouble + round(*max_element(blacks.begin(),blacks.end()) / 2);
        x = round(tempDouble);
        return vector<int>{x, static_cast<int>(round(centre[0]))};

    }
    vector<double> blacks = {};
    for (int i=0;i<stor.size();i++){
        if (i < 0 ) {
        }
        if (i%2==1){
            blacks.push_back(stor[i]);
        }
    }
    if (blacks.empty()){
        return vector<int>{(int) round(centerX), (int) round(centerY)};
    }
    eyes=0;
    for (int i=1;i<stor.size();i+=2){
        if (*max_element(blacks.begin(),blacks.end())==stor[i]) {
            eyes = i;
            break;
        }
    }
    tempDouble = 0;
    for (int kk=0;kk<eyes;kk++){
        tempDouble=tempDouble + stor[kk];
    }
    tempDouble=tempDouble + round(*max_element(blacks.begin(),blacks.end()) / 2);
    tempDouble=tempDouble + con;
    y = round(tempDouble);
    x = round( (y-centerY)*k+centerX );
    return vector<int>{x,y};
}


// extern "C"
vector<double> detectExt(cv::Mat frame)
{
    double ratE;
    cv::Mat img = frame;
    vector<double> ret;
    cv::Mat dst;
    cv::cvtColor(frame, dst, cv::COLOR_BGR2GRAY);

    dlib::array2d<dlib::bgr_pixel> dimg;
    dlib::assign_image(dimg, dlib::cv_image<uchar>(dst));

    std::vector<dlib::rectangle> dets = detector(dimg);
    if (dets.size() == 0)
    {
        ret.push_back(-1);
        ret.push_back(-1);
        ret.push_back(-1);
        ret.push_back(-1);
        return ret;
    }
    std::vector<dlib::full_object_detection> shapes;
    for (int i = 0; i < dets.size(); i++)
    {
        dlib::full_object_detection shape = sp(dimg, dets[i]);
        shapes.push_back(shape);
    }
    
    if (shapes.empty())
    {
        ret.push_back(-1);
        ret.push_back(-1);
        ret.push_back(-1);
        ret.push_back(-1);
        return ret;
    }
    else
    {
        dlib::full_object_detection face = shapes[0];
        double res[2][4][2] = {};
        res[0][0][0] = face.part(37).x();
        res[0][0][1] = face.part(37).y();
        res[0][1][0] = face.part(38).x();
        res[0][1][1] = face.part(38).y();
        res[0][2][0] = face.part(40).x();
        res[0][2][1] = face.part(40).y();
        res[0][3][0] = face.part(41).x();
        res[0][3][1] = face.part(41).y();
        res[1][0][0] = face.part(43).x();
        res[1][0][1] = face.part(43).y();
        res[1][1][0] = face.part(44).x();
        res[1][1][1] = face.part(44).y();
        res[1][2][0] = face.part(46).x();
        res[1][2][1] = face.part(46).y();
        res[1][3][0] = face.part(47).x();
        res[1][3][1] = face.part(47).y();
        cv::Mat sli_gray;
        cv::Mat sli;
        cv::Mat binary;
        cv::Mat temp;
        cv::Rect m_select;
        cv::Mat sli_blur;
        double e1[4][2] = {};
        double maxX, maxY, minX, minY;
        double dX,dY;
        double centerX,centerY,xieLv;
        for (int i = 0; i < 4; i++)
        {
            e1[i][0] = res[0][i][0];
            e1[i][1] = res[0][i][1];
        }
        maxX = max(e1[0][0], max(e1[1][0], max(e1[2][0], e1[3][0])));
        minX = min(e1[0][0], min(e1[1][0], min(e1[2][0], e1[3][0])));
        maxY = max(e1[0][1], max(e1[1][1], max(e1[2][1], e1[3][1])));
        minY = min(e1[0][1], min(e1[1][1], min(e1[2][1], e1[3][1])));
        dX = maxX - minX;
        dY = maxY - minY;
        maxX = maxX + dX * 0.5;
        minX = minX - dX * 0.5;
        maxY = maxY + dY * 0.5;
        minY = minY - dY * 0.5;
        m_select = cv::Rect(minX, minY, maxY-minY, maxX-minX);
        sli = img(m_select);
        cv::cvtColor(sli, temp, cv::COLOR_BGR2GRAY);
        temp.copyTo(sli_gray);
        cv::threshold(sli_gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        temp.copyTo(binary);
        cv::medianBlur(sli_gray, temp, 1);
        temp.copyTo(sli_blur);
        vector<cv::Vec4f> circles;
        cv::HoughCircles(sli_blur, circles, cv::HOUGH_GRADIENT, 1, 250, 13, 2, 10, (maxY - minY));
        vector<int> xy;
        if (circles.empty())
        {
            // 使用
            xy = geteyes(
                    binary,
                    vector<double>{
                        round(sli.rows/2),
                        round(sli.cols/2)+1
                    }
                );
            ret.push_back(xy[0]+minX);
            ret.push_back(xy[1]+minY);
        }
        else
        {
            cv::Vec4f thisCircle;
            thisCircle = circles[0];
            xy = geteyes(
                    binary,
                    vector<double>{
                        thisCircle[0],
                        thisCircle[1]
                    }
                );
            ret.push_back(xy[0]+minX);
            ret.push_back(xy[1]+minY);
        }
        ///////////////// 右耳

        for (int i = 0; i < 4; i++)
        {
            e1[i][0] = res[1][i][0];
            e1[i][1] = res[1][i][1];
        }
        maxX = max(e1[0][0], max(e1[1][0], max(e1[2][0], e1[3][0])));
        minX = min(e1[0][0], min(e1[1][0], min(e1[2][0], e1[3][0])));
        maxY = max(e1[0][1], max(e1[1][1], max(e1[2][1], e1[3][1])));
        minY = min(e1[0][1], min(e1[1][1], min(e1[2][1], e1[3][1])));
        dX = maxX - minX;
        dY = maxY - minY;
        maxX = maxX + dX * 0.5;
        minX = minX - dX * 0.5;
        maxY = maxY + dY * 0.5;
        minY = minY - dY * 0.5;
        m_select = cv::Rect(minX, minY, maxY-minY, maxX-minX);
        sli = img(m_select);
        cv::cvtColor(sli, temp, cv::COLOR_BGR2GRAY);
        temp.copyTo(sli_gray);
        cv::threshold(sli_gray, temp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        temp.copyTo(binary);
        cv::medianBlur(sli_gray, temp, 1);
        temp.copyTo(sli_blur);
        vector<cv::Vec4f> c;
        circles = c;
        cv::HoughCircles(sli_blur, circles, cv::HOUGH_GRADIENT, 1, 250, 13, 2, 10, (maxY - minY));
        vector<int> xy1;
        xy = xy1;
        if (circles.empty())
        {
            xy = geteyes(
                    binary,
                    vector<double>{
                        round(sli.rows/2),
                        round(sli.cols/2)+1
                    }
                );
            ret.push_back(xy[0]+minX);
            ret.push_back(xy[1]+minY);
        }
        else
        {
            cv::Vec4f thisCircle;
            thisCircle = circles[0];
            xy = geteyes(
                    binary,
                    vector<double>{
                        thisCircle[0],
                        thisCircle[1]
                    }
                );
            ret.push_back(xy[0]+minX);
            ret.push_back(xy[1]+minY);
        }
        return ret;
    }
    
}

void init(string path){
    // string path = "./shape_predictor_68_face_landmarks.dat";
    detector = dlib::get_frontal_face_detector();
    dlib::deserialize(path) >> sp;
}

void destroy(){
    free(&detector);
    free(&sp);
}

// int main(){return 0;}

int main()
{
    init("./shape_predictor_68_face_landmarks.dat");
    
    
    while (1){
        string fn;
        cin>>fn;
        cv::Mat frame = cv::imread(fn);
        vector<double> r = detectExt(frame);  
        cout << r[0] << ";;" << r[1] << ";;"<<r[2]<<";;"<<r[3] << endl;  
    }
    // cv::Mat frame = cv::imread("./1.png");
    // vector<double> r = detectExt(frame);
    // cout << r[0] << ";;" << r[1] << ";;"<<r[2]<<";;"<<r[3] << endl;
    return 0;
}
