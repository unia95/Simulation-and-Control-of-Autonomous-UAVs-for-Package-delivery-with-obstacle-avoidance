#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <ctime>
#include <cmath>
#include "bits/time.h"
#include <vector>
#include <math.h>


#include <opencv2/dnn.hpp>
#include <fstream>
#include <sstream>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h> 
#include <apriltag/common/getopt.h>
#include <apriltag/apriltag_pose.h>
}

#define _USE_MATH_DEFINES
namespace enc = sensor_msgs::image_encodings;
using namespace std::chrono_literals;
using namespace std;
using namespace cv;
using namespace dnn;
using std::placeholders::_1;

// NEURAL NETWORK'S PARAMETERS
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;  // Width of network's input image
int inpHeight = 416; // Height of network's input image
vector<string> classes;

apriltag_family_t *tf = NULL; 
apriltag_detector_t *td = NULL;

//CAMERA PARAMETERS FOR APRILTAG'S DETECTION
double box_tagsize = 0.21; //the tag's size in the AirSim's environment (in meters)
double landing_tagsize = 0.55;
int fx = 336;
int fy = 336;
int cx = 336;
int cy = 188;

Mat lastframe,lastframe2;
double x, y, z;
String qp_ball_detected;
bool isQPBallDetected = false;
int i = 0;
auto box_detected = std_msgs::msg::String();
auto landing_spot_detected = std_msgs::msg::String();

string str, outputFile;
Net net;

class ImgViewSubscriber : public rclcpp::Node {
public:
    ImgViewSubscriber() : Node("img_view") {

        //publishers and subscribers initialization
        s_bottom = this->create_subscription<sensor_msgs::msg::Image>("/airsim_node/PX4/bottom_custom/Scene", 20, std::bind(&ImgViewSubscriber::bottom_callback, this, _1)); 
		s_front = this->create_subscription<sensor_msgs::msg::Image>("/airsim_node/PX4/front_center_custom/Scene", 20, std::bind(&ImgViewSubscriber::front_callback, this, _1));
       
        z_coord = this->create_publisher<std_msgs::msg::String>("z_coord", 1);
        x_coord = this->create_publisher<std_msgs::msg::String>("x_coord", 1);
        y_coord = this->create_publisher<std_msgs::msg::String>("y_coord", 1);

        isBoxDetected = this->create_publisher<std_msgs::msg::String>("isBoxDetected", 1);
        qp_BallDetected = this->create_publisher<std_msgs::msg::String>("qp_BallDetected", 1);
        isLandingSpotDetected = this->create_publisher<std_msgs::msg::String>("isLandingSpotDetected", 1);

        timer_ = this->create_wall_timer(1000ms, std::bind(&ImgViewSubscriber::coordinates_callback, this));

        int n = cuda::getCudaEnabledDeviceCount();
        RCLCPP_INFO(this-> get_logger(),"You have %i Cuda enabled device in your system", n);

        //AprilTag's detector initialization
        try {
            tf= tagStandard41h12_create();
            td = apriltag_detector_create();
            apriltag_detector_add_family(td, tf);
        }

        catch (Exception e) {
            RCLCPP_INFO(this-> get_logger(), "Something went wrong with the AprilTag Detector");
        }

        //Neural Network's initialization
        String modelConfiguration = "/home/unia/dev_ws/src/major_tom_cpp/src/yolov4.cfg"; //modify it by adding your configuration's path
        String modelWeights = "/home/unia/dev_ws/src/major_tom_cpp/src/yolov4.weights";  //modify it by adding your configuration's path
        net = readNetFromDarknet(modelConfiguration, modelWeights);
        string classesFile = "/home/unia/dev_ws/src/major_tom_cpp/src/obj.names";  //modify it by adding your configuration's path
        ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line)) classes.push_back(line);
        net.setPreferableBackend(DNN_BACKEND_CUDA);
        net.setPreferableTarget(DNN_TARGET_CUDA);
    }

    private:
    void bottom_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cuda::GpuMat src, dst, gray_img;
        Mat img, gray;
        cv_bridge::CvImagePtr cv_ptr;
           
        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        src.upload(cv_ptr->image);

            if(src.channels()==3){
                lastframe = cv_ptr->image;
                cuda::cvtColor(src, gray_img, COLOR_BGR2GRAY);
                gray_img.download(gray);

                image_u8_t im = { .width = gray.cols,
                                    .height = gray.rows,
                                    .stride = gray.cols,
                                    .buf = gray.data
                };

                apriltag_detection_info_t info;

                zarray_t *detections = apriltag_detector_detect(td, &im);

                for (int i = 0; i < zarray_size(detections); i++) {
                    apriltag_detection_t *det;
                    zarray_get(detections, i, &det);
                    stringstream ss;
                    ss << det->id;
                    String text = ss.str();
                    //RCLCPP_INFO(this-> get_logger(), "I detected the tag n. %s", text.c_str());

                    if (strcmp(text.c_str(), "5") == 0){
                        info.det = det;
                        info.tagsize = box_tagsize;
                        info.fx = fx;
                        info.fy = fy;
                        info.cx = cx;
                        info.cy = cy;

                        apriltag_pose_t pose;
                        estimate_tag_pose(&info, &pose);

                        string mat;

                        // the following code prints the matrix t and the matrix R, t is the translation matrix and indicates the coordinates of the tag, R is the rotation matrix.

                        /*for (int i = 0; i < pose.R->nrows; i++) {
                            for (int j = 0; j < pose.R->ncols; j++) {
                                mat = mat + to_string(pose.R->data[((i)*(pose.R)->ncols + (j))]);
                            }
                            //RCLCPP_INFO(this-> get_logger(), "Row [%i]: %s", i, mat.c_str());
                            mat.clear();
                            
                        }*/

                        for (int i = 0; i < pose.t->nrows; i++) {
                            for (int j = 0; j < pose.t->ncols; j++) {
                                mat = mat + to_string(pose.t->data[((i)*(pose.t)->ncols + (j))]);
                            }
                                //RCLCPP_INFO(this-> get_logger(),"Row [%i]: %s", i, mat.c_str());
                            mat.clear();
                        }

                        //Box's position and positive detection message (ROS)

                        box_detected.data = "true";
                            
                        x = roundf(pose.t->data[0] * 100)/100;
                        y = roundf(pose.t->data[1] * 100)/100;
                        z = roundf(pose.t->data[2] * 100)/100;

                        //RCLCPP_INFO(this-> get_logger(),"x : %f\ny : %f\nz: %f", y, x, z);
                        isBoxDetected->publish(box_detected);
                    }

                    else if (strcmp(text.c_str(), "95") == 0){
                        info.det = det;
                        info.tagsize = landing_tagsize;
                        info.fx = fx;
                        info.fy = fy;
                        info.cx = cx;
                        info.cy = cy;

                        apriltag_pose_t pose;
                        estimate_tag_pose(&info, &pose);

                        string mat;


                        for (int i = 0; i < pose.t->nrows; i++) {
                            for (int j = 0; j < pose.t->ncols; j++) {
                                mat = mat + to_string(pose.t->data[((i)*(pose.t)->ncols + (j))]);
                            }
                            mat.clear();
                        }

                         //Delivery spot's position and positive detection message (ROS)

                        landing_spot_detected.data = "true";
                            
                        x = roundf(pose.t->data[0] * 100)/100;
                        y = roundf(pose.t->data[1] * 100)/100;
                        z = roundf(pose.t->data[2] * 100)/100;

                            //RCLCPP_INFO(this-> get_logger(),"x : %f\ny : %f\nz: %f", y, x, z);

                        isLandingSpotDetected -> publish(landing_spot_detected);
                    }
                }
                    //for showing the camera image in a separate window. As an advice, leave it commented.
                    //imshow("Bottom Camera", cv_ptr -> image);
            }

            else {
                    //If the camera loses a frame due to any reason, the last catched frame is shown
                RCLCPP_INFO(this-> get_logger(), "I lost a frame on the bottom camera"); //for error handling's purpose only
                    //imshow("Bottom Camera", lastframe);   
            }
        }
        
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_INFO(this-> get_logger(), "BOTTOM CAMERA: There was an error creating cv_bridge pointer");
                return;
            }

            //waitKey(1);
        }

    private:
    void front_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      	cv_bridge::CvImagePtr cv_ptr;
        cuda::GpuMat cudaImg, cudaRGB;
        Mat img, frame, blob;
        vector<Vec3f> circles; 
        auto qp_ball_detected = std_msgs::msg::String();
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            cudaImg.upload(cv_ptr -> image);

            if(cudaImg.channels() == 3){
                isQPBallDetected = false;
                lastframe2 = cv_ptr->image;
                cudaImg.download(frame);
                blobFromImage(frame, blob, 1/255.0, cv::Size(inpWidth, inpHeight), Scalar(0,0,0), true, false);

                net.setInput(blob); //sets the net's input

                //getting the indexes and the names of the output layers
                static vector<String> names;
                if (names.empty()){
                    vector<int> outLayers = net.getUnconnectedOutLayers();
                    vector<String> layersNames = net.getLayerNames();
                    names.resize(outLayers.size());
                    for (size_t i = 0; i < outLayers.size(); ++i)
                    names[i] = layersNames[outLayers[i] - 1];
                }


                vector<Mat> outs;
                net.forward(outs, names); //finds the net's output

                vector<int> classIds;
                vector<float> confidences;
                vector<Rect> boxes;

                //iteration through the output to get the scores for each detection
                for (size_t i = 0; i < outs.size(); ++i) {
                float* data = (float*)outs[i].data;
                    for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols){
                        Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                        Point classIdPoint;
                        double confidence;
                        // Get the value and location of the maximum score
                        minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                        if (confidence > confThreshold){
                            int centerX = (int)(data[0] * frame.cols);
                            int centerY = (int)(data[1] * frame.rows);
                            int width = (int)(data[2] * frame.cols);
                            int height = (int)(data[3] * frame.rows);
                            int left = centerX - width / 2;
                            int top = centerY - height / 2;
                
                            classIds.push_back(classIdPoint.x);
                            confidences.push_back((float)confidence);
                            boxes.push_back(Rect(left, top, width, height));
                        }
                    }
                }

                vector<int> indices;
                NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices); //for eliminating redundant overlapping bounding boxes with lower confidence scores
                //computing just the optimal bounding boxes
                for (size_t i = 0; i < indices.size(); ++i){
                    int idx = indices[i];
                    Rect box = boxes[idx];
                    int left = box.x;
                    int top = box.y;
                    int right = box.x + box.width;
                    int bottom = box.y + box.height;
                    float conf = confidences[idx];
                    int classId = classIds[idx];
                    
                    //computing the position of the ball's center and its distance from the camera
                    double x_pos = (box.x) + (box.width/2);
                    double y_pos = (box.y) + (box.height/2);
                    double distance = (80 * 350.75) / box.width;

                    if((x_pos < 470 && x_pos > 200) && (y_pos < 300 && y_pos > 80) && distance < 1000){
                        isQPBallDetected = true;
                        qp_ball_detected.data = to_string(isQPBallDetected);
                        qp_BallDetected -> publish(qp_ball_detected);
                        cout << "Dangerous tennis ball spotted at " << distance / 100 << " m" << endl;
                    }

                    //FOR TESTING PURPOSES ONLY, LEAVE IT COMMENTED

                    //Draw a rectangle displaying the bounding box
                    /*rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);
                    
    
                    //Get the label for the class name and its confidence score
                    string label = format("%.2f", conf);
                    if (!classes.empty()){
                        CV_Assert(classId < (int)classes.size());
                        label = classes[classId] + ":" + label;
                    }
    
                    //Display the label at the top of the bounding box
                    int baseLine;
                    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    top = max(top, labelSize.height);
                    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
                    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);*/
                }

                //computes and displays the inference time for a frame (for evaluating the performances)
                /*vector<double> layersTimes;
                double freq = getTickFrequency() / 1000;
                double t = net.getPerfProfile(layersTimes) / freq;
                string label = format("Inference time for a frame : %.2f ms", t);
                putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));*/
        
                //for showing the camera image in a separate window. As an advice, leave it commented.
                //imshow("Results", frame);
                
                }

                else{
                    RCLCPP_INFO(this-> get_logger(), "I lost a frame on the front camera"); //for error handling's purpose only
                    //imshow("Front Camera", lastframe2);
                }
            }

            catch (cv_bridge::Exception& e)
            {
                RCLCPP_INFO(this-> get_logger(), "FRONT CAMERA: There was an error creating cv_bridge pointer");
                return;
                
            }

            //waitKey(1);
        }

    //Sends the box and the delivery spot's coordinates to the Autopilot's script (ROS)
    private:
    void coordinates_callback(){
    auto z_string = std_msgs::msg::String();
    z_string.data = to_string(z); 

    auto x_string = std_msgs::msg::String();
    x_string.data = to_string(y); 

    auto y_string = std_msgs::msg::String();
    y_string.data = to_string(x); 

    z_coord->publish(z_string);
    //these coordinates are flipped due to the differences in the reference systems
    y_coord->publish(y_string);
    x_coord->publish(x_string);
    }
    

    //publishers and subscribers definition
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s_bottom;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s_front;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr z_coord;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr x_coord;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr y_coord;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr isBoxDetected;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr isLandingSpotDetected;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qp_BallDetected;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);    
  rclcpp::spin(std::make_shared<ImgViewSubscriber>());
  rclcpp::shutdown();
  return 0;
}