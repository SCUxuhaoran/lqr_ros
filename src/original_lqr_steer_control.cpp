/*************************************************************************
	> File Name: lqr_steer_control.cpp
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Wed Apr 17 11:48:46 2019
 ************************************************************************/

#include<iostream>
#include<limits>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<sys/time.h>
#include<Eigen/Eigen>
#include"cubic_spline.h"
#include"motion_model.h"
#include"cpprobotics_types.h"
#include<unistd.h>
#include<json/json.h>

//#define DT 0.1
//#define L 0.5
//#define KP 1.0
//#define MAX_STEER 45.0/180*M_PI

using namespace cpprobotics;

//Global Params
Vec_f wx;
Vec_f wy;
float DT;
float L;
float KP;
float max_steer_degree;
float MAX_STEER;
float max_speed;
float target_speed;

std::ofstream outFile;

void readFileJson(std::string json_root) {
    Json::Reader reader;
    Json::Value root;
    //从文件中读取，保证当前文件有demo.json文件  
    std::ifstream input(json_root, std::ios::binary);
    if (!input.is_open()) {
        std::cout << "Error opening file\n";
        return;
    }
    if (reader.parse(input, root)) {
        //读取根节点信息  
	DT = root["DT"].asFloat();
	L = root["L"].asFloat();
	KP = root["KP"].asFloat();
	max_steer_degree = root["max_steer_degree"].asFloat();
	MAX_STEER = max_steer_degree/180*M_PI;
	max_speed = root["max_speed"].asFloat();
        target_speed = max_speed / 3.6;

        //读取数组信息 
        for (unsigned int i = 0; i < root["circuit_wx"].size(); i++) {
            float achx = root["circuit_wx"][i].asFloat();
	    wx.push_back(achx);
        }
	for (unsigned int i = 0; i < root["circuit_wy"].size(); i++) {
            float achy = root["circuit_wy"][i].asFloat();
	    wy.push_back(achy);
        }
    }
    input.close();
};

cv::Point2i cv_offset(float x, float y, int image_width=2000, int image_height=2000) {
  cv::Point2i output;
  //output.x = int(x * 100) + 100;
  //output.y = image_height - int(y * 100) - 200;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/2;
  return output;
};

Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed){
  Vec_f speed_profile(ryaw.size(), target_speed);

  float direction = 1.0;
  for(unsigned int i=0; i < ryaw.size()-1; i++){
    float dyaw = std::abs(ryaw[i+1] - ryaw[i]);
    float switch_point = (M_PI/4.0< dyaw) && (dyaw<M_PI/2.0);

    if (switch_point) direction = direction * -1;
    if (direction != 1.0) speed_profile[i]= target_speed * -1;
    else speed_profile[i]= target_speed;

    if (switch_point) speed_profile[i] = 0.0;
  }

  speed_profile[speed_profile.size()-1] = 0.0;
  return speed_profile;
};


float calc_nearest_index(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, int &ind){
  float mind = std::numeric_limits<float>::max();
  for(unsigned int i=0; i<cx.size(); i++){
    float idx = cx[i] - state.x;
    float idy = cy[i] - state.y;
    float d_e = idx*idx + idy*idy;

    if (d_e<mind){
      mind = d_e;
      ind = i;
    }
  }
  float dxl = cx[ind] - state.x;
  float dyl = cy[ind] - state.y;
  float angle = YAW_P2P(cyaw[ind] - std::atan2(dyl, dxl));
  if (angle < 0) mind = mind * -1;

  return mind;
};

Eigen::Matrix4f solve_DARE(Eigen::Matrix4f A, Eigen::Vector4f B, Eigen::Matrix4f Q, float R){
  Eigen::Matrix4f X = Q;
  int maxiter = 150;
  float eps = 0.01;

  for(int i=0; i<maxiter; i++){
    Eigen::Matrix4f Xn = A.transpose()*X*A-A.transpose()*X*B/(R+B.transpose()*X*B) * B.transpose()*X*A+Q;
    Eigen::Matrix4f error = Xn - X;
    if (error.cwiseAbs().maxCoeff()<eps){
      return Xn;
    }
    X = Xn;
  }

  return X;
};

Eigen::RowVector4f dlqr(Eigen::Matrix4f A, Eigen::Vector4f B, Eigen::Matrix4f Q, float R){
  Eigen::Matrix4f X = solve_DARE(A, B ,Q, R);
  Eigen::RowVector4f K = 1.0/(B.transpose()*X*B + R) * (B.transpose()*X*A);
  return K;
};

float lqr_steering_control(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, int& ind, float& pe, float& pth_e){
  float e = calc_nearest_index(state, cx, cy, cyaw, ind);

  float k = ck[ind];
  float th_e = YAW_P2P(state.yaw - cyaw[ind]);

  Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
  A(0, 0) = 1.0;
  A(0 ,1) = DT;
  A(1 ,2) = state.v;
  A(2 ,2) = 1.0;
  A(2 ,3) = DT;

  Eigen::Vector4f B = Eigen::Vector4f::Zero();
  B(3) = state.v/L;

  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
  float R = 1;

  // gain of lqr
  Eigen::RowVector4f K = dlqr(A, B, Q, R);

  Eigen::Vector4f x = Eigen::Vector4f::Zero();
  x(0) = e;
  x(1) = (e-pe)/DT;
  x(2) = th_e;
  x(3) = (th_e-pth_e)/DT;

  float ff = std::atan2((L*k), (double)1.0);
  float fb = YAW_P2P((-K * x)(0));
  float delta = ff+fb;

  pe = e;
  pth_e = th_e;
  return delta;
};


void update (State& state, float a, float delta){

  if (delta >= MAX_STEER) delta = MAX_STEER;
  if (delta <= - MAX_STEER) delta = - MAX_STEER;

  state.x = state.x + state.v * std::cos(state.yaw) * DT;
  state.y = state.y + state.v * std::sin(state.yaw) * DT;
  state.yaw = state.yaw + state.v / L * std::tan(delta) * DT;
  state.v = state.v + a * DT;

  outFile << setiosflags(std::ios::fixed) << std::setprecision(2) << state.x << "," << state.y << "," << state.yaw << std::endl;
};

void closed_loop_prediction(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal){
  float T = 500.0;
  float goal_dis = 0.5;
  float stop_speed = 0.05;

  State state(0.0, 2.0, 1.5, 0.0);

  float time_ = 0.0;
  Vec_f x;
  x.push_back(state.x);
  Vec_f y;
  y.push_back(state.y);
  Vec_f yaw;
  yaw.push_back(state.yaw);
  Vec_f v;
  v.push_back(state.v);
  Vec_f t;
  t.push_back(0.0);

  float e = 0;
  float e_th = 0;
  int ind = 0;


  cv::namedWindow("lqr", cv::WINDOW_NORMAL);
  int count = 0;


  cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
  for(unsigned int i=1; i<cx.size(); i++){
    cv::line(
      bg,
      cv_offset(cx[i-1], cy[i-1], bg.cols, bg.rows),
      cv_offset(cx[i], cy[i], bg.cols, bg.rows),
      cv::Scalar(0, 0, 0),
      10);
  }

  while (T >= time_){
    float di = lqr_steering_control(state, cx, cy, cyaw, ck, ind, e, e_th);
    float ai = KP * (speed_profile[ind]-state.v);
    update(state, ai, di);

    if (std::abs(state.v) <= stop_speed) ind += 1;

    float dx = state.x - goal[0];
    float dy = state.y - goal[1];
    if (std::sqrt(dx*dx + dy*dy) <= goal_dis) {
      std::cout<<("Goal")<<std::endl;
      sleep(1);
      cv::imwrite("lqr_origin_01.jpg", bg);
      sleep(2);
      break;
    }

    cv::circle(
      bg,
      cv_offset(state.x, state.y, bg.cols, bg.rows),
      10, cv::Scalar(0, 0, 255), -1);

    //save image in build/bin/pngs
    // struct timeval tp;
    // gettimeofday(&tp, NULL);
    // long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    // std::string int_count = std::to_string(ms);
    // cv::imwrite("./pngs/"+int_count+".png", bg);
    cv::imshow("lqr", bg);
    cv::waitKey(5);
  }
};

int main(){
  //Vec_f wx({0.0, 6.0,  12.5, 10.0, 7.5, 3.0, -1.0});
  //Vec_f wy({0.0, -3.0, -5.0,  6.5, 3.0, 5.0, -2.0});
  //Vec_f wx({0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.5, 2.0, 2.0, 2.5, 3.0, 3.0, 3.0, 2.0, 1.0});
  //Vec_f wy({0.0, 1.0, 2.0, 2.5, 2.0, 1.5, 1.0, 1.5, 2.0, 2.5, 2.0, 1.0, 0.0, 0.0, 0.0});
  
  //read config from JSON file
  std::string json_root = "/home/huanyu/robot_ws/src/my_lqr/config/origin_config.json";
  std::string csv_file = "/home/huanyu/robot_ws/data0.csv";
  readFileJson(json_root);
  outFile.open(csv_file, std::ios::out);

  Spline2D csp_obj(wx, wy);
  Vec_f r_x;
  Vec_f r_y;
  Vec_f ryaw;
  Vec_f rcurvature;
  Vec_f rs;
  for(float i=0; i<csp_obj.s.back(); i+=0.1){
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }
  //float target_speed = 10.0 / 3.6;
  Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);
  closed_loop_prediction(r_x, r_y, ryaw, rcurvature, speed_profile, {{wx.back(), wy.back()}});

}
