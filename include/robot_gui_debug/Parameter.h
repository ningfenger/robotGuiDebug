// gui调参模型战车端
// Created by ningfeng on 2019/11/14.
// QQ：2387143434 TEL:13693487198
//
#include <time.h>
#include <iostream>
#include <vector>
#include <map>
#include <stdarg.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <fstream>
#include <ros/ros.h>
#include "ros/package.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "robot_gui_debug/ParameterAdjustMsg.h"
#include "robot_gui_debug/ParameterShow.h"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <unistd.h>
#include <cmath>
using namespace std;
using namespace cv;
using namespace chrono;
#ifndef ROBOT_GUI_PARAMETER_PARAMETER_H
#define ROBOT_GUI_PARAMETER_PARAMETER_H

#endif //ROBOT_GUI_PARAMETER_PARAMETER_H
typedef struct
{
    vector<int *> intParameterPoints;
    vector<float *> floatParameterPoints;
    string parameterNames;
} ParameterPoints;

class Parameter
{
private:
    vector<string> intParameters;   // 存储int类型的变量名
    vector<string> floatParameters; // 存储float类型的变量名
    //！！！以上两个容器中的变量不允许重名
    int *intParametersValues;     // 存储int类型的变量值
    float *floatParametersValues; // 存储float类型的变量值
    map<string, int> intLoc;      // 存储int变量名及其值的对应关系
    map<string, int> floatLoc;    // 存储float变量名及其值的对应关系
    string path;                  //存储yaml配置文件位置
    system_clock::time_point start;
public:
    Parameter();

    /* 带参构造函数，传入ROS节点的句柄
     * 需要调参及借助本类中的方法发布topic时，务必使用此构造函数
     */
    // Parameter(ros::NodeHandle n_);

    /* 作用：获取int类型指定变量的值
     * 参数：
     * parameterName：变量名
     * */
    int getIntValue(string parameterName);

    /* 作用：获取float类型指定变量的值
     * 参数：
     * parameterName：变量名
     * */
    float getFloatValue(string parameterName);

    /* 作用：指定要调参的变量
     * 参数：
     * parameterNameList：变量名列表
     * 返回值：存储指向所要调参的变量的指针组的容器
        * */
    ParameterPoints parametersNeed(initializer_list<string> parameterNameList);

    /* 作用：根据收到的消息调整变量
    * 参数：
    * msg：接收到的调参消息
    */
    void parameterDebugCallback(const robot_gui_debug::ParameterAdjustMsg::ConstPtr &msg);

    /* 作用：将目前的调参结果写入文件
     * */
    void writeToFile();

    /*作用：将要回传绘图的变量组格式化为发送给主机端的字符串
    * 参数：
    * valuesNameList： 所有需要传给主机端的变量名，注意顺序需要与发布时的变量顺序一致
    * valuesLoc: 所有需要回传显示的变量的绘制位置
    * 返回值：发送给主机端格式的字符串
    */
   string valuesBack(initializer_list<string> valuesNameList,initializer_list<int> valuesLoc);

    /*作用：将要回传绘图的变量值组加上时间戳并格式化为发送给主机端的字符串
    * 参数：
    * values： 所有需要回传显示的变量的值，注意顺序需要与建立连接是需要回传的变量顺序一致
    * 返回值：发送给主机端格式的字符串
    */
   string valuesShow(initializer_list<float> values);
   
};