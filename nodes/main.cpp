/*code is far away from bug with the animal protecting
    *  ┏┓　　　┏┓
    *┏┛┻━━━┛┻┓
    *┃　　　　　　　┃ 　
    *┃　　　━　　　┃
    *┃　┳┛　┗┳　┃
    *┃　　　　　　　┃
    *┃　　　┻　　　┃
    *┃　　　　　　　┃
    *┗━┓　　　┏━┛
    *　　┃　　　┃神兽保佑
    *　　┃　　　┃代码无BUG！
    *　　┃　　　┗━━━┓
    *　　┃　　　　　　　┣┓
    *　　┃　　　　　　　┏┛
    *　　┗┓┓┏━┳┓┏┛
    *　　　┃┫┫　┃┫┫
    *　　　┗┻┛　┗┻┛ 
    *　　　
    */
#include "robot_gui_debug/Parameter.h"
#include <unistd.h>
#include <thread>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "std_msgs/String.h"

using namespace std;

string valueNameLoc; //全局变量，指定需要调参的变量
string values; //全局变量，制定需要调参的变量的值

void show(int &i_a, int &i_b, int &i_c, int &i_d, float &f_a, float &f_b, float &f_c, Parameter parameter)
{
    auto start = system_clock::now();
    while (1)
    {
        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        double time= double(duration.count()) * microseconds::period::num / microseconds::period::den;
        // cout << "i_a:" << i_a << " i_b:" << i_b << " i_c:" << i_c << " i_d:" << i_d << endl;
        // cout << "f_a:" << f_a << " f_b:" << f_b << " f_c:" << f_c << endl;
        usleep(30000);
        values = parameter.valuesShow({f_a*(float)sin(time), f_b*(float)cos(time), f_c*(float)(time)});
    }
}

// server的回调函数
bool showCallback(robot_gui_debug::ParameterShow::Request &req,robot_gui_debug::ParameterShow::Response &res)
{
    int connect = int(req.connect);
    // 0代表第一次请求链接，获取变量名及其位置
    if (connect == 0)
    {
        res.parameterToShow = valueNameLoc;
        // cout<<res.parameterToShow<<endl;
    }

    // 1代表请求变量值列表
    else if (connect == 1)
    {
        res.parameterToShow = values;
    }

    return true;
}
void parametersDebug(ros::NodeHandle n, Parameter &parameter)
{
    // server用于与主机端链接，返回对应参数
    ros::ServiceServer guidebugServer = n.advertiseService("parameter/show", showCallback);
    // subscriber用于获取调参命令
    ros::Subscriber sub = n.subscribe("parameter/debug", 1, &Parameter::parameterDebugCallback, &parameter);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_gui_debug_subscriber");
    ros::NodeHandle n;
    Parameter parameter;
    
    // 将要调参的变量以及其要放置的格式化为字符串并赋值给全局变量
    valueNameLoc = parameter.valuesBack({"f_a", "f_b", "f_c"}, {1, 1, 2});
    /**
     * 此处调用parametersNeed方法，以容器形式传入待调参数列表，可以获得对应参数的指针
     * 将这些指针作为参数，传入对应的函数内，即可实时调整
     * 对于新增的参数，请修改Parameter.cpp中的参数，只需添加对应的参数名
     * 对于要调整的参数，请在客户端修改配置文件
     * */

    ParameterPoints parameterPoints = parameter.parametersNeed({"i_a", "i_b", "i_c", "i_d", "f_a", "f_b", "f_c"});
    int *i_a = parameterPoints.intParameterPoints[0];
    int *i_b = parameterPoints.intParameterPoints[1];
    int *i_c = parameterPoints.intParameterPoints[2];
    int *i_d = parameterPoints.intParameterPoints[3];
    float *f_a = parameterPoints.floatParameterPoints[0];
    float *f_b = parameterPoints.floatParameterPoints[1];
    float *f_c = parameterPoints.floatParameterPoints[2];

    // 服务连接待调
    // ros::ServiceClient connectClient=n.serviceClient<std_msgs::String> ("connect_request");
    // std_msgs::String parameterNames;
    // parameterNames.data = parameterPoints.parameterNames;
    // connectClient.call(parameterNames);
    thread showThread([&]() {
        show(*i_a, *i_b, *i_c, *i_d, *f_a, *f_b, *f_c, parameter);
    });
    thread listenThread([&]() {
        parametersDebug(n, parameter);
    });
    showThread.join();
    listenThread.join();
    return 0;
}
