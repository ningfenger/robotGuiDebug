# Robomaster GUI调参模块简易说明

v1.0 2019年11月14日

v1.1 2019年12月12日

v1.2 2019年12月19日

## 1. 结构说明

模块基于ROS，分为战车端和客户端，战车端使用c++，客户端使用python

## 2. 功能说明

两端借助topic通信，客户端可以实时调整战车端的参数，并保存在两端。

## 3.使用说明

### CmakeList说明

本模块使用了静态库文件libyaml-cpp.a 以实现yaml文件读取元组的功能，部分电脑上可能会出现报错，如果遇到报错，请在生成节点时自行链接此静态库：

```cmake
target_link_libraries(NODENAME ${CMAKE_HOME_DIRECTORY}/robot_gui_debug/lib/libyaml-cpp.a)
```

其中NODENAME为自行定义的节点名

### 战车端：

配置文件的书写请按照 robot_gui_debug/res/gui_debug_robot.yaml 格式，将所有需要调参的变量按照类型写入

```yaml
int:
  i_a: 1327
  i_b: 125
  i_c: 456414
  i_d: 6564
float:
  f_a: 4.53499985
  f_b: 589
  f_c: 3.66000009
```

通过Parameter类生成对象后可以通过parametersNeed方法传入待调参数，以获得对应的指针，函数返回值是一个结构体，包含两个容器，分别存放了int和float类型的两种指针

例：

```c
ParameterPoints parameterPoints = parameter.parametersNeed({"i_a", "i_b", "i_c", "f_a", "f_b", "f_c"});
    int *i_a = parameterPoints.intParameterPoints[0];
    int *i_b = parameterPoints.intParameterPoints[1];
    int *i_c = parameterPoints.intParameterPoints[2];
    float *f_a = parameterPoints.floatParameterPoints[0];
    float *f_b = parameterPoints.floatParameterPoints[1];
    float *f_c = parameterPoints.floatParameterPoints[2];
```

其中指针顺序与传入的参数名在对应类型中的顺序一致

注意，务必确认传入的参数名已在配置文件中写明，并且一致

```c++
void show(int &i_a, int &i_b, int &i_c, float &f_a, float &f_b, float &f_c)
{
    while (1)
    {
        cout << "i_a:" << i_a << " i_b:" << i_b << " i_c:" << i_c << endl;
        cout << "f_a:" << f_a << " f_b:" << f_b << " f_c:" << f_c << endl;
        usleep(2000000);
    }
}

    thread showThread([&]() {
        show(*i_a, *i_b, *i_c, *f_a, *f_b, *f_c);
    });
```

传入对象的回调函数，即可以实时调参

```c++
void parametersDebug(ros::NodeHandle n, Parameter &parameter)
{
    ros::Subscriber sub = n.subscribe("parameter/debug", 1, &Parameter::parameterDebugCallback, &parameter);
    ros::spin();
}
```



#### cpp例程

```cpp
#include "robot_gui_debug/Parameter.h"
#include <unistd.h>
#include <thread>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "std_msgs/String.h"
using namespace std;
// 此函数是为方便展示，每隔两秒打印一遍所有参数
void show(int &i_a, int &i_b, int &i_c,int &i_d, float &f_a, float &f_b, float &f_c)
{
    while (1)
    {
        cout << "i_a:" << i_a << " i_b:" << i_b << " i_c:" << i_c <<" i_d:"<<i_d<< endl;
        cout << "f_a:" << f_a << " f_b:" << f_b << " f_c:" << f_c << endl;
        usleep(2000000);
    }
}
// 回调函数，每次收到topic都会对对应的参数作修改
void parametersDebug(ros::NodeHandle n, Parameter &parameter)
{
    ros::Subscriber sub = n.subscribe("parameter/debug", 1, &Parameter::parameterDebugCallback, &parameter);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_gui_debug_subscriber");
    ros::NodeHandle n;
    Parameter parameter;
    /**
     * 此处调用parametersNeed方法，以容器形式传入待调参数列表，可以获得对应参数的指针
     * 将这些指针作为参数，传入对应的函数内，即可实时调整
     * 要调整参数，请修改 配置文件robot_gui_debug/res/gui_debug_robot.yaml， 并修改如下方法中对应的参数
     * */
  
    ParameterPoints parameterPoints = parameter.parametersNeed({"i_a", "i_b", "i_c","i_d", "f_a", "f_b", "f_c"});
    int *i_a = parameterPoints.intParameterPoints[0];
    int *i_b = parameterPoints.intParameterPoints[1];
    int *i_c = parameterPoints.intParameterPoints[2];
    int *i_d = parameterPoints.intParameterPoints[3];
    float *f_a = parameterPoints.floatParameterPoints[0];
    float *f_b = parameterPoints.floatParameterPoints[1];
    float *f_c = parameterPoints.floatParameterPoints[2];
   // 显示线程
    thread showThread([&]() {
        show(*i_a, *i_b, *i_c,*i_d, *f_a, *f_b, *f_c);
    });
    // 订阅topic的线程
    thread listenThread([&]() {
        parametersDebug(n, parameter);
    });
    showThread.join();
    listenThread.join();
    return 0;
}

```

其余接口参见parameter.h

### 客户端：

如未安装tkinter包，请运行

```cmd
sudo apt-get install python3-tk
pip install future
```

安装

将robot_gui_debug/res/gui_debug_host.ini中的对应位置修改为自己的变量，如果需要一键显示战场端的图像回传网页，请讲配置文件的IP修改为战车的IP地址。

如出现节点名或话题名冲突，请在配置文件中根据自己的需要修改。

运行节点

```
rosrun robot_gui_debug host.py
```

### 变量回传示波使用说明

#### 战车端

图像回传使用server实现，故需要建立server服务

```c++
ros::ServiceServer guidebugServer = n.advertiseService("parameter/show", showCallback);
```

```c++
bool showCallback(robot_gui_debug::ParameterShow::Request &req, robot_gui_debug::ParameterShow::Response &res)
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
```

服务建立时，connect变量为0，代表请求链接，此时需要返回全部需要调参的变量名以及其需要画图的位置

可以通过Parameter类中的 valuesBack函数实现

建立完成后，主机端请求变量值事，connect为1，只需讲变量值字符串返回即可

变量值字符串可通过valuesShow函数实现，详见Parameter.h文件

##### cpp例程

```c++
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
#include "robot_gui_debug/ParameterShow.h"
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
bool showCallback(robot_gui_debug::ParameterShow::Request &req, robot_gui_debug::ParameterShow::Response &res)
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
```



#### 主机端

主机端需要修改配置文件中（gui_debug_host.ini）的三个内容

```ini
need_value_show = 0 #是否需要示波
sampling_time = 0.03 #采样率，即每隔多长时间向机器人请求一次变量值
server_name = parameter/show # 与机器人请求建立连接的服务名
```

## 4.其他

对于本模块的任何建议和需求，请与作者联系，QQ:2387143434















