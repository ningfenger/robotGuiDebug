//
// Created by ningfeng on 2019/11/14.
// Update by ningfeng on 2019/12/12
/*
19.12.12更新
配置文件不再需要指定绝对路径
19.12.19更新
增加变量回传图像绘制功能，可以绘制所给变量的变化波形
*/
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

Parameter::Parameter()
{
    path = ros::package::getPath("robot_gui_debug") + "/res/gui_debug_robot.yaml";
    YAML::Node config = YAML::LoadFile(path);
    for (YAML::const_iterator it = config["int"].begin(); it != config["int"].end(); ++it)
        intParameters.push_back(it->first.as<string>());
    for (YAML::const_iterator it = config["float"].begin(); it != config["float"].end(); ++it)
        floatParameters.push_back(it->first.as<string>());
    int count = 0;
    intParametersValues = new int[intParameters.size()];
    floatParametersValues = new float[floatParameters.size()];
    for (YAML::const_iterator it = config["int"].begin(); it != config["int"].end(); ++it)
    {
        intParametersValues[count] = it->second.as<int>();
        intLoc[intParameters[count]] = count;
        count++;
    }
    count = 0;
    for (YAML::const_iterator it = config["float"].begin(); it != config["float"].end(); ++it)
    {
        floatParametersValues[count] = it->second.as<float>();
        floatLoc[floatParameters[count]] = count;
        count++;
    }
    start  = system_clock::now();
}

int Parameter::getIntValue(string parameterName)
{
    if (intLoc.find(parameterName) != intLoc.end())
        return intParametersValues[intLoc[parameterName]];
    cout << "Filed to load to value, please to check the parameter name." << endl;
    return 0;
}

float Parameter::getFloatValue(string parameterName)
{
    if (floatLoc.find(parameterName) != floatLoc.end())
        return floatParametersValues[floatLoc[parameterName]];
    cout << "Filed to load to value, please to check the parameter name." << endl;
    return 0;
}

ParameterPoints Parameter::parametersNeed(initializer_list<string> parameterNameList)
{
    ParameterPoints parameterPoints;
    string parameterNames;
    for (auto &parameter : parameterNameList)
    {
        if (intLoc.find(parameter) != intLoc.end())
            parameterPoints.intParameterPoints.push_back(&intParametersValues[intLoc[parameter]]);
        else if (floatLoc.find(parameter) != floatLoc.end())
            parameterPoints.floatParameterPoints.push_back(&floatParametersValues[floatLoc[parameter]]);
        else
        {
            cout << "Can not find the parameter:" << parameter << "!" << endl;
            continue;
        }
        parameterNames += " ";
        parameterNames += parameter;
    }
    parameterPoints.parameterNames = parameterNames;
    return parameterPoints;
}

void Parameter::parameterDebugCallback(const robot_gui_debug::ParameterAdjustMsg::ConstPtr &msg)
{
    if (msg->name == "write")
    {
        YAML::Node config = YAML::LoadFile(path);
        ofstream fout(path);
        for (int i = 0; i < intParameters.size(); i++)
            config["int"][intParameters[i]] = intParametersValues[i];
        for (int i = 0; i < floatParameters.size(); i++)
            config["float"][floatParameters[i]] = floatParametersValues[i];
        fout << config;
        fout.close();
        cout << "Write to file successfully!" << endl;
        return;
    }
    if (intLoc.find(msg->name) != intLoc.end())
        intParametersValues[intLoc[msg->name]] = msg->intValue;
    else if (floatLoc.find(msg->name) != floatLoc.end())
        floatParametersValues[floatLoc[msg->name]] = msg->floatValue;
    else
    {
        cout << "Can not find the parameter:" << msg->name << "!" << endl;
        return;
    }
    cout << "Agjust the parameter:" << msg->name << " successfully!" << endl;
}

void Parameter::writeToFile()
{
    path = ros::package::getPath("robot_gui_debug") + "/res/gui_debug_robot2.yaml";
    cout << path << endl;
    YAML::Node config = YAML::LoadFile(path);
    ofstream fout(path);
    for (int i = 0; i < intParameters.size(); i++)
        config["int"][intParameters[i]] = intParametersValues[i];
    for (int i = 0; i < floatParameters.size(); i++)
        config["float"][floatParameters[i]] = floatParametersValues[i];
    fout << config;
    fout.close();
}

string Parameter::valuesBack(initializer_list<string> valuesNameList, initializer_list<int> valuesLoc)
{
    if (valuesLoc.size() != valuesNameList.size())
    {
        cout << "Error! The number of locs and the values not match!";
        return "Fasle";
    }
    string backString;
    for (auto &valueName : valuesNameList)
    {
        backString += valueName;
        backString += ' ';
    }
    for (auto &valueLoc : valuesLoc)
    {
        backString += to_string(valueLoc);
        backString += ' ';
    }
    return backString;
}

string Parameter::valuesShow(initializer_list<float> values)
{
    auto thisTime=system_clock::now();
    auto time=duration_cast<microseconds>(thisTime- start);
    double time_d= double(time.count()) * microseconds::period::num / microseconds::period::den;
  //  cout << time_d<< endl;
    string backString;
    backString += to_string( time_d);
    backString += ' ';
    for (auto &value : values)
    {
        backString += to_string(value);
        backString += ' ';
    }
    return backString;
}