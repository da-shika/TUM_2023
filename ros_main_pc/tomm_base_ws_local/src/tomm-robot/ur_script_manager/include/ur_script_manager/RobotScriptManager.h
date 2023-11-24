#ifndef UR_ROBOT_MANAGER_ROBOTSCRIPTMANAGER_H
#define UR_ROBOT_MANAGER_ROBOTSCRIPTMANAGER_H

#include <QVector>
#include <QString>

#include <ros/ros.h>

#include <ur_script_manager/ScriptLoader.h>
#include <ur_script_manager/ScriptManager.h>

#include <ur_script_manager/setScriptManagerState.h>
#include <ur_script_manager/getScriptManagerStates.h>


namespace tum_ics_ur_robot_manager{

class RobotScriptManager : public ScriptManager
{
private:
    static QVector<QString> progNames();
    static Command getCommand(
            const QString& name,
            bool enable);

private:
    ros::NodeHandle m_node;
    ros::ServiceServer m_setState;
    ros::ServiceServer m_getStates;

    ScriptLoader m_loader;

    QVector<bool> m_states; // same order as progNames

public:
    RobotScriptManager(
            const QString &pcIpAddr,
            const QString &robotIpAddr,
            quint16 managerPort = 50001,
            quint16 scriptLoaderPort = 30001);

    ~RobotScriptManager();

    bool start(const QString& scriptFilePath);
    bool start();

    bool send(const QString& name, bool enable);

private:
    void setState(const QString& name, bool enable);
    bool getState(const QString& name);

    bool setStateCallback(
            ur_script_manager::setScriptManagerState::Request &req,
            ur_script_manager::setScriptManagerState::Response &res);

    bool getStatesCallback(
            ur_script_manager::getScriptManagerStates::Request &req,
            ur_script_manager::getScriptManagerStates::Response &res);
};

}

#endif // UR_ROBOT_MANAGER_ROBOTSCRIPTMANAGER_H
