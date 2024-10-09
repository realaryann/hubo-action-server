#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <QTimer>
#include <QEventLoop>
#include <QCoreApplication>
#include "Reader.h"
#include <QThread>
#include "hubo_core/JointInformation.h"
#include "hubo_core/Terminator.h"
#include "hubo_core/PODOClient.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>
#include "custom_interfaces/action/huboaction.hpp"


enum MELKCOMMAND
{
    MELK_NO_ACT = 100,
    MELK_GO_WALKREADYPOS,
    MELK_GO_HOMEPOS,
    MELK_WHEELCMD,
    MELK_GOTODES,
    MELK_INFINITY,
    MELK_MOVEHAND,
    MELK_SHOW_INFO,
    MELK_SHOW_JOINT_VALUES,
    MELK_MOVE_JOINTS,
    MELK_DEMO_MOVEHAND,
    MELK_DEMO_INITIALPOSITION,
    MELK_DRAW_RECTANGLE,
};
/*
if (msg.data == std::string("home_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to home position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_GO_HOMEPOS;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    } else if (msg.data == std::string("wheel_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to wheel position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_WHEELCMD;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    } else if (msg.data == std::string("walking_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to walking position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_GO_WALKREADYPOS;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    }
*/

class MyActionServer : public rclcpp::Node
{
public:
  using Move = custom_interfaces::action::Huboaction;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("hubo_action_server", options), timer(new QTimer(this))
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
      this,
      "hubo_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

  }

private:
  PODOClient* client;
  QTimer *timer;
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  std::shared_ptr<const Move::Goal> storegoal;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with message %s", std::string(goal->msg).c_str());
    (void)uuid;
    storegoal = goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // Jumps to execute?
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
    
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
    // Jumps to execu-te?
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
    // Jumps to execute?
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();
    if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }
    // Podo Goal routing
    if (std::string(storegoal->msg) == std::string("home_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to home position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_GO_HOMEPOS;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    } else if (std::string(storegoal->msg) == std::string("wheel_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to wheel position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_WHEELCMD;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    } else if (std::string(storegoal->msg) == std::string("walking_position")) {
        RCLCPP_INFO(this->get_logger(), "Sending robot to walking position");
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = 10;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
        cmd.COMMAND_DATA.USER_COMMAND = MELK_GO_WALKREADYPOS;
        LAN_GUI2PODO tempDATA;
        memcpy(&(tempDATA.UserCMD), &cmd, sizeof(USER_COMMAND));
        QByteArray tempSendData = QByteArray::fromRawData((char *)&tempDATA, sizeof(LAN_GUI2PODO));
        client->RBSendData(tempSendData);
    }
    // End 
    if (rclcpp::ok()) {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    
  }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  Terminator *terminator = new Terminator();
  QThread *terminatorThread = new QThread();
  terminator->moveToThread(terminatorThread);
  QObject::connect(terminatorThread, &QThread::started, terminator, &Terminator::startChecking);
  QObject::connect(terminatorThread, &QThread::finished, terminator, &QObject::deleteLater);
  QObject::connect(terminatorThread, &QThread::finished, terminatorThread, &QObject::deleteLater);
  terminatorThread->start();
  rclcpp::init(argc, argv);
  PODOClient client;
  client.RBConnect();
  Reader *reader = new Reader(&client);
  QThread *readerThread = new QThread;
  reader->moveToThread(readerThread);
  QObject::connect(readerThread, &QThread::started, reader, &Reader::startLoop);
  QObject::connect(readerThread, &QThread::finished, reader, &QObject::deleteLater);
  QObject::connect(readerThread, &QThread::finished, readerThread, &QObject::deleteLater);
  readerThread->start();

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
