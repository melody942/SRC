#ifndef PUBFORCE_H
#define PUBFORCE_H

#include "ros/ros.h"
#include "geomagic_control/DeviceButtonEvent.h"
#include "geometry_msgs/PoseStamped.h"
#include "geomagic_control/DeviceFeedback.h"
#include  <vector>

// 单轴输出的最大力 官网显示最大3.3N 测试可用1N
#define MAX_FORCE   1
#define RELEASE 0
#define FORCE_FEEDBACK  1
#define LOCK    2

//位姿与按键订阅者的回调函数
void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg);
void cb_button(const geomagic_control::DeviceButtonEvent::ConstPtr &bt);
//释放手柄
void touch_release(void);
//在指定位置锁住touch手柄
void touch_lock(geometry_msgs::PoseStamped* pose);

#endif // !PUBFORCE_H
