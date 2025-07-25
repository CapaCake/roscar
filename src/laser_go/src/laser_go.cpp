#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"
#include "std_msgs/String.h"

#define freq 1440 // 每转一圈雷达扫描次数
typedef struct    // 极坐标系下的点
{
    double range;
    double theta;
} Point_polar;

typedef struct // 直角坐标系下的点
{
    double x;
    double y;
} Point_rectangular;

class PubAndSub
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub_stop;

public:
    PubAndSub()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 5);
        sub_ = n_.subscribe("/scan", 5, &PubAndSub::callback, this);
        sub_stop = n_.subscribe("/mode", 5, &PubAndSub::callback_stop, this);
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
    void callback_stop(const std_msgs::String::ConstPtr &stop_msg);
    unsigned char carMode = 0;
    double car_speed = 1576.0;
};

Control::PID pid;
void PubAndSub::callback_stop(const std_msgs::String::ConstPtr &stop_msg)
{
    if (strcmp(stop_msg->data.c_str(), "Mode0") == 0)
    {
        carMode = 0;
        car_speed = 1572.0;
    }
    if (strcmp(stop_msg->data.c_str(), "Mode1") == 0)
    {
        carMode = 1;
        car_speed = 1475.0;
    }
    if (strcmp(stop_msg->data.c_str(), "Mode2") == 0)
    {
        carMode = 2;
        car_speed = 1400.0;
    }
}
void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    int flag_N = 0, flag_P = 0;
    int negetiveNum = 0, positiveNum = 0;
    int i, j = 0;
    int num = 0, time = 1;
    double range = 0, error = 0, negetiveSum = 0, positiveSum = 0, N_aver = 0, P_aver = 0, XN = 0, XP = 0, P = 0, N = 0;
    geometry_msgs::Twist twist;
    Point_polar pp[30] = {0, 0};
    Point_rectangular pr[30] = {0, 0};

    for (i = 1; i < freq; i++)
    {
        if (laser->ranges[i - 1] - laser->ranges[i] >= 2.0)       // laser->ranges[i-1] - laser->ranges[i] >= 2.0[i-3]呢？
            if (laser->ranges[i] > 0.4 && laser->ranges[i] < 2.5) // 10.30.16.54
            {
                pp[j].range = laser->ranges[i];
                pp[j].theta = i * laser->angle_increment + laser->angle_min; // 获得2.5米范围内各锥桶的极坐标 if(laser->ranges[i] > 0.75 && laser->ranges[i] < 2.5)
                j++;
            }
    }
    for (i = 0; i < 30; i++)
    {
        if (pp[i].range)
        {
            pr[i].x = pp[i].range * sin(pp[i].theta);
            pr[i].y = pp[i].range * cos(pp[i].theta); // 获得2.5米范围内各锥桶的直角坐标(激光雷达为原点)
            if (pr[i].y >= -0.2)                      // 排除后方距离超过1米的锥桶
            {
                num++;
                ROS_INFO("Cone barrel:(%d,%.2f,%.2f)", num, pr[i].x, pr[i].y);
            }
            else
            {
                pr[i].x = 0;
                pr[i].y = 0;
            }
            if (pr[i].x > 0)
            {
                positiveNum++;                // 计算到的左端锥桶数量
                positiveSum += pr[i].x + 0.2; // 左端锥桶x坐标之和
                //   ROS_INFO("positiveNum:%d",positiveNum);
            }
            if (pr[i].x < 0)
            {
                negetiveNum++; // 计算到的右端锥桶数量
                negetiveSum += pr[i].x - 0.28;
                //   ROS_INFO("negetiveNum:%d",negetiveNum);
            }
        }
        /*将极坐标范围改成直坐标范围*/

        /*   if(pr[i].x>0)
               {
               if(pr[i].x >= 1.9)
               {
                   positiveSum -= pr[i].x;
                   positiveNum --;
               }
               }
           else if(pr[i].x<0)
               {
               if(pr[i].x <= -1.9)
               {
                   negetiveSum -= pr[i].x;
                   negetiveNum --;
               }
               }*/
    }

    P = 1.70;
    N = 1.775;
    N_aver = positiveSum / positiveNum;
    P_aver = negetiveSum / negetiveNum;
    XP = P * positiveSum / positiveNum;
    XN = N * negetiveSum / negetiveNum;
    //  ROS_INFO("N,P:(%.2f,%.2f)",N,P);
    //  ROS_INFO("previous:N_aver,P_aver,XP,XN:(%.2f,%.2f,%.2f,%.2f)\n",N_aver,P_aver,XP,XN);
    //  ROS_INFO("previous:N_aver,P_aver:(%.2f,%.2f)\n",N_aver,P_aver);
    //  ROS_INFO("previous:P_weig,N_weig:(%.2f,%.2f)\n",XP,XN);
    for (i = 0; i < 30; i++)
    {
        if (pr[i].x > 0)
        {
            if (pr[i].x <= 0.4 || pr[i].x >= 1.9) //|| pr[i].x >= 1.9
            {
                positiveSum -= pr[i].x + 0.2;
                positiveNum--;
                flag_P = 1;
            }
        }
        if (pr[i].x < 0)
        {
            if (pr[i].x >= -0.4 || pr[i].x <= -1.9)
            {
                negetiveSum -= pr[i].x - 0.28;
                negetiveNum--;
                flag_N = 1;
            }
        }
        if (pr[i].x > 0)
            if (pr[i].x >= P * positiveSum / positiveNum && flag_P == 0) // pr[i].x >= 1.70*positiveSum/positiveNum
            {

                positiveSum -= pr[i].x;
                positiveNum--;
            }
            else if (pr[i].x < 0)
                if (pr[i].x <= N * negetiveSum / negetiveNum && flag_N == 0) // 1.65在我把场地放大后很好,pr[i].x <= 1.775*negetiveSum/negetiveNum
                {
                    negetiveSum -= pr[i].x;
                    negetiveNum--;
                }
        P_aver = positiveSum / positiveNum;
        N_aver = negetiveSum / negetiveNum;
        XP = P * positiveSum / positiveNum;
        XN = N * negetiveSum / negetiveNum;
        flag_N = 0;
        flag_P = 0;
    }
    error = negetiveSum + positiveSum; // 通过锥桶的坐标算出误差(加权)

    // ROS_INFO("N,P:(%.2f,%.2f)",N,P);
    //    ROS_INFO("modify:N_aver,P_aver,XP,XN:(%.2f,%.2f,%.2f,%.2f)\n",N_aver,P_aver,XP,XN);
    ROS_INFO("modify:P_aver,N_aver:(%.2f,%.2f)\n", P_aver, N_aver);
    ROS_INFO("modify:P_weig,N_weig:(%.2f,%.2f)\n", XP, XN);

    ROS_INFO("previous.error:%.2f\n", error);
    // error = pid.PIDIncremental(error);//增量式PID控制器，参数在/src/Control.h Init函数中调节
    error = pid.PIDPositional(error); // 位置式PID控制器
    // error *= 25.0;
    twist.angular.z = 90 + error; // 将误差换算成角度并发布
    twist.linear.x = car_speed;   // 速度
    if (abs(error) <= 10)
        twist.linear.x += 10;
    if (twist.angular.z > 180)
        twist.angular.z = 180;
    if (twist.angular.z < 0)
        twist.angular.z = 0;

    twist.angular.y = 0;
    twist.angular.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    pub_.publish(twist);
    ROS_INFO("PID.error:%.2f\n", error);
    num = 0;
    // time++;
    ROS_INFO("pSum,Num,nSum,Nun:(%.2f,%d,%.2f,%d)\n", positiveSum, positiveNum, negetiveSum, negetiveNum); // nNum,%.2f,negetiveNum"pSum pNum nSum:(%.2f,%.2f,%.2f)\n",positiveSum,positiveNum,negetiveSum
                                                                                                           /* if(time>999)
                                                                                                            {
                                                                                                                time=0;
                                                                                                            }*/
    ROS_INFO("speed:%f\n", twist.linear.x);
    ROS_INFO("------------------------");
}

int main(int argc, char *argv[])
{
    pid.Init();
    ros::init(argc, argv, "laser_go");
    PubAndSub PAS;
    ros::spin();
    return 0;
}
