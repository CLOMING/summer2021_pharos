/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

//library
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdint.h>
#include <iostream>

serial::Serial ser;
std::string setPortName("/dev/ttyUSB0");
int BaudrateNum(115200);
int hzNum_I(100);
double hzNum_D(100);
//std::vector<uint8_t> testHex;

void write_callback(const std_msgs::String::ConstPtr& msg){
    /*
     * 시리얼에서 msg를 받으면 ser에 (msg->data[std::string 형]) 으로 바꿔서 대입함.
     */
//    testHex.clear();
//    testHex.push_back(0x68);
    ROS_INFO_STREAM("Writing to serial port : " << std::endl << msg->data);
    ser.write(msg->data);
//    ser.write(testHex);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("setPort", setPortName, "/dev/ttyUSB0");
    pnh.param<int>("setBaudrate", BaudrateNum, 115200);
    pnh.param<int>("setHz", hzNum_I, 100);

    ros::Subscriber write_sub = nh.subscribe("settingValue", 1, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/ebimu/read", 1);
    ros::Publisher okPub = nh.advertise<std_msgs::String>("okTopic", 1);

    hzNum_D = (double)hzNum_I; // ros::Rate loop_rate(hzNum_D); --> 여기에서 double형을 필요로 하는 것 같음.


    /*
     * v01 190221 1402, 한지완
     * 출처 : github
     * 우선 터미널 창에서 cd /dev/tty를 쳐서 USB 포트가 어디에 설정되어 있는지 확인함.
     * 보통 USB0인데 imu가 무엇인지 아는게 무엇보다 중요하고. 그걸 설정함. -> 조만간 런치파일화 할 예정
     * imu의 보드레이트는 기본적으로 115200으로 설정되어 있으므로 수정함.
     *
     * hz 는 위의 define매크로 로 변경하여 바꿀 수 있다. 곧 런치화 할 예정.
     *
     * v02 190221 1552, 한지완
     * 추가 기능 :
     * 1. 런치파일에서 USB0와 같은 설정을 입력 할 수 있도록 함.
     */
    try
    {
        ser.setPort(setPortName); // ACM0 에서 USB0 으로 수정함. "/dev/ttyUSB0"
        ser.setBaudrate(BaudrateNum); // 비트레이트 수정함.
        serial::Timeout to = serial::Timeout::simpleTimeout(hzNum_I);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(hzNum_D);
//    ser.write("<sof1>"); // serial write 가능! 만세!
    while(ros::ok())
    {
        ros::spinOnce();

        if(ser.available())
        {
            std_msgs::String result;
            result.data = ser.read(ser.available());
            read_pub.publish(result);
            if(result.data == "<ok>")
            {
                okPub.publish(result);
            }
        }
        loop_rate.sleep();
    }

}

