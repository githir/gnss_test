/*
ConvGpsToLL 

※　カノニカル入力処理　＆　同期　通信方式

カノニカル：１行単位でread関数が動く
同期：データが来るまでread関数が待機

Author Nuk1 kondo copy right suzuki motor corp. 
2017.10.12

*/

#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/NavSatFix.h>
#include <gnss_suzuki/StringWithStamped.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <termios.h>
#define BAUDRATE B38400  //ボーレート

typedef struct {
   int    latitude_degree;  //緯度(度)
   double latitude_minute;  //緯度(分)
   double latitude;         //緯度

   int    longitude_degree; //経度(度)
   double longitude_minute; //経度(分)
   double longitude;        //経度
} GPGGA_FORMAT;

class ConvGpsToLL{
   public:
     ConvGpsToLL();   //breief: constructor

     bool inital_port(); //@breief: open port, store current port settings and set port setting
     bool read_serial_port();  //@breief: read serial port and input data to buf
     bool convert_data();  //@breief: convert serial data to longitude latitude topic
     bool publish_topic(int ret);  //@breief: publish longitude latitude topic
     bool close_port();  //@breief: close port and restorre port settings
      
   private:
     ros::Publisher ll_pub, gpgga_pub;
      
     GPGGA_FORMAT gpgga_data_;
      
     int fd; //ファイルディスクリプタ
     std::string data_str; //生データ保管
     struct termios newtio, oldtio;  // シリアル通信設定の構造体
     std::vector<std::string> elements;  //bunkatu-data
    
     std::string GPS_PORT_NUM; //port_namae

};

ConvGpsToLL::ConvGpsToLL()
{
   ros::NodeHandle private_nh("~");
   ll_pub = private_nh.advertise<sensor_msgs::NavSatFix>("/NavSatFix_msg", 1);
   gpgga_pub = private_nh.advertise<gnss_suzuki::StringWithStamped>("/GPGGA_msg", 1);

   private_nh.param("GPS_PORT", GPS_PORT_NUM, std::string("/dev/ttyUSB0"));
   ROS_INFO("port:%s, baudrate:%d", GPS_PORT_NUM.c_str(), BAUDRATE);
}

bool ConvGpsToLL::inital_port(){

  const char* GPS_PORT = GPS_PORT_NUM.c_str();
  fd = open(GPS_PORT, O_RDONLY);     // デバイスをオープンする
  if (fd < 0) {
      printf("open error\n");
      return -1;
  }
 
  tcgetattr(fd, &oldtio);     //現在のシリアルポートの設定を待避させる
   
  bzero(&newtio, sizeof(newtio));  //数値ゼロで埋める（初期化）
  /*
    BAUDRATE: ボーレートの設定
    CRTSCTS : 出力のハードウェアフロー制御
    CS8     : 8n1 (8 ビット，ノンパリティ，ストップビット 1)
    CLOCAL  : ローカル接続，モデム制御なし
    CREAD   : 受信文字(receiving characters)を有効にする．
 */
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
 
  /*
    IGNPAR  : パリティエラーのデータは無視する
    ICRNL   : CR を NL に対応させる(これを行わないと，他のコンピュータで
              CR を入力しても，入力が終りにならない)
    それ以外の設定では，デバイスは raw モードである(他の入力処理は行わない)
  */

  newtio.c_iflag = IGNPAR;  
  newtio.c_oflag = 0; // Raw モードでの出力

  /*
    ICANON  : カノニカル入力を有効にする
    全てのエコーを無効にし，プログラムに対してシグナルは送らせない
  */
  newtio.c_lflag = ICANON;

  //モデムラインをクリアし，ポートの設定を有効にする
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  
  return(0);

}

bool ConvGpsToLL::read_serial_port(){

  char buf[1000]="";
  if(read(fd,buf,500) == -1){  //1行(改行まで）read
    printf("read error\n");
    return -1;
  }

  data_str = std::string(buf);

  //受信チェックGPGGA文字受信してたらOK
   if(strncmp(buf, "$GPGGA", 6)!=0){
     ROS_INFO("no sentence GPGGA: %s", buf);
     return 1;
   }

  //NULL check
   elements.clear();
   boost::split(elements, data_str, boost::is_any_of(","));  // split sentence the word ","
   for(int i=0; i<13; i++){
     if(elements[i].empty()){
        ROS_INFO("sum elements is empty: %s", buf);
        return 1;
     }
     i++;
   }

   ROS_INFO("Correct GPS data accepted: %s", buf);
   return 0;

}


bool ConvGpsToLL::convert_data(){

   //elements[2]が緯度情報ddmm.mmmm
   gpgga_data_.latitude_degree = boost::lexical_cast<int>(elements[2].substr(0, 2));
   gpgga_data_.latitude_minute = boost::lexical_cast<double>(elements[2].substr(2, 7));
   gpgga_data_.latitude = (double) gpgga_data_.latitude_degree + gpgga_data_.latitude_minute / 60.0;   
   //elements[4]が経度情報dddmm.mmmm
   gpgga_data_.longitude_degree = boost::lexical_cast<int>(elements[4].substr(0, 3));
   gpgga_data_.longitude_minute = boost::lexical_cast<double>(elements[4].substr(3, 7));  
   gpgga_data_.longitude = (double) gpgga_data_.longitude_degree + gpgga_data_.longitude_minute / 60.0;

   return 0;
}

bool ConvGpsToLL::publish_topic(int ret){

   if(ret == 0){
     sensor_msgs::NavSatFix ll_topic;
     ll_topic.header.seq++;
     ll_topic.header.stamp = ros::Time::now();
     ll_topic.header.frame_id = "GPS_Frame";
     ll_topic.latitude = gpgga_data_.latitude;
     ll_topic.longitude = gpgga_data_.longitude;
     ll_pub.publish(ll_topic);
   }

   gnss_suzuki::StringWithStamped gpgga_topic;
   gpgga_topic.header.stamp = ros::Time::now();
   gpgga_topic.header.frame_id = "GPS_Frame";
   gpgga_topic.data = data_str;
   gpgga_pub.publish(gpgga_topic);

   return 0;
}

bool ConvGpsToLL::close_port(){

  tcsetattr(fd, TCSANOW, &oldtio);  /* 退避させた設定に戻す */
  close(fd);
  
}
  
int main(int argc, char *argv[]){
   ros::init(argc, argv, "ConvGpsToLL");
   int ret;
   ConvGpsToLL convgpstoll;
   int i=0;

   if( convgpstoll.inital_port() ){
      ROS_ERROR("at inital_port, sumthing error occurs");
      convgpstoll.close_port();  //initial　異常終了
      return -1;
   }
          
   ros::Rate pub_rate(200);
   while(ros::ok()){
     i++;
     ret = convgpstoll.read_serial_port();
     if(ret == -1){
       ROS_ERROR("at read_serial_port, sumthing error occurs");
     }else if(ret == 1){   //can read but incorrect data
       convgpstoll.publish_topic(ret);
     }else if(ret == 0){
       convgpstoll.convert_data();
       convgpstoll.publish_topic(ret);
     }
     pub_rate.sleep();    //ちょっと待つ
   }
    
   convgpstoll.close_port(); //正常終了
   return 0;
}
