/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*TODO:flight_control_sample will by replace by flight_sample in the future*/
#include "flight_control_demo.hpp"
#include "flight_sample.hpp"

//#define OPEN_CV_INSTALLED

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<opencv2/imgproc/imgproc.hpp>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "facedetectcnn.h"
#include<cmath>

#define BUFFER_SIZE (256 * 1024 * 1024)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * 3)
#define DETECT_BUFFER_SIZE 0x20000

using namespace cv;
static uint8_t detect_buffer[DETECT_BUFFER_SIZE];


#endif

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include<iostream>
#include<queue>
#define DEVICE_NAME "/dev/ttyAMA0"
#define OPENMV_BUFFER_SIZE (64 * 1024)

static uint64_t get_time(void)
{
	timeval time;
	gettimeofday(&time, nullptr);
	uint64_t result = time.tv_sec;

	return (result * 1000000 + time.tv_usec);
}

static int init(int fd)
{
        struct termios oldtio;
        struct termios newtio;

        if (tcgetattr(fd, &oldtio) != 0)
        {
                printf("Error at tcgetattr\n");
                return -1;
        }

        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag |= CS8;
//	newtio.c_cflag |= CSTOPB;
//	newtio.c_cflag |= PARENB;
//	newtio.c_cflag &= ~CSIZE;
        newtio.c_cflag &= ~CSTOPB;
        newtio.c_cflag &= ~PARENB;

        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);

        if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        {
                printf("Error at tcgetattr\n");
                return -2;
        }

        return 0;
}

static char buffer1[OPENMV_BUFFER_SIZE];

float MidValue(float num[])
{
    for(int i=0; i<5; i++)
    {
            for(int j=i; j<5; j++)
            {
                     if(num[i]<num[j])
                     {
                                  float temp=num[i];
                                  num[i] = num[j];
                                  num[j] = temp;
                     }
            }
    }

    return num[2];
}



class datas{
public:
    datas(float data){
        m_data=data;
    }
    float m_data;
};



using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + att + Landing                          |"
      << std::endl;
  std::cout
      << "| [v] Monitored Takeoff + vel + Landing                          |"
      << std::endl;
  std::cout
      << "| [p] Monitored Takeoff + pos + Landing                          |"
      << std::endl;
  std::cout
      << "| [x] Monitored Takeoff + movebypos + Landing                    |"
      << std::endl;
  std::cout
      << "| [y] Monitored Takeoff + movebyvel + Landing                    |"
      << std::endl;
  std::cout
      << "| [z] Monitored Takeoff + movebyatt + Landing                    |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Vision Control + Landing               |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;

  char inputChar;
  std::cin >> inputChar;


  VideoCapture cap(0);
  printf("Start detection.\n");
  //cap.release();
  if(!cap.isOpened())
  {
     cout<<"can't open this camera"<<endl;
     return -1;
   }
   cap.set(CAP_PROP_FRAME_WIDTH, 1280);
   cap.set(CAP_PROP_FRAME_HEIGHT, 720);
   Mat frame1;
   int ij = 0;

   int fd = open(DEVICE_NAME, O_RDWR | O_NOCTTY);
   if (fd < 0)
   {
          printf("Error at open\n");
          return 0;
   }

   if (init(fd) != 0)
   {
          printf("Error at init\n");
          close(fd);
          return 0;
   }

   float x[5];
   float y[5];
   float z[5];
   float fx;
   float fy;
   float fz;
   float mid_fx;
   float mid_fy;
   float mid_fz = -201.0f;
   queue<datas> qx;
   queue<datas> qy;
   queue<datas> qz;


   float distance_real_x = -1.0;
   float distance_real_y = -1.0;
   float distance_real_h = -1.0;
   int distance_img_w;
   int distance_img_h;
   int img_x = 160;
   int img_y = 120;
   float pile = 318;
   float f = 599;

  //distance_real_h = 201.0f;

  switch (inputChar) {
    case 'a':
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      {
        uint64_t start1 = get_time();
        //clock_t startTime, endTime1;
        //startTime = clock();//计时开始
        DSTATUS("Roll+\n");
        int i = 0;
        while(i < 5)
        {
          vehicle->control->attitudeAndVertPosCtrl(30.0f, 0.0f, 0.0f, 0.0f);
          usleep(200 * 1000); i++;}
        usleep(5000000);
        uint64_t end1 = get_time();
        std::cout << "Roll+ duration: " << (end1 - start1) / 1000000 << std::endl;
        /*DSTATUS("Roll-\n");
        i = 0;
        while(i < 5)
        {
          vehicle->control->attitudeAndVertPosCtrl(-30.0f, 0.0f, 0.0f, 0.0f);
          usleep(200 * 1000); i++;}

        //moveByattitudeAndVertPosCtrl(vehicle, -1.0f, 0.0f, 0.0f, 0.0f);
        endTime1 = clock();//计时开始
        std::cout << "The attx time is:" <<(double)(endTime1 - startTime) / 1000000 << "s" << std::endl;
        */
        //std::cout << "detection duration: " << end1 - start1 << std::endl;
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'v':
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      {
        uint64_t start1 = get_time();
        DSTATUS("Velx+\n");
        int i = 0;
        while(i < 1)
        {
          vehicle->control->velocityAndYawRateCtrl(1.0f, 0.0f, 0.0f, 0.0f);
          usleep(200 * 1000); i++;}
        usleep(5000000);
        uint64_t end1 = get_time();
        std::cout << "Velx duration: " << (end1 - start1) / 1000000 << std::endl;
      /*DSTATUS("Velx-\n");
      i = 0;
      while(i < 5)
      {
        vehicle->control->velocityAndYawRateCtrl(-1.0f, 0.0f, 0.0f, 0.0f);
        usleep(200 * 1000); i++;}
      */
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'p':
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      {
        uint64_t start1 = get_time();
        DSTATUS("Px+\n");
        vehicle->control->positionAndYawCtrl(1.0f, 0.0f, 0.0f, 0.0f);
        //vehicle->control->positionAndYawCtrl(1.0f, 0.0f, 0.0f, 0.0f);
        usleep(5000000);
        uint64_t end1 = get_time();
        std::cout << "Px duration: " << (end1 - start1) / 1000000 << std::endl;
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'x':
      DSTATUS("Position control!\n");
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      while(1)
      {
        uint64_t start1 = get_time();
        DSTATUS("Px1\n");
        float inputPosx, inputPosy, inputPosz, inputyaw;
        std::cout << "Posx: ";
        std::cin >> inputPosx;
        std::cout << "Posy: ";
        std::cin >> inputPosy;
        std::cout << "Posz: ";
        std::cin >> inputPosz;
        std::cout << "Yaw: ";
        std::cin >> inputyaw;
        moveByPositionOffset(vehicle, inputPosx, inputPosy, inputPosz, inputyaw);
        uint64_t end0 = get_time();
        std::cout << "Velocity+ duration: " << (end0 - start1) / 100000 << "s/10" << std::endl;
        if(abs(inputPosx) < 0.001 && abs(inputPosy) < 0.001 && 
          abs(inputPosz) < 0.001 && abs(inputyaw) < 0.001)
        {
          int inputstop;
          std::cout << "Land or not: ";
          std::cin >> inputstop;
          if(inputstop != 0){ break;}
        }
        usleep(5000000);
        
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'y':
      DSTATUS("Velocity control!\n");
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      while(1)
      {
        DSTATUS("Vx1\n");
        float inputVelx, inputVely, inputVelz, inputyaw;
        std::cout << "Velx: ";
        std::cin >> inputVelx;
        std::cout << "Vely: ";
        std::cin >> inputVely;
        std::cout << "Velz: ";
        std::cin >> inputVelz;
        std::cout << "Yaw: ";
        std::cin >> inputyaw;
        uint64_t start1 = get_time();
        moveByVelocity(vehicle, inputVelx, inputVely, inputVelz, 0.0f);
        uint64_t end0 = get_time();
        std::cout << "Velocity+ duration: " << (end0 - start1) / 100000 << "s/10" << std::endl;
        
        if(abs(inputVelx) < 0.001 && abs(inputVely) < 0.001 && 
          abs(inputVelz) < 0.001 && abs(inputyaw) < 0.001)
        { 
          int inputstop;
          std::cout << "Land or not: ";
          std::cin >> inputstop;
          if(inputstop != 0){ break;}
        }
        usleep(5000000);
        /*DSTATUS("Vx-1\n");
        end0 = get_time();
        moveByVelocity(vehicle, -1.0f, 0.0f, 0.0f, 0.0f);
        uint64_t end1 = get_time();
        std::cout << "Velocity- duration: " << (end1 - end0) / 100000 << "s/10" << std::endl;
        usleep(5000000);*/
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'z':
      DSTATUS("Attitude control!\n");
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");
      while(1)
      {      
        DSTATUS("Pitch10\n");
        float inputroll, inputpitch, inputyaw, inputPosz;
        std::cout << "Roll: ";
        std::cin >> inputroll;
        std::cout << "Pitch: ";
        std::cin >> inputpitch;
        std::cout << "Yaw: ";
        std::cin >> inputyaw;
        std::cout << "Posz: ";
        std::cin >> inputPosz;
        uint64_t start1 = get_time();
        moveByAttitudeAndVertOffset(vehicle, inputroll, inputpitch, inputyaw, inputPosz);
        uint64_t end0 = get_time();
        std::cout << "Attitude+ duration: " << (end0 - start1) / 100000 << "s/10" << std::endl;
        
        if(abs(inputroll) < 0.001 && abs(inputpitch) < 0.001 && 
          abs(inputyaw) < 0.001 && abs(inputPosz) < 0.001)
        { 
          int inputstop;
          std::cout << "Land or not: ";
          std::cin >> inputstop;
          if(inputstop != 0){ break;}
        }
        usleep(5000000);
        /*DSTATUS("Pitch-10\n");
        end0 = get_time();
        moveByAttitudeAndVertOffset(vehicle, 0.0f, -10.0f, 0.0f, 0.0f);
        uint64_t end1 = get_time();
        std::cout << "Attitude- duration: " << (end1 - end0) / 100000 << "s/10" << std::endl;
        usleep(5000000);*/
      }
      DSTATUS("Landing over!\n");
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      DSTATUS("Take off over!\n");

      while(mid_fz < -200.0f)
      {
        cap>>frame1;
        Rect rect(160, 0, 960, 720);
        Mat frame = frame1(rect);
        //imwrite("my.jpg",frame);
        resize(frame, frame, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_LINEAR);
        uint64_t start = get_time();
        int *pResults = facedetect_cnn(detect_buffer, (uint8_t *)frame.data, FRAME_WIDTH, FRAME_HEIGHT, 3 * FRAME_WIDTH);
        uint64_t end = get_time();
        std::cout << "detection duration1: " << end - start << std::endl;
        //imwrite("my1.jpg",frame);
        //size = frame.size;
        //cout << "size:" << frame.size << endl;
        for(int i = 0; i < (pResults ? *pResults : 0); i++)
        {
          short * p = ((short*)(pResults+1))+142*i;
          int confidence = p[0];
          int x = p[1];
          int y = p[2];
          int w = p[3];
          int h = p[4];

          img_x = 160;
          img_y = 120;

          distance_img_w = x - img_x;
          distance_img_h = y - img_y;

          int dis_min = w < h ? w : h;
  
          distance_real_x = pile/w*distance_img_w;
          distance_real_y = pile/h*distance_img_h;
          distance_real_h = pile/dis_min/3*f;

          cout << "distance_real_x:" << distance_real_x << endl;
          cout << "distance_real_y:" << distance_real_y << endl;
          cout << "distance_real_h:" << distance_real_h << endl;

          ij ++;
          char sScore[1024];
          snprintf(sScore, 1024, "x = %f y = %f h = %f score = %d", distance_real_x, distance_real_y, distance_real_h, confidence);
          cv::putText(frame, sScore, cv::Point(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.1, cv::Scalar(0, 255, 0), 1);       
          
          //draw face rectangle
          rectangle(frame, Rect(x, y, w, h), Scalar(0, 255, 0), 2);        
        }
        imshow("Camera Capture", frame);

        int count = 0;
        int NumberOfBytesRead = 0;
        while (NumberOfBytesRead < 24)
        {
                   int dwBytesRead = read(fd, buffer1 + NumberOfBytesRead, OPENMV_BUFFER_SIZE);
                   if (dwBytesRead < 1)
                   {
                           ++count;
                           usleep(10 * 1000);
                           if (count > 100)
                           {
                                   count = 0;
                                   printf("Openmv did not work\n");
                           }
                           continue;
                   }
                   NumberOfBytesRead += dwBytesRead;
                   printf("%d byte(s) received\n", dwBytesRead);
        }
        float *b = (float *)buffer1;
        float tx, ty, tz, rx, ry, rz;
        tx = b[0];
        ty = b[1];
        tz = b[2];
        rx = b[3];
        ry = b[4];
        rz = b[5];

        printf("tx:%f, ty:%f, tz:%f\n", tx, ty, tz);

        if((tx==ty && ty==tz && tx==tz && tx==-1)&&(distance_real_x==distance_real_y && distance_real_y==distance_real_h && distance_real_x==distance_real_h && distance_real_x==-1))
        {
           fx = -1.0;
           fy = -1.0;
           fz = -1.0;
           cout<<"Both raspberry and openmv can not detect the pile"<<endl;
        }
        else if (not(tx==ty && ty==tz && tx==tz && tx==-1))
        {
           fx = tx;
           fy = ty;
           fz = tz;
           cout<<"Using openmv detection result"<<endl;
        }
        else
        {

           fx = distance_real_x;
           fy = distance_real_y;
           fz = distance_real_h;
           cout<<"Using raspberry detection result"<<endl;

         }
        printf("fx:%f, fy:%f, fz:%f\n", fx, fy, fz);

        datas qx_data(fx);
        datas qy_data(fy);
        datas qz_data(fz);

        while(qx.size()<5)
        {
        qx.push(qx_data);
        qy.push(qy_data);
        qz.push(qz_data);
        }
        queue<datas>qx_bak = qx;
        queue<datas>qy_bak = qy;
        queue<datas>qz_bak = qz;
        for (int i = 0; i < 5; i++)
        {
                   x[i] = qx_bak.front().m_data;
                   y[i] = qy_bak.front().m_data;
                   z[i] = qz_bak.front().m_data;
                   qx_bak.pop();
                   qy_bak.pop();
                   qz_bak.pop();
                   printf("x[%d]:%f  ", i, x[i]);
                   printf("y[%d]:%f  ", i, y[i]);
                   printf("z[%d]:%f\n", i, z[i]);
        }
        mid_fx = MidValue(x);
        mid_fy = MidValue(y);
        mid_fz = MidValue(z);

        qx.pop();
        qy.pop();
        qz.pop();

        printf("mid_fx:%f, mid_fy:%f, mid_fz:%f\n", mid_fx, mid_fy, mid_fz);
        if(!(fabs(mid_fx+1)<0.01f && fabs(mid_fy+1)<0.01f && fabs(mid_fz+1)<0.01f)){
        if( fabs(mid_fx) < 100 && fabs(mid_fy) < 100){
          moveByPositionOffset(vehicle, mid_fx / 1000, mid_fy / 1000, -0.2f, 0);
          DSTATUS("Step h over!\n");
        } else{
          moveByPositionOffset(vehicle, mid_fx / 1000, mid_fy / 1000, 0, 0);
          DSTATUS("Step xy over!\n");
        }
        } else {
            mid_fz = -201.0f;
            moveByPositionOffset(vehicle, mid_fx / 1000, mid_fy / 1000, 0.2f, 0);
            DSTATUS("Step up over!\n");

        }

        end = get_time();
        std::cout << "detection duration2: " << end - start << std::endl;
        waitKey(20);
      }
      close(fd);
      monitoredLanding(vehicle);
      
      break;

    /*! @NOTE: case 'c' only support for m210 V2*/
    case 'c':
      /*! Turn off rtk switch */
      ErrorCode::ErrorCodeType ret;
      ret = vehicle->flightController->setRtkEnableSync(
          FlightController::RtkEnabled::RTK_DISABLE, 1);
      if (ret != ErrorCode::SysCommonErr::Success) {
        DSTATUS("Turn off rtk switch failed, ErrorCode is:%8x", ret);
      } else {
        DSTATUS("Turn off rtk switch successfully");
      }

      /*!  Take off */
      monitoredTakeoff(vehicle);

      /*! Move to higher altitude */
      moveByPositionOffset(vehicle, 0, 0, 30, 0);

      /*! Move a short distance*/
      moveByPositionOffset(vehicle, 10, 0, 0, -30);

      /*! Set aircraft current position as new home location */
      setNewHomeLocation(vehicle);

      /*! Set new go home altitude */
      setGoHomeAltitude(vehicle, 50);

      /*! Move to another position */
      moveByPositionOffset(vehicle, 40, 0, 0, 0);

      /*! go home and  confirm landing */
      goHomeAndConfirmLanding(vehicle, 1);
      break;

    default:
      break;
  }

  return 0;
}
