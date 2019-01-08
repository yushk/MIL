#include <opencv2/opencv.hpp>
#include "utils.h"
#include <iostream>
#include <sstream>
#include "MIL.h"
#include <stdio.h>
#include<netinet/in.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <cstdlib>
using namespace cv;
using namespace std;
//Global variables
Rect box;
bool drawing_box = false;
bool gotBB = false;
bool tl = false;
bool rep = false;
bool fromfile=false;
string video;

void readBB(char* file){
  ifstream bb_file (file);
  string line;
  getline(bb_file,line);
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str())-x;// = (int)file["bb_w"];
  int h = atoi(y2.c_str())-y;// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
}
//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param){
  switch( event ){
  case CV_EVENT_MOUSEMOVE:
    if (drawing_box){
        box.width = x-box.x;
        box.height = y-box.y;
    }
    break;
  case CV_EVENT_LBUTTONDOWN:
    drawing_box = true;
    box = Rect( x, y, 0, 0 );
    break;
  case CV_EVENT_LBUTTONUP:
    drawing_box = false;
    if( box.width < 0 ){
        box.x += box.width;
        box.width *= -1;
    }
    if( box.height < 0 ){
        box.y += box.height;
        box.height *= -1;
    }
    gotBB = true;
    break;
  }
}

void print_help(char** argv){
  printf("use:\n     %s -p /path/parameters.yml\n",argv[0]);
  printf("-s    source video\n-b        bounding box file\n-tl  track and learn\n-r     repeat\n");
}

void read_options(int argc, char** argv,VideoCapture& capture,FileStorage &fs){
  for (int i=0;i<argc;i++){
      if (strcmp(argv[i],"-b")==0){
          if (argc>i){
              readBB(argv[i+1]);
              gotBB = true;
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-s")==0){
          if (argc>i){
              video = string(argv[i+1]);
              printf("open video\n");
              capture.open(video);
              fromfile = true;
          }
          else
            print_help(argv);

      }
      if (strcmp(argv[i],"-p")==0){
          if (argc>i){
              fs.open(argv[i+1], FileStorage::READ);
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-tl")==0){
          tl = true;
      }
      if (strcmp(argv[i],"-r")==0){
          rep = true;
      }
  }
}
float KPspeed=1.0,
KIspeed=0,
KDspeed=0,
speedErr=0,
latsSpeedErr=0,
speedIntegral=0;
// 计算速度
int PIDspeed(float setX,float currX){
  int speed=0;
  speedErr = setX - currX;
  speedIntegral += speedErr;
  speed = KPspeed * speedErr + KIspeed*speedIntegral + KDspeed*(speedErr -latsSpeedErr );
  latsSpeedErr = speedErr;
  return speed;
}
float KPdirection=1.0,
KIdirection=0,
KDdirection=0,
directionErr=0,
latsDirectionErr=0,
directionIntegral=0;

// 计算方向
int PIDdirection(float setWidth,float currWidth){
  int direction=0;
  directionErr = setWidth - currWidth;
  directionIntegral += directionErr;
  direction = KPdirection * directionErr + KIdirection*directionIntegral + KDdirection*(directionErr -latsDirectionErr );
  latsDirectionErr = directionErr;
  return direction;
}



int main(int argc, char * argv[]){
  const unsigned short SERVERPORT = 2001;
  const int MAXSIZE = 1024;
  const char* SERVER_IP = "192.168.8.1";
  char DATA[20];
  char wifisend[20];
  char wifisenderror[20] = "A000000000000000000";
  int sock, recvBytes;
  char buf[MAXSIZE];
  sockaddr_in serv_addr;
  const string address = "http://192.168.8.1:8083/?action=stream.mjpg";

  VideoCapture capture;
  capture.open(0);
  FileStorage fs;
  //Read options
  read_options(argc,argv,capture,fs);
  //Init camera
  if (!capture.isOpened())
  {
	cout << "capture device failed to open!" << endl;
    return 1;
  }

   // 通过wifi 获取小车摄像头视频数据

  // if (!capture.open(address))
  // {
	//   cout << "wifi failed to open!" << endl;
  //   return 1;
  // }
  
  //与小车通信，与路由简历socket 链接 
    // if( (sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    // {
    //     printf("socket create fail!\n");
    //     exit(1);
    // }
    bzero( &serv_addr, sizeof(serv_addr) );
    serv_addr.sin_family =  AF_INET;
    serv_addr.sin_port = htons(SERVERPORT);
    serv_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // if( connect(sock, (sockaddr*)&serv_addr, sizeof(sockaddr)) == -1)
    // {
    //     printf("connect error\n");
    //     exit(1);
    // }else{
    //   printf("connect success\n");
    // }


  //Register mouse callback to draw the bounding box
  cvNamedWindow("MIL",CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback("MIL", mouseHandler, NULL );
  //TLD framework
  TLD tld;
  //Read parameters file
  tld.read(fs.getFirstTopLevelNode());
  Mat frame;
  Mat last_gray;
  Mat first;
  if (fromfile){
      capture >> frame;
      cvtColor(frame, last_gray, CV_RGB2GRAY);
      frame.copyTo(first);
  }else{
      capture.set(CV_CAP_PROP_FRAME_WIDTH,680);
      capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  }

  ///Initialization
GETBOUNDINGBOX:
  while(!gotBB)
  {
    if (!fromfile){
      capture >> frame;
    }
    else
      first.copyTo(frame);
    cvtColor(frame, last_gray, CV_RGB2GRAY);
    drawBox(frame,box);
    imshow("MIL", frame);
    if (cvWaitKey(33) == 'q')
	    return 0;
  }
  if (min(box.width,box.height)<(int)fs.getFirstTopLevelNode()["min_win"]){
      cout << "Bounding box too small, try again." << endl;
      gotBB = false;
      goto GETBOUNDINGBOX;
  }
  //Remove callback
  cvSetMouseCallback( "MIL", NULL, NULL );
  // printf("Initial Bounding Box = x:%f y:%f h:%f w:%f\n",box.x,box.y,box.width,box.height);
  //Output file
  FILE  *bb_file = fopen("log.txt","w");
  //TLD initialization
  tld.init(last_gray,box,bb_file);

  ///Run-time
  Mat current_gray;
  BoundingBox pbox;
  vector<Point2f> pts1;
  vector<Point2f> pts2;
  bool status=true;
  int frames = 1;
  int detections = 1;
  int sp,dr;

  float boxX = box.x+(box.width/2); // 记录初始位置
  float boxWidth = box.width;
  float currBoxX,currBoxWidth;
  int count =0;
REPEAT:
  while(capture.read(frame)){
    //get frame
    cvtColor(frame, current_gray, CV_RGB2GRAY);
    //Process Frame
    tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
    //Draw Points
    

    if (status){
      count++;
      currBoxX = pbox.x+(pbox.width/2); 
      currBoxWidth = pbox.width;
      sp = PIDspeed(boxWidth, currBoxWidth);
      dr = PIDdirection(boxX,currBoxX);

      sprintf(DATA,"S%dD%dE",sp,dr);
      // if(count>2){
        // count=0;
      DATA[strlen(DATA)] = '\0';
      write(sock, DATA, strlen(DATA));
      printf("DATA:%s,,,,len%d\n",DATA,strlen(DATA));
      // }
      drawBox(frame,pbox);
      detections++;
      fprintf(bb_file,"boxX:%f,boxWidth:%f\n",boxX,boxWidth);
      fprintf(bb_file,"currBoxX:%f,currBoxWidth:%f\n",currBoxX,currBoxWidth);
      fprintf(bb_file,"speed:%d,direction:%d\n",sp,dr);
      fprintf(bb_file,"DATA:%s\n",DATA);
      fprintf(bb_file,"x:%d,y:%d,width:%d,height:%d\n",pbox.x,pbox.y,pbox.width,pbox.height);
      // write(sock, DATA, strlen(DATA));

    }
    //Display
    imshow("MIL", frame);
    //swap points and images
    swap(last_gray,current_gray);
    pts1.clear();
    pts2.clear();
    frames++;
    printf("Detection rate: %d/%d\n",detections,frames);
    if (cvWaitKey(33) == 'q')
      break;
  }
  if (rep){
    rep = false;
    tl = false;
    fclose(bb_file);
    bb_file = fopen("final_detector.txt","w");
    //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
    capture.release();
    capture.open(video);
    goto REPEAT;
  }
  fclose(bb_file);
  return 0;
}
