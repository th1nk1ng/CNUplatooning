#include "zhelpers.h"
#include "Altino.h"
#include <unistd.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstring>
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/socket.h>
#include <pthread.h>
#include <errno.h>
#include <stdio.h> 
#include <time.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#define MATH_MIN3(x,y,z)		( (y) <= (z) ? ((x) <= (y) ? (x) : (y)) : ((x) <= (z) ? (x) : (z)) )
#define MATH_MAX3(x,y,z)		( (y) >= (z) ? ((x) >= (y) ? (x) : (y)) : ((x) >= (z) ? (x) : (z)) )

using namespace cv;

using namespace std;

void init();
bool myIpCheck(char* compareIp);
void* watchDog(void* Para);
void separate(char* ip);
void duplicate(int num, char* address);
void receive();

void* context = zmq_ctx_new();
void* publisher = zmq_socket(context, ZMQ_PUB);
void* subscriber = zmq_socket(context, ZMQ_SUB);
char group[1];

void auto_drive_leader(int road_identify_value);
void auto_drive_nonleader(int road_identify_value);
void speed_adjust(int value);
void colorExtraction(Mat& input);
int laneDetection(Mat& input);

int robot_speed;
int road_identify_value;
char sign_value[20];
char color_value[20];
char color_value_red[20];
char traffic_light;

SensorData infrared_sensor;
int my_position;
int my_num;
int redcount = 0;
int greencount = 0;
int merge_count[10] = { 0, };
int seperate_count;

/*오픈소스 : http://darkpgmr.tistory.com/66 */
struct hsv_color {
	unsigned char h;        // Hue: 0 ~ 255 (red:0, green: 85, blue: 171)
	unsigned char s;        // Saturation: 0 ~ 255
	unsigned char v;        // Value: 0 ~ 255
};

/*RGB값을 HSV로 변환하는 코드*/
hsv_color RGB2HSV(unsigned char r, unsigned char g, unsigned char b)
{
	int rgb_min, rgb_max;
	rgb_min = MATH_MIN3((int)b, (int)g, (int)r);
	rgb_max = MATH_MAX3((int)b, (int)g, (int)r);

	hsv_color hsv;
	hsv.v = rgb_max;
	if (hsv.v == 0) {
		hsv.h = hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * (rgb_max - rgb_min) / hsv.v;
	if (hsv.s == 0) {
		hsv.h = 0;
		return hsv;
	}

	if (rgb_max == (int)r) {
		hsv.h = 0 + 43 * ((int)g - (int)b) / (rgb_max - rgb_min);
	}
	else if (rgb_max == (int)g) {
		hsv.h = 85 + 43 * ((int)b - (int)r) / (rgb_max - rgb_min);
	}
	else /*(rgb_max == rgb.b)*/ {
		hsv.h = 171 + 43 * ((int)r - (int)g) / (rgb_max - rgb_min);
	}

	return hsv;
}


/* 각 자동차들의 정보를 저장하는 Node*/
class NodeAddr{
public:
	char* address;			//로봇의 주소
	char group;				//속해있는 군집
	int position;			//리더여부
	void *subscriber;		//상태 체크 subscriber
	int num;				//고유번호
	static int groupNum;	//군집갯수

	NodeAddr(char* address, char group, int num, int position){
		this->address = address;
		this->position = position;
		this->group = group;
		this->num = num;
		subscriber = zmq_socket(context, ZMQ_SUB);
		zmq_connect(subscriber, address);
		zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "K", 1);
	};
};

vector<NodeAddr*> nodeList;
int NodeAddr::groupNum = 1;
char sender[50];
char* myaddress;

int err;
struct sched_param param;
pthread_attr_t attr;
pthread_t th;

int main(void){

	/* Node가 들어갈 자리를 만들고 실제 Node저장*/
	nodeList.reserve(4);
	nodeList.push_back(new NodeAddr("tcp://192.168.0.105:5555", 'A', 1, 1));
	nodeList.push_back(new NodeAddr("tcp://192.168.0.120:5555", 'A', 2, 0));


	init();
	/*자신의 정보 체크*/
	for (int i = 0; i < nodeList.size(); i++){
		if (strcmp(nodeList[i]->address, myaddress) == 0){
			my_position = nodeList[i]->position;		//리더인지 비리더인지
			my_num = nodeList[i]->num;	//자신의 번호가 몇 번인지
		}
	}

	VideoCapture cap(-1);

	if (!cap.isOpened())
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}

	//카메라 프레임 크기
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

	Mat frame;

	int ccc = 0;
	while (1){
		bool bSuccess = cap.read(frame);
		flip(frame, frame, -1);

		int road_identity_value = laneDetection(frame);
		if (ccc % 10 == 0){
			colorExtraction(frame);
			if (ccc >= 30)
				ccc = 0;
		}
		ccc++;

		receive();		//군집내에서 원하는 토픽을 받아온다

		/* 로봇이 빨간색을 인지한 경우*/
		if (strcmp(color_value_red, "red") == 0){
			/*빨간불이므로 정지->속도를 0으로 변경*/
			robot_speed = 0;
			Go(robot_speed, robot_speed);

			/*리더인 경우, 비리더들에게 속도를 0으로 바꾸라는 메시지 전달*/
			if (my_position == 1) {
				/*군집 내에 Publish*/
				s_sendmore(publisher, group);
				s_send(publisher, "spd 0");
				sprintf(color_value_red, "%s", NULL);
			}
			strcpy(color_value_red, "");
			do{
				strcpy(color_value_red, "");	//값 초기화
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				colorExtraction(frame);

			} while (strcmp(color_value_red, "red") == 0);	//빨간불일 때 반복

		}

		/*로봇이 초록색을 인지한 경우*/
		else if (strcmp(color_value, "green") == 0){
			/*초록불이므로 출발->속도를 300으로 변경*/
			robot_speed = 300;
			Go(robot_speed, robot_speed);

			/*리더인 경우, 비리더들에게 속도를 300으로 바꾸라는 메시지 전달*/
			if (my_position == 1) {
				/*군집 내에 Publish*/
				s_sendmore(publisher, group);
				s_send(publisher, "spd 300");
				sprintf(color_value, "%s", NULL);
			}
			strcpy(color_value, "");	//color_value값 초기화	
		}
		/*로봇이 파란색을 인지한 경우, 오른쪽 표지판이므로 우회전*/
		else if (strcmp(color_value, "blue") == 0){
			Steering2(127, 0);	//바퀴를 오른쪽으로 회전
			Go(650, 0);	//우회전하기 위해 왼쪽 바퀴 속도 증가
			waitKey(5500);	//5.5초간 유지
			do {
				Steering2(127, 0);	//바퀴를 오른쪽으로 회전
				Go(650, 0);			//우회전하기 위해 왼쪽 바퀴 속도 증가
				sprintf(color_value, "%s", NULL);
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				road_identity_value = laneDetection(frame);

			} while (!(-45 <= road_identity_value && road_identity_value <= 45));	//길이 곡선인 경우, 계속 반복
			strcpy(color_value, "");	//color_value 값 초기화
		}

		/*로봇이 하늘색을 인지한 경우, 왼쪽 표지판이므로 좌회전*/
		else if (strcmp(color_value, "light_blue") == 0){
			Steering2(255, 0);		//바퀴를 왼쪽으로 회전
			Go(0, 650);				//좌회전하기 위해 오른쪽 바퀴 속도 증가
			waitKey(5500);			//5.5초간 유지
			do {
				Steering2(255, 0);	//바퀴를 왼쪽으로 회전
				Go(0, 650);			//좌회전하기 위해 오른쪽 바퀴 속도 증가
				sprintf(color_value, "%s", NULL);
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				road_identity_value = laneDetection(frame);
			} while (!(-45 <= road_identity_value && road_identity_value <= 45));	//길이 곡선인 경우, 계속 반복
			strcpy(color_value, "");	//color_value 값 초기화
		}
		/* 위 4가지 색 이외의 색을 인식한 경우*/
		else{
			/* count 값 초기화*/
			redcount = 0;
			greencount = 0;

			/*비리더의 속도가 0인 경우*/
			if (my_position == 0 && robot_speed == 0){
				Go(0, 0);
				continue;	//리더의 명령 전까지는 다른 코드를 실행하지 않고 그대로 정지한 상태 유지
			}

			/*그 외의 경우 주행*/
			else{
				/*로봇의 속도를 300으로 하여 주행*/
				robot_speed = 300;
				Go(robot_speed, robot_speed);

				/*리더인 경우, 비리더들에게 속도를 300으로 하라는 메시지 전달*/
				if (my_position == 1) {
					/*군집 내에 Publish*/
					s_sendmore(publisher, group);
					s_send(publisher, "spd 300");
				}
			}
		}

		if (my_position == 1){	//리더인 경우, 리더 자율주행 코드 실행
			auto_drive_leader(road_identity_value);
		}
		else {	//비리더인 경우, 비리더 자율주행 코드 실행
			auto_drive_nonleader(road_identity_value);
		}

		if (!bSuccess)
		{
			cout << "Cannot read a frame from camera" << endl;
			continue;
		}

		if (waitKey(1) == 27)
		{
			cout << "Exit" << endl;
			break;
		}
	}

	/*thread 종료*/
	pthread_join(th, 0);
	return 0;
}

/*비리더 자율주행 메소드*/
void auto_drive_nonleader(int road_identify_value){
	/*길이 인식한 값이 음수일 경우, 왼쪽으로 휘어진 도로*/
	if (road_identify_value<0){
		if (road_identify_value == -80){	//왼쪽에서 나올 수 있는 최대값인 -80일 경우
			road_identify_value = -100;		//더 급격한 회전을 위해 값을 -100으로 변경
		}
		Steering2(128 - road_identify_value, 0); //바퀴 방향을 해당 값만큼 왼쪽으로 변경
	}

	/*길을 인식한 값이 양수일 경우, 오른쪽으로 휘어진 도로*/
	else if (road_identify_value>0){
		if (road_identify_value == 80){	//오른쪽에서 나올 수 있는 최대값인 80일 경우
			road_identify_value = 100;	//더 급격한 회전을 위해 값을 100으로 변경
		}
		Steering2(road_identify_value, 0);	//바퀴 방향을 해당 값만큼 오른쪽으로 변경
	}

	/*길을 인식한 값이 0일 경우, 직선 도로*/
	else if (road_identify_value == 0){
		Steering2(0, 0);	//바퀴 방향을 가운데로 변경
	}
	Go(robot_speed, robot_speed);	//현재 속도로 주행
	strcpy(color_value, "");			//color_value 값 초기화
}

/*리더 자율주행 메소드*/
void auto_drive_leader(int road_identify_value){
	/*길을 인식한 값이 음수일 경우, 왼쪽으로 휘어진 도로*/
	if (road_identify_value<0){
		if (road_identify_value == -80){	//왼쪽에서 나올 수 있는 최대값인 -80일 경우
			road_identify_value = -100;		//더 급격한 회전을 위해 값을 -100으로 변경
		}
		Steering2(128 - road_identify_value, 0);//바퀴 방향을 해당 값만큼 왼쪽으로 변경
	}

	/*길을 인식한 값이 양수일 경우, 오른쪽으로 휘어진 도로*/
	else if (road_identify_value>0){
		if (road_identify_value == 80){	//오른쪽에서 나올 수 있는 최대값인 80일 경우
			road_identify_value = 100;	//더 급격한 회전을 위해 값을 100으로 변경
		}
		Steering2(road_identify_value, 0);	//바퀴 방향을 해당 값만큼 오른쪽으로 변경
	}

	/*길을 인식한 값이 0일 경우, 직선 도로*/
	else if (road_identify_value == 0){
		Steering2(0, 0);	//바퀴 방향을 가운데로 변경
	}
	Go(robot_speed, robot_speed);	//현재 속도로 주행
}

/*속도를 변경하는 메소드*/
void speed_adjust(int value){
	robot_speed = value;	//파라미터 값으로 속도 변경 후,
}

int laneDetection(Mat& input){
	cv::Mat image = input;

	//	cv::namedWindow("Original Image");
	//	cv::imshow("Original Image", image);

	// 케니 알고리즘 적용

	cv::Mat contours;

	cv::Canny(image, // 그레이레벨 영상
		contours, // 결과 외곽선
		240,  // 낮은 경계값
		250);  // 높은 경계값

	// 넌제로 화소로 외곽선을 표현하므로 흑백 값을 반전

	cv::Mat contoursInv; // 반전 영상

	cv::threshold(contours, contoursInv, 100, 255, cv::THRESH_BINARY_INV);
	// 밝기 값이 128보다 작으면 255가 되도록 설정

	//cv::namedWindow("Canny Contours");

	//cv::imshow("Canny Contours", contoursInv);

	int preferedHeight = image.size().height*0.6, searchThreshold = image.size().width / 2 * 0.95;
	int left = -1, right = -1;

	for (int i = contoursInv.size().width / 2; i < contoursInv.size().width / 2 + searchThreshold; i++) {

		if (contoursInv.at<unsigned char>(preferedHeight, i) == 0) {
			right = i;
			break;
		}
	}
	for (int i = contoursInv.size().width / 2; i > contoursInv.size().width / 2 - searchThreshold; i--) {
		if (contoursInv.at<unsigned char>(preferedHeight, i) == 0) {
			left = i;
			break;
		}
	}

	int returnValue = 0;
	if ((right + left) / 2 > contoursInv.size().width / 2 + 15) {
		returnValue = (right + left) / 2 - contoursInv.size().width / 2;

	}
	else if ((right + left) / 2 < contoursInv.size().width / 2 - 15) {
		returnValue = (right + left) / 2 - contoursInv.size().width / 2;

	}
	else {
		returnValue = 0;
	}

	if (right != -1 && left != -1 && (right - left) <  contoursInv.size().width*0.3)
		left = -1;

	if (right == -1){ //right가 비었을때
		int left2 = -1;
		for (int i = contoursInv.size().width / 2; i > contoursInv.size().width / 2 - searchThreshold; i--) {
			if (contoursInv.at<unsigned char>(preferedHeight - 10, i) == 0) {
				left2 = i;
				break;
			}
		}

		if (left - left2 > 0){ // 왼쪽회전
			return -80;
		}
		else // 우측회전 {
			return 80;
		}


	}
	else if (left == -1){ // left 가 비었을떄
		int right2 = -1;
		for (int i = contoursInv.size().width / 2; i < contoursInv.size().width / 2 + searchThreshold; i++) {
			if (contoursInv.at<unsigned char>(preferedHeight - 10, i) == 0) {
				right2 = i;
				break;
			}
		}
		if (right - right2 > 0){ // 왼쪽회전
			return -80;
		}
		else // 우측회전 {
			return 80;
		}
	}
	return returnValue;
}

/*색상 검출 코드*/
/*오픈 소스 : http://egloos.zum.com/newstyle/v/2707246 참고 */
void colorExtraction(Mat& input){
	IplImage* frame = new IplImage(input);	//현재 카메라 영상 저장

	char c;
	unsigned char  B, G, R;
	int x, y;

	bool check[7][320][240];	//frame에 해당 색상이 차지하는 비율을 저장하기 위한 배열

	if (frame == NULL) return;

	for (int i = 0; i < frame->height; i++)
	for (int j = 0; j < frame->width; j++)
	{
		CvScalar v = cvGet2D(frame, i, j);	//CvScalar : 색을 저장하는 구조체
		//frame의 (j, i)에 있는 픽셀의 색 값을 v에 저장

		if (v.val[0] == NULL || v.val[1] == NULL || v.val[2] == NULL)
			continue;
		B = v.val[0];
		G = v.val[1];
		R = v.val[2];

		hsv_color rgb_to_hsv = RGB2HSV(R, G, B);

		/*특정 색의 HSV 값에 범위를 주고 범위에 해당되면 check배열에 true로 저장*/
		if ((int)rgb_to_hsv.h >= 242 && (int)rgb_to_hsv.h <= 249 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//red
			x = j;
			y = i;
			check[0][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 70 && (int)rgb_to_hsv.h <= 89 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//green
			x = j;
			y = i;
			check[1][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 135 && (int)rgb_to_hsv.h <= 153 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//blue
			x = j;
			y = i;
			check[2][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 60 && (int)rgb_to_hsv.h <= 70 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//lgreen
			x = j;
			y = i;
			check[3][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 114 && (int)rgb_to_hsv.h <= 124 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//lblue
			x = j;
			y = i;
			check[4][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 160 && (int)rgb_to_hsv.h <= 175 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//purple
			x = j;
			y = i;
			check[5][x][y] = true;
		}
		else if ((int)rgb_to_hsv.h >= 229 && (int)rgb_to_hsv.h <= 237 && (int)rgb_to_hsv.s >= 102 && (int)rgb_to_hsv.v >= 102)
		{//rose
			x = j;
			y = i;
			check[6][x][y] = true;
		}

		else{
			x = j;
			y = i;
			check[0][x][y] = false;
			check[1][x][y] = false;
			check[2][x][y] = false;
			check[3][x][y] = false;
			check[4][x][y] = false;
			check[5][x][y] = false;
			check[6][x][y] = false;

		}
	}

	int red_count = 0;
	int green_count = 0;
	//int yellow_count = 0;
	int blue_count = 0;
	int light_green_count = 0;
	int light_blue_count = 0;
	int rose_count = 0;
	int purple_count = 0;


	bool color_detect = false, front_detect = false;
	/*check배열을 검사하며 픽셀에 어떤 색이 저장되어있는지 확인하고 count*/
	for (int i = 0; i + 50 < frame->width; i = i + 10){
		for (int j = 0; j + 50 < frame->height; j = j + 10){
			for (int k = i; k < i + 50; k++){
				for (int l = j; l < j + 50; l++){
					if (check[0][k][l] == true)
						red_count++;
					if (check[1][k][l] == true)
						green_count++;
					if (check[2][k][l] == true)
						blue_count++;
					if (check[3][k][l] == true)
						light_green_count++;
					if (check[4][k][l] == true)
						light_blue_count++;
					if (check[5][k][l] == true)
						purple_count++;
					if (check[6][k][l] == true)
						rose_count++;

				}
			}
			//신호 인식
			//프레임의 40*40*0.75 이상 같은 색이 검출되어야 인식하도록
			if ((red_count >= 40 * 40 * 0.75 || green_count >= 40 * 40 * 0.75) && !color_detect){

				if (red_count >= 40 * 40 * 0.75){
					strcpy(color_value_red, "red");
					return;
				}
				if (green_count >= 40 * 40 * 0.75){
					printf("green\n");
					return;
				}
				color_detect = true;
			}
			//로봇 뒤의 스티커 색상 (로봇인지 장애물인지 판단할 때 사용)
			if ((blue_count >= 40 * 40 * 0.75 || light_blue_count >= 50 * 50 * 0.75 || purple_count >= 40 * 40 * 0.75 || rose_count >= 50 * 50 * 0.75 || light_green_count >= 50 * 50 * 0.75) && !front_detect){
				if (blue_count >= 40 * 40 * 0.75){
					strcpy(color_value, "blue");
					return;
				}
				if (light_green_count >= 50 * 50 * 0.75){
					//strcpy(color_value,"light_green");
					return;
				}
				if (light_blue_count >= 50 * 50 * 0.75){
					//strcpy(color_value,"light_blue");
					return;
				}
				if (purple_count >= 40 * 40 * 0.75){
					strcpy(color_value, "purple");
					return;
				}
				if (rose_count >= 50 * 50 * 0.75){
					//strcpy(color_value,"rose");
					return;
				}
				front_detect = true;
			}

			red_count = 0;
			green_count = 0;
			//yellow_count=0;
			blue_count = 0;
			//orange_count=0;
			purple_count = 0;
		}
	}
}

/* 자신의 ip를 확인하는 함수*/
bool myIpCheck(char* compareIp){
	char hostname[256];
	size_t hostnamelen = 256;

	struct ifaddrs *ifp = NULL;
	if (getifaddrs(&ifp) < 0){
		printf("getifaddrs() fail\n");
		return false;
	}

	struct ifaddrs *ifa = NULL;
	for (ifa = ifp; ifa; ifa = ifa->ifa_next){
		socklen_t salen;
		char ip[125];
		if (ifa->ifa_addr->sa_family == AF_INET){
			salen = sizeof(struct sockaddr_in);
		}

		if (getnameinfo(ifa->ifa_addr, salen, ip, sizeof(ip), NULL, 0, NI_NUMERICHOST) < 0){
			continue;
		}
		if (strstr(compareIp, ip) != NULL){
			return true;
		}
	}

	return false;
}

/* 실시간으로 로봇들의 상태를 확인하는 함수*/
void* watchDog(void* Para){
	char* address;
	char* contents;
	char* token;

	while (1){
		/* 상태 check 메시지 Publish*/
		s_sendmore(publisher, "K");
		s_send(publisher, sender);

		if (strcmp(sender, "check") != 0){
			sprintf(sender, "%s", "check");
		}

		for (int i = 0; i < nodeList.size(); i++){
			address = s_recv(nodeList[i]->subscriber, 1);
			contents = s_recv(nodeList[i]->subscriber, 1);
			if (contents != NULL){
				token = strtok(contents, " ");

				/* 군집 분리 메시지일때*/
				if (strcmp(token, "sep") == 0){
					sprintf(sender, "%s", "check");
					separate(token = strtok(NULL, " "));
				}
				/*군집 합병 메시지일때*/
				if (strcmp(token, "dup") == 0){
					sprintf(sender, "%s", "check");
					duplicate(atoi(token = strtok(NULL, " ")), token = strtok(NULL, " "));
				}
			}
		}
		s_sleep(200);
	}
}

/*군집 분리 함수*/
void separate(char* ip){
	char temp[1];
	for (int i = 0; i < nodeList.size(); i++){

		/*분리되는 로봇의 ip를 찾아 leader로 설정 후 군집을 새로 만든다.*/
		if (strcmp(ip, nodeList[i]->address) == 0){
			temp[0] = nodeList[i]->group;
			nodeList[i]->position = 1;

			/* 군집 내에서 분리하고자 하는 로봇의 뒤에 있는 모든 로봇들의 군집을 바꾼다.*/
			for (int j = i; j < nodeList.size(); j++){
				if (nodeList[j]->group == temp[0]){
					nodeList[j]->group = (char)((int)'A' + NodeAddr::groupNum);
				}
				/* 군집 내 상태확인 하는 subscribe 정보도 변경해준다.*/
				if (strcmp(myaddress, nodeList[j]->address) == 0){
					zmq_connect(subscriber, nodeList[i]->address);
					zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, temp, 1);
				}
			}
			NodeAddr::groupNum++;	//군집갯수를 증가시킨다.
		}
	}
}

/* 군집내에서 원하는 토픽을 받아오는 함수*/
void receive(){
	/* 군집내 리더로봇이 보낸 메시지를 받는다.*/
	char *address = s_recv(subscriber, 1);
	char *contents = s_recv(subscriber, 1);

	if (contents != NULL){
		char* token = strtok(contents, " ");
		/* speed 변경 메시지를 받았을 때*/
		if (strcmp(token, "spd") == 0){
			speed_adjust(atoi(token = strtok(NULL, " ")));
		}
		/* direction 변경 메시지를 받았을 때*/
		else if (strcmp(token, "drt") == 0){
		}
	}
	free(address);
	free(contents);
}

/* 군집 합병 함수*/
void duplicate(int num, char* address){
	vector<NodeAddr*> temp;
	char grouptemp1[1], grouptemp2[1];
	char* leader_address;
	bool leader_found = false;

	for (int i = 0; i < nodeList.size(); i++){

		/* 합병하고자 하는 군집을 확인*/
		if (nodeList[i]->num == num && !leader_found){
			grouptemp1[0] = nodeList[i]->group;
			leader_address = nodeList[i]->address;
			leader_found = true;
		}
		/* 합병당하는 군집을 확인*/
		if (strcmp(nodeList[i]->address, address) == 0){
			grouptemp2[0] = nodeList[i]->group;
		}
	}

	/* 합병에 관련없는 로봇들을 임시 List에 저장*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group != grouptemp1[0] && nodeList[i]->group != grouptemp2[0]){
			temp.push_back(new NodeAddr(nodeList[i]->address, nodeList[i]->group, nodeList[i]->num, nodeList[i]->position));
		}
	}

	/* 합병하고자 하는 군집의 로봇들을 임시 List에 저장*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group == grouptemp1[0]){
			temp.push_back(new NodeAddr(nodeList[i]->address, nodeList[i]->group, nodeList[i]->num, nodeList[i]->position));
		}
	}

	/* 합병당하는 군집의 로봇들을 설정변경 후 임시 List에 저장*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group == grouptemp2[0]){
			if (strcmp(nodeList[i]->address, myaddress) == 0){
				zmq_connect(subscriber, leader_address);
				zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, grouptemp1, 1);
			}
			temp.push_back(new NodeAddr(nodeList[i]->address, grouptemp1[0], nodeList[i]->num, 0));
		}
	}

	/* 임시 List를 List에 연결*/
	nodeList = temp;
}

/* 로봇의 초기설정*/
void init(){
	robot_speed = 300;
	seperate_count = 0;
	strcpy(color_value, "green");

	for (int i = 0; i < nodeList.size(); i++){
		/* List에 저장된 자기자신의 정보를 가져온다.*/
		if (myIpCheck(nodeList[i]->address)){
			for (int j = 0; j < nodeList.size(); j++){
				if (nodeList[i]->group == nodeList[j]->group && nodeList[j]->position){
					/* Subscriber 설정*/
					zmq_connect(subscriber, nodeList[j]->address);
					group[0] = nodeList[j]->group;
					zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, group, 1);
				}
			}
			myaddress = nodeList[i]->address;
		}
	}

	/* Publisher 설정*/
	zmq_bind(publisher, "tcp://*:5555");
	/* Arduino Serial Port Open*/
	Open();

	/* 로봇 상태 체크 메시지 설정*/
	sprintf(sender, "%s", "check");

	char* address;
	char* contents;
	bool check[nodeList.size()];

	/* 로봇 상태 메시지 확인*/
	for (int i = 0; i < nodeList.size(); i++){
		s_sendmore(publisher, "K");
		s_send(publisher, "check");
		address = s_recv(nodeList[i]->subscriber, 1);
		contents = s_recv(nodeList[i]->subscriber, 1);

		if (contents == NULL){
			i = -1;
		}
		s_sleep(200);
	}

	/* thread 정책 설정*/
	param.sched_priority = 60;
	err = pthread_attr_init(&attr);

	err = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if (err){
		printf("error\n");
	}
	err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (err){
		printf("error\n");
	}
	err = pthread_attr_setschedparam(&attr, &param);
	if (err){
		printf("error\n");
	}
	err = pthread_create(&th, &attr, watchDog, strerror(err));
	if (err != 0){
		printf("error\n");
	}
}
