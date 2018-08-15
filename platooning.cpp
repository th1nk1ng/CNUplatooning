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

/*���¼ҽ� : http://darkpgmr.tistory.com/66 */
struct hsv_color {
	unsigned char h;        // Hue: 0 ~ 255 (red:0, green: 85, blue: 171)
	unsigned char s;        // Saturation: 0 ~ 255
	unsigned char v;        // Value: 0 ~ 255
};

/*RGB���� HSV�� ��ȯ�ϴ� �ڵ�*/
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


/* �� �ڵ������� ������ �����ϴ� Node*/
class NodeAddr{
public:
	char* address;			//�κ��� �ּ�
	char group;				//�����ִ� ����
	int position;			//��������
	void *subscriber;		//���� üũ subscriber
	int num;				//������ȣ
	static int groupNum;	//��������

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

	/* Node�� �� �ڸ��� ����� ���� Node����*/
	nodeList.reserve(4);
	nodeList.push_back(new NodeAddr("tcp://192.168.0.105:5555", 'A', 1, 1));
	nodeList.push_back(new NodeAddr("tcp://192.168.0.120:5555", 'A', 2, 0));


	init();
	/*�ڽ��� ���� üũ*/
	for (int i = 0; i < nodeList.size(); i++){
		if (strcmp(nodeList[i]->address, myaddress) == 0){
			my_position = nodeList[i]->position;		//�������� �񸮴�����
			my_num = nodeList[i]->num;	//�ڽ��� ��ȣ�� �� ������
		}
	}

	VideoCapture cap(-1);

	if (!cap.isOpened())
	{
		cout << "Cannot open camera" << endl;
		return -1;
	}

	//ī�޶� ������ ũ��
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

		receive();		//���������� ���ϴ� ������ �޾ƿ´�

		/* �κ��� �������� ������ ���*/
		if (strcmp(color_value_red, "red") == 0){
			/*�������̹Ƿ� ����->�ӵ��� 0���� ����*/
			robot_speed = 0;
			Go(robot_speed, robot_speed);

			/*������ ���, �񸮴��鿡�� �ӵ��� 0���� �ٲٶ�� �޽��� ����*/
			if (my_position == 1) {
				/*���� ���� Publish*/
				s_sendmore(publisher, group);
				s_send(publisher, "spd 0");
				sprintf(color_value_red, "%s", NULL);
			}
			strcpy(color_value_red, "");
			do{
				strcpy(color_value_red, "");	//�� �ʱ�ȭ
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				colorExtraction(frame);

			} while (strcmp(color_value_red, "red") == 0);	//�������� �� �ݺ�

		}

		/*�κ��� �ʷϻ��� ������ ���*/
		else if (strcmp(color_value, "green") == 0){
			/*�ʷϺ��̹Ƿ� ���->�ӵ��� 300���� ����*/
			robot_speed = 300;
			Go(robot_speed, robot_speed);

			/*������ ���, �񸮴��鿡�� �ӵ��� 300���� �ٲٶ�� �޽��� ����*/
			if (my_position == 1) {
				/*���� ���� Publish*/
				s_sendmore(publisher, group);
				s_send(publisher, "spd 300");
				sprintf(color_value, "%s", NULL);
			}
			strcpy(color_value, "");	//color_value�� �ʱ�ȭ	
		}
		/*�κ��� �Ķ����� ������ ���, ������ ǥ�����̹Ƿ� ��ȸ��*/
		else if (strcmp(color_value, "blue") == 0){
			Steering2(127, 0);	//������ ���������� ȸ��
			Go(650, 0);	//��ȸ���ϱ� ���� ���� ���� �ӵ� ����
			waitKey(5500);	//5.5�ʰ� ����
			do {
				Steering2(127, 0);	//������ ���������� ȸ��
				Go(650, 0);			//��ȸ���ϱ� ���� ���� ���� �ӵ� ����
				sprintf(color_value, "%s", NULL);
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				road_identity_value = laneDetection(frame);

			} while (!(-45 <= road_identity_value && road_identity_value <= 45));	//���� ��� ���, ��� �ݺ�
			strcpy(color_value, "");	//color_value �� �ʱ�ȭ
		}

		/*�κ��� �ϴû��� ������ ���, ���� ǥ�����̹Ƿ� ��ȸ��*/
		else if (strcmp(color_value, "light_blue") == 0){
			Steering2(255, 0);		//������ �������� ȸ��
			Go(0, 650);				//��ȸ���ϱ� ���� ������ ���� �ӵ� ����
			waitKey(5500);			//5.5�ʰ� ����
			do {
				Steering2(255, 0);	//������ �������� ȸ��
				Go(0, 650);			//��ȸ���ϱ� ���� ������ ���� �ӵ� ����
				sprintf(color_value, "%s", NULL);
				bSuccess = cap.read(frame);
				flip(frame, frame, -1);
				road_identity_value = laneDetection(frame);
			} while (!(-45 <= road_identity_value && road_identity_value <= 45));	//���� ��� ���, ��� �ݺ�
			strcpy(color_value, "");	//color_value �� �ʱ�ȭ
		}
		/* �� 4���� �� �̿��� ���� �ν��� ���*/
		else{
			/* count �� �ʱ�ȭ*/
			redcount = 0;
			greencount = 0;

			/*�񸮴��� �ӵ��� 0�� ���*/
			if (my_position == 0 && robot_speed == 0){
				Go(0, 0);
				continue;	//������ ��� �������� �ٸ� �ڵ带 �������� �ʰ� �״�� ������ ���� ����
			}

			/*�� ���� ��� ����*/
			else{
				/*�κ��� �ӵ��� 300���� �Ͽ� ����*/
				robot_speed = 300;
				Go(robot_speed, robot_speed);

				/*������ ���, �񸮴��鿡�� �ӵ��� 300���� �϶�� �޽��� ����*/
				if (my_position == 1) {
					/*���� ���� Publish*/
					s_sendmore(publisher, group);
					s_send(publisher, "spd 300");
				}
			}
		}

		if (my_position == 1){	//������ ���, ���� �������� �ڵ� ����
			auto_drive_leader(road_identity_value);
		}
		else {	//�񸮴��� ���, �񸮴� �������� �ڵ� ����
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

	/*thread ����*/
	pthread_join(th, 0);
	return 0;
}

/*�񸮴� �������� �޼ҵ�*/
void auto_drive_nonleader(int road_identify_value){
	/*���� �ν��� ���� ������ ���, �������� �־��� ����*/
	if (road_identify_value<0){
		if (road_identify_value == -80){	//���ʿ��� ���� �� �ִ� �ִ밪�� -80�� ���
			road_identify_value = -100;		//�� �ް��� ȸ���� ���� ���� -100���� ����
		}
		Steering2(128 - road_identify_value, 0); //���� ������ �ش� ����ŭ �������� ����
	}

	/*���� �ν��� ���� ����� ���, ���������� �־��� ����*/
	else if (road_identify_value>0){
		if (road_identify_value == 80){	//�����ʿ��� ���� �� �ִ� �ִ밪�� 80�� ���
			road_identify_value = 100;	//�� �ް��� ȸ���� ���� ���� 100���� ����
		}
		Steering2(road_identify_value, 0);	//���� ������ �ش� ����ŭ ���������� ����
	}

	/*���� �ν��� ���� 0�� ���, ���� ����*/
	else if (road_identify_value == 0){
		Steering2(0, 0);	//���� ������ ����� ����
	}
	Go(robot_speed, robot_speed);	//���� �ӵ��� ����
	strcpy(color_value, "");			//color_value �� �ʱ�ȭ
}

/*���� �������� �޼ҵ�*/
void auto_drive_leader(int road_identify_value){
	/*���� �ν��� ���� ������ ���, �������� �־��� ����*/
	if (road_identify_value<0){
		if (road_identify_value == -80){	//���ʿ��� ���� �� �ִ� �ִ밪�� -80�� ���
			road_identify_value = -100;		//�� �ް��� ȸ���� ���� ���� -100���� ����
		}
		Steering2(128 - road_identify_value, 0);//���� ������ �ش� ����ŭ �������� ����
	}

	/*���� �ν��� ���� ����� ���, ���������� �־��� ����*/
	else if (road_identify_value>0){
		if (road_identify_value == 80){	//�����ʿ��� ���� �� �ִ� �ִ밪�� 80�� ���
			road_identify_value = 100;	//�� �ް��� ȸ���� ���� ���� 100���� ����
		}
		Steering2(road_identify_value, 0);	//���� ������ �ش� ����ŭ ���������� ����
	}

	/*���� �ν��� ���� 0�� ���, ���� ����*/
	else if (road_identify_value == 0){
		Steering2(0, 0);	//���� ������ ����� ����
	}
	Go(robot_speed, robot_speed);	//���� �ӵ��� ����
}

/*�ӵ��� �����ϴ� �޼ҵ�*/
void speed_adjust(int value){
	robot_speed = value;	//�Ķ���� ������ �ӵ� ���� ��,
}

int laneDetection(Mat& input){
	cv::Mat image = input;

	//	cv::namedWindow("Original Image");
	//	cv::imshow("Original Image", image);

	// �ɴ� �˰��� ����

	cv::Mat contours;

	cv::Canny(image, // �׷��̷��� ����
		contours, // ��� �ܰ���
		240,  // ���� ��谪
		250);  // ���� ��谪

	// ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ��� ���� ����

	cv::Mat contoursInv; // ���� ����

	cv::threshold(contours, contoursInv, 100, 255, cv::THRESH_BINARY_INV);
	// ��� ���� 128���� ������ 255�� �ǵ��� ����

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

	if (right == -1){ //right�� �������
		int left2 = -1;
		for (int i = contoursInv.size().width / 2; i > contoursInv.size().width / 2 - searchThreshold; i--) {
			if (contoursInv.at<unsigned char>(preferedHeight - 10, i) == 0) {
				left2 = i;
				break;
			}
		}

		if (left - left2 > 0){ // ����ȸ��
			return -80;
		}
		else // ����ȸ�� {
			return 80;
		}


	}
	else if (left == -1){ // left �� �������
		int right2 = -1;
		for (int i = contoursInv.size().width / 2; i < contoursInv.size().width / 2 + searchThreshold; i++) {
			if (contoursInv.at<unsigned char>(preferedHeight - 10, i) == 0) {
				right2 = i;
				break;
			}
		}
		if (right - right2 > 0){ // ����ȸ��
			return -80;
		}
		else // ����ȸ�� {
			return 80;
		}
	}
	return returnValue;
}

/*���� ���� �ڵ�*/
/*���� �ҽ� : http://egloos.zum.com/newstyle/v/2707246 ���� */
void colorExtraction(Mat& input){
	IplImage* frame = new IplImage(input);	//���� ī�޶� ���� ����

	char c;
	unsigned char  B, G, R;
	int x, y;

	bool check[7][320][240];	//frame�� �ش� ������ �����ϴ� ������ �����ϱ� ���� �迭

	if (frame == NULL) return;

	for (int i = 0; i < frame->height; i++)
	for (int j = 0; j < frame->width; j++)
	{
		CvScalar v = cvGet2D(frame, i, j);	//CvScalar : ���� �����ϴ� ����ü
		//frame�� (j, i)�� �ִ� �ȼ��� �� ���� v�� ����

		if (v.val[0] == NULL || v.val[1] == NULL || v.val[2] == NULL)
			continue;
		B = v.val[0];
		G = v.val[1];
		R = v.val[2];

		hsv_color rgb_to_hsv = RGB2HSV(R, G, B);

		/*Ư�� ���� HSV ���� ������ �ְ� ������ �ش�Ǹ� check�迭�� true�� ����*/
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
	/*check�迭�� �˻��ϸ� �ȼ��� � ���� ����Ǿ��ִ��� Ȯ���ϰ� count*/
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
			//��ȣ �ν�
			//�������� 40*40*0.75 �̻� ���� ���� ����Ǿ�� �ν��ϵ���
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
			//�κ� ���� ��ƼĿ ���� (�κ����� ��ֹ����� �Ǵ��� �� ���)
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

/* �ڽ��� ip�� Ȯ���ϴ� �Լ�*/
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

/* �ǽð����� �κ����� ���¸� Ȯ���ϴ� �Լ�*/
void* watchDog(void* Para){
	char* address;
	char* contents;
	char* token;

	while (1){
		/* ���� check �޽��� Publish*/
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

				/* ���� �и� �޽����϶�*/
				if (strcmp(token, "sep") == 0){
					sprintf(sender, "%s", "check");
					separate(token = strtok(NULL, " "));
				}
				/*���� �պ� �޽����϶�*/
				if (strcmp(token, "dup") == 0){
					sprintf(sender, "%s", "check");
					duplicate(atoi(token = strtok(NULL, " ")), token = strtok(NULL, " "));
				}
			}
		}
		s_sleep(200);
	}
}

/*���� �и� �Լ�*/
void separate(char* ip){
	char temp[1];
	for (int i = 0; i < nodeList.size(); i++){

		/*�и��Ǵ� �κ��� ip�� ã�� leader�� ���� �� ������ ���� �����.*/
		if (strcmp(ip, nodeList[i]->address) == 0){
			temp[0] = nodeList[i]->group;
			nodeList[i]->position = 1;

			/* ���� ������ �и��ϰ��� �ϴ� �κ��� �ڿ� �ִ� ��� �κ����� ������ �ٲ۴�.*/
			for (int j = i; j < nodeList.size(); j++){
				if (nodeList[j]->group == temp[0]){
					nodeList[j]->group = (char)((int)'A' + NodeAddr::groupNum);
				}
				/* ���� �� ����Ȯ�� �ϴ� subscribe ������ �������ش�.*/
				if (strcmp(myaddress, nodeList[j]->address) == 0){
					zmq_connect(subscriber, nodeList[i]->address);
					zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, temp, 1);
				}
			}
			NodeAddr::groupNum++;	//���������� ������Ų��.
		}
	}
}

/* ���������� ���ϴ� ������ �޾ƿ��� �Լ�*/
void receive(){
	/* ������ �����κ��� ���� �޽����� �޴´�.*/
	char *address = s_recv(subscriber, 1);
	char *contents = s_recv(subscriber, 1);

	if (contents != NULL){
		char* token = strtok(contents, " ");
		/* speed ���� �޽����� �޾��� ��*/
		if (strcmp(token, "spd") == 0){
			speed_adjust(atoi(token = strtok(NULL, " ")));
		}
		/* direction ���� �޽����� �޾��� ��*/
		else if (strcmp(token, "drt") == 0){
		}
	}
	free(address);
	free(contents);
}

/* ���� �պ� �Լ�*/
void duplicate(int num, char* address){
	vector<NodeAddr*> temp;
	char grouptemp1[1], grouptemp2[1];
	char* leader_address;
	bool leader_found = false;

	for (int i = 0; i < nodeList.size(); i++){

		/* �պ��ϰ��� �ϴ� ������ Ȯ��*/
		if (nodeList[i]->num == num && !leader_found){
			grouptemp1[0] = nodeList[i]->group;
			leader_address = nodeList[i]->address;
			leader_found = true;
		}
		/* �պ����ϴ� ������ Ȯ��*/
		if (strcmp(nodeList[i]->address, address) == 0){
			grouptemp2[0] = nodeList[i]->group;
		}
	}

	/* �պ��� ���þ��� �κ����� �ӽ� List�� ����*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group != grouptemp1[0] && nodeList[i]->group != grouptemp2[0]){
			temp.push_back(new NodeAddr(nodeList[i]->address, nodeList[i]->group, nodeList[i]->num, nodeList[i]->position));
		}
	}

	/* �պ��ϰ��� �ϴ� ������ �κ����� �ӽ� List�� ����*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group == grouptemp1[0]){
			temp.push_back(new NodeAddr(nodeList[i]->address, nodeList[i]->group, nodeList[i]->num, nodeList[i]->position));
		}
	}

	/* �պ����ϴ� ������ �κ����� �������� �� �ӽ� List�� ����*/
	for (int i = 0; i < nodeList.size(); i++){
		if (nodeList[i]->group == grouptemp2[0]){
			if (strcmp(nodeList[i]->address, myaddress) == 0){
				zmq_connect(subscriber, leader_address);
				zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, grouptemp1, 1);
			}
			temp.push_back(new NodeAddr(nodeList[i]->address, grouptemp1[0], nodeList[i]->num, 0));
		}
	}

	/* �ӽ� List�� List�� ����*/
	nodeList = temp;
}

/* �κ��� �ʱ⼳��*/
void init(){
	robot_speed = 300;
	seperate_count = 0;
	strcpy(color_value, "green");

	for (int i = 0; i < nodeList.size(); i++){
		/* List�� ����� �ڱ��ڽ��� ������ �����´�.*/
		if (myIpCheck(nodeList[i]->address)){
			for (int j = 0; j < nodeList.size(); j++){
				if (nodeList[i]->group == nodeList[j]->group && nodeList[j]->position){
					/* Subscriber ����*/
					zmq_connect(subscriber, nodeList[j]->address);
					group[0] = nodeList[j]->group;
					zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, group, 1);
				}
			}
			myaddress = nodeList[i]->address;
		}
	}

	/* Publisher ����*/
	zmq_bind(publisher, "tcp://*:5555");
	/* Arduino Serial Port Open*/
	Open();

	/* �κ� ���� üũ �޽��� ����*/
	sprintf(sender, "%s", "check");

	char* address;
	char* contents;
	bool check[nodeList.size()];

	/* �κ� ���� �޽��� Ȯ��*/
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

	/* thread ��å ����*/
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
