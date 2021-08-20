
#include <iostream>
#include "mydata.h"
#include <vector>

#define ObsNum 1//障碍物点的个数
#define ObsNum_ ObsNum*2
#define pi 3.1415926

using namespace std;

struct Points
{
	double x0; double y0; double x1; double y1;
	double x2; double y2; double x3; double y3;

};
struct ObsBuff
{
	double x;
	double y;
	int parity;
	int Dir;
	int row;
	int rang;
	int order;
	int flag;
	//int follow;
	int Num;//第几次割草
};
struct OutBuff_1
{
	double x;
	double y;
	int flag;
	int order;
	int dir;
	int Num;//第几次割草
};
struct ecef {
	double x;
	double y;
	double z;
	double theta;
};
struct gpsMessage {
	double lat;
	double lon;
	double height;
	double theta;
};

int i = 0;
int flagGoal = 1;
int m = 0;//标志位
int n = 0;
bool flag_finish = false;
ObsBuff Obs[ObsNum] = {};//障碍物点要进行运算的中介
ObsBuff Obs_[ObsNum_] = {};//存放×2的障碍物点，
OutBuff_1 OutB_1[100] = {};//存储不加障碍物点
double OutB_2[200][2] = {};//最终输出的点
double InBuff_1[5][2];//第0个为起点，1-4为四个点坐标
double InObsBuff_2[ObsNum][2];//录入障碍物点
double Vehicle_With = 1;//车宽
double K1, K2, K3, K4;//K1=1->2,...K4=4->1
double K1_, K2_, K3_, K4_;//取倒数
double X_1, Y_1, X_2, Y_2, X_3, Y_3, X_4, Y_4;//四个点的坐标，一个函数负责给四个点赋值
double X_0, Y_0;//起点
double ObstaclLength = 1;//障碍物的膨胀系数
double D1, D2, D3, D4;//四个顶点的距离
double X, Y;//中间变量
int B;//确定方向的
int N;
double Obstacle_Length_1 = 0.8;//障碍物的横边
double Obstacle_Length_2 = 1.5;//障碍物的竖边
int OutB_1Num = 0;
int n1 = 0;

gpsMessage gpsOb[ObsNum], ORIGIN;
ecef xyOb[ObsNum];

ecef gps2ecef(gpsMessage gpsMsg) {       //  gpsMsg是上面那个经纬高数据
	double a = 6378137;
	double b = 6356752.3142;
	double f = (a - b) / a;
	double e_sq = f * (2 - f);
	double lamb = gpsMsg.lat / 180 * pi;
	double phi = gpsMsg.lon / 180 * pi;
	double s = sin(lamb);
	double N = a / sqrt(1 - e_sq * s * s);
	ecef Xy;
	double sin_lambda = sin(lamb);
	double cos_lambda = cos(lamb);
	double sin_phi = sin(phi);
	double cos_phi = cos(phi);
	Xy.x = (gpsMsg.height + N) * cos_lambda * cos_phi;
	Xy.y = (gpsMsg.height + N) * cos_lambda * sin_phi;
	Xy.z = (gpsMsg.height + (1 - e_sq) * N) * sin_lambda;
	return Xy;
}

ecef ecef2enu(ecef ecefXy, gpsMessage gpsOrigin) {        //入口参数是ecef一个结构体xyz坐标和原点（第一个点）的gps经纬高数据
	double a = 6378137;
	double b = 6356752.3142;
	double f = (a - b) / a;
	double e_sq = f * (2 - f);
	double lamb = gpsOrigin.lat / 180 * pi;
	double phi = gpsOrigin.lon / 180 * pi;
	double s = sin(lamb);
	double N = a / sqrt(1 - e_sq * s * s);
	ecef Xy;
	double sin_lambda = sin(lamb);
	double cos_lambda = cos(lamb);
	double sin_phi = sin(phi);
	double cos_phi = cos(phi);
	ecef origin = gps2ecef(gpsOrigin);    //原点gps信息转化成ecef作为原坐标
	double x0 = origin.x;
	double y0 = origin.y;
	double z0 = origin.z;

	double xd = ecefXy.x - x0;
	double yd = ecefXy.y - y0;
	double zd = ecefXy.z - z0;

	double t = -cos_phi * xd - sin_phi * yd;

	Xy.x = -sin_phi * xd + cos_phi * yd;
	Xy.y = t * sin_lambda + cos_lambda * zd;
	Xy.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;          //？
	return Xy;
}

void getObXy(Json::Value root)//传入障碍物的gps点###
{

	for (size_t i = 0; i < root["obstacle"].size(); i++)
	{
		gpsOb[i].lon = root["obstacle"][i]["lon"].asDouble();
		gpsOb[i].lat = root["obstacle"][i]["lat"].asDouble();
		gpsOb[i].height = root["obstacle"][i]["height"].asDouble();
		gpsOb[i].theta = 0;
	}
	for (int i = 0; i < root["obstacle"].size(); i++)
	{
		xyOb[i] = ecef2enu(gps2ecef(gpsOb[i]), ORIGIN);
		Obs[i].x = xyOb[i].x;
		Obs[i].y = xyOb[i].y;
	}
}

bool cmp(ObsBuff a, ObsBuff b)//
{
	if (a.row != b.row)
		return a.row < b.row;//升序
	if (a.rang != b.rang)
		if (a.Dir == 1)//向下走，//1向下，2向上
		{
			return a.rang > b.rang;//降序
		}
		else if (a.Dir == 2)//1向下，2向上
		{
			return a.rang < b.rang;//升序
		}
}

void CalOutB_1()
{
	//......
	//m=0,是起点
	int m = 1;
	OutB_1[m].x = X_1;//将第一个顶点作为OutBuff的第2个点
	OutB_1[m].y = Y_1;
	//X = OutB_1[m].x;
	//Y = OutB_1[m].y;
	m++;//此时m=2

	int n, i;//
	i = 0;
	n = (int)(D2 / Vehicle_With);//L2上车身长段数

	for (int j = 0; j <= n; j++)//不超过d2
	{
		OutB_1[m].x = (double)((Vehicle_With * i) / sqrt(pow(K2, 2) + 1)) + X_2;
		OutB_1[m].y = (double)((K2 * Vehicle_With * i) / sqrt(pow(K2, 2) + 1)) + Y_2;
		OutB_1[m + 1].x = (double)((Vehicle_With * (i + 1)) / sqrt(pow(K2, 2) + 1)) + X_2;
		OutB_1[m + 1].y = (double)(K2 * (Vehicle_With * (i + 1)) / sqrt(pow(K2, 2) + 1)) + Y_2;
		OutB_1[m + 2].x = (double)(Vehicle_With * (i + 1) / sqrt(pow(K4, 2) + 1));
		OutB_1[m + 2].y = (double)(K4 * Vehicle_With * (i + 1) / sqrt(pow(K4, 2) + 1));
		OutB_1[m + 3].x = (double)(Vehicle_With * (i + 2) / sqrt(pow(K4, 2) + 1));
		OutB_1[m + 3].y = (double)(K4 * Vehicle_With * (i + 2) / sqrt(pow(K4, 2) + 1)); //循环一次计算出四个坐标点，第一列x第二列y
		i = i + 2; m = m + 4;
		if (Vehicle_With * i >= D4)
			break;//下一次会超出d4，结束

	}

	for (int i = 0; i < 100; i++)  //统计出问题？？
	{
		if (OutB_1[i].x != 0.0)
		{
			OutB_1Num++;
		}
	}
	OutB_1Num = OutB_1Num + 2;
	std::cout << "OutB_1Num = " << OutB_1Num << std::endl;
}

void CalculateObs(void)
{
	int a = 0;		//距离边的距离的取整，判断是奇数还是偶数，奇数向下，偶数向上
	int b = 0;
	double c = 0;
	double Distance_1 = 0;//第几列的距离
	double Distance_2 = 0;//第几行的距离

	double D_1 = 0;
	double D_2 = 0;//距离变量
	int n = 0;//Obs_的下标

	for (int i = 0; i < ObsNum; i++) //对Obs结构体数组中每个值进行处理
	{
		//障碍物点到直线K1的距离
		Distance_1 = fabs(K1 * Obs[i].x - Obs[i].y) / sqrt(pow(K1, 2) + 1);//求点到1点和2点距离，1点是原点
		a = (int)(Distance_1 / Vehicle_With);//a是个整除偏小的值
		b = (int)(a % 2);
		if (b != 0)//判断a的奇偶，如果b！=0；->b=1,则是奇数
		{
			Obs[i].parity = 0;//为奇数
			Obs[i].Dir = 1;//向下走
		}
		if (b == 0)//a是偶数
		{
			Obs[i].parity = 1;//为偶数
			Obs[i].Dir = 2;//向上走
		}
		Obs[i].row = a;

		Distance_2 = fabs(K4 * Obs[i].x - Obs[i].y) / sqrt(pow(K4, 2) + 1);
		Obs[i].rang = (int)Distance_2;
	}

	//将Obs进行×2变为Obs_

	for (int i = 0; i < ObsNum; i++)
	{
		if (Obs[i].Dir == 1)//向下走
		{
			D_1 = fmod((fabs(K1 * Obs[i].x - Obs[i].y) / sqrt(pow(K1, 2) + 1)), Vehicle_With);//取余数
			X = -D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].x;
			Y = -K1_ * D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].y;//横着过去了
			D_2 = Obstacle_Length_2 / 2;//障碍物的竖边
			Obs_[n].x = D_2 / sqrt(pow(K1, 2) + 1) + X;
			Obs_[n].y = K1 * D_2 / sqrt(pow(K1, 2) + 1) + Y;
			//
						// Obs_[n].x=Obs[i].x;
						// Obs_[n].y=Obs[i].y;

			Obs_[n].row = Obs[i].row;
			Obs_[n].rang = Obs[i].rang;
			Obs_[n].Dir = 1;
			Obs_[n].Num = 1;//Num=1,第一个割草点，Num=2,第二个割草点

			D_1 = Vehicle_With - D_1;
			X = D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].x;
			Y = K1_ * D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].y;//横着过去了
			D_2 = Obstacle_Length_2 / 2;//障碍物的竖边
			Obs_[n + 1].x = -D_2 / sqrt(pow(K1, 2) + 1) + X;
			Obs_[n + 1].y = -K1 * D_2 / sqrt(pow(K1, 2) + 1) + Y;
			//
						// Obs_[n+1].x=Obs[i].x;
						// Obs_[n+1].y=Obs[i].y;

			Obs_[n + 1].row = Obs[i].row + 1;
			Obs_[n + 1].rang = Obs[i].rang;
			Obs_[n + 1].Dir = 2;
			Obs_[n + 1].Num = 2;//Num=1,第一个割草点，Num=2,第二个割草点
			n++;
			n++;
		}
		if (Obs[i].Dir == 2)//向上走
		{
			D_1 = fmod((fabs(K1 * Obs[i].x - Obs[i].y) / sqrt(pow(K1, 2) + 1)), Vehicle_With);//取余数
			X = -D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].x;
			Y = -K1_ * D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].y;//横着过去了
			D_2 = Obstacle_Length_2 / 2;//障碍物的竖边
			Obs_[n].x = -D_2 / sqrt(pow(K1, 2) + 1) + X;
			Obs_[n].y = -K1 * D_2 / sqrt(pow(K1, 2) + 1) + Y;

			// Obs_[n].x=Obs[i].x;
			// Obs_[n].y=Obs[i].y;

			Obs_[n].row = Obs[i].row;
			Obs_[n].rang = Obs[i].rang;
			Obs_[n].Dir = 2;
			Obs_[n].Num = 1;//Num=1,第一个割草点，Num=2,第二个割草点

			D_1 = Vehicle_With - D_1;
			X = D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].x;
			Y = K1_ * D_1 / sqrt(pow(K1_, 2) + 1) + Obs[i].y;//横着过去了
			D_2 = Obstacle_Length_2 / 2;//障碍物的竖边
			Obs_[n + 1].x = D_2 / sqrt(pow(K1, 2) + 1) + X;
			Obs_[n + 1].y = K1 * D_2 / sqrt(pow(K1, 2) + 1) + Y;

			// Obs_[n+1].x=Obs[i].x;
			// Obs_[n+1].y=Obs[i].y;

			Obs_[n + 1].row = Obs[i].row + 1;
			Obs_[n + 1].rang = Obs[i].rang;
			Obs_[n + 1].Dir = 1;
			Obs_[n + 1].Num = 2;//Num=1,第一个割草点，Num=2,第二个割草点
			n++;
			n++;
		}
	}

	sort(Obs_, Obs_ + ObsNum_, cmp);//对Obs_结构体数组进行排序
	//如果在一条线上，有很多障碍物，添加标志位
	for (int i = 0; i < ObsNum_; i++)
	{
		if (Obs_[i].row == Obs_[i + 1].row)
		{
			Obs_[i].flag = 1;//后面有相同的数
		}
	}
	//添加order，添加障碍物在数组的顺序
	for (int i = 0; i < ObsNum_; i++)
	{
		Obs_[i].order = i;
	}
	//对数组已经处理完了

	//打印Obs数组
	//cout << "打印Obs数组" << endl;
	//for (int i = 0; i < ObsNum; i++)
	//{
	//	cout << Obs[i].x << "  " << Obs[i].y << endl;
	//}
	// cout << "打印Obs数组的row" << endl;
	// for (int i = 0; i < ObsNum; i++)
	// {
	// 	cout << Obs[i].row << endl;
	// }

	//遍历Obs_[]，为OutB_1添加后面有障碍物的标志位
	for (int i = 0; i < ObsNum_; i++)
	{
		OutB_1[Obs_[i].row * 2 + 1].flag = 1;//这个点的下一个点为障碍物点,1是标志位
		OutB_1[Obs_[i].row * 2 + 1].order = Obs_[i].order;//对应障碍物在障碍物数组的位置
		OutB_1[Obs_[i].row * 2 + 1].dir = Obs_[i].Dir;//1向下，2向上
		OutB_1[Obs_[i].row * 2 + 1].Num = Obs_[i].Num;//Num=1,第一个割草点，Num=2,第二个割草点

		// OutB_1[Obs[i].row * 2 + 1 + 2].flag = 1;//下一个点也是障碍物点，
		// OutB_1[Obs[i].row * 2 + 1 + 2].order = Obs[i].order;//对应障碍物在障碍物数组的位置
		// if (OutB_1[Obs[i].row * 2 + 1].dir == 1)
		// {
		// 	OutB_1[Obs[i].row * 2 + 1 + 2].dir = 2;//1向下，2向上
		// }
		// else
		// {
		// 	OutB_1[Obs[i].row * 2 + 1 + 2].dir = 1;//1向下，2向上
		// }
	}

	//for (int i = 0; i < OutB_1Num; i++)
	//{
	//	if (OutB_1[i].flag == 1)
	//	{
	//		cout << i << endl;
	//	}
	//}
}

void ObsToOutB_2(void)
{
	int m, n;
	m = 1;//X_1,Y_1点位OutBuff1的第1（2）个数组
	n = 1;
	for (m = 1; m < OutB_1Num; m++)//遍历OutB_1数组
	{
		if (OutB_1[m].flag == 1)//下个点是障碍物点
		{

			//cout << "下一个点是障碍物点" << endl;
			OutB_2[n][0] = OutB_1[m].x;//先传入正常点
			OutB_2[n][1] = OutB_1[m].y;
			std::cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << std::endl;
			n++;
			/*m++;*/ //等会再m++
			//开始传入障碍物点怎么走
			std::cout << "开始避障,并到达避障起始点" << std::endl;
			OutB_2[n][0] = Obs_[OutB_1[m].order].x;
			OutB_2[n][1] = Obs_[OutB_1[m].order].y;//对应在在ObsBuff的点
			X = OutB_2[n][0];
			Y = OutB_2[n][1];
			std::cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << std::endl;
			n++;
			if (OutB_1[m].dir == 2)//1向下，2向上
			{
				B = 1;
				if (OutB_1[m].Num == 1)//Num=1,第一个割草点，Num=2,第二个割草点
				{//向上，第一个点
					N = 1;//N=1,第一个割草点，N=-1,第二个割草点
	//-1是第一个点和第三个点的方向是相反的
				//B是朝上走还是朝下走
					OutB_2[n][0] = -Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//走第一个点，横着走
					OutB_2[n][1] = -K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					std::cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << std::endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第二个点
					OutB_2[n][0] = Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + X;//走第二个点，竖着走
					OutB_2[n][1] = K1 * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + Y;
					std::cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << std::endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第三个
					OutB_2[n][0] = Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//横着走
					OutB_2[n][1] = K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					std::cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << std::endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					cout << "结束避障动作" << endl;
				}
				if (OutB_1[m].Num == 2)//Num=1,第一个割草点，Num=2,第二个割草点
				{	//向上第二个点
					N = -1;//N=1,第一个割草点，N=-1,第二个割草点
					//-1是第一个点和第三个点的方向是相反的
					//B是朝上走还是朝下走
					OutB_2[n][0] = Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//走第一个点，横着走
					OutB_2[n][1] = K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第二个点
					OutB_2[n][0] = Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + X;//走第二个点，竖着走
					OutB_2[n][1] = K1 * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第三个
					OutB_2[n][0] = -Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//横着走
					OutB_2[n][1] = -K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					cout << "结束避障动作" << endl;
				}
			}
			if (OutB_1[m].dir == 1)//奇数，向下走
			{
				B = -1;
				if (OutB_1[m].Num == 1)//Num=1,第一个割草点，Num=2,第二个割草点
				{
					//向下第一个点
					N = 1;//N=1,第一个割草点，N=-1,第二个割草点
	//-1是第一个点和第三个点的方向是相反的
				//B是朝上走还是朝下走
					OutB_2[n][0] = -Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//走第一个点，横着走
					OutB_2[n][1] = -K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第二个点
					OutB_2[n][0] = -Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + X;//走第二个点，竖着走
					OutB_2[n][1] = -K1 * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第三个
					OutB_2[n][0] = Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//横着走
					OutB_2[n][1] = K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					cout << "结束避障动作" << endl;
				}
				if (OutB_1[m].Num == 2)//Num=1,第一个割草点，Num=2,第二个割草点
				{
					//向下第二个点
					N = -1;//N=1,第一个割草点，N=-1,第二个割草点
					//-1是第一个点和第三个点的方向是相反的
					//B是朝上走还是朝下走
					OutB_2[n][0] = Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//走第一个点，横着走
					OutB_2[n][1] = K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第二个点
					OutB_2[n][0] = -Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + X;//走第二个点，竖着走
					OutB_2[n][1] = -K1 * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					//走第三个
					OutB_2[n][0] = -Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//横着走
					OutB_2[n][1] = -K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
					cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
					X = OutB_2[n][0];
					Y = OutB_2[n][1];

					n++;
					cout << "结束避障动作" << endl;
				}
			}

			// 给OutB_1加入Num标志位

			//cout << "B=  " << B << endl;
			//开始进行障碍物绕点动作

			//-1是第一个点和第三个点的方向是相反的
			//B是朝上走还是朝下走
			// OutB_2[n][0] = -N*B * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//走第一个点，横着走
			// OutB_2[n][1] = -N*B * K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
			// cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
			// X = OutB_2[n][0];
			// Y = OutB_2[n][1];

			// n++;
			// //走第二个点
			// OutB_2[n][0] = N*B * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + X;//走第二个点，竖着走
			// OutB_2[n][1] = N*B * K1 * Obstacle_Length_2 / sqrt(pow(K1, 2) + 1) + Y;
			// cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
			// X = OutB_2[n][0];
			// Y = OutB_2[n][1];

			// n++;
			// //走第三个
			// OutB_2[n][0] = N*B * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + X;//横着走
			// OutB_2[n][1] = N*B * K1_ * Obstacle_Length_1 / sqrt(pow(K1_, 2) + 1) + Y;
			// cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
			// X = OutB_2[n][0];
			// Y = OutB_2[n][1];

			// n++;
			// cout << "结束避障动作" << endl;
			//结束障碍物绕点动作	
			//m++;//补上m++
		}
		//如果不是障碍物点，则继续
		else
		{
			OutB_2[n][0] = OutB_1[m].x;
			OutB_2[n][1] = OutB_1[m].y;
			cout << "第" << n << "个点" << OutB_2[n][0] << "   " << OutB_2[n][1] << endl;
			n++;
		}

		//m++;
	}
	n1 = n;
	X = 0;
	Y = 0;//清空X，Y
}

void test1(void)//测试实验
{
	CalOutB_1();
	CalculateObs();
	ObsToOutB_2();
	//for (int i = 0; i < OutB_1Num; i++)
	//{
	//	if (OutB_1[i].flag == 1)
	//	{
	//		cout << OutB_1[i].x << "    " << OutB_1[i].y << endl;
	//	}
	//}
}

void Init(void)
{
	X_0 = InBuff_1[0][0];
	Y_0 = InBuff_1[0][1];
	X_1 = InBuff_1[1][0];
	Y_1 = InBuff_1[1][1];
	X_2 = InBuff_1[2][0];
	Y_2 = InBuff_1[2][1];
	X_3 = InBuff_1[3][0];
	Y_3 = InBuff_1[3][1];
	X_4 = InBuff_1[4][0];
	Y_4 = InBuff_1[4][1];

	K1 = (Y_2 - Y_1) / (X_2 - X_1);
	K2 = (Y_3 - Y_2) / (X_3 - X_2);
	K3 = (Y_4 - Y_3) / (X_4 - X_3);
	K4 = (Y_1 - Y_4) / (X_1 - X_4);

	K1_ = -1 / K1;
	K2_ = -1 / K2;
	K3_ = -1 / K3;
	K4_ = -1 / K4;

	D1 = sqrt(pow((Y_2 - Y_1), 2) + pow((X_2 - X_1), 2));
	D2 = sqrt(pow((Y_3 - Y_2), 2) + pow((X_3 - X_2), 2));
	D3 = sqrt(pow((Y_4 - Y_3), 2) + pow((X_4 - X_3), 2));
	D4 = sqrt(pow((Y_1 - Y_4), 2) + pow((X_1 - X_4), 2));

}

void gpsLitCallBack(std::vector<std::vector<double> > edge, Json::Value root)
{
	InBuff_1[0][0] = edge[0][0];
	InBuff_1[0][1] = edge[0][1];
	InBuff_1[1][0] = edge[0][0];
	InBuff_1[1][1] = edge[0][1];
	InBuff_1[2][0] = edge[1][0];
	InBuff_1[2][1] = edge[1][1];
	InBuff_1[3][0] = edge[2][0];
	InBuff_1[3][1] = edge[2][1];
	InBuff_1[4][0] = edge[3][0];
	InBuff_1[4][1] = edge[3][1];

	getObXy(root);
	Init();
	test1();
	flag_finish = true;
}

void Printf(void)
{
	//int point[4][2] = { {1,9},{9,9},{5,5},{5,1} };

		//Graph graph = Graph(800, 640);
		//graph.drawCoordinateAxis(60, 50, 1, 1);//结合窗口合理设置(´-﹏-`；)
		//graph.setPointColor(Graph::COLOR_RED);//设置点颜色
		//graph.setTextColor(Graph::COLOR_BLUE);//设置文本颜色

		//for (int i = 0; i < 4; i++) {
		//	graph.drawPoint(point[i][0], point[i][1]);//画点
		//	graph.drawNum(point[i][0], point[i][1], i + 1);//写数字
		//}
		////设置线格式
		//graph.setLineWidth(1);
		//graph.setLineColor(Graph::COLOR_GREEN);
		////画线
		//graph.drawLine(point[0][0], point[0][1], point[2][0], point[2][1]);
		//graph.drawLine(point[1][0], point[1][1], point[2][0], point[2][1]);
		//graph.drawLine(point[3][0], point[3][1], point[2][0], point[2][1]);
		//graph.show();
	int a = 0;
	for (int i = 0; i < 100; i++)
	{
		if (OutB_2[i][0] != 0)//统计有多少是非空的
		{
			a++;
		}
	}

	// for (int i = 0; i < a; i++)
	// {
	// 	//graph.drawPoint(OutB_2[i][0], OutB_2[i][1]);//画点
	// 	cout << "第" << i << "个任务点的  x=" << OutB_2[i][0] << "   y=" << OutB_2[i][1] << endl;
	// }
	//graph.show();
}


void inCallBack(std::vector<double>& origin)
{
	ORIGIN.lon = origin[0];                                //使得map的最西点为原点
	ORIGIN.lat = origin[1];
	ORIGIN.height = origin[2];
	ORIGIN.theta = 0;
}

void sortBound(std::vector<std::pair<double, double>>& boundV)
{
	vector<pair<double, double>> boundV1(4);
	for (int i = 0; i < 4; i++)
		boundV1[i] = boundV[i];
	double maxX, minX, maxY, minY;
	double k[4];
	for (int i = 0; i < 4; i++)
		k[i] = 0;
	maxX = boundV1[0].first;
	maxY = boundV1[0].second;
	minX = boundV1[0].first;
	minY = boundV1[0].second;
	for (int i = 1; i < 4; i++)
	{
		if (boundV1[i].first < minX) {//最西点 1
			k[0] = i;
			minX = boundV1[i].first;
		}
		if (boundV1[i].first > maxX) {//最东点 3
			k[2] = i;
			maxX = boundV1[i].first;
		}
		if (boundV1[i].second < minY) {
			k[3] = i;
			minY = boundV1[i].second;
		}
		if (boundV1[i].second > maxY) {
			k[1] = i;
			maxY = boundV1[i].second;
		}
	}
	for (int i = 0; i < 4; i++)
		boundV[i] = boundV1[k[i]];
}


std::vector<std::vector<double> > path_planning(std::vector<std::vector<double> >& edge, std::vector<double>& origin, Json::Value root)//###
{
	inCallBack(origin);//原点
	std::vector<std::pair<double, double>> boundV(4);
	for (size_t i = 0; i < boundV.size(); i++)//排序
	{
		boundV[i].first = edge[i][0];
		boundV[i].second = edge[i][1];
	}
	sortBound(boundV);
	for (size_t i = 0; i < boundV.size(); i++)
	{
		edge[i][0] = boundV[i].first;
		edge[i][1] = boundV[i].second;

	}
	gpsLitCallBack(edge, root);   //传入任务角点和障碍物信息，规划路径数组
	std::vector<std::vector<double> > mission(n1, std::vector<double>(2));
	for (size_t i = 0; i < n1; i++)
	{
		mission[i][0] = OutB_2[i][0];
		mission[i][1] = OutB_2[i][1];
	}
	return mission;
}