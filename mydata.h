#ifndef MYDATA_INCLUDED
#define MYDATA_INCLUDED

#include <queue>
#include <json/json.h>
#include <vector>
#include <string>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

typedef struct 
{
	double x;
	double y;
}Origin;

typedef struct 
{
	std::string command;
	boost::interprocess::interprocess_mutex mutex;   //实例内匿名互斥锁即可，外部文件管理可以使用具名
}shared_memory;

extern const char* COM;
extern const char* MAP_FILE;
extern const char* MISSION_FILE;
extern const char* RESULT_FILE;
extern const char* COMMAND_FILE;
extern const char* address_;
extern const char* port_;
extern bool END;
extern std::vector<std::vector<double> > mission;
extern std::queue<std::string> mq;
extern std::size_t count_;

std::string parse2write(std::string str, std::vector<std::vector<double> > sum);
std::string load_file(const char* address);
bool check_file(const char* address);
std::vector<std::vector<double> >
path_planning(std::vector<std::vector<double> >& edge, std::vector<double>& origin, Json::Value root);
#endif