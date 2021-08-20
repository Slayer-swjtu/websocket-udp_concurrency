#include "mydata.h"
#include <fstream>

std::size_t count_ = 0;
const char* COM = "COM9";//串口设备号
const char* MAP_FILE = "F:/gcrobot_server/api/maps/robots_maps.json";//地图文件地址
const char* MISSION_FILE = "F:/gcrobot_server/api/mission/group/mission.json";//任务文件地址
const char* RESULT_FILE = "F:/gcrobot_server/api/mission/group/mission_result.json";//路径规划文件地址
const char* COMMAND_FILE = "F:/gcrobot_server/api/robots/remote_command.json";//遥控文件地址

const char* address_ = "0.0.0.0";//以太网连接  ipv4
const char* port_ = "8001";//websocket的port号
bool END = false;
std::vector<std::vector<double> > mission;
std::queue<std::string> mq;

std::string parse2write(std::string str, std::vector<std::vector<double> > sum)
{
	Origin origin;
	Json::Reader reader(Json::Features::strictMode());
	Json::Value root;
	if (reader.parse(str, root))
	{
		std::cout << "接受的JSON字符串解析正确\n";
		Json::Value _root;
		Json::Value state;
		if (count_ == 0)
		{
			origin.x = root["x"].asDouble();
			origin.y = root["y"].asDouble();
			count_++;
		}
		state["code"] = Json::Value(20000);
		Json::Value s;
		s["id"] = Json::Value(10);
		s["name"] = Json::Value("gc1");
		s["sn"] = Json::Value("123456");
		s["robot_sn"] = Json::Value("123456");
		s["state"] = Json::Value("UNKNOWN");
		s["x"] = root["x"];
		s["y"] = root["y"];
		s["enable"] = Json::Value(true);
		s["power"] = root["power"];
		s["oil"] = root["oil"];
		state["data"].append(s);
		std::ofstream os;
		os.open("F:/gcrobot_server/api/robots/robots_state.json", std::ios::binary);//这里依旧是写文件
		os << state.toStyledString();
		os.close();

		_root = root;
		_root["x"] = root["x"].asDouble() + 50;
		_root["y"] = root["y"].asDouble() - 50;
		//path 1元
		int index = root["index"].asInt();
		Json::Value __leaf;
		Json::Value __leaf__;
		__leaf__["x"] = Json::Value(origin.x);
		__leaf__["y"] = Json::Value(origin.y);
		__leaf__["z"] = Json::Value(0.0);
		__leaf.append(__leaf__);
		__leaf__["x"] = Json::Value(sum[0][0]);
		__leaf__["y"] = Json::Value(sum[0][1]);
		__leaf__["z"] = Json::Value(0.0);
		__leaf.append(__leaf__);
		_root["path"].append(__leaf);
		//path 2元
		Json::Value _leaf;
		for (int i = 0; i < sum.size(); i++)
		{
			Json::Value _leaf_;
			_leaf_["x"] = Json::Value(sum[i][0]);
			_leaf_["y"] = Json::Value(sum[i][1]);
			_leaf_["z"] = Json::Value(0.0);
			_leaf.append(_leaf_);
		}_root["path"].append(_leaf);
		if (!index < 0)
		{
			for (int i = index; i < sum.size(); i++)
			{
				_root["scan"].append(sum[i][0]);
				_root["scan"].append(sum[i][1]);
			}
		}
		else
		{
			for (int i = 0; i < sum.size(); i++)
			{
				_root["scan"].append(sum[i][0]);
				_root["scan"].append(sum[i][1]);
			}
		}
		_root["sn"] = Json::Value("123456");
		_root["name"] = Json::Value("gc1");
		_root["id"] = Json::Value(10);
		Json::FastWriter w;
		return w.write(_root);
	}
	else
	{
		std::cout << "JSON字符串解析错误\n";
		root["code"] = 20000;
		root["name"] = "gc1";
		root["id"] = 10;
		return root.toStyledString();
	}
}

std::string load_file(const char* address)
{
	std::ifstream fin(address, std::ios::binary);
	std::istreambuf_iterator<char>  beg(fin), end;
	std::string  str(beg, end);
	fin.close();
	return str;
}

bool check_file(const char* address)
{
	bool flag;
	std::ifstream fin_(address, std::ios::binary);
	fin_.seekg(0, std::ios::end);
	std::streampos fp_ = fin_.tellg();
	if (int(fp_) != 0)
		flag = true;
	else
		flag = false;
	fin_.close(); fin_.clear();
	return flag;
}

