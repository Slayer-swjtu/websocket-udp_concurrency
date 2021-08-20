#include "udp.h"
#include "websocket.h"
#include <fstream>
//#include <boost/interprocess/shared_memory_object.hpp>
//#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

using namespace boost::asio;
using namespace boost::interprocess;


struct mutex_remove
{
	mutex_remove() { named_mutex::remove("fstream_named_mutex"); }
	~mutex_remove() { named_mutex::remove("fstream_named_mutex"); }
};


bool check_reception(std::string& str, const char* m, serial_port &sp1)
{
	boost::asio::write(sp1, buffer((char*)str.data(), str.length()));
	std::string serialbuf;
	std::size_t count = boost::asio::read(sp1, boost::asio::dynamic_buffer(serialbuf));
	if (serialbuf[0] == '*' && serialbuf.back() == '$')
	{
		serialbuf = serialbuf.substr(1, serialbuf.length() - 2);
		Json::Value root; Json::Reader reader;
		reader.parse(serialbuf, root);
		if (!root[m].isNull())
		{
			std::cout << "收到机器人对" << m << "的反馈信息!\n";
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

void set_process(serial_port &sp1)
{
	bool check;
	do
	{
		check = check_file(MAP_FILE);
	} while (!check);
	std::string str = load_file(MAP_FILE);

	while (1)
	{
		if (check_reception(str, "got",sp1))
			break;
	}

	str.clear();

	do
	{
		check = check_file(MISSION_FILE);
	} while (!check);

	std::vector<std::vector<double> > edge(4, std::vector<double>(2, 0));
	std::vector<double> origin(3, 0);
	Json::Value ob;

	Json::Reader reader; Json::Value root;
	std::ifstream  in(MISSION_FILE, std::ios::binary);

	if (reader.parse(in, root))//写任务角点
	{
		Json::Value leaf_; Json::FastWriter writer_;
		leaf_["sn"] = root["sn"];
		leaf_["corner_points"] = root["corner_points"];
		std::string serialbuf = writer_.write(leaf_);

		while (1)
		{
			if (check_reception(serialbuf, "corner_points",sp1))
				break;
		}

		for (unsigned int i = 0; i < root["corner_points"].size(); i++)
		{
			edge[i][0] = root["corner_points"][i]["x"].asDouble();
			edge[i][1] = root["corner_points"][i]["y"].asDouble();
		}
	}in.close();

	{
		std::ifstream _(MAP_FILE, std::ios::binary);
		Json::Reader reader; Json::Value _root; int j = 0;
		if (reader.parse(_, _root))
		{
			for (unsigned int i = 0; i < _root["data"].size(); i++)
			{
				if (_root["data"][i]["used"].asBool() == true)
				{
					origin[0] = _root["data"][i]["boundary"][j]["lon"].asDouble();
					origin[1] = _root["data"][i]["boundary"][j]["lat"].asDouble();
					origin[2] = _root["data"][i]["boundary"][j]["height"].asDouble();
					ob["obstacle"] = _root["data"][i]["obstacle"];
				}
			}
		}
		_.close();
		mission = path_planning(edge, origin, ob);
	}

	{
		std::vector<std::vector<double> >& v = mission;
		Json::Value root; Json::Value leaf; Json::Value xyz; Json::Value xyz_;
		xyz_["x"] = 0.0; xyz_["y"] = 0.0; xyz_["z"] = 0.0; xyz_["w"] = 0.0;

		for (size_t i = 0; i < v.size(); i++)
		{
			Json::Value stem;
			xyz["x"] = v[i][0]; xyz["y"] = v[i][1]; xyz["z"] = 0.0;
			leaf["position"] = xyz;
			leaf["rotation"] = xyz_;
			stem.append(leaf);

			if ((i + 1) < v.size())
			{
				xyz["x"] = v[i + 1][0]; xyz["y"] = v[i + 1][1]; xyz["z"] = 0.0;
				leaf["position"] = xyz;
				leaf["rotation"] = xyz_;
				stem.append(leaf);
			}
			else
			{
				xyz["x"] = v[0][0]; xyz["y"] = v[0][1]; xyz["z"] = 0.0;
				leaf["position"] = xyz;
				leaf["rotation"] = xyz_;
				stem.append(leaf);
			}
			root.append(stem);
		}
		Json::Value earth; earth.append(root);
		Json::FastWriter wr;
		std::ofstream os;
		os.open(RESULT_FILE);
		os << wr.write(earth);
		os.close();
	}
	str = "{\"set\":true}";
	while (1)
	{
		if (check_reception(str, "set",sp1))//最后一次的键是啥？
			break;
	}
	return;
}

void thread_serial(serial_port &sp1)//串口线程内打开共享内存映射
{
	try 
	{
		//struct shm_remove
		//{
		//	~shm_remove() { shared_memory_object::remove("MySharedMemory"); }
		//} remover;
		//shared_memory_object shm(open_only,"MySharedMemory",read_write);//应该是串口线程往队列写，websocket逐个去读，命令发送才是内存映射
		//mapped_region region(shm,read_write);
		//void* addr = region.get_address();
		//shared_memory* data = static_cast<shared_memory*>(addr);
		//Open or create the named mutex
		named_mutex mutex(open_or_create, "fstream_named_mutex");
		for (;;)
		{
			{
				mutex_remove __;//析构里放锁
				scoped_lock<named_mutex> lock(mutex);
				std::ifstream file(COMMAND_FILE, std::ios::binary);
				std::ostringstream  tmp;
				tmp << file.rdbuf();
				std::string  str = tmp.str();
				boost::asio::write(sp1, buffer((char*)str.data(), str.length()));
				file.close();
				std::fstream file_(COMMAND_FILE,std::ios::out);
				file_.close();
				////读临界区，串口发送指令是依照共享内存的，串口收到消息给终端是队列！
				//{
				//	scoped_lock<interprocess_mutex> lock(data->mutex);
				//	if (!data->command.empty())
				//	{
				//		std::cout << data->command;
				//		boost::asio::write(sp1, buffer((char*)data->command.data(), data->command.length()));
				//		data->command.clear();
				//	}
				//	//这里解锁
				//}
			}
			std::string serialbuf;
			std::size_t count = boost::asio::read_until(sp1, boost::asio::dynamic_buffer(serialbuf), '$');//阻塞访问io设备
			if (serialbuf[0] == '*' && serialbuf.back() == '$')
			{
				std::cout<<count<<"\n"<< serialbuf << std::endl;
				mq.push(serialbuf.substr(1, serialbuf.length() - 2));
			}
		}
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
	catch (boost::interprocess::interprocess_exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
	return;
}

void thread_udp(Udp &udp, std::thread &th,serial_port &sp1)
{
	udp.udp_start(th,sp1);
	return;
}


int main()
{
	//运行前请保证串口连接正确
	//历史遗留问题: 匿名局部互斥锁版本，httpserver映射后不能写数据进去，找不到问题，先重新采用具名方式实现
	io_service iosev;
	serial_port sp1(iosev, COM);
	sp1.set_option(serial_port::baud_rate(115200));
	sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp1.set_option(serial_port::parity(serial_port::parity::none));
	sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp1.set_option(serial_port::character_size(8));

	Udp udp; Websc websc;
	//set_process(sp1);

	std::thread t1(&thread_serial, std::ref(sp1)); //t1.detach();   串口不能detach，分离后本地句柄和线程id全部为0，理论上无法在外部使用winapi terminate了，毕竟std没有提供外部结束的方法
	std::thread t2(&thread_udp, std::ref(udp), std::ref(t1), std::ref(sp1)); t2.detach();

	websc.server_start();

	while (!END)
		Sleep(1000);

	return 0;
}