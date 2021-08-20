#ifndef UDP_INCLUDED
#define UDP_INCLUDED

#include <boost/asio.hpp>
#include <windows.h>
#include "mydata.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::udp;

class Udp
{
public:
	//void udp_start(std::thread &th);
	void udp_start(std::thread& th,boost::asio::serial_port &sp1);
	void udp_stopcommand(boost::asio::serial_port& sp1);
private:
	enum { length_ = 1024 };
	const size_t port = 8888;
};
#endif