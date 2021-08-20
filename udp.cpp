#include "udp.h"
#include <string>
#include <iostream>



void Udp::udp_stopcommand(boost::asio::serial_port &sp1)
{
	std::string str = "{}";//��ͣ����
	boost::asio::write(sp1, boost::asio::buffer((char*)str.data(), str.length()));
}

void Udp::udp_start(std::thread &th,boost::asio::serial_port &sp1)
{
	boost::asio::io_context io_context;
	udp::socket sock(io_context, udp::endpoint(udp::v4(), this->port));
	std::cout << "init udp!\n";
	for (;;)
	{
		char data[this->length_];
		udp::endpoint sender_endpoint;
		size_t length = sock.receive_from(
			boost::asio::buffer(data, length_), sender_endpoint);
		sock.send_to(boost::asio::buffer(data, length), sender_endpoint);
		//��Ҫ����Э��
		//�յ�udp��֪ͨ���ڻ�����ղ������µ�����
		
		TerminateThread(th.native_handle(), 0);
		udp_stopcommand(sp1);
		break;
	}
}