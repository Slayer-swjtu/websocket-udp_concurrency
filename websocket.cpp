#include "websocket.h"
#include <iostream>


void do_session(tcp::socket socket)
{
	try
	{
		//std::cout << "init new";
		websocket::stream<tcp::socket> ws{ std::move(socket) };
		ws.set_option(websocket::stream_base::decorator(
			[](websocket::response_type& res)
			{
				res.set(http::field::server,
					std::string(BOOST_BEAST_VERSION_STRING) +
					" websocket-server-sync");
			}));
		ws.accept();
		std::cout << "--达成连接--" << std::endl;
		for (;;)
		{
			beast::flat_buffer buffer;
			ws.text(true/*ws.got_text()*/);
			if (mq.empty())
				continue;
			else
			{
				std::string temp = mq.front();
				mq.pop();
				temp = parse2write(temp, mission);
				ws.write(net::buffer(temp, temp.length()));
			}
		}
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
	catch (beast::system_error const& se)
	{
		if (se.code() != websocket::error::closed)
			std::cerr << "Error: " << se.code().message() << std::endl;
	}
	return;
}

void Websc::server_start()
{
	std::cout << "进入web构建\n";
	net::io_context ioc{ 1 };
	auto const address = net::ip::make_address(address_);
	auto const port = static_cast<unsigned short>(std::atoi(port_));
	tcp::acceptor acceptor{ ioc, { address,port} };
	//for(;;)
	tcp::socket socket{ ioc };
	acceptor.accept(socket);//连接阻塞，没有新连接就会hang
	std::thread(&do_session, std::move(socket)).detach();//创建线程即分离，在有一定延时的情况下可以创建多级连接，目前是一辆
	for (;;)
	{
		//hang这个线程
		Sleep(1000);
	}
	return;
}

