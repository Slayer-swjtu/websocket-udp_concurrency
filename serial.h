//#ifndef SERIAL_INCLUDED
//#define SERIAL_INCLUDED
//#include <boost/bind.hpp>
//#include <boost/asio.hpp>
//#include "mydata.h"
//
//using namespace boost::asio;
////io_service iosev;
////serial_port sp1(iosev, COM);//初始化设备名在这，省去传参引用值
//
//class Serial
//{
//public:
//	Serial();
//	~Serial();
//private:
//
//};
//Serial::Serial()
//{
//	io_service iosev;
//	serial_port sp1(iosev, COM);
//	sp1.set_option(serial_port::baud_rate(115200));
//	sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
//	sp1.set_option(serial_port::parity(serial_port::parity::none));
//	sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
//	sp1.set_option(serial_port::character_size(8));
//}
//
//
//#endif