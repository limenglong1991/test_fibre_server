#include "my_thread.h"
#include <fibre/posix_tcp.hpp>
#include <fibre/posix_udp.hpp>
my_thread::my_thread(QObject *parent) : QObject(parent)
{

}
void my_thread::on_server()
{
    //serve_on_tcp(9910);
    serve_on_udp(9910);
}
