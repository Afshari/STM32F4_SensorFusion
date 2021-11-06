
#ifndef __TCP_SERVER_H__
#define __TCP_SERVER_H__


#include "tcp_server.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"

#include <algorithm>
#include <string>
#include <functional>


using std::string;
using std::function;

void tcp_server_init(function<string (struct pbuf*)> response_handler);
void tcp_server_init();

#endif /* __TCP_SERVER_H__ */
