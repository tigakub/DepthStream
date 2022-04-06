//
//  Connection.cpp
//  SocketServer
//
//  Created by Edward Janne on 4/1/22.
//

#include "Connection.hpp"
#include "Server.hpp"

void Connection::shutdownNotification() {
    if(server) server->sockDidShutdown(this);
}

