/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include "server.h"
#include "client.h"
#include "logging.h"

#include <memory>
#include <functional>
#include <cstdint>
#include <signal.h>
#include <boost/bind.hpp>

Server::Server(Node& node, unsigned int port)
    : node(node)
    , signals(io)
    , endpoint(boost::asio::ip::tcp::v4(), (uint16_t)port)
    , acceptor(io, endpoint)
    , socket(io)
{
    LOG("Listening on port " << port);

    signals.add(SIGTERM);
    signals.add(SIGINT);
}

Server::~Server()
{
}

void Server::run()
{
    signals.async_wait(boost::bind(
        &Server::stop,
        shared_from_this(),
        boost::asio::placeholders::error,
        boost::asio::placeholders::signal_number));

    begin_accept();

    io.run();
}

void Server::stop(const boost::system::error_code& ec, int signal_number)
{
    if (ec)
    {
        ERROR("Error waiting on signals: " << ec.message());
    }
    else
    {
        LOG("Signal " << signal_number << " received, stopping server");
    }
    acceptor.close();
    clients.stop_all();
}

void Server::begin_accept()
{
    acceptor.async_accept(
        socket,
        boost::bind(
            &Server::end_accept,
            shared_from_this(),
            boost::asio::placeholders::error));
}

void Server::end_accept(const boost::system::error_code& ec)
{
    if (!acceptor.is_open())
    {
        return;
    }

    if (ec)
    {
        ERROR("Error accepting connection: " << ec.message());
    }
    else
    {
        auto client = std::make_shared<Client>(node, clients, std::move(socket));
        clients.start(client);
    }

    begin_accept();
}
