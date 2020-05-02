/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#pragma once

#include <cstdint>
#include <vector>
#include <boost/asio.hpp>

class Clients;
class Node;

class Client : public std::enable_shared_from_this<Client>
{
public:
    Client(Node& node, Clients& clients, boost::asio::ip::tcp::socket socket);
    ~Client();

    void start();
    void stop();

    void publish(const std::string& topic, const std::vector<uint8_t>& msg);

private:
    Node& node;
    Clients& clients;

    boost::asio::ip::tcp::socket socket;

    uint8_t temp[1024*1024];
    std::vector<uint8_t> buffer;

    void handle_read(const boost::system::error_code& ec, std::size_t length);
    void handle_write(const boost::system::error_code& ec);

    void handle_add_subscriber();
    void handle_add_publisher();
    void handle_publish();

    uint32_t get32le(size_t offset) const;
};
