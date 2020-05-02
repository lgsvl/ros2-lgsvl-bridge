/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include "client.h"
#include "clients.h"
#include "node.h"
#include "logging.h"

#include <memory>
#include <string>
#include <functional>
#include <boost/bind.hpp>

enum
{
    OP_ADD_SUBSCRIBER = 1,
    OP_ADD_PUBLISHER  = 2,
    OP_PUBLISH        = 3,
};

Client::Client(Node& node, Clients& clients, boost::asio::ip::tcp::socket s)
    : node(node)
    , clients(clients)
    , socket(std::move(s))
{
    auto endpoint = socket.remote_endpoint();
    LOG("Client [" << endpoint.address() << ":" << endpoint.port() << "] connected");
}

Client::~Client()
{
}

void Client::start()
{
    socket.async_read_some(
        boost::asio::buffer(temp, sizeof(temp)),
        boost::bind(
            &Client::handle_read,
            shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void Client::stop()
{
    socket.close();
}

void Client::handle_read(const boost::system::error_code& ec, std::size_t size)
{
    if (!ec)
    {
        DEBUG("Received " << size << " bytes");
        buffer.insert(buffer.end(), temp, temp + size);

        size_t size = buffer.size();

        while (size >= sizeof(uint8_t))
        {
            if (buffer[0] == OP_ADD_SUBSCRIBER)
            {
                handle_add_subscriber();
            }
            else if (buffer[0] == OP_ADD_PUBLISHER)
            {
                handle_add_publisher();
            }
            else if (buffer[0] == OP_PUBLISH)
            {
                handle_publish();
            }
            else
            {
                ERROR("Unknown operation received from client (" << uint32_t(buffer[0]) << "), disconnecting client");
                clients.stop(shared_from_this());
                return;
            }

            if (size == buffer.size())
            {
                break;
            }
            size = buffer.size();
        }

        start();
        return;
    }

    if (ec == boost::asio::error::eof)
    {
        // remote side closed connection
        LOG("Client disconnected");
        clients.stop(shared_from_this());
        node.remove(shared_from_this());
        return;
    }

    if (ec != boost::asio::error::operation_aborted)
    {
        ERROR("Client read failed, disconnecting: " << ec.message());
        clients.stop(shared_from_this());
        node.remove(shared_from_this());
        return;
    }
}

void Client::handle_write(const boost::system::error_code& ec)
{
    if (ec && ec != boost::asio::error::operation_aborted)
    {
        ERROR("Client write failed, disconnecting: " << ec.message());
        clients.stop(shared_from_this());
        node.remove(shared_from_this());
        return;
    }
}

// [OP_ADD_SUBSCRIBER] [topic] [type]
void Client::handle_add_subscriber()
{
    if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
    {
        DEBUG("handle_add_subscriber too short header");
        return;
    }

    size_t offset = sizeof(uint8_t);

    uint32_t topic_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + topic_length > buffer.size())
    {
        DEBUG("handle_add_subscriber short1 " << (offset + topic_length) << " " << buffer.size());
        return;
    }

    std::string topic((char*)&buffer[offset], topic_length);
    offset += topic_length;

    uint32_t type_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + type_length > buffer.size())
    {
        DEBUG("handle_add_subscriber short2 " << (offset + type_length) << " " << buffer.size());
        return;
    }

    std::string type((char*)&buffer[offset], type_length);
    offset += type_length;

    DEBUG("OP_ADD_SUBSCRIBER, topic = " << topic << ", type = " << type);

    node.add_subscriber(topic, type, shared_from_this());

    buffer.erase(buffer.begin(), buffer.begin() + offset);
}

// [OP_ADD_PUBLISHER] [topic] [type]
void Client::handle_add_publisher()
{
    if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
    {
        DEBUG("handle_add_publisher too short header");
        return;
    }

    size_t offset = sizeof(uint8_t);

    uint32_t topic_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + topic_length > buffer.size())
    {
        DEBUG("handle_add_publisher short1 " << (offset + topic_length) << " " << buffer.size());
        return;
    }

    std::string topic((char*)&buffer[offset], topic_length);
    offset += topic_length;

    uint32_t type_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + type_length > buffer.size())
    {
        DEBUG("handle_add_publisher short2 " << (offset + type_length) << " " << buffer.size());
        return;
    }

    std::string type((char*)&buffer[offset], type_length);
    offset += type_length;

    DEBUG("OP_ADD_PUBLISHER, topic = " << topic << ", type = " << type);

    node.add_publisher(topic, type, shared_from_this());

    buffer.erase(buffer.begin(), buffer.begin() + offset);
}

// [OP_PUBLISH] [topic] [message]
void Client::handle_publish()
{
    if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > buffer.size())
    {
        return;
    }

    size_t offset = sizeof(uint8_t);

    uint32_t topic_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + topic_length > buffer.size())
    {
        return;
    }

    std::string topic((char*)&buffer[offset], topic_length);
    offset += topic_length;

    uint32_t message_length = get32le(offset);
    offset += sizeof(uint32_t);
    if (offset + message_length > buffer.size())
    {
        return;
    }

    std::vector<uint8_t> message(&buffer[offset], &buffer[offset] + message_length);
    offset += message_length;

    DEBUG("OP_PUBLISH, topic = " << topic);

    node.publish(topic, message);

    buffer.erase(buffer.begin(), buffer.begin() + offset);
}

void Client::publish(const std::string& topic, const std::vector<uint8_t>& msg)
{
    std::vector<uint8_t> data;
    data.reserve(sizeof(uint8_t) + sizeof(uint32_t) + topic.size() + sizeof(uint32_t) + msg.size());

    data.push_back(OP_PUBLISH);

    data.push_back(uint8_t(topic.size() >> 0));
    data.push_back(uint8_t(topic.size() >> 8));
    data.push_back(uint8_t(topic.size() >> 16));
    data.push_back(uint8_t(topic.size() >> 24));
    data.insert(data.end(), (uint8_t*)topic.data(), (uint8_t*)topic.data() + topic.size());

    data.push_back(uint8_t(msg.size() >> 0));
    data.push_back(uint8_t(msg.size() >> 8));
    data.push_back(uint8_t(msg.size() >> 16));
    data.push_back(uint8_t(msg.size() >> 24));
    data.insert(data.end(), msg.data(), msg.data() + msg.size());

    boost::asio::async_write(
        socket,
        boost::asio::buffer(data.data(), data.size()),
        boost::bind(
            &Client::handle_write,
            shared_from_this(),
            boost::asio::placeholders::error));
}

uint32_t Client::get32le(size_t offset) const
{
    return buffer[offset + 0]
        | (buffer[offset + 1] << 8)
        | (buffer[offset + 2] << 16)
        | (buffer[offset + 3] << 24);
}
