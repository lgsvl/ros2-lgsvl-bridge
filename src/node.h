/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#pragma once

#include "types.h"

#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <rcl/rcl.h>

class Client;

class Node
{
public:
    Node(rcl_allocator_t* alloc, rcl_context_t* context);
    ~Node();

    void remove(std::shared_ptr<Client> client);

    void add_subscriber(const std::string& topic, const std::string& type, std::shared_ptr<Client> client);
    void add_publisher(const std::string& topic, const std::string& type, std::shared_ptr<Client> client);

    void publish(const std::string& topic, const std::vector<uint8_t>& data);

private:
    rcl_allocator_t* alloc;
    rcl_context_t* context;
    rcl_node_t node;
    rcl_wait_set_t wait;

    std::mutex mutex;
    std::queue<std::function<void()>> actions;

    volatile bool running;
    std::thread thread;
    struct Publisher
    {
        rcl_publisher_t pub;
        const MessageType* type;
        std::unordered_set<std::shared_ptr<Client>> clients;
    };

    typedef std::unordered_map<std::string, Publisher> Publishers;
    Publishers publishers;

    struct Subscriber
    {
        // keep rcl_subscription_t as first member of this struct
        // other code relies on this fact!
        rcl_subscription_t sub;
        const MessageType* type;
        std::string topic;
        std::unordered_set<std::shared_ptr<Client>> clients;
    };

    typedef std::vector<std::unique_ptr<Subscriber>> Subscribers;
    Subscribers subscribers;

    MessageTypes types;

    void execute();
    void handle_message(Subscriber* sub);

    Node(const Node&) = delete;
    Node& operator = (const Node&) = delete;
};
