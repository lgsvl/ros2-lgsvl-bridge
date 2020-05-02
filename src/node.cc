/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include "node.h"
#include "client.h"
#include "logging.h"

#include <cstring>
#include <chrono>

Node::Node(rcl_allocator_t* alloc, rcl_context_t* context)
    : running(true)
    , alloc(alloc)
    , context(context)
    , node(rcl_get_zero_initialized_node())
    , wait(rcl_get_zero_initialized_wait_set())
{
    rcl_node_options_t opts = rcl_node_get_default_options();
    rcl_ret_t rc;

    rc = rcl_node_init(&node, "bridge", "lgsvl", context, &opts);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_init failed: " << rc);
    }

    thread = std::thread(&Node::execute, this);
}

Node::~Node()
{
    running = false;
    thread.join();

    rcl_ret_t rc;

    for (const auto& it : subscribers)
    {
        rc = rcl_subscription_fini(&it->sub, &node);
        if (rc != RCL_RET_OK)
        {
            ERROR("= rcl_subscription_fini failed: " << rc);
        }
    }

    for (auto it : publishers)
    {
        rc = rcl_publisher_fini(&it.second.pub, &node);
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_publisher_fini failed: " << rc);
        }
    }

    rc = rcl_wait_set_fini(&wait);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_wait_set_fini failed: " << rc);
    }

    rc = rcl_node_fini(&node);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_node_fini failed: " << rc);
    }
}

void Node::execute()
{
    size_t sub_count = 0;
    rcl_ret_t rc;

    const unsigned int timeout = 100; // msec
    while (running)
    {
        size_t new_sub_count;
        {
            std::lock_guard<std::mutex> lock(mutex);
            while (!actions.empty())
            {
                auto action = actions.front();
                action();
                actions.pop();
            }
            new_sub_count = subscribers.size();
        }

        if (new_sub_count == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
            continue;
        }

        if (new_sub_count != sub_count)
        {
            sub_count = new_sub_count;

            rc = rcl_wait_set_fini(&wait);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_fini failed: " << rc);
            }

            rc = rcl_wait_set_init(&wait, sub_count, 0, 0, 0, 0, 0, context, *alloc);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_init failed: " << rc);
            }
        }
        else
        {
            rc = rcl_wait_set_clear(&wait);
            if (rc != RCL_RET_OK)
            {
                ERROR("rcl_wait_set_clear failed: " << rc);
                return;
            }
        }

           for (const auto& it : subscribers)
           {
               size_t index;
               rc = rcl_wait_set_add_subscription(&wait, &it->sub, &index);
               if (rc != RCL_RET_OK)
               {    
                   ERROR("rcl_wait_set_add_subscription failed: " << rc);
               }
           }

        rc = rcl_wait(&wait, RCL_MS_TO_NS(timeout));
        if (rc == RCL_RET_TIMEOUT)
        {
            continue;
        }
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_wait failed: " << rc);
        }

        for (size_t i=0; i<sub_count; i++)
        {
            if (wait.subscriptions[i])
            {
                Subscriber* sub = (Subscriber*)wait.subscriptions[i];
                handle_message(sub);
            }
        }
    }
}

void Node::remove(std::shared_ptr<Client> client)
{
    auto action = [=]()
    {
        for (auto it = subscribers.begin(); it != subscribers.end(); /* empty */)
        {
            Subscriber* sub = it->get();
            if (sub->clients.find(client) != sub->clients.end())
            {
                DEBUG("Removing client from subscribers on " << sub->topic << " topic");
                sub->clients.erase(client);
                if (sub->clients.empty())
                {
                    LOG("Removing subscriber on " << sub->topic << " topic");

                    rcl_ret_t rc = rcl_subscription_fini(&sub->sub, &node);
                    if (rc != RCL_RET_OK)
                    {
                        ERROR("rcl_subscription_fini failed: " << rc);
                    }

                    it = subscribers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            else
            {
                ++it;
            }
        }
    };

    {
        std::lock_guard<std::mutex> lock(mutex);
        actions.push(action);
    }

    for (auto it = publishers.begin(), eit = publishers.end(); it != eit; /* empty */)
    {
        if (it->second.clients.find(client) != it->second.clients.end())
        {
            DEBUG("Removing client from publishers on " << it->first << " topic");
            it->second.clients.erase(client);
            if (it->second.clients.empty())
            {
                LOG("Removing publisher on " << it->first << " topic");

                rcl_ret_t rc = rcl_publisher_fini(&it->second.pub, &node);
                if (rc != RCL_RET_OK)
                {
                    ERROR("rcl_publisher_fini failed: " << rc);
                }

                it = publishers.erase(it);
            }
            else
            {
                ++it;
            }
        }
        else
        {
            ++it;
        }
    }
}

void Node::add_subscriber(const std::string& topic, const std::string& type, std::shared_ptr<Client> client)
{
    const MessageType* message_type = types.get(type);
    if (message_type == NULL)
    {
        return;
    }

     auto action = [=]()
     {
        for (auto it = subscribers.begin(), eit = subscribers.end(); it != eit; ++it)
        {
            Subscriber* sub = it->get();
            if (sub->topic == topic)
            {
                sub->clients.insert(client);
                return;
            }
        }

        std::unique_ptr<Subscriber> s(new Subscriber);
        s->sub = rcl_get_zero_initialized_subscription();
        s->type = message_type;
        s->topic = topic;
        s->clients.insert(client);

        rcl_subscription_options_t sub_opt = rcl_subscription_get_default_options();
        rcl_ret_t rc = rcl_subscription_init(&s->sub, &node, message_type->type_support, topic.c_str(), &sub_opt);
        if (rc != RCL_RET_OK)
        {
            ERROR("rcl_subscription_init failed: " << rc);
            return;
        }

        auto it = subscribers.insert(subscribers.end(), std::move(s));
        assert((void*)it->get() == (void*)&it->get()->sub);

        LOG("Subscribed " << type << " on " << topic);
     };

    std::lock_guard<std::mutex> lock(mutex);
    actions.push(action);
}

void Node::add_publisher(const std::string& topic, const std::string& type, std::shared_ptr<Client> client)
{
    auto it = publishers.find(topic);
    if (it != publishers.end())
    {
        it->second.clients.insert(client);
        return;
    }

    const MessageType* message_type = types.get(type);
    if (message_type == NULL)
    {
        return;
    }

    rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

    rcl_ret_t rc = rcl_publisher_init(&pub, &node, message_type->type_support, topic.c_str(), &pub_opt);
    if (rc != RCL_RET_OK)
    {
        ERROR("rcl_publisher_init failed: " << rc);
        return;
    }

    Publisher p;
    p.pub = pub;
    p.type = message_type;
    p.clients.insert(client);

    publishers.insert(std::make_pair(topic, p));

    LOG("Publishing " << type << " on " << topic);
}

void Node::publish(const std::string& topic, const std::vector<uint8_t>& data)
{
    auto it = publishers.find(topic);
    if (it == publishers.end())
    {
        ERROR("No publisher registered on topic " << topic << ", ignorning message");
        return;
    }

    const MessageType* type = it->second.type;
    rcl_publisher_t* pub = &it->second.pub;

    void* msg = malloc(type->size);
    if (msg)
    {
        if (type->init(msg))
        {
            DEBUG("Unserializing message for " << topic << " topic");
            if (Unserialize(msg, type->introspection, data))
            {
                rcl_ret_t rc = rcl_publish(pub, msg, NULL);
                if (rc != RCL_RET_OK)
                {
                    ERROR("rcl_publish failed: " << rc);
                }
            }

            type->fini(msg);
        }
        else
        {
            ERROR("Message init failed for " << topic << " topic");
        }

        free(msg);
    }
    else
    {
        ERROR("Out of memory when unserializing message on " << topic << " topic");
    }
}

void Node::handle_message(Subscriber* sub)
{
    rcl_ret_t rc;

    void* msg = malloc(sub->type->size);
    if (msg)
    {
        if (sub->type->init(msg))
        {
            rmw_message_info_t info;
            rc = rcl_take(&sub->sub, msg, &info, NULL);
            if (rc == RCL_RET_OK)
            {
                DEBUG("New message received " << sub->topic << " topic");

                std::vector<uint8_t> data;
                data.reserve(4096);

                DEBUG("Serializing message for " << sub->topic << " topic");
                Serialize(msg, sub->type->introspection, data);

                for (auto client : sub->clients)
                {
                    client->publish(sub->topic, data);
                }
            }
            else if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED)
            {
                // everything's fine, no message, ignore this
            }
            else
            {
                ERROR("rcl_take failed: "<< rc);
            }

            sub->type->fini(msg);
        }
        else
        {
            ERROR("Message init failed on " << sub->topic << " topic");
        }

        free(msg);
    }
    else
    {
        ERROR("Out of memory when receiving message " << sub->topic << " topic");
    }
}
