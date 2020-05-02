/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include <rosidl_generator_c/message_type_support_struct.h>

struct MessageType
{
    const rosidl_message_type_support_t* type_support;
    const rosidl_message_type_support_t* introspection;

    size_t size;
    bool (*init)(void* msg);
    void (*fini)(void* msg);
};

class MessageTypes
{
public:
    MessageTypes();
    ~MessageTypes();

    // thread-safe
    const MessageType* get(const std::string& type);

private:
    typedef std::unordered_map<std::string, void*> Libraries;
    typedef std::unordered_map<std::string, MessageType> Messages;
    
    Libraries libraries;
    Messages messages;

    std::mutex mutex;

    void* load_lib(const std::string& name);
    static void unload_lib(void* handle);
    static void* getsym(void* lib, const std::string& name);

    MessageTypes(const MessageTypes&) = delete;
    MessageTypes& operator = (const MessageTypes&) = delete;
};

bool Unserialize(void* msg, const rosidl_message_type_support_t* type, const std::vector<uint8_t>& data, size_t offset = 0);
void Serialize(void* msg, const rosidl_message_type_support_t* type, std::vector<uint8_t>& data);
