/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

#include "types.h"
#include "logging.h"

#include <cassert>
#include <cstring>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rosidl_typesupport_introspection_c/field_types.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>

#ifndef _WIN32
#  include <dlfcn.h>
#else
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif

MessageTypes::MessageTypes()
{
}

MessageTypes::~MessageTypes()
{
    for (auto it : libraries)
    {
        unload_lib(it.second);
    }
}

const MessageType* MessageTypes::get(const std::string& type)
{
    std::lock_guard<std::mutex> lock(mutex);

    auto it = messages.find(type);
    if (it != messages.end())
    {
        return &it->second;
    }

    DEBUG("Searching for " << type << " type");

    size_t split = type.find('/');
    if (split == std::string::npos)
    {
        ERROR("No '/' in message type " << type);
        return NULL;
    }

    std::string package = type.substr(0, split);
    std::string name = type.substr(split + 1);

    // type_support
    // {
    void* type_support = load_lib(package + "__rosidl_typesupport_c");
    if (!type_support)
    {
        return NULL;
    }
    std::string type_support_symbol_name = "rosidl_typesupport_c__get_message_type_support_handle__" + package + "__msg__" + name;
    void* type_support_symbol = getsym(type_support, type_support_symbol_name);
    if (!type_support_symbol)
    {
        ERROR("Cannot get " << type_support_symbol_name << " symbol in type_support library for " << type << " type");
        return NULL;
    }
    auto get_type_support_hande = (const rosidl_message_type_support_t* (*)(void))type_support_symbol;
    // }

    // introspection
    // {
    void* introspection = load_lib(package + "__rosidl_typesupport_introspection_c");
    if (!introspection)
    {
        return NULL;
    }
    std::string introspection_symbol_name = "rosidl_typesupport_introspection_c__get_message_type_support_handle__" + package + "__msg__" + name;
    void* introspection_symbol = getsym(introspection, introspection_symbol_name);
    if (!introspection_symbol)
    {
        ERROR("Cannot get " << introspection_symbol_name << " symbol in introspection library for " << type << " type");
        return NULL;
    }
    auto get_introspection_hande = (const rosidl_message_type_support_t* (*)(void))introspection_symbol;
    // }

    // generator
    // {
    void* generator = load_lib(package + "__rosidl_runtime_c");
    if (!generator)
    {
        return NULL;
    }
    std::string init_name = package + "__msg__" + name + "__init";
    void* init_symbol = getsym(generator, init_name);
    if (!init_symbol)
    {
        ERROR("Cannot get " << init_name << " symbol in generator library for " << type << " type");
        return NULL;
    }
    std::string fini_name = package + "__msg__" + name + "__fini";
    void* fini_symbol = getsym(generator, fini_name);
    if (!fini_symbol)
    {
        ERROR("Cannot get " << fini_name << " symbol in generator library for " << type << " type");
        return NULL;
    }
    // }

    LOG("Loaded type support for " << type << " type");

    MessageType mtype;
    mtype.type_support = get_type_support_hande();
    mtype.introspection = get_introspection_hande();
    mtype.size = ((const rosidl_typesupport_introspection_c__MessageMembers*)mtype.introspection->data)->size_of_;
    mtype.init = (bool (*)(void*))init_symbol;
    mtype.fini = (void (*)(void*))fini_symbol;

    it = messages.insert(std::make_pair(type, mtype)).first;
    return &it->second;
}

void* MessageTypes::load_lib(const std::string& name)
{
    auto it = libraries.find(name);
    if (it != libraries.end())
    {
        return it->second;
    }

    std::string lib;
#if defined(__linux__)
    lib = "lib" + name + ".so";
#elif defined(__APPLE__)
    lib = "lib" + name + ".dylib";
#elif defined(_WIN32)
    lib = name + ".dll";
#else
    return NULL;
#endif

    DEBUG("Loading " << lib << " library");

    void* handle;
#if defined(__linux__) || defined(__APPLE__)
    handle = dlopen(lib.c_str(), RTLD_LAZY);
#elif defined(_WIN32)
    handle = (void*)LoadLibraryA(lib.c_str());
#endif

    if (handle == NULL)
    {
        ERROR("Cannot load " << lib << " library");
        return NULL;
    }

    it = libraries.insert(std::make_pair(name, handle)).first;
    return it->second;
}

void MessageTypes::unload_lib(void* handle)
{
#if defined(__linux__) || defined(__APPLE__)
    dlclose(handle);
#elif defined(_WIN32)
    FreeLibrary((HMODULE)handle);
#endif
}

void* MessageTypes::getsym(void* lib, const std::string& name)
{
#if defined(__linux__) || defined(__APPLE__)
    return dlsym(lib, name.c_str());
#elif defined(_WIN32)
    return GetProcAddress((HMODULE)lib, name.c_str());
#else
    return NULL;
#endif
}

static_assert(sizeof(bool) == 1);

#define UNS_CHECK_SIZE(need) \
    if (offset + need > size) \
    { \
        ERROR("Not enough data in unserialization for " << member->name_ << ", need = " << need << ", offset = " << offset << ", size = " << size); \
        return false; \
    }

#define UNS_SIMPLE(type, ptr)            \
    UNS_CHECK_SIZE(sizeof(type));        \
    *(type*)ptr = *(type*)&data[offset]; \
    offset += sizeof(type)

#define UNS_STRING(str)                                                          \
    uint32_t length;                                                             \
    UNS_SIMPLE(uint32_t, &length);                                               \
    UNS_CHECK_SIZE(length);                                                      \
    if (!rosidl_runtime_c__String__assignn(str, (char*)&data[offset], length)) \
    {                                                                            \
        ERROR("Failed to assign string for " << member->name_);                  \
        return false;                                                            \
    }                                                                            \
    offset += length

#define UNS_SIMPLE_CASE(id, type, ptr)                      \
    case rosidl_typesupport_introspection_c__ROS_TYPE_##id: \
    {                                                       \
        UNS_SIMPLE(type, ptr);                              \
        break;                                              \
    }

#define UNS_SIMPLE_ARR_CASE(id, idtype, type, ptr, count)                                                             \
    case rosidl_typesupport_introspection_c__ROS_TYPE_##id:                                                           \
    {                                                                                                                 \
        size_t byte_count = count * sizeof(type);                                                                     \
        UNS_CHECK_SIZE(byte_count);                                                                                   \
        if (member->array_size_ == 0)                                                                                 \
        {                                                                                                             \
            auto arr = (rosidl_runtime_c__##idtype##__Sequence*)ptr;                                                \
            if (!rosidl_runtime_c__##idtype##__Sequence__init(arr, count))                                          \
            {                                                                                                         \
                ERROR("Failed to allocate primitive array for " << member->name_ << " for " << count << " elements"); \
                return false;                                                                                         \
            }                                                                                                         \
            ptr = arr->data;                                                                                          \
        }                                                                                                             \
        else                                                                                                          \
        {                                                                                                             \
            std::memset((uint8_t*)ptr + sizeof(type) * count, 0, sizeof(type) * (member->array_size_ - count));       \
        }                                                                                                             \
        std::memcpy(ptr, &data[offset], byte_count);                                                                  \
        offset += byte_count;                                                                                         \
        break;                                                                                                        \
    }

struct Reader
{
    const uint8_t* data;
    size_t size;
    size_t offset;

    Reader(const std::vector<uint8_t>& data)
        : data(data.data())
        , size(data.size())
        , offset(0)
    {
    }

    bool read(void* msg, const rosidl_message_type_support_t* type)
    {
        auto info = (const rosidl_typesupport_introspection_c__MessageMembers*)type->data;

        for (uint32_t i=0; i<info->member_count_; i++)
        {
            auto member = info->members_ + i;

            void* ptr = (uint8_t*)msg + member->offset_;
            if (member->is_array_)
            {
                uint32_t count;
                UNS_SIMPLE(uint32_t, &count);
                if (member->array_size_ != 0)
                {
                    // constant size array
                    if (count != 0 && count != member->array_size_)
                    {
                        ERROR("Incorrect array " << member->name_ << " element count, expected = " << member->array_size_ << ", got = " << count);
                        return false;
                    }
                }

                switch (member->type_id_)
                {
                UNS_SIMPLE_ARR_CASE(FLOAT,   float,   float,    ptr, count);
                UNS_SIMPLE_ARR_CASE(DOUBLE,  double,  double,   ptr, count);
                UNS_SIMPLE_ARR_CASE(BOOLEAN, boolean, bool,     ptr, count);
                UNS_SIMPLE_ARR_CASE(UINT8,   uint8,   uint8_t,  ptr, count);
                UNS_SIMPLE_ARR_CASE(INT8,    int8 ,   int8_t,   ptr, count);
                UNS_SIMPLE_ARR_CASE(UINT16,  uint16,  uint16_t, ptr, count);
                UNS_SIMPLE_ARR_CASE(INT16,   int16,   int16_t,  ptr, count);
                UNS_SIMPLE_ARR_CASE(UINT32,  uint32,  uint32_t, ptr, count);
                UNS_SIMPLE_ARR_CASE(INT32,   int32,   int32_t,  ptr, count);
                UNS_SIMPLE_ARR_CASE(UINT64,  uint64,  uint64_t, ptr, count);
                UNS_SIMPLE_ARR_CASE(INT64,   int64,   int64_t,  ptr, count);

                case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
                {
                    rosidl_runtime_c__String* strings;
                    if (member->array_size_ == 0)
                    {
                        // dynamic size array
                        auto stringseq = (rosidl_runtime_c__String__Sequence*)ptr;
                        if (!rosidl_runtime_c__String__Sequence__init(stringseq, count))
                        {
                            ERROR("Failed to allocate string array for " << member->name_ << " for " << count << " elements");
                            return false;
                        }
                        strings = stringseq->data;
                    }
                    else // constant size array
                    {
                        strings = (rosidl_runtime_c__String*)ptr;
                    }

                    for (uint32_t i=0; i<count; i++)
                    {
                        UNS_STRING(&strings[i]);
                    }
                    break;
                }

                case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
                {
                    void* arr;
                    if (member->array_size_ == 0)
                    {
                        // dynamic size array
                        assert(member->resize_function);
                        if (!member->resize_function(ptr, count))
                        {
                            ERROR("Failed to allocate message array for " << member->name_ << " for " << count << " elements");
                            return false;
                        }
                        arr = ptr;
                    }
                    else // constant size array
                    {
                        // member is static size array, we need address of "array", silly
                        // why does get_function doesn't accept array directly in this case? :/
                        arr = &ptr;
                    }

                    for (uint32_t i=0; i<count; i++)
                    {
                        if (!read(member->get_function(arr, i), member->members_))
                        {
                            return false;
                        }
                    }
                    break;
                }

                // case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
                default:
                    ERROR("Unsupported type " << uint32_t(member->type_id_) << " for " << member->name_);
                    return false;
                }
            }
            else // member is not an array
            {
                switch (member->type_id_)
                {
                UNS_SIMPLE_CASE(FLOAT,   float,    ptr);
                UNS_SIMPLE_CASE(DOUBLE,  double,   ptr);
                UNS_SIMPLE_CASE(BOOLEAN, bool,     ptr);
                UNS_SIMPLE_CASE(UINT8,   uint8_t,  ptr);
                UNS_SIMPLE_CASE(INT8,    int8_t,   ptr);
                UNS_SIMPLE_CASE(UINT16,  uint16_t, ptr);
                UNS_SIMPLE_CASE(INT16,   int16_t,  ptr);
                UNS_SIMPLE_CASE(UINT32,  uint32_t, ptr);
                UNS_SIMPLE_CASE(INT32,   int32_t,  ptr);
                UNS_SIMPLE_CASE(UINT64,  uint64_t, ptr);
                UNS_SIMPLE_CASE(INT64,   int64_t,  ptr);

                case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
                {
                    auto str = (rosidl_runtime_c__String*)ptr;
                    UNS_STRING(str);
                    break;
                }

                case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
                {
                    if (!read(ptr, member->members_))
                    {
                        return false;
                    }
                    break;
                }

                // case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
                // case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
                default:
                    ERROR("Unsupported type " << uint32_t(member->type_id_) << " for " << member->name_);
                    return false;
                }
            }
        }
        return true;
    }
};

bool Unserialize(void* msg, const rosidl_message_type_support_t* type, const std::vector<uint8_t>& data, size_t offset)
{
    Reader reader(data);
    bool ok = reader.read(msg, type);
    if (reader.offset != reader.size)
    {
        // NOTE: if you see this message, verify that your C# structure has correct fields!
        // Compare with .msg file, including other referenced messages - make sure all the types match exactly
        LOG("Did not fully use all data for unserialization, offset=" << reader.offset << ", size=" << reader.size);
    }
    return ok;
}


#define SER_SIMPLE(type, ptr)                            \
    {                                                    \
        type var = *(type*)ptr;                          \
        data.resize(data.size() + sizeof(type));         \
        *(type*)&data[data.size() - sizeof(type)] = var; \
    }

#define SER_STRING(ptr)                                         \
    {                                                           \
        auto str = (rosidl_runtime_c__String*)ptr;            \
        uint32_t length = (uint32_t)str->size;                  \
        SER_SIMPLE(uint32_t, &length);                          \
        data.insert(data.end(), str->data, str->data + length); \
    }

#define SER_SIMPLE_CASE(id, type, ptr)                      \
    case rosidl_typesupport_introspection_c__ROS_TYPE_##id: \
    {                                                       \
        SER_SIMPLE(type, ptr);                              \
        break;                                              \
    }

#define SER_SIMPLE_ARR_CASE(id, idtype, type, ptr)                                  \
    case rosidl_typesupport_introspection_c__ROS_TYPE_##id:                         \
    {                                                                               \
        uint32_t arrcount;                                                          \
        void* arrdata;                                                              \
        if (member->array_size_ == 0)                                               \
        {                                                                           \
            auto arr = (rosidl_runtime_c__##idtype##__Sequence*)ptr;              \
            arrcount = (uint32_t)arr->size;                                         \
            arrdata = arr->data;                                                    \
        }                                                                           \
        else                                                                        \
        {                                                                           \
            arrcount = (uint32_t)member->array_size_;                               \
            arrdata = ptr;                                                          \
        }                                                                           \
        SER_SIMPLE(uint32_t, &arrcount);                                            \
        uint32_t byte_count = arrcount * sizeof(type);                              \
        data.insert(data.end(), (uint8_t*)arrdata, (uint8_t*)arrdata + byte_count); \
        break;                                                                      \
     }

void Serialize(void* msg, const rosidl_message_type_support_t* type, std::vector<uint8_t>& data)
{
    auto info = (const rosidl_typesupport_introspection_c__MessageMembers*)type->data;

    for (uint32_t i=0; i<info->member_count_; i++)
    {
        auto member = info->members_ + i;

        void* ptr = (uint8_t*)msg + member->offset_;
        if (member->is_array_)
        {
            switch (member->type_id_)
            {
            SER_SIMPLE_ARR_CASE(FLOAT,   float,   float,    ptr);
            SER_SIMPLE_ARR_CASE(DOUBLE,  double,  double,   ptr);
            SER_SIMPLE_ARR_CASE(BOOLEAN, boolean, bool,     ptr);
            SER_SIMPLE_ARR_CASE(UINT8,   uint8,   uint8_t,  ptr);
            SER_SIMPLE_ARR_CASE(INT8,    int8 ,   int8_t,   ptr);
            SER_SIMPLE_ARR_CASE(UINT16,  uint16,  uint16_t, ptr);
            SER_SIMPLE_ARR_CASE(INT16,   int16,   int16_t,  ptr);
            SER_SIMPLE_ARR_CASE(UINT32,  uint32,  uint32_t, ptr);
            SER_SIMPLE_ARR_CASE(INT32,   int32,   int32_t,  ptr);
            SER_SIMPLE_ARR_CASE(UINT64,  uint64,  uint64_t, ptr);
            SER_SIMPLE_ARR_CASE(INT64,   int64,   int64_t,  ptr);

            case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
            {
                const rosidl_runtime_c__String* strings;
                uint32_t count;
                if (member->array_size_ == 0)
                {
                    // dynamic size array
                    auto stringseq = (rosidl_runtime_c__String__Sequence*)ptr;
                    strings = stringseq->data;
                    count = (uint32_t)stringseq->size;
                }
                else // constant size array
                {
                    strings = (rosidl_runtime_c__String*)ptr;
                    count = (uint32_t)member->array_size_;
                }
                SER_SIMPLE(uint32_t, &count);
                for (uint32_t i=0; i<count; i++)
                {
                    SER_STRING(&strings[i]);
                }
                break;
            }

            case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
            {
                void* arr;
                uint32_t count;
                if (member->array_size_ == 0)
                {
                    // dynamic size array
                    assert(member->size_function);
                    count = (uint32_t)member->size_function(ptr);
                    arr = ptr;
                }
                else // constant size array
                {
                    count = (uint32_t)member->array_size_;

                       // member is static size array, we need address of "array", silly
                       // why does get_function doesn't accept array directly in this case? :/
                    arr = &ptr;
                }
                SER_SIMPLE(uint32_t, &count);
                for (uint32_t i=0; i<count; i++)
                {
                    Serialize(member->get_function(arr, i), member->members_, data);
                }
                break;
            }
            
            // case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
            default:
                ERROR("Unsupported type " << uint32_t(member->type_id_) << " for " << member->name_);
                return;
            }
        }
        else // member is not an array
        {
            switch (member->type_id_)
            {
            SER_SIMPLE_CASE(FLOAT,   float,    ptr);
            SER_SIMPLE_CASE(DOUBLE,  double,   ptr);
            SER_SIMPLE_CASE(BOOLEAN, bool,     ptr);
            SER_SIMPLE_CASE(UINT8,   uint8_t,  ptr);
            SER_SIMPLE_CASE(INT8,    int8_t,   ptr);
            SER_SIMPLE_CASE(UINT16,  uint16_t, ptr);
            SER_SIMPLE_CASE(INT16,   int16_t,  ptr);
            SER_SIMPLE_CASE(UINT32,  uint32_t, ptr);
            SER_SIMPLE_CASE(INT32,   int32_t,  ptr);
            SER_SIMPLE_CASE(UINT64,  uint64_t, ptr);
            SER_SIMPLE_CASE(INT64,   int64_t,  ptr);

            case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
            {
                SER_STRING(ptr);
                break;
            }

            case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
            {
                Serialize(ptr, member->members_, data);
                break;
            }

            // case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
            // case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
            default:
                ERROR("Unsupported type " << uint32_t(member->type_id_) << " for " << member->name_);
                return;
            }
        }
    }
}
