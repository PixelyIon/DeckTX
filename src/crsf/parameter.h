/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#pragma once

#include <chrono>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include <common/error.h>

#include "protocol.h"

namespace crsf {
template <typename T>
struct IntegerParameter {
    T value;
    T min;
    T max;
    std::string units;

    constexpr IntegerParameter(std::span<u8> data) {
        auto ptr{data.data()};
        
        value = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);
        min = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);
        max = *reinterpret_cast<T*>(ptr);
        ptr += sizeof(T);

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct FloatParameter {
    float value;
    float min;
    float max;
    u8 precision;
    u32 step;
    std::string units;

    FloatParameter(std::span<u8> data) {
        auto ptr{data.data()};

        value = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);
        min = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);
        max = *reinterpret_cast<float*>(ptr);
        ptr += sizeof(float);

        precision = *ptr++;
        step = *reinterpret_cast<u32*>(ptr);
        ptr += sizeof(u32);

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct TextSelectionParameter {
    u8 selectedIndex;
    u8 minIndex;
    u8 maxIndex;
    u8 defaultIndex;
    std::vector<std::string> options;
    std::string units;

    TextSelectionParameter(std::span<u8> data) {
        auto ptr{data.data()};
        std::string_view optionsString{reinterpret_cast<char*>(ptr)};
        for (size_t start{}, end{}; end != std::string_view::npos; start = end + 1) {
            end = optionsString.find(';', start);
            options.emplace_back(optionsString.substr(start, end - start));
        }
        ptr += optionsString.size() + 1;

        constexpr char LuaSymbolArrowUp{'\xc0'};
        constexpr char LuaSymbolArrowDown{'\xc1'};

        for (std::string& option : options) {
            for (size_t i{}; i < option.size(); i++) {
                if (option[i] == LuaSymbolArrowUp) {
                    option.replace(i, 1, "↑");
                    i++;
                } else if (option[i] == LuaSymbolArrowDown) {
                    option.replace(i, 1, "↓");
                    i++;
                }
            }
        }

        selectedIndex = *ptr++;
        minIndex = *ptr++;
        maxIndex = *ptr++;
        defaultIndex = *ptr++;

        units = std::string(reinterpret_cast<char*>(ptr));
    }
};

struct FolderParameter {
    std::string displayName;

    constexpr FolderParameter(std::span<u8> data) {
        if (!data.empty())
            displayName = std::string(reinterpret_cast<char*>(data.data()));
    }
};

struct InfoParameter {
    std::string display;

    constexpr InfoParameter(std::span<u8> data) {
        display = std::string(reinterpret_cast<char*>(data.data()));
    }
};

struct CommandParameter {
    CommandStep step;
    std::string status;

    constexpr CommandParameter(std::span<u8> data) {
        step = static_cast<CommandStep>(data[0]);
        status = std::string(reinterpret_cast<char*>(data.data() + 1));
    }
};

using ParameterValue = std::variant<std::monostate, IntegerParameter<u8>, IntegerParameter<i8>, IntegerParameter<u16>, IntegerParameter<i16>, IntegerParameter<u32>, IntegerParameter<i32>, FloatParameter, TextSelectionParameter, FolderParameter, InfoParameter, CommandParameter>;

constexpr ParameterValue GetParameterValue(std::span<u8> data, ParameterType type) {
    switch (type) {
        case ParameterType::Uint8:
            return IntegerParameter<u8>(data);
        case ParameterType::Int8:
            return IntegerParameter<i8>(data);
        case ParameterType::Uint16:
            return IntegerParameter<u16>(data);
        case ParameterType::Int16:
            return IntegerParameter<i16>(data);
        case ParameterType::Uint32:
            return IntegerParameter<u32>(data);
        case ParameterType::Int32:
            return IntegerParameter<i32>(data);
        case ParameterType::Float:
            return FloatParameter(data);
        case ParameterType::TextSelection:
            return TextSelectionParameter(data);
        case ParameterType::Folder:
            return FolderParameter(data);
        case ParameterType::Info:
            return InfoParameter(data);
        case ParameterType::Command:
            return CommandParameter(data);
        default:
            throw Exception("Unknown parameter type: {}", static_cast<int>(type));
    }
}

/**
 * @brief An abstraction over a CRSF parameter, processing every chunk of data until the parameter is complete.
 */
struct Parameter {
    u8 fieldIndex{};
    u8 parentIndex{};
    u8 chunkIndex{};
    i16 lastChunksRemaining{-1};
    bool complete{};
    bool requestedChunk{};
    bool activeCommand{};
    std::chrono::time_point<std::chrono::steady_clock> requestTime{};
    std::chrono::time_point<std::chrono::steady_clock> updateTime{};

    std::string name{};
    ParameterType type{};
    std::vector<u8> rawData;
    ParameterValue value{};

#pragma pack(push, 1)
    struct FirstChunkData {
        u8 parentIndex;
        ParameterType type;
    };
    static_assert(sizeof(FirstChunkData) == 2);
#pragma pack(pop)

    constexpr Parameter(u8 fieldIndex)
        : fieldIndex{fieldIndex} {}

    constexpr void ProcessChunk(std::span<u8> chunk, u8 chunksRemaining) {
        if (lastChunksRemaining > 0 && chunksRemaining != lastChunksRemaining - 1) {
            fmt::print("Chunk order mismatch: expected {}, got {}\n", lastChunksRemaining - 1, chunksRemaining);
            return;
        }
        lastChunksRemaining = chunksRemaining;

        rawData.insert(rawData.end(), chunk.begin(), chunk.end());
        chunkIndex++;
        requestedChunk = false;

        if (chunksRemaining == 0) {
            auto ptr{rawData.data()};

            FirstChunkData* firstChunkData{reinterpret_cast<FirstChunkData*>(ptr)};
            parentIndex = firstChunkData->parentIndex;
            type = firstChunkData->type;
            ptr += sizeof(FirstChunkData);

            name = std::string(reinterpret_cast<char*>(ptr));
            ptr += name.size() + 1;
            value = GetParameterValue(std::span<u8>(ptr, rawData.size() - (ptr - rawData.data())), type);

            complete = true;
            rawData.clear();
            updateTime = std::chrono::steady_clock::now();

            if (activeCommand) {
                CommandParameter& command{std::get<CommandParameter>(value)};
                if (command.step == CommandStep::Idle)
                    activeCommand = false;
            }
        }
    }

    constexpr void Reset() {
        chunkIndex = 0;
        lastChunksRemaining = -1;
        requestedChunk = false;
        rawData.clear();
        complete = false;
        if (!activeCommand)
            value = std::monostate{};
    }
};
}