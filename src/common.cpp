/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <iomanip>
#include <pt/common.h>

namespace pt {

std::string indent(const std::string& string, int amount) {
    /* This could probably be done faster (it's not
        really speed-critical though) */
    std::istringstream iss(string);
    std::ostringstream oss;
    std::string spacer(amount, ' ');
    bool firstLine = true;
    for (std::string line; std::getline(iss, line); ) {
        if (!firstLine)
            oss << spacer;
        oss << line;
        if (!iss.eof())
            oss << endl;
        firstLine = false;
    }
    return oss.str();
}

bool endsWith(const std::string& value, const std::string& ending) {
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

std::string toLower(const std::string& value) {
    std::string result;
    result.resize(value.size());
    std::transform(value.begin(), value.end(), result.begin(), ::tolower);
    return result;
}

bool toBool(const std::string& str) {
    std::string value = toLower(str);
    if (value == "false")
        return false;
    else if (value == "true")
        return true;
    else
        throw PathTracerException("Could not parse boolean value \"%s\"", str);
}

int toInt(const std::string& str) {
    char* end_ptr = nullptr;
    int result = (int)strtol(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw PathTracerException("Could not parse integer value \"%s\"", str);
    return result;
}

unsigned int toUInt(const std::string& str) {
    char* end_ptr = nullptr;
    unsigned int result = (int)strtoul(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw PathTracerException("Could not parse integer value \"%s\"", str);
    return result;
}

float toFloat(const std::string& str) {
    char* end_ptr = nullptr;
    float result = (float)strtof(str.c_str(), &end_ptr);
    if (*end_ptr != '\0')
        throw PathTracerException("Could not parse floating point value \"%s\"", str);
    return result;
}

Eigen::Vector3f toVector3f(const std::string& str) {
    std::vector<std::string> tokens = tokenize(str);
    if (tokens.size() != 3)
        throw PathTracerException("Expected 3 values");
    Eigen::Vector3f result;
    for (int i = 0; i < 3; ++i)
        result[i] = toFloat(tokens[i]);
    return result;
}

std::vector<std::string> tokenize(const std::string& string, const std::string& delim, bool includeEmpty) {
    std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
    std::vector<std::string> tokens;

    while (lastPos != std::string::npos) {
        if (pos != lastPos || includeEmpty)
            tokens.push_back(string.substr(lastPos, pos - lastPos));
        lastPos = pos;
        if (lastPos != std::string::npos) {
            lastPos += 1;
            pos = string.find_first_of(delim, lastPos);
        }
    }

    return tokens;
}

}