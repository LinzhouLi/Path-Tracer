#include <pt/common.h>


namespace pt {

std::string indent(const std::string& string, int amount) {
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

std::string toLower(const std::string& value) {
    std::string result;
    result.resize(value.size());
    std::transform(value.begin(), value.end(), result.begin(), ::tolower);
    return result;
}

float toFloat(const std::string& str) {
    char* end_ptr = nullptr;
    float result = (float)strtof(str.c_str(), &end_ptr);
    if (*end_ptr != '\0')
        throw PathTracerException("Could not parse floating point value \"%s\"", str);
    return result;
}

bool endsWith(const std::string& value, const std::string& ending) {
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

std::string getFolderPath(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    std::string baseDir = pos == std::string::npos ? "" : path.substr(0, pos);
    if (!baseDir.empty()) {
        if (baseDir[baseDir.length() - 1] != DIR_SEP) baseDir += DIR_SEP;
    }
    return baseDir;
}

};