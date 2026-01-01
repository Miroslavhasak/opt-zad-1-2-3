#pragma once
#include <string>
#include <sstream>

struct PoseCommand {
    enum class Type { MOVE, LINEAR } type;
    float x, y, z;
    float roll, pitch, yaw;
    float velocity; // mandatory
};

enum class ParseStatus {
    OK,
    EMPTY_LINE,
    UNKNOWN_COMMAND,
    MISSING_POSITION,
    INVALID_POSITION,
    MISSING_ORIENTATION,
    INVALID_ORIENTATION,
    MISSING_VELOCITY,
    INVALID_VELOCITY,
    TOO_MANY_TOKENS
};

class CommandParser {
public:
    static ParseStatus parse(const std::string& line, PoseCommand& cmd) {
        if (line.empty()) return ParseStatus::EMPTY_LINE;

        std::istringstream ss(line);
        std::string token;

        // 1. Command type
        if (!(ss >> token)) return ParseStatus::UNKNOWN_COMMAND;
        if (token == "MOVE") cmd.type = PoseCommand::Type::MOVE;
        else if (token == "LINEAR") cmd.type = PoseCommand::Type::LINEAR;
        else return ParseStatus::UNKNOWN_COMMAND;

        // 2. Position
        if (!(ss >> token)) return ParseStatus::MISSING_POSITION;
        if (!parseTriple(token, cmd.x, cmd.y, cmd.z, 'P')) return ParseStatus::INVALID_POSITION;

        // 3. Orientation
        if (!(ss >> token)) return ParseStatus::MISSING_ORIENTATION;
        if (!parseTriple(token, cmd.roll, cmd.pitch, cmd.yaw, 'R', "RPY")) return ParseStatus::INVALID_ORIENTATION;

        // 4. Velocity (mandatory)
        if (!(ss >> token)) return ParseStatus::MISSING_VELOCITY;
        if (token.size() < 3 || token.substr(0,2) != "V=") return ParseStatus::INVALID_VELOCITY;
        cmd.velocity = strToFloat(token.substr(2));

        // 5. Check for extra tokens
        if (ss >> token) return ParseStatus::TOO_MANY_TOKENS;

        return ParseStatus::OK;
    }

private:
    // Parse P(x,y,z) or RPY(x,y,z)
    static bool parseTriple(const std::string& token, float &a, float &b, float &c,
                            char type, const std::string& prefix="P") {
        size_t start = token.find('(');
        size_t end = token.find(')');
        if (start == std::string::npos || end == std::string::npos || end <= start)
            return false;

        std::string numbers = token.substr(start+1, end-start-1);
        std::istringstream ss(numbers);
        std::string item;

        if (!std::getline(ss, item, ',')) return false;
        a = strToFloat(item);
        if (!std::getline(ss, item, ',')) return false;
        b = strToFloat(item);
        if (!std::getline(ss, item, ',')) return false;
        c = strToFloat(item);

        return true;
    }

    static float strToFloat(const std::string& s) {
        std::istringstream ss(s);
        float val = 0;
        ss >> val;
        return val;
    }
};

// ------------------- Usage Example -------------------
//
// LINEAR P(0.0,0.1,0.15) RPY(3.14,0,0) V=0.05