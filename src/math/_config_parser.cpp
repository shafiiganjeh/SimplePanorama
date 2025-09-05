
#include "_config_parser.h"

namespace conf{

    enum pan::Blending ConfigParser::stringToMethod(const std::string& str) {
        if (str == "NO_BLEND") return pan::NO_BLEND;
        if (str == "SIMPLE_BLEND") return pan::SIMPLE_BLEND;
        if (str == "MULTI_BLEND") return pan::MULTI_BLEND;
        return pan::SIMPLE_BLEND;
    }


    std::string ConfigParser::methodToString(pan::Blending method) {

        switch(method) {
            case pan::NO_BLEND: return "NO_BLEND";
            case pan::SIMPLE_BLEND: return "SIMPLE_BLEND";
            case pan::MULTI_BLEND: return "MULTI_BLEND";
            default: return "SIMPLE_BLEND";
        }

    }


    void ConfigParser::write_cfg(const std::string& filename) {
        std::ofstream file(filename);
        for (const auto& key : entries_order) {
            file << "\n"<< key << "=" << entries[key].first() << "\n";
        }
    }


    void ConfigParser::read_cfg(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            auto delimiterPos = line.find('=');
            if (delimiterPos == std::string::npos) continue;

            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 1);

            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            if (entries.find(key) != entries.end()) {
                entries[key].second(value);

            }


        }

    }


}


