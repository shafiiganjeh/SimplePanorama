
#ifndef CONFP_H
#define CONFP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <functional>
#include "_panorama.h"


namespace conf{

    class ConfigParser {

    public:

        explicit ConfigParser(pan::config* cfg) : config_(cfg) {

        registerEntry("Threads", [&]() { return std::to_string(config_->threads); },
                          [&](const std::string& val) { config_->threads = util::stringToInt(val); });
        std::vector<std::string> keyOrder;

        registerEntry("Focal", [&]() { return util::floatToString(config_->focal); },
                          [&](const std::string& val) { config_->focal = util::stringToFloat(val); });

        registerEntry("Init_size", [&]() { return std::to_string(config_->init_size); },
                          [&](const std::string& val) { config_->init_size = util::stringToInt(val); });

        registerEntry("Method", [&]() { return methodToString(config_->blend); },
                          [&](const std::string& val) { config_->blend = stringToMethod(val); });

        registerEntry("Gain_Compensation", [&]() { return config_->gain_compensation ? "true" : "false"; },
                          [&](const std::string& val) { config_->gain_compensation = (val == "true"); });

        registerEntry("Blend_Intensity", [&]() { return config_->blend_intensity ? "true" : "false"; },
                        [&](const std::string& val) { config_->blend_intensity = (val == "true"); });

        registerEntry("Cut", [&]() { return config_->cut ? "true" : "false"; },
                          [&](const std::string& val) { config_->cut = (val == "true"); });

        registerEntry("Use_Cut", [&]() { return config_->cut_seams ? "true" : "false"; },
                          [&](const std::string& val) { config_->cut_seams = (val == "true"); });

        registerEntry("Bands", [&]() { return std::to_string(config_->bands); },
                          [&](const std::string& val) { config_->bands = util::stringToInt(val); });

        registerEntry("Blend_Sigma", [&]() { return util::doubleToString(config_->sigma_blend); },
                          [&](const std::string& val) { config_->sigma_blend = util::stringToDouble(val); });

        registerEntry("Straighten", [&]() { return config_->straighten ? "true" : "false"; },
                          [&](const std::string& val) { config_->straighten = (val == "true"); });

        registerEntry("Projection", [&]() { return projectionToString(config_->proj); },
                          [&](const std::string& val) { config_->proj = stringToProjection(val); });

        registerEntry("Fix_center", [&]() { return config_->fix_center ? "true" : "false"; },
                          [&](const std::string& val) { config_->fix_center = (val == "true"); });

        registerEntry("Stretch", [&]() { return stretchToString(config_->stretching); },
                          [&](const std::string& val) { config_->stretching = stringToStretch(val); });

        registerEntry("Lambda", [&]() { return util::floatToString(config_->lambda); },
                          [&](const std::string& val) { config_->lambda = util::stringToFloat(val); });

        registerEntry("Adjustment", [&]() { return config_->fast ? "true" : "false"; },
                        [&](const std::string& val) { config_->fast = (val == "true"); });

        registerEntry("Max_Images_Per_Match", [&]() { return std::to_string(config_->max_images_per_match); },
                          [&](const std::string& val) { config_->max_images_per_match = util::stringToInt(val); });

        registerEntry("Max_Keypoints", [&]() { return std::to_string(config_->max_keypoints); },
                          [&](const std::string& val) { config_->max_keypoints = util::stringToInt(val); });

        registerEntry("RANSAC_iterations", [&]() { return std::to_string(config_->RANSAC_iterations); },
                          [&](const std::string& val) { config_->RANSAC_iterations = util::stringToInt(val); });

        registerEntry("x_Margin", [&]() { return std::to_string(config_->x_margin); },
                          [&](const std::string& val) { config_->x_margin = util::stringToInt(val); });

        registerEntry("min_overlap", [&]() { return util::floatToString(config_->min_overlap); },
                          [&](const std::string& val) { config_->min_overlap = util::stringToFloat(val); });

        registerEntry("overlap_inl_match", [&]() { return util::floatToString(config_->overlap_inl_match); },
                          [&](const std::string& val) { config_->overlap_inl_match = util::stringToFloat(val); });

        registerEntry("overlap_inl_keyp", [&]() { return util::floatToString(config_->overlap_inl_keyp); },
                          [&](const std::string& val) { config_->overlap_inl_keyp = util::stringToFloat(val); });

        registerEntry("confidence", [&]() { return util::floatToString(config_->conf); },
                          [&](const std::string& val) { config_->conf = util::stringToFloat(val); });

        registerEntry("nfeatures", [&]() { return std::to_string(config_->nfeatures); },
                          [&](const std::string& val) { config_->nfeatures = util::stringToInt(val); });

        registerEntry("nOctaveLayers", [&]() { return std::to_string(config_->nOctaveLayers); },
                          [&](const std::string& val) { config_->nOctaveLayers = util::stringToInt(val); });

        registerEntry("contrastThreshold", [&]() { return util::doubleToString(config_->contrastThreshold); },
                          [&](const std::string& val) { config_->contrastThreshold = util::stringToDouble(val); });

        registerEntry("edgeThreshold", [&]() { return util::doubleToString(config_->edgeThreshold); },
                          [&](const std::string& val) { config_->edgeThreshold = util::stringToDouble(val); });

        registerEntry("sigma_sift", [&]() { return util::doubleToString(config_->sigma_sift); },
                          [&](const std::string& val) { config_->sigma_sift = util::stringToDouble(val); });


        }

        void read_cfg(const std::filesystem::path& filename);

        void write_cfg(const std::filesystem::path& filename);

    private:

        enum pan::Blending stringToMethod(const std::string& str);
        std::string methodToString(pan::Blending method);

        enum pan::Projection stringToProjection(const std::string& str);
        std::string projectionToString(pan::Projection method);

        enum pan::Stretch stringToStretch(const std::string& str);
        std::string stretchToString(pan::Stretch method);

        std::map<std::string, std::pair<std::function<std::string()>,std::function<void(const std::string&)>>> entries;
        std::vector<std::string> entries_order;

        void registerEntry(const std::string& name,std::function<std::string()> read,std::function<void(const std::string&)> write) {
            entries[name] = {read, write};
            entries_order.push_back(name);
        }

        pan::config* config_;

    };



}
#endif

