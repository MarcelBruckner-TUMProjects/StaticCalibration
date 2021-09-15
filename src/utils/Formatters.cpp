//
// Created by brucknem on 25.05.21.
//
#include "StaticCalibration/utils/Formatters.hpp"

#include <iomanip>
#include <StaticCalibration/utils/RenderUtils.hpp>

namespace static_calibration {
    namespace utils {

        std::string toYAML(const calibration::CameraPoseEstimationBase &estimator) {
            YAML::Emitter out;
            out << YAML::BeginMap;

            out << YAML::Key << "translation";
            out << YAML::Comment("east, north, height");
            out << YAML::Value << YAML::BeginSeq;
            auto translation = estimator.getTranslation();
            out << YAML::Value << translation.x() << translation.y() << translation.z();
            out << YAML::EndSeq;

            out << YAML::Key << "rotation";
            out << YAML::Comment("west-east axis, south-north axis, zero-height axis");
            out << YAML::Value << YAML::BeginSeq;
            auto rotation = estimator.getRotation();
            out << YAML::Value << rotation.x() << rotation.y() << rotation.z();
            out << YAML::EndSeq;

            out << YAML::Key << "intrinsics";
            out << YAML::Comment("f_x, ratio, c_x, c_y, skew");
            out << YAML::Value << estimator.getIntrinsics();

            out << YAML::EndMap;
            return out.c_str();
        }

        template<typename T>
        void pushArg(tinyxml2::XMLPrinter &printer, const std::string &name, T value) {
            printer.OpenElement("arg");
            printer.PushAttribute("name", name.c_str());
            printer.PushAttribute("value", value);
            printer.CloseElement();
        }

        std::string
        toROStf2Node(const calibration::CameraPoseEstimationBase &estimator, const std::string &measurementPoint,
                     const std::string &cameraName) {
            tinyxml2::XMLPrinter printer;
            printer.OpenElement("launch");

            printer.PushComment("Author: Marcel Bruckner");
            printer.PushComment("Automatically generated - DO NOT EDIT!");

            std::string parent = "map";
            std::string child = measurementPoint + "_" + cameraName + "_base";
            printer.PushComment("Frames");
            pushArg(printer, "frame_mp_base", parent.c_str());
            pushArg(printer, "frame_mp_cam", child.c_str());

            printer.PushComment("Translations");
            auto translation = static_calibration::utils::translationToROStf2(estimator.getTranslation());
            pushArg(printer, "translation_x", translation.x());
            pushArg(printer, "translation_y", translation.y());
            pushArg(printer, "translation_z", translation.z());

            printer.PushComment("Rotations");
            auto rotation = static_calibration::utils::rotationToROStf2(estimator.getRotation());
            pushArg(printer, "yaw", rotation.x());
            pushArg(printer, "pitch", rotation.y());
            pushArg(printer, "roll", rotation.z());
            pushArg(printer, "yaw_rad", "$(eval yaw * pi / 180.)");
            pushArg(printer, "pitch_rad", "$(eval pitch * pi / 180.)");
            pushArg(printer, "roll_rad", "$(eval roll * pi / 180.)");

            printer.PushComment("Transformation Node");
            printer.OpenElement("node");
            printer.PushAttribute("pkg", "tf2_ros");
            printer.PushAttribute("type", "static_transform_publisher");
            printer.PushAttribute("name", (parent + "2" + child).c_str());

            std::stringstream args;
            args << "$(arg translation_x) " << "$(arg translation_y) " << "$(arg translation_z) ";
            args << "$(arg yaw_rad) " << "$(arg pitch_rad) " << "$(arg roll_rad) ";
            args << "$(arg frame_mp_base)" << " " << "$(arg frame_mp_cam)";

            printer.PushAttribute("args", args.str().c_str());
            printer.CloseElement();

            printer.CloseElement();
            return printer.CStr();
        }

        std::string
        toROSParamsIntrinsics(const calibration::CameraPoseEstimationBase &estimator,
                              const std::string &measurementPoint,
                              const std::string &cameraName) {
            auto intrinsics = estimator.getIntrinsics();

            std::stringstream ss;
            ss << std::fixed << std::setprecision(12);
            ss << intrinsics[0] << ", " << 0 << ", " << intrinsics[2] << ", " << 0 << ", " << intrinsics[1] << ", "
               << intrinsics[3] << ", " << 0 << ", " << 0 << ", " << 1;

            YAML::Emitter out;
            out << YAML::Comment("Author: Marcel Bruckner") << YAML::Newline;
            out << YAML::Comment("Automatically generated - DO NOT EDIT!") << YAML::Newline;
            out << YAML::Comment("For easy copying: " + ss.str()) << YAML::Newline;

            out << YAML::BeginMap;
            out << YAML::Key << "intrinsics/" + measurementPoint + "_" + cameraName;
            out << YAML::BeginSeq;
            out << intrinsics[0] << YAML::Comment("Focal length X [px]") << 0 << intrinsics[2]
                << YAML::Comment("Principal point X [px]")
                << 0 << intrinsics[1] << YAML::Comment("Focal length Y [px]") << intrinsics[3]
                << YAML::Comment("Principal point Y [px]")
                << 0 << 0 << YAML::Comment("Skew") << 1;
            out << YAML::EndSeq;
            out << YAML::EndMap;

            return out.c_str();
        }
    }
}
