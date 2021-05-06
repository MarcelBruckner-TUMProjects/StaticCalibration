//
// Created by brucknem on 18.01.21.
//

#include "CSVWriter.hpp"

#include <utility>
#include <iomanip>

namespace static_calibration {
    namespace evaluation {

        template<typename T>
        CSVWriter &CSVWriter::operator<<(const T &val) {
            std::stringstream ss;
            ss << std::fixed;
            ss << std::setprecision(20);
            ss << val;
            fs_ << ss.str() << separator_;
            return *this;
        }

        CSVWriter::CSVWriter(const std::string &filename, bool append, std::string separator)
                : fs_(), separator_(std::move(separator)) {
            fs_.exceptions(std::ios::failbit | std::ios::badbit);
            if (append) {
                fs_.open(filename, std::ofstream::app);
            } else {
                fs_.open(filename);
            }
        }

        CSVWriter::CSVWriter(const std::string &filename, const std::string &separator) :
                CSVWriter(filename, false, separator) {}

        CSVWriter::~CSVWriter() {
            flush();
            fs_.close();
        }

        void CSVWriter::flush() {
            fs_.flush();
        }

        void CSVWriter::newline() {
            fs_ << std::endl;
            flush();
        }

        CSVWriter &CSVWriter::operator<<(CSVWriter &(*val)(CSVWriter &)) {
            return val(*this);
        }

        CSVWriter &CSVWriter::operator<<(const char *val) {
            fs_ << '"' << val << '"' << separator_;
            return *this;
        }

        CSVWriter &CSVWriter::operator<<(const std::string &val) {
            fs_ << '"' << val << '"' << separator_;
            return *this;
        }

#ifdef WITH_OPENCV
        CSVWriter &CSVWriter::operator<<(const cv::Rect &val) {
            *this << val.x << val.y << val.width << val.height;
            return *this;
        }

        CSVWriter &CSVWriter::operator<<(const cv::Point2d &val) {
            *this << val.x << val.y;
            return *this;
        }
#endif //WITH_OPENCV

        CSVWriter &CSVWriter::operator<<(const std::vector<double> &val) {
            for (const auto &v : val) {
                *this << v;
            }
            return *this;
        }

        CSVWriter::CSVWriter(const boost::filesystem::path &filename, const std::string &separator) :
                CSVWriter(filename.string(), separator) {}

        CSVWriter::CSVWriter(const boost::filesystem::path &filename, bool append, std::string separator) :
                CSVWriter(filename.string(), append, separator) {}

        CSVWriter &CSVWriter::operator<<(const Eigen::Vector3d &val) {
            *this << val.x() << val.y() << val.z();
            return *this;
        }

        template CSVWriter &CSVWriter::operator<<(const int &);

        template CSVWriter &CSVWriter::operator<<(const double &);

        template CSVWriter &CSVWriter::operator<<(const float &);

        template CSVWriter &CSVWriter::operator<<(const bool &);

        template CSVWriter &CSVWriter::operator<<(const long &);

        CSVWriter &newline(CSVWriter &file) {
            file.newline();
            return file;
        }

        CSVWriter &flush(CSVWriter &file) {
            file.flush();
            return file;
        }
    }
}
