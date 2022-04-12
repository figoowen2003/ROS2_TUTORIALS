#include <cmath>

// #include "polygon_plugins/polygon_plugins.hpp"

#include <polygon_base/regular_polygon.hpp>

namespace polygon_plugins
{
    class Square : public polygon_base::RegularPolygon {
    public:
        void initialize(double sideLen) override {
            sideLength_ = sideLen;
        }

        double area() override {
            return sideLength_ * sideLength_;
        }

    private:
        double sideLength_;
    };

    class Triangle : public polygon_base::RegularPolygon {
    public:
        void initialize(double sideLen) override {
            sideLength_ = sideLen;
        }

        double area() override {
            return 0.5 * sideLength_ * getHeight();
        }

        double getHeight() {
            return sqrt((sideLength_ * sideLength_) - ((sideLength_ / 2) * (sideLength_ / 2)));
        }
    private:
        double sideLength_;
    };
}  // namespace polygon_plugins

#include<pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)