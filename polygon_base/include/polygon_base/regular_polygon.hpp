#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
    class RegularPolygon
    {
    public:
        // 因为pluginlib所加载的类的构造函数不可以带参数，因此需要这个函数来完成参数的初始化
        virtual void initialize(double side_length) = 0;
        virtual double area() = 0;
        virtual ~RegularPolygon(){}

    protected:
        RegularPolygon(){}  // 表示不允许基类直接实例化，需要借助与子类完成实例化
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP