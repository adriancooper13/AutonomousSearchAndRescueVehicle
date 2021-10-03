#include <cmath>
#include <string>

class Golfball
{
    public:
        const double X;
        const double Y;
        const std::string NAME;

        Golfball(double x, double y, std::string name)
        : X(x), Y(y), NAME(name)
        {
        }

        double distance(double x, double y)
        {
            return sqrt(pow(X - x, 2) + pow(Y - y, 2));
        }
};