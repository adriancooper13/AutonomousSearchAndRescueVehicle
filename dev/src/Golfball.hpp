#include <cmath>
#include <string>

class Golfball
{
    public:
        double X;
        double Y;
        std::string NAME;

        Golfball(double x, double y, std::string name)
        {
            X = x;
            Y = y;
            NAME = std::string(name);
        }

        double distance(double x, double y)
        {
            return sqrt(pow(X - x, 2) + pow(Y - y, 2));
        }
};
