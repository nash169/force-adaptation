#include <Eigen/Core>

namespace force_adaptation {
    class Dynamics
    {
    public:
        Dynamics()
        {

        }

        ~Dynamics()
        {

        }

        void circularMotion()
        {
            
        }

        Eigen::Vector2d circle(Eigen::VectorXd angle, double radius, Eigen::Vector2d center)
        {
            Eigen::MatrixXd x(angle.rows(), 2);

            // angle.col(0)
        }

    private:
        Eigen::Vector3d _velocity, _position;

    };
    
}

