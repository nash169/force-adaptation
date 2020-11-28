#ifndef FORCEADAPTATION_PARTICLE_HPP
#define FORCEADAPTATION_PARTICLE_HPP

#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <integrator_lib/Integrate.hpp>

namespace force_adaptation {
    using namespace integrator_lib;

    class Particle {
    public:
        Particle(const double mass = 1) : _mass(mass) 
        {
            _integrator.setStep(0.001);
            _state = Eigen::VectorXd::Zero(6);
            _input = Eigen::VectorXd::Zero(3);
        }

        ~Particle() {}

        void update(const double& time)
        {
            Eigen::VectorXd pos(3), vel(3), test(6);
            pos = _state.segment(0,3);
            vel = _state.segment(2,3);

            test.tail(3) = _integrator.integrate(*this, &Particle::dynamics, time, vel, _input);
            // test.tail(3) = static_cast_ptr<integrator::ForwardEuler>(_integrator)->integrate(*this, &Particle::dynamics, time, vel, _input);

            // auto identity = [](const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
            // {
            //     return x;
            // };

            // _state.segment(0,3) = static_cast_ptr<integrator::ForwardEuler>(_integrator)->integrate(identity, time, pos, _input);
        }

        Eigen::VectorXd state()
        {
            return _state;
        }

        Eigen::VectorXd input()
        {
            return _input;
        }

        Particle& setMass(const double& mass)
        {
            _mass = mass;

            return *(this);
        }

        Particle& setState(const Eigen::VectorXd& state)
        {
            _state = state;

            return *this;
        }

        Particle& setInput(const Eigen::VectorXd& input)
        {
            _input = input;

            return *this;
        }
        
        template<typename Integrator>
        Particle& setIntegrator(std::unique_ptr<Integrator>&& integrator)
        {
            _integrator = std::move(integrator);

            return *this;
        }

        Eigen::VectorXd dynamics(const double& t, const Eigen::VectorXd& x, const Eigen::VectorXd& u)
        {
            return u  / _mass;
        }

        // Eigen::VectorXd surfaceForce(const Eigen::VectorXd& x)
        // {
        //     Eigen::VectorXd surfaceForce = Eigen::VectorXd::Zero(3);

        //     surface_force(2) = (x(1) * sin(x(0)) - x(0) * cos(x(1))) * exp(-x(2));

        //     return surface_force;
        // }

        Eigen::VectorXd surfaceForce(const Eigen::MatrixXd& x)
        {
            Eigen::VectorXd surface_force(x.rows()), negative_altitude(x.rows());
            negative_altitude = -x.col(2);

            surface_force = (x.col(1).array() * x.col(0).array().sin() - x.col(0).array() * x.col(1).array().cos()) * negative_altitude.array().exp();

            return surface_force;
        }

    protected:
        double _mass;

        Eigen::VectorXd _state, _input;

        // std::unique_ptr<AbstractIntegrator> _integrator;

        integrator::ForwardEuler _integrator;

        template<typename D, typename B>
        std::unique_ptr<D> static_cast_ptr(std::unique_ptr<B>& base)
        {
            return std::unique_ptr<D>(static_cast<D*>(base.release()));
        }
        
        template<typename D, typename B>
        std::unique_ptr<D> static_cast_ptr(std::unique_ptr<B>&& base)
        {
            return std::unique_ptr<D>(static_cast<D*>(base.release()));
        }
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_PARTICLE_HPP