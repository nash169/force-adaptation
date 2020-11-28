#ifndef FORCEADAPTATION_CONTROL_HPP
#define FORCEADAPTATION_CONTROL_HPP

#include <Eigen/Core>
#include <vector>

namespace force_adaptation {
    enum class ControlMode : unsigned int {
        OPERATIONSPACE = 1 << 0,
        CONFIGURATIONSPACE = 1 << 1
    };

    class Control {
    public:
        Control() : _control_mode(ControlMode::CONFIGURATIONSPACE), _control_ref(-1) {}

        Control(ControlMode mode, int ref = -1) : _control_mode(mode), _control_ref(ref) {}

        virtual ~Control() {}

        virtual void init() = 0;

        void setControlMode(ControlMode mode, int ref = -1)
        {
            _control_mode = mode;
        }

        ControlMode getControlMode() const { return _control_mode; }

        int getControlRef() const { return _control_ref; }

        template <typename... Args>
        Eigen::VectorXd control(const Args&... args)
        {
            size_t vec_size = 0;
            std::vector<Eigen::VectorXd> arguments = {args...};

            for (auto& arg : arguments)
                vec_size += arg.size();

            Eigen::VectorXd state(vec_size);

            int curr_index = 0;
            for (auto& arg : arguments) {
                state.segment(curr_index, arg.size()) = arg;
                curr_index += arg.size();
            }

            return update(state);
        }

        virtual Eigen::VectorXd update(const Eigen::VectorXd& state) = 0;

    protected:
        // Control mode
        ControlMode _control_mode;

        // Operational space reference
        int _control_ref;
    };

} // namespace force_adaptation

#endif // FORCEADAPTATION_CONTROL_HPP