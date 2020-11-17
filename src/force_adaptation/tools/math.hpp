#ifndef FORCEADAPTATION_TOOLS_MATH_HPP
#define FORCEADAPTATION_TOOLS_MATH_HPP

#include <Eigen/Dense>

namespace force_adaptation {
    namespace tools {
        Eigen::MatrixXd c_reshape(Eigen::MatrixXd M, int num_rows, int num_cols);
    } // namespace tools
} // namespace force_adaptation

#endif // FORCEADAPTATION_TOOLS_MATH_HPP