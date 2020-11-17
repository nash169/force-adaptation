#include "force_adaptation/tools/math.hpp"
#include <Eigen/Dense> // use Eigen/Dense for now, then it'd be better to specialize headers

namespace force_adaptation {
    namespace tools {
        Eigen::MatrixXd c_reshape(Eigen::MatrixXd M, int num_rows, int num_cols)
        {
            M.transposeInPlace();

            Eigen::Map<Eigen::MatrixXd> S(M.data(), num_cols, num_rows);

            return S.transpose();
        }
    } // namespace tools
} // namespace force_adaptation