// Dependencies
#include <kernel_lib/Kernel.hpp>
#include <utils_cpp/UtilsCpp.hpp>

#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/null_function.hpp>
#include <limbo/model/gp.hpp>

using namespace kernel_lib;
using namespace limbo;

struct KernelParams {
    struct kernel : public kernel_lib::defaults::kernel {
        PARAM_SCALAR(double, sigma_n, 0.003948873059426);
        PARAM_SCALAR(double, sigma_f, 0.484798403620202);
    };

    struct rbf : public kernel_lib::defaults::rbf {
        PARAM_SCALAR(Covariance, type, CovarianceType::DIAGONAL);
        PARAM_VECTOR(double, sigma, 0.052528133311725, 0.031951602184414, 64.962285768917100);
    };

    struct expansion : public kernel_lib::defaults::expansion {
    };
};

struct ExpansionParams {
    struct kernel : public kernel_lib::defaults::kernel {
        PARAM_SCALAR(double, sigma_f, 0.484798403620202);
    };

    struct rbf : public kernel_lib::defaults::rbf {
        PARAM_SCALAR(Covariance, type, CovarianceType::DIAGONAL);
        PARAM_VECTOR(double, sigma, 0.052528133311725, 0.031951602184414, 64.962285768917100);
    };

    struct expansion : public kernel_lib::defaults::expansion {
    };
};

struct LimboParams {
    struct kernel : public limbo::defaults::kernel {
        BO_PARAM(double, noise, pow(0.003948873059426, 2));
    };
    struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public limbo::defaults::opt_rprop {
    };
};

int main(int argc, char const* argv[])
{
    // My lib
    kernels::Rbf<KernelParams> k;
    utils::Expansion<KernelParams, kernels::Rbf<KernelParams>> psi;

    // Limbo
    using Kernel_t = kernel::SquaredExpARD<LimboParams>;
    using Mean_t = mean::NullFunction<LimboParams>;
    using GP_t = model::GP<LimboParams, Kernel_t, Mean_t>;
    GP_t gp(3, 1);

    Eigen::MatrixXd train(100, 3), test(1, 3);
    Eigen::VectorXd target(100);

    train << -0.0396, -0.0514, -0.0047,
        -0.0353, -0.0540, -0.0045,
        -0.0309, -0.0563, -0.0044,
        -0.0263, -0.0582, -0.0042,
        -0.0211, -0.0600, -0.0041,
        -0.0157, -0.0613, -0.0040,
        -0.0103, -0.0621, -0.0040,
        -0.0048, -0.0624, -0.0039,
        0.0007, -0.0623, -0.0039,
        0.0062, -0.0617, -0.0039,
        0.0115, -0.0606, -0.0040,
        0.0168, -0.0591, -0.0041,
        0.0219, -0.0571, -0.0042,
        0.0269, -0.0546, -0.0043,
        0.0311, -0.0520, -0.0044,
        0.0352, -0.0491, -0.0046,
        0.0390, -0.0459, -0.0048,
        0.0425, -0.0423, -0.0050,
        0.0457, -0.0385, -0.0053,
        0.0486, -0.0344, -0.0055,
        0.0511, -0.0301, -0.0058,
        0.0533, -0.0256, -0.0061,
        0.0551, -0.0209, -0.0064,
        0.0565, -0.0161, -0.0067,
        0.0574, -0.0112, -0.0069,
        0.0580, -0.0062, -0.0071,
        0.0581, -0.0012, -0.0073,
        0.0578, 0.0043, -0.0074,
        0.0569, 0.0097, -0.0074,
        0.0556, 0.0150, -0.0073,
        0.0539, 0.0197, -0.0072,
        0.0518, 0.0243, -0.0070,
        0.0494, 0.0286, -0.0067,
        0.0466, 0.0328, -0.0064,
        0.0434, 0.0366, -0.0061,
        0.0399, 0.0402, -0.0058,
        0.0361, 0.0435, -0.0056,
        0.0321, 0.0464, -0.0053,
        0.0278, 0.0489, -0.0051,
        0.0232, 0.0511, -0.0049,
        0.0186, 0.0528, -0.0048,
        0.0133, 0.0542, -0.0046,
        0.0078, 0.0552, -0.0046,
        0.0024, 0.0555, -0.0045,
        -0.0031, 0.0554, -0.0045,
        -0.0086, 0.0547, -0.0045,
        -0.0140, 0.0534, -0.0046,
        -0.0192, 0.0517, -0.0047,
        -0.0242, 0.0494, -0.0048,
        -0.0285, 0.0469, -0.0049,
        -0.0326, 0.0440, -0.0051,
        -0.0364, 0.0408, -0.0053,
        -0.0399, 0.0373, -0.0055,
        -0.0431, 0.0334, -0.0058,
        -0.0459, 0.0293, -0.0061,
        -0.0483, 0.0249, -0.0064,
        -0.0503, 0.0203, -0.0066,
        -0.0519, 0.0156, -0.0069,
        -0.0531, 0.0107, -0.0072,
        -0.0537, 0.0058, -0.0074,
        -0.0540, 0.0008, -0.0076,
        -0.0537, -0.0047, -0.0077,
        -0.0529, -0.0102, -0.0077,
        -0.0515, -0.0155, -0.0076,
        -0.0498, -0.0202, -0.0074,
        -0.0476, -0.0247, -0.0072,
        -0.0451, -0.0290, -0.0069,
        -0.0421, -0.0330, -0.0066,
        -0.0388, -0.0368, -0.0063,
        -0.0352, -0.0402, -0.0060,
        -0.0312, -0.0433, -0.0057,
        -0.0270, -0.0459, -0.0055,
        -0.0226, -0.0482, -0.0052,
        -0.0179, -0.0501, -0.0051,
        -0.0126, -0.0516, -0.0049,
        -0.0072, -0.0526, -0.0048,
        -0.0017, -0.0530, -0.0047,
        0.0038, -0.0529, -0.0047,
        0.0092, -0.0521, -0.0047,
        0.0145, -0.0509, -0.0048,
        0.0197, -0.0490, -0.0049,
        0.0247, -0.0467, -0.0050,
        0.0290, -0.0441, -0.0052,
        0.0330, -0.0411, -0.0054,
        0.0367, -0.0378, -0.0056,
        0.0401, -0.0341, -0.0058,
        0.0431, -0.0302, -0.0061,
        0.0458, -0.0259, -0.0064,
        0.0480, -0.0214, -0.0067,
        0.0498, -0.0168, -0.0069,
        0.0511, -0.0119, -0.0072,
        0.0520, -0.0070, -0.0074,
        0.0524, -0.0020, -0.0076,
        0.0523, 0.0034, -0.0077,
        0.0516, 0.0089, -0.0078,
        0.0504, 0.0143, -0.0077,
        0.0488, 0.0190, -0.0075,
        0.0467, 0.0235, -0.0073,
        0.0443, 0.0279, -0.0070,
        0.0414, 0.0320, -0.0067;

    test << 0, 0, 0;

    target << -0.3844,
        -0.3362,
        -0.2867,
        -0.2374,
        -0.1841,
        -0.1322,
        -0.0815,
        -0.0317,
        0.0176,
        0.0670,
        0.1169,
        0.1678,
        0.2200,
        0.2735,
        0.3230,
        0.3728,
        0.4219,
        0.4687,
        0.5110,
        0.5457,
        0.5691,
        0.5769,
        0.5644,
        0.5272,
        0.4622,
        0.3683,
        0.2478,
        0.0915,
        -0.0771,
        -0.2415,
        -0.3732,
        -0.4771,
        -0.5470,
        -0.5816,
        -0.5838,
        -0.5595,
        -0.5159,
        -0.4598,
        -0.3968,
        -0.3310,
        -0.2649,
        -0.1934,
        -0.1239,
        -0.0562,
        0.0101,
        0.0756,
        0.1407,
        0.2060,
        0.2713,
        0.3303,
        0.3882,
        0.4433,
        0.4935,
        0.5357,
        0.5657,
        0.5789,
        0.5699,
        0.5339,
        0.4673,
        0.3686,
        0.2405,
        0.0735,
        -0.1062,
        -0.2798,
        -0.4165,
        -0.5212,
        -0.5876,
        -0.6150,
        -0.6076,
        -0.5728,
        -0.5187,
        -0.4528,
        -0.3810,
        -0.3074,
        -0.2269,
        -0.1485,
        -0.0724,
        0.0018,
        0.0745,
        0.1464,
        0.2176,
        0.2881,
        0.3509,
        0.4112,
        0.4670,
        0.5155,
        0.5528,
        0.5741,
        0.5743,
        0.5479,
        0.4905,
        0.4000,
        0.2777,
        0.1132,
        -0.0689,
        -0.2497,
        -0.3957,
        -0.5111,
        -0.5876,
        -0.6234;

    std::vector<Eigen::VectorXd> train_limbo;
    for (size_t i = 0; i < train.rows(); i++)
        train_limbo.push_back(train.row(i));

    std::vector<Eigen::VectorXd> target_limbo;
    for (size_t i = 0; i < target.rows(); i++)
        target_limbo.push_back(target.row(i));

    Eigen::VectorXd test_limbo(3);
    test_limbo = test.row(0);

    Eigen::VectorXd weights = k(train, train).colPivHouseholderQr().solve(target);

    psi.setReference(train).setWeights(weights);

    Eigen::VectorXd params(4);
    params << log(0.052528133311725), log(0.031951602184414), log(64.962285768917100), log(0.484798403620202);

    gp.kernel_function().set_params(params);
    gp.compute(train_limbo, target_limbo);

    std::cout << psi(test) << std::endl;
    std::cout << gp.mu(test_limbo) << std::endl;

    return 0;
}
