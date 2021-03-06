// #define PLOTTING_ENABLED
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <ros/ros.h>

// #include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


double upper_len = 0.38;
double lower_len = 0.3612;
constexpr double pi = 3.141592654;

void forward_solution(Eigen::Matrix<double, 2, 1> & foot_end, const Eigen::Matrix<double, 2, 1> & angular)
{
    foot_end(0) = upper_len * cos(angular(0)) + lower_len * cos(angular(1) + angular(0));
    foot_end(1) = upper_len * sin(angular(0)) + lower_len * sin(angular(1) + angular(0));
}


void inverse_solution(Eigen::Matrix<double, 2, 1> & x_des, const Eigen::Matrix<double, 2, 1> & foot_end)
{
    double x = foot_end(0);
    double y = foot_end(1);
    double alpha = atan2(y, x);
    double rho = sqrt(x*x + y *y);
    double phi = acos((upper_len*upper_len + rho*rho - lower_len * lower_len)/2/upper_len/rho);
    x_des(0) = alpha - phi;
    double beta = asin(upper_len* sin(phi) / lower_len);
    x_des(1) = phi + beta;
    return;
}

double sensor[4];
void joint_states_callback(const sensor_msgs::JointStateConstPtr & joint_states_msg)
{
    sensor[0] = joint_states_msg->position[8];
    sensor[1] = joint_states_msg->position[7];
    sensor[2] = joint_states_msg->velocity[8];
    sensor[3] = joint_states_msg->velocity[7];
}


template<typename SCALAR>
struct InertialPara{
    SCALAR g;
    SCALAR m1;
    SCALAR m2;
    SCALAR Izz1;
    SCALAR Izz2;
    SCALAR l1;
    SCALAR l2;
    Eigen::Matrix<SCALAR, 3, 1> com1;
    Eigen::Matrix<SCALAR, 3, 1> com2;
};

std::fstream file;


namespace ct {
namespace core {

namespace tpl {

template <typename SCALAR>
class TestNonlinearSystem : public ControlledSystem<2, 1, SCALAR>
{
public:
    static const size_t STATE_DIM = 2;
    static const size_t CONTROL_DIM = 1;

    typedef ControlVector<1, SCALAR> control_vector_t;
    typedef StateVector<2, SCALAR> state_vector_t;

    TestNonlinearSystem() = delete;

    // constructor directly using frequency and damping coefficients
    TestNonlinearSystem(SCALAR w_n, std::shared_ptr<Controller<2, 1, SCALAR>> controller = nullptr)
            : ControlledSystem<2, 1, SCALAR>(controller, SYSTEM_TYPE::GENERAL), w_n_(w_n)
    {
    }

    TestNonlinearSystem(const TestNonlinearSystem& arg) : ControlledSystem<2, 1, SCALAR>(arg), w_n_(arg.w_n_) {}
    virtual ~TestNonlinearSystem() {}
    TestNonlinearSystem* clone() const override { return new TestNonlinearSystem(*this); }
    virtual void computeControlledDynamics(const StateVector<2, SCALAR>& state,
                                           const SCALAR& t,
                                           const control_vector_t& control,
                                           state_vector_t& derivative) override
    {
        //this is pretty much random
        derivative(0) = state(1) * state(0) + state(1) * control(0);
        derivative(1) = w_n_ * control(0) - 2.0 * w_n_ * state(1) - 3.0 * w_n_ * state(1) * control(0);
    }

private:
    SCALAR w_n_;
};



template <typename SCALAR>
class Singleleg : public ControlledSystem<4, 2, SCALAR>
{
public:
    static const size_t STATE_DIM = 4;
    static const size_t CONTROL_DIM = 2;
    typedef ControlVector<CONTROL_DIM, SCALAR> u_type;
    typedef StateVector<STATE_DIM, SCALAR> x_type;
    typedef ct::core::Time t_type;
    typedef std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controllerPtr_t;
    typedef InertialPara<SCALAR> *  inertialPtr_t;

    /* Constructor and distructor start */
    Singleleg() = delete;



    Singleleg(const Singleleg& arg)
        :   ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
            g(arg.g),
            m1(arg.m1),
            m2(arg.m2),
            Izz1(arg.Izz1),
            Izz2(arg.Izz2),
            l1(arg.l1),
            l2(arg.l2),
            com1(arg.com1),
            com2(arg.com2)
    {

    }
    virtual ~Singleleg() {}
    Singleleg* clone() const override { return new Singleleg(*this); }
    /* Constructor and distructor end */

    // constructor directly using frequency and damping coefficients
    Singleleg(inertialPtr_t ip, controllerPtr_t controller = nullptr)
        : ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>(controller, SYSTEM_TYPE::GENERAL)
    {
        if(ip == nullptr)
        {
            throw std::runtime_error("no inertial parameters!");
        }
        g = ip->g;
        m1 = ip->m1;
        m2 = ip->m2;
        Izz1 = ip->Izz1;
        Izz2 = ip->Izz2;
        l1= ip->l1;
        l2= ip->l2;
        com1 = ip->com1;
        com2 = ip->com2;
    }

    virtual void computeControlledDynamics(
            const x_type & x, const t_type & t, const u_type & u, x_type & x_dot) override
    {
        x_dot(0) = x(2);
        x_dot(1) = x(3);
        M(0, 0) = Izz1
                  + Izz2
                  + com1(0) * com1(0) * m1
                  + com2(0) * com2(0) * m2
                  + com1(1) * com1(1) * m1
                  + com2(1) * com2(1) * m2
                  + 2 * l1 * com2(0) * m2 * cos(x(1))
                  - 2 * l1 * com2(1) * m2 * sin(x(1))
                  + l1 * l1 * m2;
        M(0, 1) = Izz2 + com2(0) * com2(0) * m2 + com2(1) * com2(1) * m2 + l1 * com2(1) * m2 * sin(x(1)) + l1 * com2(0) * m2 * cos(x(1));
        M(1, 0) = com2(0) * com2(0) * m2 + Izz2 + com2(1) * com2(1) * m2 - l1 * com2(1) * m2 * sin(x(1)) + l1 * com2(0) * m2 * cos(x(1));
        M(1, 1) = Izz2 + com2(0) * com2(0) * m2 + com2(1) * com2(1) * m2;



        V(0, 0) = - l1 * com2(1) * m2 * x(3) * x(3) * cos(x(1))
                  - l1 * com2(0) * m2*x(3)* x(3) * sin(x(1))
                  - 2 * l1 * com2(1) * m2 * x(2) * x(3) * cos(x(1))
                  - 2 * l1 * com2(0) * m2 * x(2) * x(3) * sin(x(1));

        V(1, 0) = l1*com2(1)*m2*x(2)* x(2)*cos(x(1)) + l1*com2(0)*m2*x(2)* x(2) *sin(x(1));


        G(0, 0) = + g*com1(1)*m1*cos(x(0))
                  + g*com1(0)*m1*sin(x(0))
                  + g*com2(1)*m2*cos(x(0))*cos(x(1))
                  + g*com2(0)*m2*cos(x(0))*sin(x(1))
                  + g*com2(0)*m2*cos(x(1))*sin(x(0))
                  - g*com2(1)*m2*sin(x(0))*sin(x(1))
                  + g*l1*m2*cos(x(1)) * cos(x(1)) * sin(x(0))
                  + g * l1 * m2 * sin(x(0)) * sin(x(1)) * sin(x(1)) ;

        G(1, 0) = + g*com2(1)*m2*cos(x(0))*cos(x(1))
                  + g*com2(0)*m2*cos(x(0))*sin(x(1))
                  + g*com2(0)*m2*cos(x(1))*sin(x(0))
                  - g*com2(1)*m2*sin(x(0))*sin(x(1));

        double det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        M_inv(0, 0) = M(1, 1) / det;
        M_inv(0, 1) = -M(0, 1) / det;

        M_inv(1, 1) = M(0, 0) / det;
        M_inv(1, 0) = -M(1, 0) / det;

        q_dotdot = M_inv * (u - V - G);

        x_dot(2) = q_dotdot(0);
        x_dot(3) = q_dotdot(1);

    }
protected:
    SCALAR g;
    SCALAR m1;
    SCALAR m2;
    SCALAR Izz1;
    SCALAR Izz2;
    SCALAR l1;
    SCALAR l2;

    Eigen::Matrix<SCALAR, 3, 1> com1;
    Eigen::Matrix<SCALAR, 3, 1> com2;

    Eigen::Matrix<SCALAR, 2, 2> M;
    Eigen::Matrix<SCALAR, 2, 2> M_inv;
    u_type V;
    u_type G;
    Eigen::Matrix<SCALAR, 2, 1> q_dotdot;
};


} // namespace tpl

typedef tpl::TestNonlinearSystem<double> TestNonlinearSystem;
typedef tpl::Singleleg<double> Singleleg;
}  // namespace core
}  // namespace ct

ct::core::ControlVector<2> u;
int cnt =0;


// constexpr double pi = 3.141592654;


void mpc_thread()
{
    // mpc initialization
    using Singleleg = ct::core::Singleleg;
    const size_t state_dim = Singleleg::STATE_DIM;
    const size_t control_dim = Singleleg::CONTROL_DIM;
    typedef ct::core::StateVector<state_dim> x_type;
    typedef ct::core::ControlVector<control_dim> u_type;
    typedef ct::core::StateVector<2> q_type;
    typedef ct::core::Time t_type;
    
    typedef ct::optcon::TermQuadratic<state_dim, control_dim> term_t;
    typedef ct::optcon::OptConProblemBase<state_dim, control_dim,
            ct::core::ControlledSystem<state_dim, control_dim>,
            ct::core::LinearSystem<state_dim, control_dim>,
            ct::core::SystemLinearizer<state_dim, control_dim>,
            double> problem_t;
    typedef ct::optcon::NLOptConSettings settings_t;
    using FeedbackArray = ct::core::FeedbackArray<state_dim, control_dim>;
    using FeedbackMatrix =  ct::core::FeedbackMatrix<state_dim, control_dim>;
    using ControlVector = ct::core::ControlVector<control_dim>;
    typedef ct::core::ControlVectorArray<control_dim> uArrayType;
    using NLOptConSolver = ct::optcon::NLOptConSolver<state_dim, control_dim>;
    using ct::core::ControlVectorArray;
    using ct::core::StateVectorArray;

    std::fstream mpc_data;
    // std::fstream joint_angular_data;
    std::fstream RF_HFE_joint_data;
    RF_HFE_joint_data.open("/home/cda/code/ros_ws/src/mpc_for_single_leg/RF_HFE_joint_data.csv", std::ios::out);
    std::fstream RF_KFE_joint_data;
    RF_KFE_joint_data.open("/home/cda/code/ros_ws/src/mpc_for_single_leg/RF_KFE_joint_data.csv", std::ios::out);        
    mpc_data.open("/home/cda/code/ros_ws/src/mpc_for_single_leg/mpc_data.csv", std::ios::out);
    // mpc_data << "target_footend_x,target_footend_y,target_vel0,target_vel1,actual_pos0,actual_pos1,actual_vel0,actual_vel1,time\n";
    mpc_data << "target_angular1,target_angular2,actual_angular1,actual_angular2,time\n";
    RF_HFE_joint_data << "target_angular,actual_angular,target_velocity,actual_velocity,time\n";
    RF_KFE_joint_data << "target_angular,actual_angular,target_velocity,actual_velocity,time\n";
    std::fstream foot_end_position_data;
    foot_end_position_data << "actual_x,actual_y,target_x,targety,time";
    std::fstream foot_end_velocity_data;
    foot_end_velocity_data << "actual_x,actual_y,target_x,targety,time";
    Eigen::Matrix<double, 2, 1> actual_footend_pos;
    Eigen::Matrix<double, 2, 1> actual_footend_vel;
    

    Eigen::Matrix<double, 2, 1> target_footend_pos;
    Eigen::Matrix<double, 2, 1> target_footend_vel;


    Eigen::Matrix<double, 2, 1> actual_joint_ang;
    Eigen::Matrix<double, 2, 1> actual_joint_vel;

    Eigen::Matrix<double, 2, 1> target_joint_ang;
    Eigen::Matrix<double, 2, 1> target_joint_vel;


    Eigen::Matrix<double, 2, 2> jacobian;


    InertialPara<double> inertia;
    inertia.g = 9.81;
    inertia.m1 = 1.676432; // kg
    inertia.m2 = 0.27959940; //kg
    inertia.Izz1 = 0.08378037; //kgm^2
    inertia.Izz2 = 0.01005577; //kgm^2
    inertia.l1 = 0.38;         //m
    inertia.l2 = 0.3615;       //m
    inertia.com1(0) = 0.154585302583965;
    inertia.com1(1) = 0.000148813426876504;
    inertia.com1(2) = 0.0228920460487543;
    inertia.com2(0) = 0.151947583066964;
    inertia.com2(1) =  -5.03809557717005e-06;
    inertia.com2(2) = -0.0285988321147522;

    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, double>>
        leg(new Singleleg(&inertia));

    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim, double>>
        legLinearizer(new ct::core::SystemLinearizer<state_dim, control_dim, double>(leg));

    

    // 终端损失
    std::shared_ptr<term_t> finalCost(new term_t());

    const std::string cost_dir = "/home/cda/code/ros_ws/src/mpc_for_single_leg/config";
    bool verbose = false;

    // 从配置文件加载目标函数项
    ct::core::StateVector<state_dim> x_start;
    x_start << 0, 0, -1.0471975511965976 * 2, 2.0943951023931953 * 2;
    ct::core::StateVector<state_dim> x_end;
    x_end << -pi/3, pi/3*2, -pi/3*2, 2.0943951023931953 * 2;
    ct::core::ControlVector<control_dim> u_start;
    u_start << 0, 0, 0, 0;
    ct::core::ControlVector<control_dim> u_end;
    u_end << 0, 0, 0, 0;
    auto x_a = ct::core::linspace<ct::core::StateVectorArray<state_dim>>(x_start, x_end, 500);
    double time_tmp;
    double w = 1;
    for (int i=0;i<500;i++)
    {
        time_tmp = i * 0.001;
        target_footend_pos(0) = 0.400 + 0.2 * cos(w * pi * time_tmp);
        target_footend_pos(1) = 0.2 * sin(w * pi * time_tmp);
        target_footend_vel(0) = -w * pi * 0.2 * sin(w * pi * time_tmp);
        target_footend_vel(1) = w * pi * 0.2 * cos(w * pi * time_tmp);
        inverse_solution(target_joint_ang, target_footend_pos);
        jacobian(0, 0) = -upper_len * sin(target_joint_ang(0)) - lower_len * sin(target_joint_ang(0) + target_joint_ang(1));
        jacobian(0, 1) = -lower_len * sin(target_joint_ang(0) + target_joint_ang(1));
        jacobian(1, 0) = upper_len * cos(target_joint_ang(0)) + lower_len * cos(target_joint_ang(0) + target_joint_ang(1));
        jacobian(1, 1) = lower_len * cos(target_joint_ang(0) + target_joint_ang(1));
        target_joint_vel = jacobian.inverse() * target_footend_vel;
        x_a[i](0, 0) = target_joint_ang(0, 0);
        x_a[i](1, 0) = target_joint_ang(1, 0);
        x_a[i](2, 0) = target_joint_vel(0, 0);
        x_a[i](3, 0) = target_joint_vel(1, 0);
        
    }
    auto u_a = ct::core::linspace<ct::core::ControlVectorArray<control_dim>>(u_start, u_end, 500);
    ct::core::TimeArray t_a(0.001, 500);
    ct::core::StateTrajectory<state_dim> x_traj(t_a, x_a, ct::core::LIN);
    ct::core::ControlTrajectory<control_dim> u_traj(t_a, u_a, ct::core::LIN);

    // 中间过程损失
    ct::optcon::TermQuadTracking<state_dim, control_dim>::state_matrix_t Q;
    ct::optcon::TermQuadTracking<state_dim, control_dim>::control_matrix_t R;
    ct::optcon::loadMatrixCF(cost_dir + "/legcost.info", "Q", Q, "intermediateCost");
    ct::optcon::loadMatrixCF(cost_dir + "/legcost.info", "R", R, "intermediateCost");
    std::shared_ptr<ct::optcon::TermQuadTracking<state_dim, control_dim>> intermediateTrackingCost(
        new ct::optcon::TermQuadTracking<state_dim, control_dim>(Q, R, ct::core::LIN, ct::core::ZOH, false));
    intermediateTrackingCost->setStateAndControlReference(x_traj, u_traj);

    

    intermediateTrackingCost->setStateAndControlReference(x_traj, u_traj);
    auto data_array = x_traj.getDataArray();
    for (auto i:data_array)
    {
        std::cout << i.transpose() << std::endl;
    }
    
    // finalCost->loadConfigFile(cost_dir + "/legcost.info", "finalCost", verbose);

    // 目标函数类型
    
    //定义一个目标函数
    std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> costF(
        new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
    // 添加项
    costF->addIntermediateTerm(intermediateTrackingCost);
    // costF->addFinalTerm(finalCost);




    

    

    x_type x_des;
    x_des << -1.0471975511965976, 2.0943951023931953, 0, 0;
    x_type x0;
    for(size_t i=0;i<state_dim;i++)
    {
        x0(i) = sensor[i];
    }
    
    
    t_type timeHorizon = 0.5;

    
    problem_t problem(timeHorizon, x0, leg, costF, legLinearizer);
    
    settings_t ilqr_settings;
    ilqr_settings.dt = 0.001;  // the control discretization in [sec]
    ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
    ilqr_settings.discretization = settings_t::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = settings_t::NLOCP_ALGORITHM::GNMS;
    ilqr_settings.lqocp_solver = settings_t::LQOCP_SOLVER::GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = false;


    size_t K = ilqr_settings.computeK(timeHorizon);
    FeedbackArray u0_fb(K, FeedbackMatrix::Zero());
    uArrayType u0_ff(K, u_type::Zero());
    // std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
    // std::cout << "x_des = " << x_des << std::endl;

    // std::cout << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
    StateVectorArray<state_dim> x_ref_init = ct::core::linspace<StateVectorArray<state_dim>>(x0, x_des, K+1);
    // StateVectorArray<state_dim> x_ref_init(K + 1, x_des);
    NLOptConSolver::Policy_t initController(x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

    // STEP 2-C: create an NLOptConSolver instance
    NLOptConSolver iLQR(problem, ilqr_settings);

    // set the initial guess
    iLQR.setInitialGuess(initController);
    // we solve the optimal control problem and retrieve the solution
    iLQR.solve();
    using StateFeedbackController =  ct::core::StateFeedbackController<state_dim, control_dim>;
    StateFeedbackController initialSolution = iLQR.getSolution();

//  q_type p_act;
    // q_type C;

    
    // 1) settings for the iLQR instance used in MPC. Of course, we use the same settings
    // as for solving the initial problem ...
    settings_t ilqr_settings_mpc = ilqr_settings;
    // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
    ilqr_settings_mpc.max_iterations = 1;
    // and we limited the printouts, too.
    ilqr_settings_mpc.printSummary = false;
    // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
    mpc_settings.coldStart_ = false;
    // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.


    using MPC = ct::optcon::MPC<NLOptConSolver>;
    MPC ilqr_mpc(problem, ilqr_settings_mpc, mpc_settings);
    // initialize it using the previously computed initial controller
    ilqr_mpc.setInitialGuess(initialSolution);
    
    
    
    auto start_time = std::chrono::high_resolution_clock::now();
    // limit the maximum number of runs in this example
    size_t maxNumRuns = 5000;

    std::cout << "Starting to run MPC" << std::endl;
    ros::Rate rate(1000);
    double time_after_horizon;
    for(int count=0;count<maxNumRuns;count++)
    {
        double time_now = count*0.001;
        // std::cout << "++++++++++++++++++++++++++++" << std::endl;
        // cnt++;
        // std::cout << "enter for loop " << std::endl;
        // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
        if (count > 0)
        {
            for(int i=0;i<state_dim;i++)
            {
                x0(i) = sensor[i];
            }
            
            
            
        }
        actual_joint_ang(0, 0) = sensor[0];
        actual_joint_ang(1, 0) = sensor[1];

        forward_solution(actual_footend_pos, actual_joint_ang);
        time_after_horizon = time_now + 0.5;
        // target_footend_pos = 
        target_footend_pos(0) = 0.400 + 0.2 * cos(w * pi * time_after_horizon);
        target_footend_pos(1) = 0.2 * sin(w * pi * time_after_horizon);
        target_footend_vel(0) = -w * pi * 0.2 * sin(w * pi * time_after_horizon);
        target_footend_vel(1) = w * pi * 0.2 * cos(w * pi * time_after_horizon);
        inverse_solution(target_joint_ang, target_footend_pos);
        jacobian(0, 0) = -upper_len * sin(target_joint_ang(0)) - lower_len * sin(target_joint_ang(0) + target_joint_ang(1));
        jacobian(0, 1) = -lower_len * sin(target_joint_ang(0) + target_joint_ang(1));
        jacobian(1, 0) = upper_len * cos(target_joint_ang(0)) + lower_len * cos(target_joint_ang(0) + target_joint_ang(1));
        jacobian(1, 1) = lower_len * cos(target_joint_ang(0) + target_joint_ang(1));
        target_joint_vel = jacobian.inverse() * target_footend_vel;
        x_end(0) = target_joint_ang(0, 0);
        x_end(1) = target_joint_ang(1, 0);
        x_end(2) = target_joint_vel(0, 0);
        x_end(3) = target_joint_vel(1, 0);
        x_traj.eraseFront(1);
        x_traj.push_back(x_end, 0.001, false);
        intermediateTrackingCost->setStateAndControlReference(x_traj, u_traj);
        //定义一个目标函数
        std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> newCostFunction(
            new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());
        // 添加项
        newCostFunction->addIntermediateTerm(intermediateTrackingCost);
        // newCostFunction->addFinalTerm(finalCost);
        ilqr_mpc.getSolver().changeCostFunction(newCostFunction);
        auto x_target = x_traj.front();
        mpc_data << x_target(0)<< ","
            << x_target(1)<< ","
            << x_target(2)<< ","
            << x_target(3)<< ","
            << sensor[0] << ","
            << sensor[1] << ","
            << sensor[2] << ","
            << sensor[3] << ","
            << x_traj.startTime() << "\n";
        RF_HFE_joint_data << x_target(0) << ","
            << sensor[0] << ","
            << x_target(2) << ","
            << sensor[2] << ","
            << x_traj.startTime() << "\n";
        RF_KFE_joint_data << x_target(1) << ","
            << sensor[1] << ","
            << x_target(3) << ","
            << sensor[3] << ","
            << x_traj.startTime() << "\n";
         
        // mpc_data << x_target(0) << ","
        //     << x_target(1) << ","
        //     << 
        //     << actual_joint_ang(0) << ","
        //     << actual_joint_ang(1) << ","
        //     << x_traj.startTime() << "\n";
            

        // time which has passed since start of MPC
        auto current_time = std::chrono::high_resolution_clock::now();
        ct::core::Time t =
                1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

        // prepare mpc iteration
        ilqr_mpc.prepareIteration(t);

        // new optimal policy
        ct::core::StateFeedbackController<state_dim, control_dim> newPolicy;

        // timestamp of the new optimal policy
        ct::core::Time ts_newPolicy;

        current_time = std::chrono::high_resolution_clock::now();
        t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
        bool success = ilqr_mpc.finishIteration(x0, t, newPolicy, ts_newPolicy);
       
        // ilqr_mpc.
        newPolicy.computeControl(x0, 0, u);

        // std::cout << "u = " << u << std::endl;
        // we break the loop in case the time horizon is reached or solve() failed
        if (!success)
        {
            break;
        }
        std::cout << "cnt = " << cnt << std::endl;
        rate.sleep();
    }
    mpc_data.close();

    // 


    // the summary contains some statistical data about time delays, etc.
    ilqr_mpc.printMpcSummary();
    file.close();
    
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "controller");
    std_msgs::Float64 joint_torque[2];
    ros::NodeHandle n;
    ros::Subscriber joint_states_sub = n.subscribe("/pegasus2_model/joint_states", 1, joint_states_callback);

    ros::Publisher rf_hfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller02/command", 1);
    ros::Publisher rf_kfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller03/command", 1);
    // ct::core::ControlVector<2> u;
    std::thread th(mpc_thread);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    file.open("/home/cda/Desktop/data01.txt", std::ios::out);
    ros::Rate rate(1000);
    while(ros::ok())
    {
        double t = static_cast<double>(cnt)/1000;
        
        if(!u.hasNaN())
        {
            for(int i=0; i<2; i++)
            {
                joint_torque[i].data = u(i);
            }
        }

        file << sensor[0] << "\t";
        file << sensor[1] << "\t";
        file << sensor[2] << "\t";
        file << sensor[3] << "\t";
        file << u(0) << "\t";
        file << u(1) << "\t";
        file << t << "\n";

#define ENABLE_PUBLISHING
#ifdef ENABLE_PUBLISHING
        rf_hfe_pub.publish(joint_torque[0]);
        rf_kfe_pub.publish(joint_torque[1]);
#endif
        rate.sleep();
        cnt++;
    }
    th.join();
    return 0;
}
