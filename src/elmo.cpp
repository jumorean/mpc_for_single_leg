// #include <stdio.h>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <cstring>
#include <ctime>
#include <thread>
#include <semaphore.h>

#include <pthread.h>
#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <ethercat.h>
#include <csignal>
#include "my_type.h"
#include "motor.h"
#include <memory>
#include "joint.h"
#include "transform.h"
#include "io.h"
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "configDir.h"
//#include <ct/core/core.h>
#include <Eigen/Cholesky>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>


bool end_program = false;
ct::core::ControlVector<2> mpc_force;
int cnt =0;
double actual_state[4] = {0};
void mpc_thread();
void ctrl_c_function(int sig)
{
   end_program =true;
   (void)signal(SIGINT, SIG_DFL);
}


/*
 * User define  
*/
static int elmo_count = 0;

#define NSEC_PER_SEC 1000000000




// sample interval in ns, here 8us -> 125kHz
// maximum data rate for E/BOX v1.0.1 is around 150kHz
#define SYNC0TIME 8000


char IOmap[4096];
pthread_t thread1;
struct timeval tv,t1,t2;
int dorun = 0;
int deltat, tmax=0;
int64 toff;
int DCdiff;

sem_t bin_sem;
sem_t mpc_thread_sem;
pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int64 integral=0;
uint32 cyclecount;




static const double pi = 3.141592654;
typedef elmo::Motor Motor_t;
typedef Motor_t * MotorPtr_t;
typedef elmo::Joint Joint_t;
typedef Joint_t * JointPtr_t;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 2, 2> JacobianT;




const double upper_len=0.38;

const double lower_len=0.38;

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
            const x_type & x, const t_type & t, const u_type & tau, x_type & x_dot) override
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

        q_dotdot = M_inv * (tau - V - G);

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


void forward_solution(Vector2d & foot_end, const Vector2d & angular)
{
    foot_end(0) = upper_len * cos(angular(0)) + lower_len * cos(angular(1) + angular(0));
    foot_end(1) = upper_len * sin(angular(0)) + lower_len * sin(angular(1) + angular(0));
}

void inverse_solution(Vector2d & angular, const Vector2d & foot_end)
{
    double x = foot_end(0);
    double y = foot_end(1);
    double alpha = atan2(y, x);
    double rho = sqrt(x*x + y *y);
    double phi = acos((upper_len*upper_len + rho*rho - lower_len * lower_len)/2/upper_len/rho);
    angular(0) = alpha - phi;
    double beta = asin(upper_len* sin(phi) / lower_len);
    angular(1) = phi + beta;
}

// static const double err_limit = 0.00119842249053566;
static const double err_limit = 0.00119842249053566;
static double coef(double angular_err)
{
    if (angular_err >= 0)
    {
        if (angular_err > err_limit)
        {
            return 1.;
        }
        else
        {
            return angular_err / err_limit;
        }
    }
    else
    {
        return coef(-angular_err);
    }

}

static double sgn(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
class Generator{
public:
    typedef Eigen::Matrix<double, 2, 1>  Point;
    Generator()
    {
        p1 << 0.45, -0.05;
        p2 << 0.45, -0.05;
        p3 << 0.3, -0.05;
        p4 << 0.3, 0.05;
        p5 << 0.45, 0.05;
        p6 << 0.45, 0.05;
    }
    void generate(Point &p, Point & v, Point & a, const double & t)
    {
        p = p1 - 5*p1*t + 5*p2*t + 10*p1*t*t - 10*p1*t*t*t - 20*p2*t*t + 5*p1*t*t*t*t + 30*p2*t*t*t + 10*p3*t*t - p1*t*t*t*t*t - 20*p2*t*t*t*t - 30*p3*t*t*t + 5*p2*t*t*t*t*t + 30*p3*t*t*t*t + 10*p4*t*t*t - 10*p3*t*t*t*t*t - 20*p4*t*t*t*t + 10*p4*t*t*t*t*t + 5*p5*t*t*t*t - 5*p5*t*t*t*t*t + p6*t*t*t*t*t;
        v = 5*p2 - 5*p1 + 20*p1*t - 40*p2*t + 20*p3*t - 30*p1*t*t + 20*p1*t*t*t + 90*p2*t*t - 5*p1*t*t*t*t - 80*p2*t*t*t - 90*p3*t*t + 25*p2*t*t*t*t + 120*p3*t*t*t + 30*p4*t*t - 50*p3*t*t*t*t - 80*p4*t*t*t + 50*p4*t*t*t*t + 20*p5*t*t*t - 25*p5*t*t*t*t + 5*p6*t*t*t*t;
        a = 20*p1 - 40*p2 + 20*p3 - 60*p1*t + 180*p2*t - 180*p3*t + 60*p4*t + 60*p1*t*t - 20*p1*t*t*t - 240*p2*t*t + 100*p2*t*t*t + 360*p3*t*t - 200*p3*t*t*t - 240*p4*t*t + 200*p4*t*t*t + 60*p5*t*t - 100*p5*t*t*t + 20*p6*t*t*t;
    }
protected:
    Point p1;
    Point p2;
    Point p3;
    Point p4;
    Point p5;
    Point p6;
};
void control_loop()
{
    // typedef
    // Open config file, load the configuration.
    std::string config_file_name = "config.json";
    std::string config_dir_name = configDir;
    std::string config_file_full_name = config_dir_name + "/" + config_file_name;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(config_file_full_name, pt);
    int controlled_motor_count = pt.get<int>("controlled motor count");
    // Open the data file
    std::fstream error_file;
    std::fstream data_file;
    std::fstream motor_data_file;
    motor_data_file.open("/home/cda/Desktop/data/motor_data_file", std::ios::out);
    error_file.open("/home/cda/Desktop/data/error.txt", std::ios::out);
    data_file.open("/home/cda/Desktop/data/data.csv", std::ios::out);
    /* Init code start */
    std::vector<InputData_t *> input_data(elmo_count, nullptr);
    std::vector<OutputData_t *> output_data(elmo_count, nullptr);
    for(int i=0;i<elmo_count;i++)
    {
        input_data[i] = (InputData_t * )ec_slave[i+1].inputs;
        output_data[i] = (OutputData_t * )ec_slave[i+1].outputs;
    }

    std::vector<MotorPtr_t> motors(controlled_motor_count, nullptr);
    std::vector<JointPtr_t> joints(controlled_motor_count, nullptr);
    for(int i=0;i<controlled_motor_count;i++)
    {
        motors[i] = new elmo::Motor(input_data[i], output_data[i]);
        joints[i] = new Joint_t(motors[i]);
    }
    joints[0]->config(pt, "rf.hfe");
    joints[1]->config(pt, "rf.kfe");
    
    // int cnt = 0;
    double time=0;
    mpc_force.setZero();
    std::thread th(mpc_thread);
    /* Init code end */


    // Matrix definition


    motor_data_file << "actual_pos[0]" << ",";
    motor_data_file << "actual_pos[1]" << ",";
    motor_data_file << "actual_vel[0]" << ",";
    motor_data_file << "actual_vel[1]" << ",";
    motor_data_file << "actual_torque[0]" << ",";
    motor_data_file << "actual_torque[1]" << ",";
    motor_data_file << "time" << "\n";

    data_file << "actual_pos[0]" << ",";
    data_file << "actual_pos[1]" << ",";
   
    data_file << "actual_vel[0]" << ",";
    data_file << "actual_vel[1]" << ",";
   
    data_file << "actual_torque[0]" << ",";
    data_file << "actual_torque[1]" << ",";

    data_file << "mpc_torque[0]" << ",";
    data_file << "mpc_torque[1]" << ",";
   

    data_file << "time" << "\n";

    /* acyclic loop 1000ms */
    while(true) {
        sem_wait(&bin_sem);

        time = cnt * 0.001;
        for(auto i:motors)
        {
            i->status_control();
        }
        double abcd = 0;
        actual_state[0] = joints[0]->getActualAngular();
        actual_state[1] = joints[1]->getActualAngular();
        actual_state[2] = joints[0]->getActualVelocity();
        actual_state[3] = joints[1]->getActualVelocity();
        if (time < 0.5) {
            for (auto i: joints) {
                i->enable(false);
                i->setOpMode(10);
                i->torque_cmd(0);
            }
        }
        else{
            if (time < 1) {
                for (auto i: joints) {
                    i->enable(true);
                    i->setOpMode(10);
                }
            }
            else {

            }
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
            std::cout << "mpc_torque: " <<  mpc_force(0)  << "\t" << mpc_force(1) << std::endl;
            std::cout << "joint angular:" << rad2deg(joints[0]->getActualAngular())
            << "\t" << rad2deg(joints[1]->getActualAngular()) << std::endl;
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
            // tau += gravity_comp;
            for(int i=0;i<2;i++)
            {
                joints[i]->torque_cmd(mpc_force(i));
            }
        }

        // std::cout << std::dec << time << "\t" <<  std::hex << input_data[0]->status_word << "\t" << input_data[1]->status_word << "\r";
        if(time > 1)
        {
            data_file << joints[0]->getActualAngular() << ","
                    << joints[1]->getActualAngular()  << ","
                    << joints[0]->getActualVelocity() << ","
                    << joints[1]->getActualVelocity() << ","
                    << joints[0]->getActualTorque() << ","
                    << joints[1]->getActualTorque() << ","
                    << mpc_force(0) << ","
                    << mpc_force(1) << ","
                    << time << "\n";
            motor_data_file << motors[0]->getActualPosition() << ",";
            motor_data_file << motors[1]->getActualPosition() << ",";
            motor_data_file << motors[0]->getActualVelocity() << ",";
            motor_data_file << motors[1]->getActualVelocity() << ",";
            motor_data_file << motors[0]->getActualTorque() << ",";
            motor_data_file << motors[1]->getActualTorque() << ",";
            motor_data_file << time << "\n";

        }

        cnt++;


        for (auto i: input_data) {
            if ((i->status_word & 0x004fU) == 0x0008U) {
                error_file << "ERROR" << std::endl;
            }
        }

        // check if get the end program signal
        if (end_program) {
            data_file.close();
            error_file.close();
            motor_data_file.close();
            break;
        }
    }
}


void master_shutdown()
{
    for(int slave=1;slave<=ec_slavecount;slave++)
    {
        ec_dcsync0(slave, FALSE, 8000, 0); // SYNC0 off
    }
    std::cout << "Request safe operational state for all slaves\n";
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
}

static void slave_info()
{
    for(int cnt = 1; cnt <= ec_slavecount ; cnt++)
    {
        std::cout << "Slave:" << cnt << ", "
                  << "Name:" << ec_slave[cnt].name << ", "
                  << "Output size: " << ec_slave[cnt].Obits << "bits, "
                  << "Input size: " << ec_slave[cnt].Ibits << "bits, "
                  << "State: " << ec_slave[cnt].state << ", "
                  << "delay: " << ec_slave[cnt].pdelay << ", ";
        if(ec_slave[cnt].hasdc != 0) {
            std::cout << "has DC: " << "true" << std::endl;
        }
        else{
            std::cout << "has DC: " << "false" << std::endl;
        }

    }
}

void write_slave_state(uint16 target_state)
{
    ec_slave[0].state = target_state;
    /* request OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach OP state */
    ec_statecheck(0, target_state,  EC_TIMEOUTSTATE);
}


void elmo_config(const char * ifname)
{
    int result = 0;
    /* initialise SOEM, bind socket to ifname */
    result = ec_init(ifname);
    if(!result) {std::cerr << "No socket connection on " << ifname << "\n";return;}
    std::cout << "ec_init on " << ifname <<  "succeeded." << std::endl;
    /* find and auto-config slaves */

    result = ec_config_init(FALSE);
    if(result <= 0) {std::cout << "No slaves found!" << std::endl;ec_close();return;}
    std::cout << ec_slavecount << "slaves found and configured." << std::endl;
    do{}while(io_config());
    ec_config_map(&IOmap);
    ec_configdc();
    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

    /* configure DC options for every DC capable slave found in the list */
    std::cout << "DC capable : " << ec_configdc() << std::endl;

    /* check configuration */
    if (ec_slavecount == elmo_count) {
        std::cout << elmo_count << " elmo drives found" << std::endl;
        /* connect struct pointers to slave I/O pointers */
        /* read indevidual slave state and store in ec_slave[] */
        ec_readstate();
        slave_info();
        std::cout << "Request operational state for all slaves\n";
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        write_slave_state(EC_STATE_OPERATIONAL);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
            std::cout << "Operational state reached for all slaves.\n";
            if(0 != sem_init(&bin_sem, 0, 0))
            {
                perror("Semaphore initialization failed");
            }
            if(0 != sem_init(&mpc_thread_sem, 0, 0))
            {
                perror("Semaphore initialization failed");
            }
            dorun = 1;
            usleep(100000); // wait for linux to sync on DC
            for(int i=1;i<=ec_slavecount;i++)ec_dcsync0(i, TRUE, SYNC0TIME, 0); // SYNC0 on slave 1
            control_loop();
            dorun = 0;
        }
        else std::cout << "Not all slaves reached operational state.\n";
    }
    else if (ec_slavecount > elmo_count) std::cout << "elmo count is less than EtherCAT slave count.\n";
    else if (ec_slavecount < elmo_count) std::cout << "elmo count is more than EtherCAT slave count.\n";
    master_shutdown();
    ec_close();
}

/* add ns to timespec */
inline void add_timespec(struct timespec & ts, const int64 & addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts.tv_sec += sec;
    ts.tv_nsec += nsec;
    if ( ts.tv_nsec > NSEC_PER_SEC )
    {
        nsec = ts.tv_nsec % NSEC_PER_SEC;
        ts.tv_sec += (ts.tv_nsec - nsec) / NSEC_PER_SEC;
        ts.tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
inline void ec_sync(const int64 & reftime, const int64 & cycletime , int64 & offsettime)
{
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if(delta> (cycletime /2)) { delta= delta - cycletime; }
    if(delta>0){ integral++; }
    if(delta<0){ integral--; }
    offsettime = -(delta / 100) - (integral /20);
}

/* RT EtherCAT thread */
void * ecatthread( void *ptr )
{
    struct timespec   ts;
    struct timeval    tp;
    int ht;
    // int i;
    int pcounter = 0;
    int64 cycletime = 0;
    pthread_mutex_lock(&mutex);
    gettimeofday(&tp, nullptr);
    /* Convert from timeval to timespec */
    ts.tv_sec  = tp.tv_sec;
    ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    while(true)
    {
        /* calculate next cycle start */
        add_timespec(ts, cycletime + toff);
        /* wait to cycle start */
        pthread_cond_timedwait(&cond, &mutex, &ts);
        if (dorun>0)
        {
            gettimeofday(&tp, nullptr);
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            cyclecount++;
            /* calculate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, toff);
            sem_post(&bin_sem);
            sem_post(&mpc_thread_sem);
        }
        if(end_program)
        {
            break;
        }
    }
    return nullptr;
}

int main(int argc, char *argv[])
{
    int result = 0;
    std::string config_file_name = "config.json";
    std::string config_dir_name = configDir;
    std::string config_file_full_name = config_dir_name + "/" + config_file_name;
    if(argc > 1)
    {
        std::cerr << "Notice: the program parameters are invalid.\n";
        std::cerr << "Please write the configuration to the config file:\n";
        std::cerr << config_file_full_name << std::endl;
        exit(EXIT_FAILURE);
    }
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(config_file_full_name, pt);
    std::string if_name;
    try{
        elmo_count = pt.get<int>("elmo drive count");
        if_name = pt.get<std::string>("network interface name");
        std::cout << "network interface name: " << if_name << std::endl;
    }catch(...){
        std::cerr << "Please check the config file!\n";
        std::cerr << "Check" << config_file_full_name << std::endl;
    }
    (void)signal(SIGINT, ctrl_c_function);


    /* do not set priority above 49, otherwise sockets are starved */
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 30;
    result = sched_setscheduler(0, SCHED_FIFO, &schedp);
    if ( result != 0 )
    {
        std::cerr << "Set the scheduler policy to FIFO failed\n";
        std::cerr << "Please check if you executed this program as root\n";
        exit(EXIT_FAILURE);
    }

    do{
        usleep(1000);
    }while (dorun);

    dorun = 1;
    int ctime = 1000; // 1ms cycle time
    void * pVoid = &ctime;

    /* create RT thread */
    result = pthread_create( &thread1, nullptr, ecatthread, pVoid);
    if(result != 0)
    {
        std::cerr << "Thread created failed!\n";
        exit(EXIT_FAILURE);
    }

    /* start acyclic part */
    elmo_config(if_name.data());
    /* end acyclic part */


    schedp.sched_priority = 0;
    sched_setscheduler(0, SCHED_OTHER, &schedp);

    std::cout << "End program\n";

    return (0);
}

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
    typedef ct::optcon::CostFunctionQuadratic<state_dim, control_dim> cost_t_base;
    typedef ct::optcon::CostFunctionAnalytical<state_dim, control_dim> cost_t;
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

    InertialPara<double> inertia;
    inertia.g = 9.81;
    inertia.m1 = 1.8555783; // kg
    inertia.m2 = 0.318846; //kg
    inertia.Izz1 = 0.10543357199259; //kgm^2
    inertia.Izz2 = 0.016684726988; //kgm^2
    inertia.l1 = 0.38;         //m
    inertia.l2 = 0.3615;       //m
    inertia.com1(0) = 0.16437808256;
    inertia.com1(1) = 0.00038101869;
    inertia.com1(2) = 0.05985287822;
    inertia.com2(0) = 0.172254680;
    inertia.com2(1) =  0.002552009;
    inertia.com2(2) = 0.001298443;

    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, double>>
        leg(new Singleleg(&inertia));

    std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim, double>>
        legLinearizer(new ct::core::SystemLinearizer<state_dim, control_dim, double>(leg));

    // 中间过程损失
    std::shared_ptr<term_t> intermediateCost(new term_t());

    // 终端损失
    std::shared_ptr<term_t> finalCost(new term_t());

    const std::string cost_dir = "/home/cda/code/ros_ws/src/mpc_for_single_leg/config";                                  
    bool verbose = true;

    // 从配置文件加载目标函数项
    intermediateCost->loadConfigFile(cost_dir + "/legcost.info", "intermediateCost", verbose);
    finalCost->loadConfigFile(cost_dir + "/legcost.info", "finalCost", verbose);
    std::cout << "success" << std::endl;
    // 目标函数类型
    
    //定义一个目标函数
    std::shared_ptr<cost_t_base> costF(new cost_t());
    // 添加项
    costF->addIntermediateTerm(intermediateCost);
    costF->addFinalTerm(finalCost);

    

    x_type x_des;
    x_des(0) = 0;
    x_des(1) = 1.5708;
    x_type x0;
    for(size_t i=0;i<state_dim;i++)
    {
        x0(i) = actual_state[i];
    }
    
    
    t_type timeHorizon = 3.;

    
    problem_t problem(timeHorizon, x0, leg, costF, legLinearizer);
    
    settings_t ilqr_settings;
    ilqr_settings.dt = 0.002;  // the control discretization in [sec]
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
    size_t maxNumRuns = 10000;

    std::cout << "Starting to run MPC" << std::endl;
    
    for(int count=0;;count++)
    {
        sem_wait(&mpc_thread_sem);
        // std::cout << "++++++++++++++++++++++++++++" << std::endl;
        // cnt++;
        // std::cout << "enter for loop " << std::endl;
        // let's for simplicity, assume that the "measured" state is the first state from the optimal trajectory plus some noise
        if (count > 0)
        {
            for(int i=0;i<state_dim;i++)
            {
                x0(i) = actual_state[i];
            }
        }
            

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
        newPolicy.computeControl(x0, 0, mpc_force);

        // std::cout << "u = " << u << std::endl;
        // we break the loop in case the time horizon is reached or solve() failed
        if (!success)
        {
            mpc_force.setZero();
            break;
        }

    }

    // 


    // the summary contains some statistical data about time delays, etc.
    ilqr_mpc.printMpcSummary();
    
}
