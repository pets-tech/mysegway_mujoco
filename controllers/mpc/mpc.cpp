#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <zmq.h>
#include <vector>
#include <chrono>
#include <thread>

template <typename T>
constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

auto get_time() {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::milliseconds since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return since_epoch.count();
}

struct StateMsg {
    double t;
    float theta;
    float dtheta;
    float psi;
    float dpsi;
};

#pragma pack(push, 1)
struct ControlMsg {
    double t;
    float u_left;
    float u_right;
};
#pragma pack(pop)

int main(){

    constexpr int nx = 4;
    constexpr int nu = 1;
    constexpr int N  = 100;     // 0.25 sec horizon (model dt=0.005)
    constexpr double umax = 16.0;

    constexpr int nvar = nx*(N+1) + nu*N;

    constexpr int neq_dyn = nx*N;
    constexpr int neq_x0  = nx;
    constexpr int neq     = neq_dyn + neq_x0;

    constexpr int nineq   = nu*N;
    constexpr int ncon    = neq + nineq;


    Eigen::Matrix<double,nx,nx> A;
    Eigen::Matrix<double,nx,nu> B;

    A << 1.00000e+00, -2.43155e-04,  5.00000e-03, -4.05251e-07,
         0.00000e+00,  1.00031e+00,  0.00000e+00,  5.00052e-03,
         0.00000e+00, -9.72672e-02,  1.00000e+00, -2.43155e-04,
         0.00000e+00,  1.24210e-01,  0.00000e+00,  1.00031e+00;

    B << 7.48153e-04,
        -2.64970e-04,
         2.99265e-01,
        -1.05994e-01;


    Eigen::Matrix<double,nx,nx> Q = Eigen::Matrix<double,nx,nx>::Zero();
    Q.diagonal() << 50,100,2,1;

    Eigen::Matrix<double,nu,nu> R;
    R << 10;

    Eigen::Matrix<double,nx,nx> P;
    P << 6702.3796 ,  26193.66276,   2129.33867,   6223.96849,  26193.66276,
        146061.04439,  11266.30676,  34204.01214,   2129.33867,  11266.30676,
        905.55718,   2696.71682,   6223.96849,  34204.01214,   2696.71682,
        8168.68504;


    std::vector<Eigen::Triplet<double>> H_trip;

    for(int k=0;k<N;k++)
        for(int i=0;i<nx;i++)
            H_trip.emplace_back(k*nx+i, k*nx+i, Q(i,i));

    for(int i=0;i<nx;i++)
        H_trip.emplace_back(N*nx+i, N*nx+i, P(i,i));

    for(int k=0;k<N;k++){
        int idx = nx*(N+1) + k;
        H_trip.emplace_back(idx, idx, R(0,0));
    }

    Eigen::SparseMatrix<double> H(nvar,nvar);
    H.setFromTriplets(H_trip.begin(), H_trip.end());

    std::vector<Eigen::Triplet<double>> A_trip;

    for(int k=0;k<N;k++){
        for(int i=0;i<nx;i++){

            int row = k*nx + i;

            A_trip.emplace_back(row,(k+1)*nx+i,1.0);

            for(int j=0;j<nx;j++)
                A_trip.emplace_back(row,k*nx+j,-A(i,j));

            int ucol = nx*(N+1)+k;
            A_trip.emplace_back(row,ucol,-B(i,0));
        }
    }

    for(int i=0;i<nx;i++){
        int row = neq_dyn + i;
        A_trip.emplace_back(row,i,1.0);
    }

    for(int k=0;k<N;k++){
        int row = neq + k;
        int col = nx*(N+1)+k;
        A_trip.emplace_back(row,col,1.0);
    }

    Eigen::SparseMatrix<double> Acon(ncon,nvar);
    Acon.setFromTriplets(A_trip.begin(),A_trip.end());

    Eigen::VectorXd l = Eigen::VectorXd::Zero(ncon);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(ncon);

    for(int i=0;i<neq_dyn;i++){
        l(i)=0; u(i)=0;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setMaxIteration(40);
    solver.settings()->setAbsoluteTolerance(1e-3);
    solver.settings()->setRelativeTolerance(1e-3);

    solver.data()->setNumberOfVariables(nvar);
    solver.data()->setNumberOfConstraints(ncon);
    solver.data()->setHessianMatrix(H);
    solver.data()->setLinearConstraintsMatrix(Acon);

    Eigen::VectorXd grad = Eigen::VectorXd::Zero(nvar);
    solver.data()->setGradient(grad);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    if(!solver.initSolver()){
        std::cout<<"Init failed\n";
        return -1;
    }

    void* ctx = zmq_ctx_new();
    void* sub = zmq_socket(ctx,ZMQ_SUB);
    zmq_connect(sub,"tcp://127.0.0.1:5555");
    zmq_setsockopt(sub,ZMQ_SUBSCRIBE,"",0);

    void* pub = zmq_socket(ctx,ZMQ_PUB);
    zmq_bind(pub,"tcp://127.0.0.1:5556");

    std::cout<<"Sparse MPC running\n";

    double v_cmd = 0.3;
    double omega_cmd = 1.0;
    static double v_int = 0.0;

    int t_prev = 0.0;
    while(true){
        int dt = get_time() - t_prev;
        t_prev = get_time();
        // std::cout << 1000.0 / dt << "Hz \n";

        StateMsg s;
        if(zmq_recv(sub,&s,sizeof(s),0)!=sizeof(s))
            continue;

        // x = [theta, psi, dtheta, dpsi]
        Eigen::VectorXd x0(nx);
        x0 << s.theta, s.psi, s.dtheta, s.dpsi;

        const double wR = 0.085;
        const double g = 9.81;
        const double k_v = 1.0;
        const double k_w = 0.5;


        double v_current = wR * s.dtheta;

        double v_error = v_cmd - v_current;
        v_int += v_error * 0.005;

        v_int = std::clamp(v_int, -2.0, 2.0);

        double v_ref_corrected = v_cmd + 0.5 * v_error + 0.3 * v_int;

        double yaw_error = omega_cmd - s.dpsi;
        double u_yaw = k_w * yaw_error;
        u_yaw = std::clamp(u_yaw, -0.5, 0.5);


        Eigen::Vector4d x_ref;
        x_ref << 0.0,
                0.0,
                v_ref_corrected / wR,
                0.0;
        
        grad.setZero();

        for(int k=0;k<N;k++){
            for(int i=0;i<nx;i++){
                int idx = k*nx + i;
                grad(idx) = -2.0 * Q(i,i) * x_ref(i);
            }
        }

        for(int i=0;i<nx;i++){
            int idx = N*nx + i;
            grad(idx) = -2.0 * P(i,i) * x_ref(i);
        }

        solver.updateGradient(grad);

        for(int i=0;i<nx;i++){
            l(neq_dyn+i)=x0(i);
            u(neq_dyn+i)=x0(i);
        }

        for(int k=0;k<N;k++){
            l(neq+k) = -umax;
            u(neq+k) =  umax;
        }

        solver.updateBounds(l,u);

        if(solver.solveProblem()!=OsqpEigen::ErrorExitFlag::NoError)
            continue;

        Eigen::VectorXd sol = solver.getSolution();

        double u0 = sol(nx*(N+1));

        double u_left  = u0 - u_yaw;
        double u_right = u0 + u_yaw;

        if (u0 > 8.0f) u0 = 8.0f;
        if (u0 < -8.0f) u0 = -8.0f;

        if (fabs(s.psi) >= 1.7f) {
            u0 = sgn(s.psi) * 8.0f;
            u_left  = u0;
            u_right = u0;            
        }

        ControlMsg c;
        c.t = s.t;
        c.u_left = static_cast<float>(u_left);
        c.u_right = static_cast<float>(u_right);


        zmq_send(pub,&c,sizeof(c),ZMQ_DONTWAIT);
    }

    return 0;
}