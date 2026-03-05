#include <zmq.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <iostream>
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

int main() {
    const float K[4] = {-2.23607, -25.29926,  -1.49292,  -5.85518};

    void* ctx = zmq_ctx_new();

    void* sub = zmq_socket(ctx, ZMQ_SUB);
    zmq_connect(sub, "tcp://127.0.0.1:5555");
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);

    void* pub = zmq_socket(ctx, ZMQ_PUB);
    zmq_bind(pub, "tcp://127.0.0.1:5556");

    printf("LQR controller started\n");
    int i = 0;
    float u = 0.0;

    while (true) {

        StateMsg s;
        int n = zmq_recv(sub, &s, sizeof(s), 0);
        if (n != sizeof(s)) continue;

        // Simulation 500 Hz
        // Control 500 Hz
        if (i == 1) {
            if (s.t > 0.0) {
                // x = [theta, psi, dtheta, dpsi]
                u =
                    -(K[0]*(s.theta - 0.0) +
                    K[1]*s.psi +
                    K[2]*s.dtheta +
                    K[3]*s.dpsi);

                if (u > 8.0f) u = 8.0f;
                if (u < -8.0f) u = -8.0f;

                if (fabs(s.psi) >= 1.7f) {
                    u = sgn(s.psi) * 8.0f;
                }
            }
            i = 0;
        }
        ++i;

        ControlMsg c;
        c.t = s.t;
        c.u_left = u;
        c.u_right = u;

        zmq_send(pub, &c, sizeof(c), ZMQ_DONTWAIT);
    }

    zmq_close(sub);
    zmq_close(pub);
    zmq_ctx_destroy(ctx);
    return 0;
}
