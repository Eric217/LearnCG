//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

//#define MY_PI 3.1415926
//#define TWO_PI (2.0* MY_PI)

extern float angleForTriangle2;

#define  LOGE(...) do { printf(__VA_ARGS__);printf("\n"); } while(0)

class EfficiencyTool {
public:
    static std::map<int, size_t> TIMES;
    static std::map<int, size_t> TOTAL;
    static void clear() { TIMES.clear(); TOTAL.clear();}
    
    int program;
    int reportInterval;
    clock_t time;
    size_t &times;
    size_t &total;
    EfficiencyTool(int program, int interval): program(program) ,time(clock()),
    times(TIMES[program]), total(TOTAL[program]), reportInterval(interval) {
        if (reportInterval == 0) {
            reportInterval = 1;
        }
    }
    ~EfficiencyTool() {
        auto sec = clock() - time;
        times += 1;
        total += sec;
        if (times % reportInterval) return;
        auto avg = total * 1.0 / CLOCKS_PER_SEC / times;
        LOGE("fill %d avg: %.3f ms, %lu times, total %0.2f s", program, avg * 1000, times, total * 1.0 / CLOCKS_PER_SEC);
    }
};
#endif //RASTERIZER_GLOBAL_H
