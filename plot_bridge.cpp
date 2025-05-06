#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include"ev.h"
#include <stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <stdbool.h>
#include <limits.h>
#include <float.h>
#include "solar.h"
#include"battery.h"
#include "mechanic.h"
#include"vehicle.h"
#include"throttle.h"
//#include "plot_bridge.h"
#include "define.h"
#include"free.h"
#include"memalloc.h"
#include"power.h"
int main() {
    // 模擬資料
    int z;
    std::vector<double> t, alpha;
    std::cout << "Main from C++" << std::endl;
    z=main();  // 呼叫 C 實作的邏輯
    int n = 100;
    for (int i = 0; i < n; ++i) {
        t.push_back(i * 0.1);
        alpha.push_back(50 * sin(0.2 * i)); // alpha 百分比
    }

    // 寫入臨時資料檔
    std::ofstream data("alpha_data.txt");
    for (int i = 0; i < n; ++i) {
        data << t[i] << " " << alpha[i] << std::endl;
    }
    data.close();

    // 開啟 gnuplot pipe
    FILE* gp = popen("gnuplot -persistent", "w");
    if (!gp) {
        std::cerr << "Gnuplot 無法啟動\n";
        return 1;
    }

    // Gnuplot 指令
    fprintf(gp,
        "set multiplot layout 2,2 title 'All Plots'\n"
        "set ytics nomirror\n"
        "set y2tics\n"
        "set ylabel 'Throttle (%%)'\n"
        "plot 'alpha_data.txt' using 1:2 with lines linestyle 1 linecolor rgb 'black' title 'alpha(t)'\n"
        "unset multiplot\n"
    );
    
    pclose(gp);
    return 0;
}
