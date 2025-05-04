#include "plot_bridge.h"
#include <matplot/matplot.h>
using namespace matplot;

void draw_my_plot() {
    auto x = linspace(0, 2*pi, 100);
    auto y = transform(x, [](double x) { return sin(x); });
    plot(x, y);
    show();
}
