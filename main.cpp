//
// Created by Ruslan Shaiakhmetov on 20/12/22.
//

#include "two_wheel.h"

int main() {
    TwoWheel car(1.0, 1.0);
    car.update(1.0, 1.0, 0.1);
    car.update(1.0, 0.0, 0.1);
    for (int i = 0; i < 10; i++) {
        auto result = car.update(0.0, 0.0, 0.1);
        std::cout << "Position: " << result.position.str() <<" "<< result.theta << std::endl;
    }
    return 0;
}
