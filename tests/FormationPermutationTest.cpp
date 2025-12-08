#include "control/FormationPattern.h"
#include "control/AssignmentStrategy.h"

#include <iostream>
#include <algorithm>
#include <numeric>
#include <random>

bool sameRelativeOrder(const std::vector<int>& before,
    const std::vector<int>& after)
{
    // remove items not in "after"
    std::vector<int> filtered;
    for (int id : before) {
        if (std::find(after.begin(), after.end(), id) != after.end()) {
            filtered.push_back(id);
        }
    }
    return filtered == after;
}

int main() {
    FormationPattern circle =
        FormationPattern::makeCircle("perm", 10.0f, 720);

    const int N = 30;
    std::vector<int> ids(N);
    std::iota(ids.begin(), ids.end(), 0);

    auto a1 = AssignmentStrategy::assignDronesToPattern(ids, circle);

    // randomly kill half the drones
    std::vector<int> survivors = ids;
    std::shuffle(survivors.begin(), survivors.end(),
        std::mt19937{ 12345 });
    survivors.resize(N / 2);

    auto a2 = AssignmentStrategy::assignDronesToPattern(survivors, circle);

    // Extract ordering
    std::vector<int> order1, order2;
    for (auto& x : a1) order1.push_back(x.droneId);
    for (auto& x : a2) order2.push_back(x.droneId);

    bool stable = sameRelativeOrder(order1, order2);

    std::cout << (stable ?
        "PASS: relative order maintained\n" :
        "FAIL: relative order broken\n");

    return stable ? 0 : 1;
}
