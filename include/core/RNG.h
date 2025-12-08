#pragma once
#include <random>

extern std::mt19937_64 GLOBAL_RNG;

void reseedRNG(uint64_t seed);
