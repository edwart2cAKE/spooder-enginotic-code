#ifndef AUTOS_HPP
#define AUTOS_HPP

#include <string>
#include <vector>
#include <functional>

// auto functions
void test_lateral(int dist);
void test_lateral_range(int start_dist, int dist_increment, int num_tests);
void auto_skills();
void match_ladder();
void new_auto_skills_push();
void match_ring2();

// selector funcs
extern int num_autos;
extern std::string auto_names[];
extern std::vector<std::function<void>> auto_funcs;
#endif // AUTOS_HPP