#ifndef AUTOS_HPP
#define AUTOS_HPP

#include <string>
#include <vector>
#include <functional>

// auto functions
void test_lateral(int dist);
void test_lateral_range(int start_dist, int dist_increment, int num_tests);
void test_angular(int angle);
void test_angular_range(int start_angle, int angle_increment, int num_tests);

// enginotic autos
void auto_skills();
void match_ladder();
void new_auto_skills_push();
void match_ring2();

// states autos
void right_red4ring();
void states_skills();
void left_blue2ring();

// selector funcs
extern int num_autos;
extern std::string auto_names[];
extern std::vector<std::function<void>> auto_funcs;
#endif // AUTOS_HPP