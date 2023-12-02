#ifndef MDP_H
#define MDP_H

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <limits>
#include <cmath>

#include "state.h"
#include "action.h" 

struct VectorHasher {
    long long operator()(const std::vector<std::string>& v) const {
        int prime = 31;
        long long mod = 1e9 + 9;
        long long hash = 0;
        for (std::string s : v) {
            long long string_hash{};
            long long pow = 1;
            for (int i = 0; i < s.length(); ++i) {
                string_hash = (string_hash + (s[i] - '0' + 1) * pow) % mod;
                pow = (prime * pow) % mod;
            }
            hash = (hash + string_hash + (prime * hash) % mod) % mod;
        }
        return hash;
    }
};

typedef std::unordered_map<std::vector<std::string>, double, VectorHasher> map_v;


class MDP {
private:
    std::vector<State> states;
    std::vector<Action> actions;
    std::unordered_map<std::string, std::string> currentEnv;
    double discountFactor;
    bool trained;
    map_v jointStates;

    map_v init_value_func();

    State* getState(const std::string& stateName);

    Action* getAction(const std::string& actionName);

    double getTransitionProbability(const Action& action, const std::string& stateName, const std::vector<std::string>& valueNextValue);

    std::unordered_map<std::string, std::string> transition(const Action& action);

    double reward(const std::unordered_map<std::string, std::string>& env, const Action& action, const std::unordered_map<std::string, std::string>& changes);

    void valueIteration(int maxIterations = 100, double threshold = 0.001);

    std::string extractOptimalPolicy();

public:
    void solve(int maxIterations = 100, double threshold = 0.001, const std::string& algorithm = "value iteration");

    void next_steps(int n_steps = 1, bool display = true);

    MDP(std::vector<State> states, std::vector<Action> actions, std::unordered_map<std::string, std::string> currentEnv, double discountFactor)
        : states(states), actions(actions), currentEnv(currentEnv), discountFactor(discountFactor), trained(false) {}
};

#endif // MDP_H
