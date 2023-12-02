#include "mdp.h"

map_v MDP::init_value_func() {
    map_v joint_value_func;

    for (const auto& item1 : states[0].values) {
        for (const auto& item2 : states[1].values) {
            for (const auto& item3 : states[2].values) {
                joint_value_func[{item1, item2, item3}] = 0.0;
            }
        }
    }
    return joint_value_func;
}

State* MDP::getState(const std::string& stateName) {
    for (auto& state : this->states) {
        if (state.name == stateName) {
            return &state;
        }
    }
    return nullptr;
}

Action* MDP::getAction(const std::string& actionName) {
    for (auto& action : this->actions) {
        if (action.name == actionName) {
            return &action;
        }
    }
    return nullptr;
}

double MDP::getTransitionProbability(const Action& action, const std::string &stateName, const std::vector<std::string>& valueNextValue) {
    State* state = getState(stateName);
    if (stateName.empty() || action.transitionMatrices.find(stateName) == action.transitionMatrices.end()) {
        return 1.0 / state->values.size();
    }
    
    const std::string& val = valueNextValue.front();
    const std::string& nextVal = valueNextValue.back();
    const std::vector<std::vector<double>>& transitionMatrix = action.transitionMatrices.at(stateName);
    const auto valIt = std::find(state->values.begin(), state->values.end(), val);
    const auto nextValIt = std::find(state->values.begin(), state->values.end(), nextVal);
    if (valIt == state->values.end() || nextValIt == state->values.end()) {
        return 0.0;
    }
    int valIdx = std::distance(state->values.begin(), valIt);
    int nextValIdx = std::distance(state->values.begin(), nextValIt);
    return transitionMatrix[valIdx][nextValIdx];
}

std::unordered_map<std::string, std::string> MDP::transition(const Action& action) {
    std::unordered_map<std::string, std::string> nextEnv = currentEnv;
    for (const auto& entry : action.transitionMatrices) {
        const std::string& stateName = entry.first;
        const std::vector<std::vector<double>>& transitionMatrix = entry.second;
        State* state = getState(stateName);
        const auto valueIt = std::find(state->values.begin(), state->values.end(), currentEnv[stateName]);
        if (valueIt != state->values.end()) {
            int valueIdx = std::distance(state->values.begin(), valueIt);
            std::vector<double> probabilities = transitionMatrix[valueIdx];
            std::random_device rd;
            std::mt19937 gen(rd());
            std::discrete_distribution<> distribution(probabilities.begin(), probabilities.end());
            int nextValueIdx = distribution(gen);
            nextEnv[stateName] = state->values[nextValueIdx];
        }
    }
    return nextEnv;
}

double MDP::reward(const std::unordered_map<std::string, std::string>& env, const Action& action, const std::unordered_map<std::string, std::string>& changes) {
    std::unordered_map<std::string, std::string> nextEnv = env;
    for (const auto& entry : changes) {
        nextEnv[entry.first] = entry.second;
    }
double reward = 0.0;
    if (action.name == "Enroll Course") {
        if (env.at("Career Goal") == "Engineer") {
            if (nextEnv.at("Course") == "CS" || nextEnv.at("Course") == "Math") {
                reward += 10.0;
            } else {
                reward += 5.0;
            }
        } else if (env.at("Career Goal") == "Teacher" || env.at("Career Goal") == "Researcher") {
            if (nextEnv.at("Course") == "Physics" || nextEnv.at("Course") == "Chemistry" || nextEnv.at("Course") == "Math") {
                reward += 10.0;
            } else {
                reward += 5.0;
            }
        } else {
            if (nextEnv.at("Course") == "Languages") {
                reward += 10.0;
            } else {
                reward += 5.0;
            }
        }
        
        if (env.at("Course") == nextEnv.at("Course")) {
            reward -= 3.0;
        }
        
        if (nextEnv.at("GPA") == "F") {
            reward = 0.0;
        } else if (nextEnv.at("GPA") == "D") {
            reward = 2.0;
        }
    }
    
    if (action.name == "Seek Support") {
        if (env.at("GPA") == "F") {
            if (nextEnv.at("Support") == "Swaye3") {
                reward = (nextEnv.at("GPA") != "F") ? reward + 10.0 : reward + 2.0;
            } else if (nextEnv.at("Support") == "SG") {
                reward = (nextEnv.at("GPA") != "F") ? reward + 10.0 : reward + 4.0;
            } else {
                reward = (nextEnv.at("GPA") != "F") ? reward + 5.0 : reward;
            }
        }
        
        if (env.at("GPA") == "B" || env.at("GPA") == "A") {
            if (nextEnv.at("Support") == "SG") {
                reward = (nextEnv.at("GPA").compare(env.at("GPA")) <= 0) ? reward + 10.0 : reward + 2.0;
            } else {
                reward = (nextEnv.at("GPA").compare(env.at("GPA")) <= 0) ? reward + 5.0 : reward;
            }
        }
        
        if (env.at("GPA") == "C" || env.at("GPA") == "D") {
            if (nextEnv.at("Support") == "Swaye3" || nextEnv.at("Support") == "SG") {
                reward = (nextEnv.at("GPA").compare(env.at("GPA")) < 0) ? reward + 10.0 : reward + 3.0;
            } else {
                reward = (nextEnv.at("GPA").compare(env.at("GPA")) < 0) ? reward + 5.0 : reward;
            }
        }
        
        reward += 2.0;
    }
    return reward;
}

void MDP::valueIteration(int maxIterations, double threshold) {
    this->jointStates = init_value_func();
    
    int iteration = 0;
    while (iteration < maxIterations) {
        double delta = 0.0;
        for (const auto& jointStateValue : this->jointStates) {
            // const std::tuple<std::string, std::string, std::string>& jointState = jointStateValue.first;
            double v = jointStateValue.second;
            // std::unordered_map<std::string, std::string> changes = {{"Course", std::get<0>(jointState)}, {"GPA", std::get<1>(jointState)}, {"Support", std::get<2>(jointState)}};
            std::unordered_map<std::string, std::string> changes = {{"Course", jointStateValue.first[0]}, 
            {"GPA", jointStateValue.first[1]}, {"Support", jointStateValue.first[2]}};
            std::unordered_map<std::string, std::string> env = currentEnv;
            for (const auto& entry : changes) {
                env[entry.first] = entry.second;
            }
            std::vector<double> qValues;
            for (const Action& action : actions) {
                double qValue = 0.0;
                // getTransitionProbability(const Action& action, const std::string& stateName, std::vector<std::string>& valueNextValue)
                for (const auto& jointStateValuePrime : this->jointStates) {
                    // const std::tuple<std::string, std::string, std::string>& jointStatePrime = jointStateValuePrime.first;
                    double vPrime = jointStateValuePrime.second;
                    std::unordered_map<std::string, std::string> changes = {{"Course", jointStateValuePrime.first[0]}, 
                    {"GPA", jointStateValuePrime.first[1]}, {"Support", jointStateValuePrime.first[2]}};
                    double rewardValue = reward(env, action, changes);
                    const std::string& course_name("Course"); 
                    const std::string& gpa_name("GPA");
                    const std::string& support_name("Support");
                    double p0 = getTransitionProbability(action, course_name, {jointStateValue.first[0], jointStateValuePrime.first[0]});
                    double p1 = getTransitionProbability(action, gpa_name, {jointStateValue.first[1], jointStateValuePrime.first[1]});
                    double p2 = getTransitionProbability(action, support_name, {jointStateValue.first[2], jointStateValuePrime.first[2]});
                    double jointProbability = p0 * p1 * p2;
                    qValue += jointProbability * (rewardValue + discountFactor * vPrime);
                }
                qValues.push_back(qValue);
            }
            delta = std::max(delta, std::fabs(v - *std::max_element(qValues.begin(), qValues.end())));
            this->jointStates[jointStateValue.first] = *std::max_element(qValues.begin(), qValues.end());
        }
        if (delta < threshold) {
            break;
        }
        iteration++;
    }
    this->trained = true;
}

std::string MDP::extractOptimalPolicy() {
    std::vector<std::string> init_joint_state = {currentEnv.at("Course"), currentEnv.at("GPA"), currentEnv.at("Support")};
    std::string best_action;
    double best_action_value = -std::numeric_limits<double>::infinity();
    for (const Action& action : actions) {
        double action_value = 0.0;
        for (const auto& [joint_state_p, v_prime] : jointStates) {
            std::unordered_map<std::string, std::string> changes = {
                {"Course", joint_state_p[0]},
                {"GPA", joint_state_p[1]},
                {"Support", joint_state_p[2]}
            };
            double reward_val = reward(currentEnv, action, changes);
            double p0 = getTransitionProbability(action, "Course", {init_joint_state[0], joint_state_p[0]});
            double p1 = getTransitionProbability(action, "GPA", {init_joint_state[1], joint_state_p[1]});
            double p2 = getTransitionProbability(action, "Support", {init_joint_state[2], joint_state_p[2]});
            double joint_prob = p0 * p1 * p2;
            action_value += joint_prob * (reward_val + discountFactor * v_prime);
        }
        if (action_value > best_action_value) {
            best_action = action.name;
            best_action_value = action_value;
        }
    }
    return best_action;
}


void MDP::solve(int maxIterations, double threshold, const std::string& algorithm) {
    
    this->jointStates = init_value_func();
    
    if (algorithm == "value iteration") {
        valueIteration(maxIterations, threshold);
    }
    else {
        throw std::runtime_error("Algorithm not supported");
    }
    std::cout << "MDP Solved\n" << std::endl;
}

void MDP::next_steps(int n_steps, bool display) {
    for (int i = 0; i < n_steps; ++i) {
        // auto action_name = getBestAction();
        auto action_name = extractOptimalPolicy();
        Action* best_action = getAction(action_name);
        currentEnv = transition(*best_action);
        if (display) {
            std::cout << "Step " << i+1 << ": " << action_name << std::endl;
            std::cout << "Current environment:" << std::endl;
            for (const auto& pair : currentEnv) {
                std::cout << pair.first << ": " << pair.second << "   ";//std::endl;
            }
            std::cout << "\n\n";
        }
    }
}