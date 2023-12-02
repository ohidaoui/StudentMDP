#include "action.h"

Action::Action(std::string name, std::unordered_map<std::string, std::vector<std::vector<double>>> transitionMatrices)
    : name(name), transitionMatrices(transitionMatrices) {}

bool Action::operator==(const Action& other) const {
    return name == other.name && transitionMatrices == other.transitionMatrices;
}

std::string Action::toString() const {
    std::string result = "Action name: " + name + "\nAction Transition matrices:\n";
    for (const auto& entry : transitionMatrices) {
        const std::string& stateName = entry.first;
        const std::vector<std::vector<double>>& transitionMatrix = entry.second;

        result += "State name: " + stateName + "\nTransition matrix:\n";
        for (const std::vector<double>& row : transitionMatrix) {
            for (double value : row) {
                result += std::to_string(value) + " ";
            }
            result += "\n";
        }
        result += "\n";
    }
    return result;
}
