#include "state.h"

State::State(std::string name, std::vector<std::string> values) : name(name), values(values) {}

bool State::operator==(const State& other) const {
    return name == other.name && values == other.values;
}

std::string State::toString() const {
    std::string result = "State name: " + name + "\nState values:\n";
    for (const std::string& value : values) {
        result += value + "\n";
    }
    return result;
}
