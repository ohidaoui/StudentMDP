#ifndef STATE_H
#define STATE_H

#include <string>
#include <vector>

class State {
public:
    std::string name;
    std::vector<std::string> values;

public:
    State(std::string name, std::vector<std::string> values);

    bool operator==(const State& other) const;

    std::string toString() const;
};

#endif // STATE_H
