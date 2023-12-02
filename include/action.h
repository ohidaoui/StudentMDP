#ifndef ACTION_H
#define ACTION_H

#include <string>
#include <unordered_map>
#include <vector>

class Action {
public:
    std::string name;
    std::unordered_map<std::string, std::vector<std::vector<double>>> transitionMatrices;

public:
    Action(std::string name, std::unordered_map<std::string, std::vector<std::vector<double>>> transitionMatrices);

    bool operator==(const Action& other) const;

    std::string toString() const;
};

#endif // ACTION_H
