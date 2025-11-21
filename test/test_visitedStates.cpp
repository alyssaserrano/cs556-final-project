#include <iostream>
#include <set>
#include <vector>

// Function prototypes
void visitedStates(int x, int y);
bool isTraversable(int x, int y);
void logWaypoint(int x, int y);


//------------------------ MAP DEFINITION --------------------------//
// Goal positions: visit in order, last goal is (2,1)
std::set<std::pair<int, int>> goalPositions = {
    {6,2},    // Goal 1
    {8,0},    // Goal 2
    {2,1}     // Last goal, where we turn and head back
};

// Goal tracking
std::set<std::pair<int, int>> reachedGoals;
bool allGoalsVisited = false;
bool returnedToDock = false;

#define DOCK_X 0
#define DOCK_Y 0

#define NUM_X 9
#define NUM_Y 4

int visitArray[NUM_X][NUM_Y] = {0};

// ----------- Testable functions for simulation run -----------//

bool atDockingStation(int x, int y) {
    return x == DOCK_X && y == DOCK_Y;
}

bool atGoalLocation(int x, int y) {
    return goalPositions.count(std::make_pair(x, y)) > 0;
}

bool isTraversable(int x, int y) {
    if ((x == 2 && y == 0) || (x == 3 && y == 0)) return false; // Walls
    return x >= 0 && x < NUM_X && y >= 0 && y < NUM_Y;
}

void visitedStates(int x, int y) {
    if (isTraversable(x, y)) {
        visitArray[x][y] += 1;
        logWaypoint(x, y);
    }
    if (atDockingStation(x, y)) {
        std::cout << "At docking station (" << x << "," << y << ")\n";
    }
    if (atGoalLocation(x, y)) {
        std::cout << "Reached goal at (" << x << "," << y << ")\n";
        if (!reachedGoals.count(std::make_pair(x, y))) {
            reachedGoals.insert(std::make_pair(x, y));
            if (reachedGoals.size() == goalPositions.size()) {
                allGoalsVisited = true;
                std::cout << "All goals visited!\n";
            }
        }
    }
}

void logWaypoint(int x, int y) {
    std::cout << "Visited cell (" << x << "," << y << ")\n";
}

// -------- HARDCODED PATH (filled from your previous message) --------
const std::vector<std::pair<int, int>> fullPath = {
    {0,0}, {0,1}, {0,2}, {0,3},
    {1,3}, {2,3}, {3,3}, {4,3}, {5,3}, {5,2}, {5,1}, {6,1}, {6,2}, // First goal (6,2)
    {6,3}, {7,3}, {8,3}, {8,2}, {8,1}, {8,0}, // Second goal (8,0)
    {7,0}, {7,1}, {7,2}, {7,1}, {7,0}, {6,0}, {5,0}, {4,0}, {4,1}, {4,2}, {3,2}, {2,2}, {1,2}, {1,1}, {1,0}, {1,1}, {2,1}  // Last goal (2,1)
};


// --- Traverse in reverse and mark cells ---
// Question: Do we want to count EVERY time we visit the cell on the map?
void traverseReversePath(const std::vector<std::pair<int,int>>& path) {
    std::cout << "\nSimulating return with same path in reverse:\n";
    for (int i = path.size()-2; i >= 0; --i) { // -2: skip actual final endpoint
        int x = path[i].first, y = path[i].second;
        if (isTraversable(x, y)) {
            visitArray[x][y] += 1;
            logWaypoint(x, y);
        }
        if (atDockingStation(x, y) && !returnedToDock) {
            returnedToDock = true;
            std::cout << "Returned to docking station (" << x << "," << y << ") after visiting all goals!\n";
        }
    }
}

// -------------- MAIN --------------
int main() {
    // Traverse the hardcoded path "to goals"
    for (const auto& cell : fullPath) {
        visitedStates(cell.first, cell.second);
    }
    // Once all goals reached, return via the same path backwards
    if(allGoalsVisited) traverseReversePath(fullPath);

    // Print visitArray
    std::cout << "\n-- Visit Array --\n";
    for (int y = 0; y < NUM_Y; ++y) {
        for (int x = 0; x < NUM_X; ++x) {
            std::cout << visitArray[x][y] << " ";
        }
        std::cout << "\n";
    }
    return 0;
}