#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTState.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isValid(const ob::State *state)
{
    return true;
}

void plan()
{
    // construct state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    // set the bounds for the planning space
    auto bounds = ob::RealVectorBounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);

    space->setBounds(bounds);

    OMPL_INFORM("Setting bounds %f %f", bounds.low[0], bounds.high[0]);

    // construct an instance of space information
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set the StateValidityChecker
    si->setStateValidityChecker(isValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = 0;
    start[1] = 0;

    OMPL_INFORM("Starting state: %f %f", start[0], start[1]);

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 1;
    goal[1] = 2;

    OMPL_INFORM("Goal state: %f %f", goal[0], goal[1]);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states for the problem
    pdef->setStartAndGoalStates(start, goal);

    // create an planner
    auto planner = std::make_shared<og::RRTState>(si);
    // auto planner = std::make_shared<og::RRT>(si);

    
    // set the problem and planner for the planner
    planner->setProblemDefinition(pdef);

    OMPL_INFORM("Constructed planner");

    // perform setup steps for the planner
    planner->setup();

    OMPL_INFORM("Completed setup!");

    // print the setting for this space 
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    OMPL_INFORM("Starting planning");

    // setup the trajectory generator
    std::vector<double> maxV = {1, 1};
    std::vector<double> maxA = {1, 1};
    std::vector<double> maxJ = {1, 1};
    planner->setupGenerator(maxV, maxA, maxJ);

    // attempt to solve the problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(1);


    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        OMPL_INFORM("\n arrive here! \n");
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
        auto pd = ob::PlannerData(si);
        planner->getPlannerData(pd);
        

        // get all data from the planner
        for (int i = 0; i < pd.numVertices(); ++i)
        {
            auto state = pd.getVertex(i).getState();
            double *value = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            // std::cout << "Vertex " << i << ": " << value[0] << ", " << value[1] << std::endl;
        }
        double a = 0;
    }
    else
        std::cout << "No solution found" << std::endl;
}


int main()
{
    OMPL_INFORM("Starting planning");
    plan();
    OMPL_INFORM("Planning complete");
    return 0;
}