/*

Copyright (c) 2015, Luigi Palmieri, Social Robotics Laboratory

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

*/

#pragma once

// used for text debugging
#include <iostream>
#include <stdio.h>
#include <assert.h>

// stl includes
#include <algorithm>
#include <set>
#include <vector>
#include <cfloat>


#include <unistd.h>

using namespace std;


#define INF_COST 1000000

// disable warning that debugging information has lines that are truncated
// occurs in stl headers
#pragma warning( disable : 4786 )

template<class T>
class ThetaStarState;

// UserState is the users state space type
template<class UserState>
class ThetaStarSearch
{
public: // data

    enum
    {
        SEARCH_STATE_NOT_INITIALISED,
        SEARCH_STATE_SEARCHING,
        SEARCH_STATE_SUCCEEDED,
        SEARCH_STATE_FAILED,
        SEARCH_STATE_OUT_OF_MEMORY,
        SEARCH_STATE_INVALID
    };


    bool m_euclideanCost;
    bool m_useAstar;
    bool m_connectGrandParent;
    // A node represents a possible state in the search
    // The user provided state type is included inside this type

public:

    class Node
    {
    public:

        Node *parent; // used during the search to record the parent of successor nodes
        Node *child; // used after the search for the application to view the search in reverse
        double yaw;
        float g; // cost of this node + it's predecessors
        float h; // heuristic estimate of distance to goal
        float f; // sum of cumulative cost of predecessors and self and heuristic

        Node() : parent(0), child(0), yaw(0.0f), g(0.0f), h(0.0f), f(0.0f)
        {
        }

        UserState m_UserState;
    };


    // For sorting the heap the STL needs compare function that lets us compare
    // the f value of two nodes

    class HeapCompare_f
    {
    public:

        bool operator()(const Node *x, const Node *y) const
        {
            if (x->f > y->f)
                return true;

            if (x->f < y->f)
                return false;

            // We break ties among vertices with the same
            // f-values in favor of larger g-values
            if (x->f == y->f)
            {
                return x->g > y->g;
            }

            return false;
        }
    };

public: // methods

    // constructor just initialises private data
    ThetaStarSearch() : m_AllocateNodeCount(0), m_State(SEARCH_STATE_NOT_INITIALISED), m_CurrentSolutionNode(NULL),
                        m_CancelRequest(false)
    {
        m_useAstar = false;
        m_connectGrandParent = false;
    }

    // constructor just initialises private data
    ThetaStarSearch(bool euclideanCostChoice) : m_AllocateNodeCount(0), m_State(SEARCH_STATE_NOT_INITIALISED),
                                                m_CurrentSolutionNode(NULL), m_CancelRequest(false)
    {
        m_euclideanCost = euclideanCostChoice;
        m_useAstar = false;
        m_connectGrandParent = false;
    }

    ThetaStarSearch(int MaxNodes) : m_AllocateNodeCount(0), m_State(SEARCH_STATE_NOT_INITIALISED),
                                    m_CurrentSolutionNode(NULL), m_CancelRequest(false)
    {
        m_useAstar = false;
        m_connectGrandParent = false;
    }

    // set Astar Search
    void useAstar()
    {
        m_useAstar = true;
    }

    void use_connectGrandParent()
    {
        m_connectGrandParent = true;
    }

    // call at any time to cancel the search and free up all the memory
    void CancelSearch()
    {
        m_CancelRequest = true;
    }

    // Set Start and goal states
    void SetStartAndGoalStates(UserState &Start, UserState &Goal)
    {
        m_CancelRequest = false;

        m_Start = AllocateNode();
        m_Goal = AllocateNode();

        assert((m_Start != NULL && m_Goal != NULL));

        m_Start->m_UserState = Start;
        m_Goal->m_UserState = Goal;

        m_State = SEARCH_STATE_SEARCHING;

        // Initialise the AStar specific parts of the Start Node
        // The user only needs fill out the state information

        m_Start->g = 0;
        m_Start->h = m_Start->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);
        m_Start->f = m_Start->g + m_Start->h;
        m_Start->parent = m_Start; // fix it.

        // Push the start node on the Open list

        m_OpenList.push_back(m_Start); // heap now unsorted

        // Sort back element into heap
        push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());

        // Initialise counter for search steps
        m_Steps = 0;
    }

    // Advances search one step
    unsigned int SearchStep()
    {
        // Firstly break if the user has not initialised the search
        assert((m_State > SEARCH_STATE_NOT_INITIALISED) && (m_State < SEARCH_STATE_INVALID));

        // Next I want it to be safe to do a searchstep once the search has succeeded...
        if ((m_State == SEARCH_STATE_SUCCEEDED) || (m_State == SEARCH_STATE_FAILED))
        {
            return m_State;
        }

        // Failure is defined as emptying the open list as there is nothing left to
        // search...
        // New: Allow user abort
        if (m_OpenList.empty() || m_CancelRequest)
        {
            FreeAllNodes();
            m_State = SEARCH_STATE_FAILED;
            return m_State;
        }

        // Incremement step count
        m_Steps++;

        // Pop the best node (the one with the lowest f)
        Node *n = m_OpenList.front(); // get pointer to the node
        pop_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
        m_OpenList.pop_back();

        // Check for the goal, once we pop that we're done
        if (n->m_UserState.IsGoal(m_Goal->m_UserState))
        {
            // The user is going to use the Goal Node he passed in
            // so copy the parent pointer of n
            m_Goal->parent = n->parent;
            m_Goal->g = n->g;

            OMPL_DEBUG("Setting Goal orientation..");

            if (!(n->m_UserState.lineofsight(&(n->parent->m_UserState), &((m_Goal)->m_UserState))))
            {
                (m_Goal)->m_UserState.setOrientation(&(n->parent->m_UserState));
            }

            OMPL_DEBUG("Setting Goal orientation done");
            m_Goal->m_UserState.steer = n->m_UserState.steer;


            m_Goal->m_UserState.costs = m_Goal->m_UserState.getLineCost();

            // A special case is that the goal was passed in as the start state
            // so handle that here

            if (!n->m_UserState.IsSameState(m_Start->m_UserState))
            {
                FreeNode(n);

                // set the child pointers in each node (except Goal which has no child)
                Node *nodeChild = m_Goal;
                Node *nodeParent = m_Goal->parent;

                do
                {
                    nodeParent->child = nodeChild;
                    nodeChild = nodeParent;
                    nodeParent = nodeParent->parent;

#ifdef DEBUG
                    nodeParent->m_UserState.PrintNodeInfo();
                    nodeChild->m_UserState.PrintNodeInfo();
#endif

                    if (nodeParent == nodeChild && nodeChild != m_Start)
                    {
                        usleep(200);
#ifdef DEBUG
                        nodeParent->m_UserState.PrintNodeInfo();
                        nodeChild->m_UserState.PrintNodeInfo();
#endif
                        std::exit(EXIT_FAILURE);
                    }
                }
                while (nodeChild != m_Start && nodeChild != NULL); // Start is always the first node by definition
            }

            // delete nodes that aren't needed for the solution
            FreeUnusedNodes();
            m_State = SEARCH_STATE_SUCCEEDED;

            return m_State;
        }
        else // not goal
        {
            typename vector<Node *>::iterator closedlist_result;
            typename vector<Node *>::iterator openlist_result;

            m_ClosedList.push_back(n);

            // We now need to generate the successors of this node
            // The user helps us to do this, and we keep the new nodes in
            // m_Successors ...

            m_Successors.clear(); // empty vector of successor nodes to n

            // User provides this functions and uses AddSuccessor to add each successor of
            // node 'n' to m_Successors

            bool ret = n->m_UserState.GetSuccessors(this, n->parent ? &n->parent->m_UserState : NULL);

            if (!ret)
            {
                typename vector<Node *>::iterator successor;

                // free the nodes that may previously have been added
                for (successor = m_Successors.begin(); successor != m_Successors.end(); successor++)
                {
                    FreeNode((*successor));
                }

                m_Successors.clear(); // empty vector of successor nodes to n

                // free up everything else we allocated
                FreeAllNodes();

                m_State = SEARCH_STATE_OUT_OF_MEMORY;
                return m_State;
            }


            // Now handle each successor to the current node ...
            for (typename vector<Node *>::iterator successor = m_Successors.begin();
                 successor != m_Successors.end(); successor++)
            {
                /// if successor is not in closed
                for (closedlist_result = m_ClosedList.begin();
                     closedlist_result != m_ClosedList.end(); closedlist_result++)
                {
                    if ((*closedlist_result)->m_UserState.IsSameState((*successor)->m_UserState))
                    {
                        break;
                    }
                }

                if (closedlist_result != m_ClosedList.end())
                {
                    // we found this state on closed
                    // consider the next successor now
                    continue;
                }

                /// if successor is not in open set cost to inf
                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((*successor)->m_UserState))
                    {
                        break;
                    }
                }

                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                }
                else
                {
                    // State not in open list
                    // Set its g value to inf
                    (*successor)->g = INF_COST;
                    (*successor)->parent = NULL;
                }

                if (!m_connectGrandParent)
                    UpdateVertex(n, *successor);
                else
                    UpdateVertexGrandParent(n, *successor);
            }
        }

        return m_State; // Succeeded bool is false at this point.
    }

    bool UpdateVertex(Node *n, Node *successor)
    {
        typename vector<Node *>::iterator closedlist_result;
        typename vector<Node *>::iterator openlist_result;

        double tcost = 0;

        if (n->parent != NULL && n->m_UserState.lineofsight(&(n->parent->m_UserState), &((successor)->m_UserState)) &&
            !m_useAstar)
        {
            if (m_euclideanCost)
                tcost = n->parent->g + n->parent->m_UserState.GetCost((successor)->m_UserState);
            else
                tcost = n->parent->g + n->parent->m_UserState.GetCostTrajFromParent((n->parent->m_UserState),
                                                                                    (successor)->m_UserState);
            // tcost=n->parent->g+n->parent->m_UserState.GetCostTraj((successor)->m_UserState );

            if (tcost < (successor)->g)
            {
                if (n->parent == n && n->parent != m_Start)
                {
#ifdef DEBUG
                    n->parent->m_UserState.PrintNodeInfo();
                    n->m_UserState.PrintNodeInfo();
                    #endif
#ifdef DEBUG
                    (successor)->m_UserState.PrintNodeInfo();
                    #endif
                    cout << endl;
                }

                // Update cost and Parent
                double h_succ = (successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);

                (successor)->parent = n->parent;
                (successor)->yaw = (successor)->m_UserState.theta;
                (successor)->g = (float) tcost;
                (successor)->h = (float) h_succ;
                (successor)->f = (float) (tcost + h_succ);
                (successor)->m_UserState.setType(1);

                // check if successor is in open list
                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((successor)->m_UserState))
                    {
                        break;
                    }
                }

                // if in open list remove it
                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                    FreeNode((*openlist_result));
                    m_OpenList.erase(openlist_result);
                    make_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
                }

                // heap now unsorted
                m_OpenList.push_back((successor));

                // sort back element into heap
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
            }
        }
        else
        {
            // A* Case
            float newg = 0;

            if (m_euclideanCost)
                newg = n->g + n->m_UserState.GetCost((successor)->m_UserState);
            else
                newg = n->g + n->m_UserState.GetCostTraj((successor)->m_UserState);

            if (newg < (successor)->g)
            {
                // This node is the best node so far with this particular state
                // so lets keep it and set up its AStar specific data ...
                if (n->parent == n && n->parent != m_Start)
                {
#ifdef DEBUG
                    n->parent->m_UserState.PrintNodeInfo();
                    n->m_UserState.PrintNodeInfo();
                    #endif
#ifdef DEBUG
                    (successor)->m_UserState.PrintNodeInfo();
                    #endif
                    cout << endl;
                }

                double h_succ = (successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);
                (successor)->parent = n;
                (successor)->g = newg;
                (successor)->h = (float) h_succ;
                (successor)->f = (float) (newg + h_succ);

                (successor)->yaw = (successor)->m_UserState.theta;
                (successor)->m_UserState.costs = (successor)->m_UserState.getLineCost();
                (successor)->m_UserState.setType(1);


                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((successor)->m_UserState))
                    {
                        break;
                    }
                }

                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                    FreeNode((*openlist_result));
                    m_OpenList.erase(openlist_result);
                    make_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
                }

                // heap now unsorted
                m_OpenList.push_back((successor));

                // sort back element into heap
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
            }
        }

        return true;
    }

    bool UpdateVertexGrandParent(Node *n, Node *successor)
    {
        typename vector<Node *>::iterator closedlist_result;
        typename vector<Node *>::iterator openlist_result;

        double tcost = 0;

        if (n->parent->parent != NULL &&
            n->m_UserState.lineofsight(&(n->parent->parent->m_UserState), &((successor)->m_UserState)) && !m_useAstar)
        {
            if (m_euclideanCost)
                tcost = n->parent->parent->g + n->parent->parent->m_UserState.GetCost((successor)->m_UserState);
            else
                tcost = n->parent->parent->g +
                        n->parent->parent->m_UserState.GetCostTrajFromParent((n->parent->parent->m_UserState),
                                                                             (successor)->m_UserState);

            if (tcost < (successor)->g)
            {
                if (n->parent->parent == n && n->parent->parent != m_Start)
                {

#ifdef DEBUG
                    n->parent->parent->m_UserState.PrintNodeInfo();
#endif

#ifdef DEBUG
                    n->m_UserState.PrintNodeInfo();
                    (successor)->m_UserState.PrintNodeInfo();
                    #endif
                    cout << endl;
                }

                // Update cost and Parent
                double h_succ = (successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);

                (successor)->parent = n->parent->parent;
                (successor)->yaw = (successor)->m_UserState.theta;
                (successor)->g = (float) tcost;
                (successor)->h = (float) h_succ;
                (successor)->f = (float) (tcost + h_succ);
                (successor)->m_UserState.setType(1);


                // check if successor is in open list
                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((successor)->m_UserState))
                    {
                        break;
                    }
                }

                // if in open list remove it
                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                    FreeNode((*openlist_result));
                    m_OpenList.erase(openlist_result);
                    make_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
                }

                // heap now unsorted
                m_OpenList.push_back((successor));

                // sort back element into heap
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
            }
        }
        else if (n->parent != NULL &&
                 n->m_UserState.lineofsight(&(n->parent->m_UserState), &((successor)->m_UserState)) && !m_useAstar)
        {
            if (m_euclideanCost)
                tcost = n->parent->g + n->parent->m_UserState.GetCost((successor)->m_UserState);
            else
                tcost = n->parent->g + n->parent->m_UserState.GetCostTrajFromParent((n->parent->m_UserState),
                                                                                    (successor)->m_UserState);
            // tcost=n->parent->g+n->parent->m_UserState.GetCostTraj((successor)->m_UserState );

            if (tcost < (successor)->g)
            {
                if (n->parent == n && n->parent != m_Start)
                {
#ifdef DEBUG
                    n->parent->m_UserState.PrintNodeInfo();
                    n->m_UserState.PrintNodeInfo();
                    #endif
#ifdef DEBUG
                    (successor)->m_UserState.PrintNodeInfo();
                    #endif
                    cout << endl;
                }

                // Update cost and Parent
                double h_succ = (successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);

                (successor)->parent = n->parent;
                (successor)->yaw = (successor)->m_UserState.theta;
                (successor)->g = (float) tcost;
                (successor)->h = (float) h_succ;
                (successor)->f = (float) (tcost + h_succ);
                (successor)->m_UserState.setType(1);


                // check if successor is in open list
                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((successor)->m_UserState))
                    {
                        break;
                    }
                }

                // if in open list remove it
                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                    FreeNode((*openlist_result));
                    m_OpenList.erase(openlist_result);
                    make_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
                }

                // heap now unsorted
                m_OpenList.push_back((successor));

                // sort back element into heap
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
            }
        }
        else
        {
            // A* Case

            float newg = 0;

            if (m_euclideanCost)
                newg = n->g + n->m_UserState.GetCost((successor)->m_UserState);
            else
                newg = n->g + n->m_UserState.GetCostTraj((successor)->m_UserState);


            if (newg < (successor)->g)
            {
                // This node is the best node so far with this particular state
                // so lets keep it and set up its AStar specific data ...
                if (n->parent == n && n->parent != m_Start)
                {
#ifdef DEBUG
                    n->parent->m_UserState.PrintNodeInfo();
                    n->m_UserState.PrintNodeInfo();
                    #endif
#ifdef DEBUG
                    (successor)->m_UserState.PrintNodeInfo();
                    #endif
                    cout << endl;
                }

                double h_succ = (successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);
                (successor)->parent = n;
                (successor)->g = newg;
                (successor)->h = (float) h_succ;
                (successor)->f = (float) (newg + h_succ);

                (successor)->yaw = (successor)->m_UserState.theta;
                (successor)->m_UserState.costs = (successor)->m_UserState.getLineCost();
                (successor)->m_UserState.setType(1);


                for (openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++)
                {
                    if ((*openlist_result)->m_UserState.IsSameState((successor)->m_UserState))
                    {
                        break;
                    }
                }

                if (openlist_result != m_OpenList.end())
                {
                    // we found this state on open
                    FreeNode((*openlist_result));
                    m_OpenList.erase(openlist_result);
                    make_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
                }

                // heap now unsorted
                m_OpenList.push_back((successor));

                // sort back element into heap
                push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());

            }

        }

        return true;

    }

    // User calls this to add a successor to a list of successors
    // when expanding the search frontier
    bool AddSuccessor(UserState &State)
    {
        Node *node = AllocateNode();

        if (node)
        {
            node->m_UserState = State;
            node->yaw = State.theta;
            m_Successors.push_back(node);

            return true;
        }

        return false;
    }

    // Free the solution nodes
    // This is done to clean up all used Node memory when you are done with the
    // search
    void FreeSolutionNodes()
    {
        Node *n = m_Start;

        if (m_Start->child)
        {
            do
            {
                Node *del = n;
                n = n->child;
                FreeNode(del);
            }
            while (n != m_Goal);

            FreeNode(n); // Delete the goal
        }
        else
        {
            // if the start node is the solution we need to just delete the start and goal
            // nodes
            FreeNode(m_Start);
            FreeNode(m_Goal);
        }
    }

    // Functions for traversing the solution

    // Get start node
    UserState *GetSolutionStart()
    {
        m_CurrentSolutionNode = m_Start;
        if (m_Start)
        {
            return &m_Start->m_UserState;
        }
        else
        {
            return NULL;
        }
    }

    // Get next node
    UserState *GetSolutionNext()
    {
        if (m_CurrentSolutionNode)
        {
            if (m_CurrentSolutionNode->child)
            {
                Node *child = m_CurrentSolutionNode->child;
                m_CurrentSolutionNode = m_CurrentSolutionNode->child;
                return &child->m_UserState;
            }
        }

        return NULL;
    }

    // Get end node
    UserState *GetSolutionEnd()
    {
        m_CurrentSolutionNode = m_Goal;
        if (m_Goal)
        {
            return &m_Goal->m_UserState;
        }
        else
        {
            return NULL;
        }
    }

    // Step solution iterator backwards
    UserState *GetSolutionPrev()
    {
        if (m_CurrentSolutionNode)
        {
            if (m_CurrentSolutionNode->parent)
            {
                Node *parent = m_CurrentSolutionNode->parent;
                m_CurrentSolutionNode = m_CurrentSolutionNode->parent;
                return &parent->m_UserState;
            }
        }

        return NULL;
    }

    // Get final cost of solution
    // Returns FLT_MAX if goal is not defined or there is no solution
    float GetSolutionCost()
    {
        if (m_Goal && m_State == SEARCH_STATE_SUCCEEDED)
        {
            return m_Goal->g;
        }
        else
        {
            return FLT_MAX;
        }
    }

    // For educational use and debugging it is useful to be able to view
    // the open and closed list at each step, here are two functions to allow that.

    UserState *GetOpenListStart()
    {
        float f, g, h;
        return GetOpenListStart(f, g, h);
    }

    UserState *GetOpenListStart(float &f, float &g, float &h)
    {
        iterDbgOpen = m_OpenList.begin();
        if (iterDbgOpen != m_OpenList.end())
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return &(*iterDbgOpen)->m_UserState;
        }

        return NULL;
    }

    UserState *GetOpenListNext()
    {
        float f, g, h;
        return GetOpenListNext(f, g, h);
    }

    UserState *GetOpenListNext(float &f, float &g, float &h)
    {
        iterDbgOpen++;
        if (iterDbgOpen != m_OpenList.end())
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return &(*iterDbgOpen)->m_UserState;
        }

        return NULL;
    }

    Node *GetCurrentBestRawNode()
    {
        return m_OpenList.front();
    }

    UserState *GetClosedListStart()
    {
        float f, g, h;
        return GetClosedListStart(f, g, h);
    }

    UserState *GetClosedListStart(float &f, float &g, float &h)
    {
        iterDbgClosed = m_ClosedList.begin();
        if (iterDbgClosed != m_ClosedList.end())
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;

            return &(*iterDbgClosed)->m_UserState;
        }

        return NULL;
    }

    UserState *GetClosedListNext()
    {
        float f, g, h;
        return GetClosedListNext(f, g, h);
    }

    UserState *GetClosedListNext(float &f, float &g, float &h)
    {
        iterDbgClosed++;
        if (iterDbgClosed != m_ClosedList.end())
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;

            return &(*iterDbgClosed)->m_UserState;
        }

        return NULL;
    }

    // Get the number of steps

    int GetStepCount()
    {
        return m_Steps;
    }

    void EnsureMemoryFreed()
    {
        if (m_AllocateNodeCount == 0)
            std::cout << "memory clean" << std::endl;

    }

private:

    // delete all nodes
    void FreeAllNodes()
    {
        typename vector<Node *>::iterator iterOpen = m_OpenList.begin();

        while (iterOpen != m_OpenList.end())
        {
            Node *n = (*iterOpen);
            FreeNode(n);

            iterOpen++;
        }

        m_OpenList.clear();

        typename vector<Node *>::iterator iterClosed;

        for (iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed++)
        {
            Node *n = (*iterClosed);
            FreeNode(n);
        }

        m_ClosedList.clear();

        // delete the goal

        FreeNode(m_Goal);
    }

    void FreeUnusedNodes()
    {
        // iterate open list and delete unused nodes
        typename vector<Node *>::iterator iterOpen = m_OpenList.begin();

        while (iterOpen != m_OpenList.end())
        {
            Node *n = (*iterOpen);

            if (!n->child)
            {
                FreeNode(n);
            }

            iterOpen++;
        }

        m_OpenList.clear();

        // iterate closed list and delete unused nodes
        typename vector<Node *>::iterator iterClosed;

        for (iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed++)
        {
            Node *n = (*iterClosed);

            if (!n->child)
            {
                FreeNode(n);
            }
        }

        m_ClosedList.clear();
    }

    // Node memory management
    Node *AllocateNode()
    {
        m_AllocateNodeCount++;
        Node *p = new Node;
        return p;
    }

    void FreeNode(Node *node)
    {
        m_AllocateNodeCount--;
        delete node;
    }

private:

    // Heap using std:vector
    vector<Node *> m_OpenList;

    // Closed list is a std::vector.
    vector<Node *> m_ClosedList;

    // Contains the successors of the current node evaluated during the search
    vector<Node *> m_Successors;

    // Status of the State
    unsigned int m_State;

    // Counts steps
    int m_Steps;

    // Start and goal state pointers
    Node *m_Start;
    Node *m_Goal;

    Node *m_CurrentSolutionNode;


    // Two iterators that keep debug info
    typename vector<Node *>::iterator iterDbgOpen;
    typename vector<Node *>::iterator iterDbgClosed;

    // debugging : count memory allocation and free's
    int m_AllocateNodeCount;

    bool m_CancelRequest;
};


/// Template Class to use as state during the Search
template<class T>
class ThetaStarState
{
public:
    virtual ~ThetaStarState()
    {}

    virtual float
    GoalDistanceEstimate(T &nodeGoal) = 0; // Heuristic function which computes the estimated cost to the goal node
    virtual bool IsGoal(T &nodeGoal) = 0; // Returns true if this node is the goal node
    virtual bool GetSuccessors(ThetaStarSearch<T> *thetastarsearch,
                               T *parent_node) = 0; // Retrieves all successors to this node and adds them via thetastarsearch.addSuccessor()
    virtual float GetCost(T &successor) = 0; // Computes the cost of travelling from this node to the successor node
    virtual bool IsSameState(T &rhs) = 0; // Returns true if this node is the same as the rhs node
    virtual bool lineofsight(T *successor, T *parent_node)=0;

    int OPTM_ORIENTATIONS;
};
