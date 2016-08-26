#include <iostream>
#include <stdio.h>
#include <math.h>

#include "gdastar.h"
#include "stlastar.h"


using namespace std;

class MapSearchNode {
public:
	int x; // the (x,y) positions of the node
	int y;

	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 
};

static AStarSearch<MapSearchNode> astarsearch;
static std::vector< std::pair<int, int> > _map;

static int GetMap(int x, int y) {
	if (std::find(_map.begin(), _map.end(), std::make_pair(x, y)) != _map.end()) {
		return 1;
	}

	return 9;
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs ) {
	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) && (y == rhs.y) ) {
		return true;
	} else {
		return false;
	}
}

void MapSearchNode::PrintNodeInfo() {
	char str[100];
	sprintf( str, "Node position : (%d,%d)\n", x,y );

	cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal) {
	return fabsf(float(x - nodeGoal.x)) + fabsf(float(y - nodeGoal.y));
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal) {
	return (x == nodeGoal.x) && (y == nodeGoal.y);
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node) {
	int parent_x = -1; 
	int parent_y = -1; 

	if (parent_node) {
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if ((GetMap( x-1, y ) < 9) && !((parent_x == x-1) && (parent_y == y))) {
		NewNode = MapSearchNode(x-1, y);
		astarsearch->AddSuccessor(NewNode);
	}

	if( (GetMap( x, y-1 ) < 9) && !((parent_x == x) && (parent_y == y-1))) {
		NewNode = MapSearchNode(x, y-1);
		astarsearch->AddSuccessor(NewNode);
	}

	if( (GetMap( x+1, y ) < 9) && !((parent_x == x+1) && (parent_y == y))) {
		NewNode = MapSearchNode(x+1, y);
		astarsearch->AddSuccessor(NewNode);
	}

	if( (GetMap( x, y+1 ) < 9) && !((parent_x == x) && (parent_y == y+1))) {
		NewNode = MapSearchNode(x, y+1);
		astarsearch->AddSuccessor(NewNode);
	}

	return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost(MapSearchNode &successor) {
	return (float)GetMap(x, y);
}

void gdAstar::AddPoint(int x, int y) {
	_map.push_back(std::make_pair(x, y));
}

void gdAstar::ClearPoints() {
	_map.clear();
}

Vector2Array gdAstar::FindPath(int x0, int y0, int x1, int y1) {
	Vector2Array path;

	AStarSearch<MapSearchNode> astarsearch;

	// Create a start state
	MapSearchNode nodeStart;
	nodeStart.x = x0;
	nodeStart.y = y0;

	// Define the goal state
	MapSearchNode nodeEnd;
	nodeEnd.x = x1;
	nodeEnd.y = y1;

	// Set Start and goal states
	astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);

	unsigned int SearchState;
	unsigned int SearchSteps = 0;

	do {
		SearchState = astarsearch.SearchStep();

		SearchSteps++;
	} while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

	if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) {
		MapSearchNode *node = astarsearch.GetSolutionStart();

		for (;;) {
			node = astarsearch.GetSolutionNext();
			if (node == NULL) break;

			path.append(Vector2(node->x, node->y));
		}

		// Once you're done with the solution you can free the nodes up
		astarsearch.FreeSolutionNodes();
	}

	astarsearch.EnsureMemoryFreed();
	return path;
}

void gdAstar::_bind_methods() {
	ObjectTypeDB::bind_method("AddPoint", &gdAstar::AddPoint);
	ObjectTypeDB::bind_method("ClearPoints", &gdAstar::ClearPoints);
	ObjectTypeDB::bind_method("FindPath", &gdAstar::FindPath);
}

gdAstar::gdAstar() {
}

gdAstar::~gdAstar() {
}


