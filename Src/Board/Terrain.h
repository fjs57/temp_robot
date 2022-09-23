#ifndef TERRAIN_H
#define TERRAIN_H

#include <DataTypes.h>
#include <vector>
#include <set>
#include <thread>
#include <mutex>
#include <string.h>
#include <cmath>

#include "Odometry.h"

#include "Log.h"
#include "config.h"

namespace terrain {

	typedef enum {
		Success,
		PathImpossible,
		TargetReached,
		NotInitialized,
	} TerrainReturnValue;

	typedef struct {
		int x;
		int y;
	} GridCoordinates;

	typedef struct _GridPoint{
		bool possible;
		float f, g, h;
		Position pos;
		GridCoordinates coord;
		struct _GridPoint *parent;
	} GridPoint;

	typedef struct _PathChainLink{
		Position *self;
		struct _PathChainLink *next;
	} PathChainLink;

}

using namespace terrain;

class Terrain
{
private:
    std::vector<Cluster> obstacles;
    std::mutex obstacles_mutex;

    Position target;
    Position robot;
    std::mutex positions_mutex;

    std::vector<Position> path;
    std::mutex path_mutex;

    std::vector<Position> temp_path;
	std::mutex temp_path_mutex;

    std::thread process_thread;
    TerrainReturnValue process_state;
    bool process_running;
    std::mutex process_mutex;
    int downsampling_factor;
    int downsampling_counter;

    bool enabled;

    float x_max, x_min, y_max, y_min;
    int grid_size_x, grid_size_y;

    Terrain(){
    	process_running = false;
    	process_state = NotInitialized;
    	enabled = false;

    	downsampling_factor = 10;
    	downsampling_counter = 0;


    	x_max = FIELD_BOUND_X_MAX;
    	x_min = FIELD_BOUND_X_MIN;
    	y_max = FIELD_BOUND_Y_MAX;
    	y_min = FIELD_BOUND_Y_MIN;

    	grid_size_x = (int)((x_max - x_min) / PATH_PLANNER_GRID_MESH_SIZE);
    	grid_size_y = (int)((y_max - y_min) / PATH_PLANNER_GRID_MESH_SIZE);

    }

    ~Terrain(){
    	obstacles_mutex.unlock();
    	positions_mutex.unlock();
    	process_mutex.unlock();
    	path_mutex.unlock();
    	temp_path_mutex.unlock();
    	if(process_thread.joinable()) process_thread.join();
    }

    void _enable(void){
    	enabled = true;
    }

    void _disable(void){
    	enabled = false;
    }

    void _setObstacles(std::vector<Cluster> newObstacles){
    	obstacles_mutex.lock();
    	obstacles = newObstacles;
    	obstacles_mutex.unlock();
    }

    void _setTarget(Position newTarget){
    	positions_mutex.lock();
    	memcpy(&target, &newTarget, sizeof(Position));
    	positions_mutex.unlock();
    }

    void _updateRobotPosition(void){
    	positions_mutex.lock();
    	Odometry::getCurrentPosition(&robot);
    	positions_mutex.unlock();
    }

    void _computePath(void){

    	if(!enabled) return;

    	if(downsampling_counter++ >= downsampling_factor){
    		downsampling_counter=0;
    	} else return;

    	bool cond;

    	process_mutex.lock();

    	cond=process_running;
    	if(!cond) process_running = true;

    	process_mutex.unlock();

    	if(cond) return;
    	if(process_thread.joinable()) process_thread.join();

    	process_thread = std::thread([this](){
    		this->process();
    	});

    }

    void process(void){

    	_updateRobotPosition();

    	std::vector<std::vector<GridPoint>> grid(grid_size_x, std::vector<GridPoint> (grid_size_y, {true,0,0,0,{0,0,0},{0,0},nullptr} ));
    	GridPoint *start, *current, *temp, *neighbour;
    	float best_f, tentative_g;
    	bool path_found = false;



    	// fill the grid
    	for(int x=0; x<grid_size_x; x++){
    		for(int y=0; y<grid_size_y; y++){

    			GridPoint gp;
    			gp.f=100.0f;
    			gp.g=100.0f;
    			gp.pos.x = x * PATH_PLANNER_GRID_MESH_SIZE - x_max;
    			gp.pos.y = y * PATH_PLANNER_GRID_MESH_SIZE - y_max;

    			gp.possible = isPositionPossible(gp.pos);

    			gp.parent = nullptr;
    			gp.coord.x = x;
    			gp.coord.y = y;

    			heuristic(&gp);

    			memcpy(&(grid[x][y]), &gp, sizeof(GridPoint));

    			if(isStart(gp.pos)){
    				start = &(grid[x][y]);
    			}
			}
    	}

    	start->g=0;

    	std::set<GridPoint*> open;
    	std::set<GridPoint*> closed;

    	open.insert(start);


    	while(!open.empty()){


    		best_f=-1;
    		for(std::set<GridPoint*>::iterator it=open.begin(); it!=open.end(); it++){
    			if(best_f==-1 || (*it)->f<best_f){
    				best_f=(*it)->f;
    				current = (*it);
    			}
    		}

    		if(isDestination(current->pos)){

    			// reconstruct the path.
    			temp_path_mutex.lock();
    			temp_path.clear();

    			temp = current;
    			while(temp->parent != nullptr){
    				temp_path.insert(temp_path.begin(), temp->pos);
    				temp = temp->parent;
    			}

    			positions_mutex.lock();
    			temp_path.push_back(target);

    			temp_path.insert(temp_path.begin(), robot);

    			positions_mutex.unlock();
    			temp_path_mutex.unlock();

    			process_state = Success;
    			LOG_TRACE("Path Success");
    			path_found = true;
    			break;
    		}
    		// delete the current node from the open set
    		open.erase(current);

    		// search for the neighbours
    		for(int dx=-1; dx<=1; dx++){
    			for(int dy=-1; dy<=1; dy++){

    				if(dx==0 && dy==0) continue;
    				if(current->coord.x+dx < 0) continue;
    				if(current->coord.y+dy < 0) continue;
    				if(current->coord.x+dx >= grid_size_x) continue;
    				if(current->coord.y+dy >= grid_size_y) continue;

    				neighbour = &(grid[current->coord.x+dx][current->coord.y+dy]);



    				if(!neighbour->possible) continue;



    				tentative_g = current->g + distance(current, neighbour);

    				if(tentative_g < neighbour->g){



    					neighbour->parent = current;
    					neighbour->g = tentative_g;
    					neighbour->f = tentative_g + neighbour->h;
    					open.insert(neighbour);
    				}

    			}
    		}
    	}

    	if(!path_found){
			// Path Impossible
			LOG_TRACE("Path Impossible");
			process_state = PathImpossible;
    	}else {
	    	reducePath();
			updatePath();
    	}


    	process_mutex.lock();
    	process_running = false;
    	process_mutex.unlock();

    }

    float distance(GridPoint *a, GridPoint *b){
    	return std::sqrt(
    		(a->coord.x - b->coord.x)*(a->coord.x - b->coord.x) + (a->coord.y - b->coord.y)*(a->coord.y - b->coord.y)
    	);
    }

    void heuristic(GridPoint *p){
    	float d;


    	positions_mutex.lock();


    	d = std::sqrt(
    		(p->pos.x - target.x)*(p->pos.x - target.x) + (p->pos.y - target.y)*(p->pos.y - target.y)
    	);

    	positions_mutex.unlock();


    	p->h = d;
    }

    bool isDestination(Position pos){
    	float dist;

    	positions_mutex.lock();
    	dist = pos.x - target.x;
    	positions_mutex.unlock();

    	if (dist<0) dist*=-1;
    	if(dist>PATH_PLANNER_GRID_MESH_SIZE/2) return false;


    	positions_mutex.lock();

    	dist = pos.y - target.y;
    	positions_mutex.unlock();


		if (dist<0) dist*=-1;
		if(dist>PATH_PLANNER_GRID_MESH_SIZE/2) return false;

		return true;
    }

    bool isStart(Position pos){
    	float dist;


    	positions_mutex.lock();

    	dist = pos.x - robot.x;
    	positions_mutex.unlock();


    	if (dist<0) dist*=-1;
    	if(dist>PATH_PLANNER_GRID_MESH_SIZE/2) return false;


    	positions_mutex.lock();

    	dist = pos.y - robot.y;
    	positions_mutex.unlock();


		if (dist<0) dist*=-1;
		if(dist>PATH_PLANNER_GRID_MESH_SIZE/2) return false;

		return true;
    }

    bool _isProcessRunning(void){
    	return process_running;
    }

    bool isPositionPossibleTerrain(Position pos){
    	if(pos.x < FIELD_BOUND_X_MAX - PATH_PLANNER_ROBOT_RADIUS) return false;
    	if(pos.y < FIELD_BOUND_Y_MAX - PATH_PLANNER_ROBOT_RADIUS) return false;
    	if(pos.x > FIELD_BOUND_X_MIN + PATH_PLANNER_ROBOT_RADIUS) return false;
    	if(pos.x > FIELD_BOUND_Y_MIN + PATH_PLANNER_ROBOT_RADIUS) return false;
    	return true;
    }

    bool isPositionPossible(Position a){
		bool ret = true;
		float dist_sq;
		Cluster *p;

		obstacles_mutex.lock();


		do{
			if(isPositionPossibleTerrain(a)) { ret=false; break; }

			for(int i=0; i<(int)obstacles.size(); i++){

				p = &(obstacles[i]);

				dist_sq = (a.x - p->x)*(a.x - p->x)+(a.y - p->y)*(a.y - p->y);

				if(std::sqrt(dist_sq) < PATH_PLANNER_PROTECTION_AROUND_OBSTACLES){
					ret = false;
					break;
				}

			}

		}while(false);

		obstacles_mutex.unlock();
		return ret;
	}

    bool isVertexPossible(Position a, Position b){
    	bool ret = true;
    	float dist;
    	Cluster *p;
    	float x0, x1, x2, y0, y1, y2, norm_v, bh, xh, yh, xv, yv, x_max, x_min, y_max, y_min;


    	obstacles_mutex.lock();


    	do{

			for(int i=0; i<(int)obstacles.size(); i++){

				p = &(obstacles[i]);

				x0 = p->x;
				y0 = p->y;

				x1 = a.x;
				y1 = a.y;

				x2 = b.x;
				y2 = b.y;

				dist = std::abs(
						(x2 - x1) * (y1 - y0) - (x1 - x0)*(y2 - y1)
				) / std::sqrt(
						(x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)
				);

				if(dist < PATH_PLANNER_PROTECTION_AROUND_OBSTACLES){

					xv = x2 - x1;
					yv = y2 - y1;
					norm_v = std::sqrt(xv*xv+yv*yv);

					bh = ((x0 - x2)*xv + (y0 - y2)*yv)/norm_v;

					xh = x2 + bh / norm_v * xv;
					yh = y2 + bh / norm_v * yv;

					x_max = std::max(x1, x2);
					x_min = std::min(x1, x2);
					y_max = std::max(y1, y2);
					y_min = std::min(y1, y2);

					if(
							(x_min <= xh) && (xh <= x_max) &&
							(y_min <= yh) && (yh <= y_max)
					){
						ret = false;
						break;
					}

				}
			}

    	}while(false);

    	obstacles_mutex.unlock();
    	return ret;
    }

    void reducePath(void){

    	LOG_DEBUG("start reducePath");

    	std::vector<Position> oldPath, newPath;


    	temp_path_mutex.lock();
    	oldPath = temp_path;
    	temp_path_mutex.unlock();


    	PathChainLink links[oldPath.size()];
    	PathChainLink *ptr;

    	for(int i=0; i<(int)oldPath.size(); i++){
    		links[i].self = &(oldPath[i]);
    		if(i<(int)oldPath.size()-1){
    			links[i].next = &(links[i+1]);
    		} else {
    			links[i].next = nullptr;
    		}
    	}

    	ptr = &(links[0]);
    	newPath.push_back(*(ptr->self));
    	do {

			while(ptr->next->next != nullptr){
				if(isVertexPossible(*(ptr->self), *(ptr->next->next->self))){
					ptr->next = ptr->next->next;
				} else {
					break;
				}
			}

			ptr = ptr->next;
			newPath.push_back(*(ptr->self));
    	} while(ptr->next != nullptr);


//    	bool operation;
//    	ptr = &(links[0]);
//    	do {
//    		operation = false;
//    		while(ptr->next->next != nullptr) {
//				if(isVertexPossible(*(ptr->self), *(ptr->next->next->self))){
//					ptr->next = ptr->next->next;
//					operation = true;
//				}
//				ptr = ptr->next;
//			};
//    	} while(operation);
//
//    	ptr = &(links[0]);
//    	do{
//    		newPath.push_back(*(ptr->self));
//    		ptr = ptr->next;
//    	} while(ptr->next != nullptr);


    	temp_path_mutex.lock();
    	temp_path = newPath;
    	temp_path_mutex.unlock();

    	LOG_DEBUG("end reducePath");
    }

    int _printPath(char* msg){

    	int len;
    	len = sprintf(msg, ";Q");

    	path_mutex.lock();

    	for(int i=0; i<(int)path.size();i++){
    		len += sprintf(&(msg[len]), ";%.2f,%.2f", path[i].x, path[i].y);
    	}
    	path_mutex.unlock();

    	return len;
    }

    void updatePath(void){
    	LOG_DEBUG("start updatePath");
    	path_mutex.lock();
    	temp_path_mutex.lock();

    	path = temp_path;

    	path_mutex.unlock();
		temp_path_mutex.unlock();
    	LOG_DEBUG("end updatePath");

    }

public:

    static Terrain& getInstance(void){
    	static Terrain terrain;
    	return terrain;
    }

    static void enable(void){
    	getInstance()._enable();
    }

    static void setObstacles(std::vector<Cluster> newObstacles){
    	getInstance()._setObstacles(newObstacles);
    }

    static void setTarget(Position newTarget){
    	getInstance()._setTarget(newTarget);
    }

    static void computePath(void){
    	getInstance()._computePath();
    }

    static bool isProcessRunning(void){
    	return getInstance()._isProcessRunning();
    }

    static bool isPathReady(void){
    	return false;
    }

    static int printPath(char* msg){
    	return getInstance()._printPath(msg);
    }

};




#endif
