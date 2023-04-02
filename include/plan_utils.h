#include <iostream>
#include <queue>
#include <stdlib.h>

typedef std::vector<std::vector<float>> floatGrid;
typedef std::vector<std::vector<int>> intGrid;

using namespace std;
#pragma once
#define FMT_HEADER_ONLY
#include "format.h"
namespace std
{
    using fmt::format;
    using fmt::format_error;
    using fmt::formatter;
}


struct Node{
    int x, y;
    double cost;
};

struct PixelWithCost{
    int row, col;
    double cost;
    bool operator==(const PixelWithCost& p) const
    {
        return (row == p.row && col == p.col);
    }
    void print()
    {
        std::cout << "Pixel = [ " << row << ", " << col << ", " << cost << " ]" << std::endl;
    }
};

void printNode(Node n){
    std::cout << "Node: {" << n.x << 
    ", " << n.y << ", " << n.cost << "}" << std::endl;
}

void printPixel(PixelWithCost p){
    std::cout << "Pixel: {" << p.row <<
    ", " << p.col << ", " << p.cost << "}" << std::endl;
}

class Compare{
    public:
    bool operator() (Node n1, Node n2){
        if (n1.cost >= n2.cost){
            return true;
        }
        return false;
    }
};
class ComparePixel{
    public:
    bool operator() (PixelWithCost p1, PixelWithCost p2){
        if (p1.cost > p2.cost){
            return true;
        }
        return false;
    }
};

// Given a grid for the image, we want a cost grid from the heuristic
floatGrid getCostGrid(floatGrid grid, PixelWithCost goal)
{
    floatGrid cost_grid;
    cost_grid.resize(grid.size(), std::vector<float>(grid[0].size()));
    // visited.resize(grid.size(), std::vector<int>(grid[0].size()));
    for(int row = 0; row < grid.size(); ++row){
        for(int col = 0; col < grid[row].size(); ++col){
            // dist from goal
            float heuristic_cost = std::abs(goal.row - row) + std::abs(goal.col - col);
            cost_grid[row][col] = grid[row][col] + heuristic_cost;
            // visited[row][col] = 0;
        }
    }
    return cost_grid;
}

std::vector<PixelWithCost> validNeighbors(PixelWithCost p, const intGrid& visited, 
                                          const floatGrid& cost_grid, const floatGrid& grid){
    int rows = cost_grid.size();
    int cols = cost_grid[0].size();
    std::vector<PixelWithCost> neighbors;
    int offset_x[] = {0, 1, 0, -1, 1, -1, 1, -1};
    int offset_y[] = {1, 0, -1, 0, 1, 1, -1, -1};
    // 8 potential neighbors
    int i;
    // std::cout << std::format("Expanding pixel ({}, {}) with grid value = {}", p.row, p.col, grid[p.row][p.col]) << std::endl;
    for(i = 0; i < 8; ++i){
        int nrow = p.row + offset_x[i];
        int ncol = p.col + offset_y[i];
        if(nrow < 0 || nrow >= rows){
            std::cout << "( " << nrow << ", " << ncol << " ) is out of bounds row-wise" << std::endl;
            continue;
        }
        if(ncol < 0 || ncol >= cols){
            std::cout << "( " << nrow << ", " << ncol << " ) is out of bounds col-wise" << std::endl;
            continue;
        }
        if(grid[nrow][ncol] == 255.0){
            // std::cout << "( " << nrow << ", " << ncol << " ) is in collision" << std::endl;
            continue;
        }
        if(visited[nrow][ncol] == 1){
            continue;
        }
        neighbors.push_back({nrow, ncol, cost_grid[nrow][ncol]});
    }
    return neighbors;
}

bool aStarSearch(std::vector<PixelWithCost>& path, PixelWithCost start, PixelWithCost goal, 
                 const floatGrid& cost_grid, const floatGrid& grid)
{

    if(grid[goal.row][goal.col] == 255.0){
        std::cerr << "Goal is in obstacle!" << std::endl;
        return false;
    }
    if(grid[start.row][start.col] == 255.0){
        std::cerr << "Start is in obstacle!" << std::endl;
    }
    intGrid visited;
    visited.resize(grid.size(), std::vector<int>(grid[0].size()));
    for(int i = 0; i < visited.size(); ++i){
        for(int j = 0; j < visited[i].size(); ++j){
        visited[i][j] = 0;
        }
    }
    priority_queue<PixelWithCost, std::vector<PixelWithCost>, ComparePixel> pq;
    pq.push(start);
    int i = 0;
    while(!pq.empty()){
        // Get the pixel with least "cost"
        PixelWithCost current_pixel = pq.top();
        pq.pop();
        current_pixel.print();
        // Append to path
        // std::cout << "Updating path! "<< std::endl;
        path.push_back(current_pixel);
        // Updated visited
        // std::cout << "Updating visited!" << std::endl;
        visited[current_pixel.row][current_pixel.col] = 1;
        if(current_pixel == goal){
            // Reached the goal!
            std::cout << "Reached the goal!" << std::endl;
            return true;
        }
        // Else explore valid neighbors of the current pixel
        // std::cout << "Checking for valid neighbors!" << std::endl;
        std::vector<PixelWithCost> neighbors = validNeighbors(current_pixel, visited, cost_grid, grid);
        // std::cout << "Exploring neighbors!" << std::endl;
        for(auto pixel : neighbors){
            // std::cout << std::format("row = {}, col = {}, visited = {}", pixel.row, pixel.col, visited[pixel.row][pixel.col]) << std::endl;
            if(visited[pixel.row][pixel.col] == 1){
                // if already explored or been added to the heap, skip it (dont add duplicate nodes)
                continue;
            }
            std::cout << "Pushing pixel onto heap" << std::endl;          
            pq.push(pixel); // heapify happens
            visited[pixel.row][pixel.col] = 1;
        }
        i++;
    }
    std::cerr << "Could not find a path to goal!" << std::endl;
    return false;
}