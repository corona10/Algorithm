//
//  main.cpp
//  pathfinding
//
//  Created by 나동희 on 2015. 10. 29..
//  Copyright © 2015년 나동희. All rights reserved.
//

#define map_size 10
#include <iostream>
#include <list>
#include <cmath>
#include <algorithm>

class Node;

typedef struct{
    int x;
    int y;
} position;


void print_map(int (*map)[map_size]);
void astar_path(int (*map)[map_size], position st, position end);
bool isInList(std::list<Node*>& list, Node* node);
bool hComp( const Node* lhs, const Node* rhs );
bool validNode(const Node* node);


int main(int argc, const char * argv[]) {
    // insert code here...
    
    std::cout<<"====== A* Pathfinding Demo ======"
             <<"0 : empty_road\n"
             <<"# : block on road \n"
             <<"+ : path finding route\n"
             <<"@ : start point        \n"
             <<"$ : destination point  \n"
             <<"======= Initial Map======\n";
    
    position start;
    position end;
    start.x = 0;
    start.y = 0;
    
    end.x = 9;
    end.y = 9;
    
    int map[map_size][map_size] = {
         {0, 0, 0, 0, 0, 0 ,0 ,0 ,0, 0}
        ,{0, 1, 1, 0, 0, 0, 1, 0, 0, 0}
        ,{1, 1, 0, 0, 0, 0, 1, 0, 0 ,0}
        ,{0, 1, 1, 1, 0, 1, 1, 1, 0, 0}
        ,{0, 0, 0, 0, 0, 0, 1 ,0 ,0, 0}
        ,{0, 0, 0, 0, 0, 0, 0, 1, 0, 0}
        ,{0, 0, 0, 0, 0, 0, 0, 1, 0, 0}
        ,{0, 0, 0, 0, 0, 0, 0, 1, 0, 0}
        ,{0, 0, 0, 0, 0, 0, 0, 1, 0, 0}
        ,{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    print_map(map);
    std::cout<<"====== Now A* Path Finding.. ======="<<std::endl;
    astar_path(map, start, end);
    print_map(map);
    std::cout<<"======= corona10@gmail.com ======"<<std::endl;
    return 0;
}

class Node
{
public:
    int x;
    int y;
    int h;
    int g;
    int f;
    
    Node* parent;
    
    Node(){}
    Node(int gx, int gy)
    :x(gx) , y(gy)
    {}
    
    ~Node(){}
    
    // Calculate Manhattan distance It's better than Euclidean distance
    inline int get_distance(Node* dst)
    {
        return  std::abs(this->x - dst->x) + std::abs(this->y - dst->y);
    }
    
     // Calculate 'F = G + H' by Manhattan distance
    int get_f(Node* start, Node* dst)
    {
        this->g = get_distance(start);
        this->h = get_distance(dst);
        this->f = this->g + this->h;
        return this->f;
    }
    
};

// list sorting by h value
bool hComp( const Node* lhs, const Node* rhs )
{
    return (*lhs).h > (*rhs).h;   //내림차순 정렬
};

// printing map
void print_map(int (*map)[map_size])
{
    for(int i =0; i < map_size; i++)
    {
        for(int j = 0; j < map_size; j++)
        {
            if(map[i][j] == 0)
                std::cout<<"0";
            else if(map[i][j] == 1)
                std::cout<<"#";
            else if(map[i][j] == 2)
                std::cout<<"@";
            else if(map[i][j] == 3)
                std::cout<<"$";
            else if(map[i][j] == 4)
                std::cout<<"+";
        }
        std::cout<<std::endl;
    }
}

// A* Path finding algorithm
void astar_path(int (*map)[map_size], position st, position end)
{
    map[st.x][st.y] =2;
    map[end.x][end.y] = 3;
    
    std::list<Node*> open_list;
    std::list<Node*> closed_list;
    std::list<Node*> route;
    
    Node* startNode = new Node(st.x, st.y);
    Node* endNode = new Node(end.x, end.y);
    
    startNode->get_f(startNode,endNode);
    endNode->get_f(startNode,endNode);
    open_list.push_back(startNode);
    
    Node* current = 0;
    
    while(!open_list.empty())
    {
        current = open_list.front();
        
        // Add Node for 6 direction
        Node *d1 = new Node(current->x -1 , current->y - 1),
        *d2 = new Node(current->x -1 , current->y   ),
        *d3 = new Node(current->x    , current->y - 1),
        *d4 = new Node(current->x    , current->y  +1 ),
        *d5 = new Node(current->x + 1, current->y    ),
        *d6 = new Node(current->x + 1, current->y +  1);
        
        // Calculate F value for Node
        d1->get_f(startNode, endNode);
        d2->get_f(startNode, endNode);
        d3->get_f(startNode, endNode);
        d4->get_f(startNode, endNode);
        d5->get_f(startNode, endNode);
        d6->get_f(startNode, endNode);
        
        if(validNode(d1) && !isInList(closed_list, d1) && map[d1->x][d1->y] == 0)
        {
            if(!isInList(open_list, d1))
            {
                d1->parent = current;
                open_list.push_back(d1);
            }else{
                if(current-> f >= d1->f)
                    d1->parent = current;;
            }
        }
        
        if(validNode(d2) && !isInList(closed_list, d2) && map[d2->x][d2->y] == 0)
        {
            if(!isInList(open_list, d2))
            {
                d2->parent = current;
                open_list.push_back(d2);
            }else{
                if(current-> f >= d2->f)
                    d2->parent = current;
            }
        }
        if(validNode(d3) && !isInList(closed_list, d3) && map[d3->x][d3->y] == 0)
        {
            if(!isInList(open_list, d3))
            {
                d3->parent = current;
                open_list.push_back(d3);
            }else{
                if(current-> f >= d3->f)
                    d3->parent = current;
            }
        }
        if(validNode(d4) && !isInList(closed_list, d4) && map[d4->x][d4->y] == 0)
        {
            if(!isInList(open_list, d4))
            {
                d4->parent = current;
                open_list.push_back(d4);
            }else{
                if(current-> f >= d4->f)
                    d4->parent = current;
            }
        }
        if(validNode(d5) && !isInList(closed_list, d5) && map[d5->x][d5->y] == 0)
        {
            if(!isInList(open_list, d5))
            {
                d5->parent = current;
                open_list.push_back(d5);
            }else{
                if(current-> f >= d5->f)
                    d5->parent = current;
            }
        }
        if(validNode(d6) && !isInList(closed_list, d6) && map[d6->x][d6->y] == 0)
        {
            if(!isInList(open_list, d6))
            {
                d6->parent = current;
                open_list.push_back(d6);
            }else{
                if(current-> f >= d6->f)
                    d6->parent = current;
            }
        }
        open_list.pop_front();
        closed_list.push_back(current);
        open_list.sort(hComp);
        
        route.push_back(current);
    }
    
    while(true)
    {
       
        if(current->x == startNode->x && current->y == startNode->y)
        {
            map[current->x][current->y] = 2;
            delete current;
            break;
        }
        
        map[current->x][current->y] = 4;
        Node* old = current;
        current = current->parent;
        delete  old;
    }
}

// Check Node is valid or not
bool validNode(const Node* node)
{
    if(node->x >= 0 && node->y >= 0 && node->x < map_size && node->y < map_size)
        return true;
    
    return false;
}

// Check Node is wheter in the list or not
bool isInList(std::list<Node*>& list, Node* node)
{
    bool result = false;
    
    for(auto iter = list.begin(); iter != list.end(); iter++)
    {
        if((*iter)->x == node->x && (*iter)->y == node->y)
        {
            return true;
        }
    }
    return result;
}

