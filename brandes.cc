#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <map>
#include <stack>

typedef std::set<int> vertices;
typedef std::vector<int> neighbours;
typedef std::map<int, neighbours> edges;
typedef std::map<int, double> results;

void read_graph(vertices &actors, edges &neighbourhood);

void print_edges(edges &neighbourhoods);

void calculate_betweenness_centrality(vertices &actors, edges &neighbourhood);

void calculate_betweenness_centrality(int actor,
                                      vertices &actors,
                                      edges &neighbourhood,
                                      results &bc);

int main() {
    vertices actors;
    edges neighbourhood;

    /// Reading graph.
    read_graph(actors, neighbourhood);

    calculate_betweenness_centrality(actors, neighbourhood);

//    print_edges(neighbourhood);

    return 0;
}

void calculate_betweenness_centrality(vertices &actors, edges &neighbourhood) {

    results bc;

    for (auto actor : actors) {
        bc.insert(std::pair<int, int>(actor, 0.0));
    }

    for (auto actor : actors) {
        calculate_betweenness_centrality(actor, actors, neighbourhood, bc);
    }

    for (auto actor : bc) {
        if (neighbourhood.find(actor.first) != neighbourhood.end()) {
            std::cout << actor.first << " " << actor.second << std::endl;
        }
    }
}

void calculate_betweenness_centrality(int actor,
                                      vertices &actors,
                                      edges &neighbourhood,
                                      results &bc) {
    std::stack<int> stack;

    /// Initialization of additional structures.
    std::map<int, int> sigma;
    std::map<int, int> distance;
    std::map<int, double> delta;
    std::map<int, std::vector<int>> predecessors;

    for (auto vertice : actors) {
        sigma.insert(std::pair<int, int>(vertice, 0));
        distance.insert(std::pair<int, int>(vertice, -1));
        delta.insert(std::pair<int, double>(vertice, 0.0));
    }


    sigma[actor] = 1;
    distance[actor] = 0;

    std::deque<int> queue;
    queue.push_back(actor);

    while (!queue.empty()) {
        int vertex = queue.front();
        queue.pop_front();

        stack.push(vertex);

        if (neighbourhood.find(vertex) != neighbourhood.end()) {
            for (auto neighbour : neighbourhood.at(vertex)) {
                if (distance.at(neighbour) < 0) {
                    queue.push_back(neighbour);
                    distance.at(neighbour) = distance.at(vertex) + 1;
                }

                if (distance.at(neighbour) == distance.at(vertex) + 1) {
                    sigma.at(neighbour) += sigma.at(vertex);

                    if (predecessors.find(neighbour) == predecessors.end()) {
                        predecessors.insert(std::pair<int, std::vector<int>>(neighbour, {}));
                    }

                    predecessors.at(neighbour).push_back(vertex);
                }
            }
        }
    }

    while (!stack.empty()) {
        int w = stack.top();
        stack.pop();

        if (predecessors.find(w) != predecessors.end()) {
            for (auto v : predecessors.at(w)) {
                delta[v] += (sigma[v] / sigma[w])*(1 + delta[w]);
            }
        }

        if (w != actor) {
            bc[w] += delta[w];
        }
    }
}

void read_graph(vertices &actors, edges &neighbourhood) {
    std::string line;

    while (std::getline(std::cin, line)) {
        std::istringstream iss(line);
        int v;
        int w;

        iss >> v;
        iss >> w;

        //todo temp
        if (v == -1) break;

        actors.insert(v);
        actors.insert(w);

        if (neighbourhood.find(v) == neighbourhood.end()) {
            neighbourhood.insert(std::pair<int, neighbours>(v, {}));
        }

        neighbourhood.at(v).push_back(w);
    }

}