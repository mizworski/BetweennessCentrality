#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <map>
#include <stack>
#include <mutex>
#include <thread>
#include <future>

typedef std::set<int> vertices;
typedef std::vector<int> neighbours;
typedef std::map<int, neighbours> edges;
typedef std::map<int, double> results;

const int threads_number = 5;

void read_graph(vertices &actors, edges &neighbourhood);

void calculate_betweenness_centrality(const vertices &actors, const edges &neighbourhood);

void calculate_betweenness_centrality(const int actor,
                                      const vertices &actors,
                                      const edges &neighbourhood,
                                      results &bc);

void thread_do(std::deque<int> &vertices_to_process,
               const vertices &actors,
               const edges &neighbourhood,
               std::promise<results> &result,
               std::mutex &mutex);

void update_results(results &bc, results &result);

int main() {
    vertices actors;
    edges neighbourhood;

    /// Reading graph.
    read_graph(actors, neighbourhood);

    calculate_betweenness_centrality(actors, neighbourhood);

    return 0;
}

void calculate_betweenness_centrality(const vertices &actors, const edges &neighbourhood) {

    results bc;
    std::mutex mut;


    std::vector<std::promise<results>> promises(threads_number);

    std::vector<std::thread> threads;

    std::deque<int> vertices_to_process(actors.begin(), actors.end());


    for (int i = 0; i < threads_number; ++i) {
        threads.push_back(std::thread(
                [&vertices_to_process, &actors, &neighbourhood, &promises, i, &mut]
                {
                    thread_do(vertices_to_process, actors, neighbourhood, promises[i], mut);
                }));
    }

    for (auto actor : actors) {
        bc.insert(std::pair<int, int>(actor, 0.0));
    }

    for (unsigned int i = 0; i < threads.size(); ++i) {
        std::future<results> result_future = promises.at(i).get_future();
        results result = result_future.get();

        update_results(bc, result);

        threads[i].join();
    }

    for (auto actor : bc) {
        if (neighbourhood.find(actor.first) != neighbourhood.end()) {
            std::cout << actor.first << " " << actor.second << std::endl;
        }
    }

}

void update_results(results &bc, results &result) {
    for (auto res : result) {
        bc.at(res.first) += res.second;
    }
}

void thread_do(std::deque<int> &vertices_to_process,
               const vertices &actors,
               const edges &neighbourhood,
               std::promise<results> &result,
               std::mutex &mutex) {
    results bc;

    do {
        int actor;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (vertices_to_process.empty()) {
                break;
            }
            actor = vertices_to_process.front();
            vertices_to_process.pop_front();
        }

        calculate_betweenness_centrality(actor, actors, neighbourhood, bc);

    } while (true);

    result.set_value(bc);
}

void calculate_betweenness_centrality(const int actor,
                                      const vertices &actors,
                                      const edges &neighbourhood,
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

    sigma.at(actor) = 1;
    distance.at(actor) = 0;

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
                delta.at(v) += ((double) sigma.at(v) / sigma.at(w) ) * (1 + delta.at(w));
            }
        }

        if (w != actor) {
            if (bc.find(w) == bc.end()) {
                bc.insert(std::pair<int, int>(w, 0.0));
            }

             bc.at(w) += delta.at(w);
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