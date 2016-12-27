#include <iostream>
#include <vector>
#include <set>
#include <sstream>
#include <map>
#include <stack>
#include <mutex>
#include <thread>
#include <future>
#include <stdlib.h>
#include <fstream>
#include "brandes.h"

unsigned int additional_threads_number;

void single_thread_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc);

int main(int argc, char *argv[]) {
    vertices actors;
    edges neighbourhood;
    results bc;

    /// Setting initial conditions.
    additional_threads_number = std::atoi(argv[1]) - 1;
    std::string input_filename(argv[2]);
    std::string output_filename(argv[3]);

    /// Reading graph.
    read_graph(actors, neighbourhood, input_filename);

    /// Running algorithm.
    if (additional_threads_number > 0) {
        multi_threaded_betweenness_centrality(actors, neighbourhood, bc);
    } else {
        single_thread_betweenness_centrality(actors, neighbourhood, bc);
    }

    /// Writing results.
    write_results(neighbourhood, bc, output_filename);

    return 0;
}

void write_results(edges &neighbourhood, results &results, std::string &output_filename) {
    std::ofstream output_file(output_filename);

    for (auto actor : results) {
        /// Writes only if there was edge coming out of vertice.
        if (neighbourhood.find(actor.first) != neighbourhood.end()) {
            output_file << actor.first << " " << actor.second << std::endl;
        }
    }
}

void single_thread_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc) {
    /// Initialize results data structure.
    for (auto actor : actors) {
        bc.insert(std::pair<int, int>(actor, 0.0));
    }

    /// Calculates algorithm.
    for (auto actor : actors) {
        calculate_betweenness_centrality(actor, actors, neighbourhood, bc);
    }
}

void multi_threaded_betweenness_centrality(vertices &actors, edges &neighbourhood, results &bc) {
    std::mutex mut;
    std::vector<std::promise<results>> promises(additional_threads_number);
    std::vector<std::thread> threads;
    std::deque<int> vertices_to_process(actors.begin(), actors.end());

    /// Launches additional threads to run algorithm.
    for (int i = 0; i < additional_threads_number; ++i) {
        threads.push_back(std::thread(
                [&vertices_to_process, &actors, &neighbourhood, &promises, i, &mut] {
                    thread_do(vertices_to_process, actors, neighbourhood, promises[i], mut);
                }));
    }

    /// Initialize data structure with results.
    for (auto actor : actors) {
        bc.insert(std::pair<int, int>(actor, 0.0));
    }

    /// Waiting for results from each thread.
    for (unsigned int i = 0; i < threads.size(); ++i) {
        std::future<results> result_future = promises.at(i).get_future();
        results result = result_future.get();

        update_results(bc, result);
        threads[i].join();
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
                /// No more vertices to process.
                break;
            }
            actor = vertices_to_process.front();
            vertices_to_process.pop_front();
        }

        calculate_betweenness_centrality(actor, actors, neighbourhood, bc);

    } while (true);
    /// Saves results and terminates.
    result.set_value(bc);
}

void calculate_betweenness_centrality(const int actor,
                                      const vertices &actors,
                                      const edges &neighbourhood,
                                      results &bc) {
    std::stack<int> stack;

    std::map<int, int> sigma;
    std::map<int, int> distance;
    std::map<int, double> delta;
    std::map<int, std::vector<int>> predecessors;

    /// Initialization of additional structures.
    for (auto vertice : actors) {
        sigma.insert(std::pair<int, int>(vertice, 0));
        distance.insert(std::pair<int, int>(vertice, -1));
        delta.insert(std::pair<int, double>(vertice, 0.0));
    }

    /// Brandes algorithm for specified vertice.
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
                delta.at(v) += ((double) sigma.at(v) / sigma.at(w)) * (1 + delta.at(w));
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

void read_graph(vertices &actors, edges &neighbourhood, std::string &filename) {
    std::string line;

    std::ifstream input_file(filename);

    while (std::getline(input_file, line)) {
        std::istringstream iss(line);
        int v;
        int w;

        iss >> v;
        iss >> w;

        actors.insert(v);
        actors.insert(w);

        if (neighbourhood.find(v) == neighbourhood.end()) {
            neighbourhood.insert(std::pair<int, neighbours>(v, {}));
        }

        neighbourhood.at(v).push_back(w);
    }

}