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
#include <algorithm>
#include "brandes.h"

unsigned int additional_threads_number;

void single_thread_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc);

int main(int argc, char *argv[]) {
    vertices actors;
    edges neighbourhood;

    /// Setting initial conditions.
    additional_threads_number = std::atoi(argv[1]) - 1;
    std::string input_filename(argv[2]);
    std::string output_filename(argv[3]);

    /// Reading graph.
    read_graph(actors, neighbourhood, input_filename);

    results bc(actors.size(), 0.0);
    /// Running algorithm.
    if (additional_threads_number > 0) {
        multi_threaded_betweenness_centrality(actors, neighbourhood, bc);
    } else {
        single_thread_betweenness_centrality(actors, neighbourhood, bc);
    }

    /// Writing results.
    write_results(actors, neighbourhood, bc, output_filename);

    return 0;
}

void single_thread_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc) {
    for (unsigned long i = 0; i < actors.size(); ++i) {
        calculate_betweenness_centrality(i, actors, neighbourhood, bc);
    }
}

void multi_threaded_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc) {
    std::mutex mut;
    std::vector<std::promise<results>> promises(additional_threads_number);
    std::vector<std::thread> threads;
    unsigned long vertices_to_process = actors.size();
    /// Launches additional threads to run algorithm.
    for (int i = 0; i < additional_threads_number; ++i) {
        threads.push_back(std::thread(
                [&vertices_to_process, &actors, &neighbourhood, &promises, i, &mut] {
                    thread_do(vertices_to_process, actors, neighbourhood, promises[i], mut);
                }));
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
    for (unsigned long i = 0; i < bc.size(); ++i) {
        bc.at(i) += result.at(i);
    }
}

void thread_do(unsigned long &vertices_to_process,
               const vertices &actors,
               const edges &neighbourhood,
               std::promise<results> &result,
               std::mutex &mutex) {
    results bc(actors.size(), 0.0);

    do {
        unsigned long actor;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (vertices_to_process == 0) {
                /// No more vertices to process.
                break;
            }
            actor = --vertices_to_process;
        }

        calculate_betweenness_centrality(actor, actors, neighbourhood, bc);

    } while (true);
    /// Saves results and terminates.
    result.set_value(bc);
}

void calculate_betweenness_centrality(const unsigned long actor,
                                      const vertices &actors,
                                      const edges &neighbourhood,
                                      results &bc) {
    std::stack<unsigned long> stack;

    std::vector<int> sigma(actors.size(), 0);
    std::vector<int> distance(actors.size(), -1);
    std::vector<double> delta(actors.size(), 0.0);
    std::vector<std::vector<unsigned long>> predecessors(actors.size());

    /// Brandes algorithm for specified vertice.
    sigma.at(actor) = 1;
    distance.at(actor) = 0;

    std::deque<unsigned long> queue;
    queue.push_back(actor);

    while (!queue.empty()) {
        unsigned long vertex = queue.front();
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

                    predecessors.at(neighbour).push_back(vertex);
                }
            }
        }
    }

    while (!stack.empty()) {
        unsigned long w = stack.top();
        stack.pop();

        for (auto v : predecessors.at(w)) {
            delta.at(v) += ((double) sigma.at(v) / sigma.at(w)) * (1 + delta.at(w));
        }

        if (w != actor) {
            bc.at(w) += delta.at(w);
        }
    }
}

void read_graph(vertices &actors, edges &neighbourhood, const std::string &filename) {
    std::string line;
    unsigned long count = 0;

    std::ifstream input_file(filename);

    while (std::getline(input_file, line)) {
        std::istringstream iss(line);
        unsigned long v;
        unsigned long w;

        iss >> v;
        iss >> w;

        if (actors.find(v) == actors.end()) {
            actors.insert(std::pair<unsigned long, unsigned long>(v, count++));
        }

        if (actors.find(w) == actors.end()) {
            actors.insert(std::pair<unsigned long, unsigned long>(w, count++));
        }

        if (neighbourhood.find(actors.at(v)) == neighbourhood.end()) {
            neighbourhood.insert(std::pair<int, neighbours>(actors.at(v), {}));
        }

        neighbourhood.at(actors.at(v)).push_back(actors.at(w));
    }

}

void write_results(const vertices &actors, const edges &neighbourhood, const results &results,
                   const std::string &output_filename) {
    std::ofstream output_file(output_filename);

    std::vector<std::pair<unsigned long, double>> sorted_results;

    for (auto actor : actors) {
        if (neighbourhood.find(actor.second) != neighbourhood.end()) {
            sorted_results.push_back(std::pair<unsigned long, double>(actor.first, results.at(actor.second)));
        }
    }

    std::sort(sorted_results.begin(), sorted_results.end());

    for (auto result : sorted_results) {
        output_file << result.first << " " << result.second << std::endl;
    }
}