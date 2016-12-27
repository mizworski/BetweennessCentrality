//
// Created by michal on 12/25/16.
//

#ifndef PW2_BRANDES_H
#define PW2_BRANDES_H

typedef std::set<int> vertices;
typedef std::vector<int> neighbours;
typedef std::map<int, neighbours> edges;
typedef std::map<int, double> results;

void write_results(edges &neighbourhood, results &results, std::string &output_filename);

void read_graph(vertices &actors, edges &neighbourhood, std::string &filename);

void multi_threaded_betweenness_centrality(vertices &actors, edges &neighbourhood, results &bc);

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

#endif //PW2_BRANDES_H
