//
// Created by michal on 12/25/16.
//

#ifndef PW2_BRANDES_H
#define PW2_BRANDES_H

typedef std::map<unsigned long, unsigned long> vertices;
typedef std::vector<unsigned long> neighbours;
typedef std::map<unsigned long, neighbours> edges;
typedef std::vector<double> results;

void write_results(const vertices &actors, const edges &neighbourhood, const results &results, const std::string &output_filename);

void read_graph(vertices &actors, edges &neighbourhood, const std::string &filename);

void multi_threaded_betweenness_centrality(const vertices &actors, const edges &neighbourhood, results &bc);

void calculate_betweenness_centrality(const unsigned long actor,
                                      const vertices &actors,
                                      const edges &neighbourhood,
                                      results &bc);

void thread_do(unsigned long &vertices_to_process,
               const vertices &actors,
               const edges &neighbourhood,
               std::promise<results> &result,
               std::mutex &mutex);

void update_results(results &bc, results &result);

#endif //PW2_BRANDES_H
