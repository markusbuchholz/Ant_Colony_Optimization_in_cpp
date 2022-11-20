//Markus Buchholz

#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <numeric>
#include <random>
#include <stdlib.h> 
#include <time.h>   

int NODES = 5;
int START = 0;
int GOAL = 4;
int EVOLUTION = 550;
float alfa = 1;
float beta = 2;
float evaporation = 0.2;

//----------------------------------------------------------------------------------------------------

class Ant
{

public:
    std::vector<std::vector<float>> distance;
    std::vector<std::vector<float>> visibility;

    static std::vector<std::vector<float>> pheromone;
    std::vector<int> ant_way_i;
    Ant(std::vector<std::vector<float>> dist) : distance(dist)
    {

        for (auto &ii : dist)
        {

            std::vector<float> vis;

            for (auto &jj : ii)
            {
                if (jj == 0.0)
                {
                    vis.push_back(0.0);
                }
                else
                {

                    vis.push_back(1.0 / jj);
                }
            }
            this->visibility.push_back(vis);
        }
    }
};

std::vector<float> ph(NODES, 0.1);
std::vector<std::vector<float>> Ant::pheromone(NODES, ph);

//----------------------------------------------------------------------------------------------------

void pheromoneUpdate(std::vector<std::vector<int>> antWay_i, std::vector<float> ph_change_i)
{

    for (auto &ii : Ant::pheromone)
    {

        for (int jj = 0; jj < ii.size(); jj++)
        {
            ii[jj] = (1 - evaporation) * ii[jj];
        }
    }

    for (auto &ii : antWay_i)
    {

        ii.push_back(ii[0]);
    }

    for (int ij = 0; ij < ph_change_i.size(); ij++)
    {

        for (int yi = 0; yi < antWay_i[0].size() - 1; yi++)
        {

            Ant::pheromone[antWay_i[ij][yi]][antWay_i[ij][yi + 1]] = Ant::pheromone[antWay_i[ij][yi]][antWay_i[ij][yi + 1]] + ph_change_i[ij];
        }
    }
}

//----------------------------------------------------------------------------------------------------

float computeWayCostAnt_i(Ant ant, std::vector<int> way)
{

    float cost = 0.0;

    for (int ii = 0; ii < way.size() - 1; ii++)
    {

        cost = cost + ant.distance[way[ii]][way[ii + 1]];
    }

    return 1.0 / cost;
}

//----------------------------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::uniform_real_distribution<float> distrib(0.0, 1.0);
    return distrib(engine);
}

//----------------------------------------------------------------------------------------------------

void shortestPath(std::vector<std::vector<int>> antWay_i, Ant ant)
{

    std::vector<std::vector<int>> antWay_opt;

    for (auto &ix : antWay_i)
    {
        std::vector<int> ant_opt_i;
        ant_opt_i.push_back(START);

        for (auto &jx : ix)
        {

            ant_opt_i.push_back(jx);

            if (jx == GOAL)
            {
                break;
            }
        }
        antWay_opt.push_back(ant_opt_i);
    }

    float cost = 0.0;
    std::vector<int> shortestPath;

    for (int ii = 0; ii < antWay_opt.size(); ii++)
    {

        float ci = 0.0;
        for (int jj = 0; jj < antWay_opt[ii].size() - 1; jj++)
        {
            ci = ci + ant.distance[antWay_opt[ii][jj]][antWay_opt[ii][jj + 1]];
        }
        if (ci > cost)
        {
            cost = ci;
            shortestPath = antWay_opt[ii];
        }
    }

    std::cout << "cost : " << cost << "\n";
    std::cout << "path ------"
              << "\n";

    for (auto &ii : shortestPath)
    {
        std::cout << ii << " ,";
    }
    std::cout << "\n";
}

//----------------------------------------------------------------------------------------------------

void runAnts(std::vector<Ant> ants)
{
    std::vector<int> aWi(NODES, 0);
    std::vector<std::vector<int>> antWay_i(NODES, aWi);

    for (int ii = 0; ii < EVOLUTION; ii++)
    {
        std::vector<float> ph_change_i; // (NODES, 0.0);
        for (int jj = 0; jj < ants.size(); jj++)
        {
            std::vector<int> visited(NODES, 1);
            std::vector<int> ant_optimal_way_i(NODES, 1);
            int actual_node = START;  // start node
            visited[actual_node] = 0; // node 0 is visited = 0

            ant_optimal_way_i[0] = actual_node;

            for (int node = 0; node < NODES - 1; node++)
            {

                std::vector<float> ij_prob;
                std::vector<float> transition_prob;
                std::vector<float> accum_transition_prob;
                for (int nx = 0; nx < NODES - 0; nx++)
                {

                    float ph = Ant::pheromone[actual_node][nx];
                    float fx = visited[nx] * std::pow(ph, alfa) * std::pow(ants[jj].visibility[actual_node][nx], beta);

                    ij_prob.push_back(fx);
                }
                float s = std::accumulate(ij_prob.begin(), ij_prob.end(), 0.0);

                for (auto &ii : ij_prob)
                {

                    transition_prob.push_back(ii / s);
                }

                float accum_i = 0;
                int i = 0;
                for (auto &ii : transition_prob)
                {

                    accum_i = (accum_i + ii) * visited[i];
                    accum_transition_prob.push_back(accum_i);
                    i++;
                }

                double r = generateRandom();

                int next_node = 0;
                float sx = 0.0;
                for (int iX = 0; iX < NODES; iX++)
                {

                    sx = sx + accum_transition_prob[iX];
                    if (r <= sx)
                    {
                        next_node = iX;
                        break;
                    }
                }

                actual_node = next_node;
                visited[next_node] = 0;
                ant_optimal_way_i[node] = actual_node;
                if (next_node == GOAL)
                {
                    break;
                }
            }
            antWay_i[jj] = ant_optimal_way_i;
            ph_change_i.push_back(computeWayCostAnt_i(ants[jj], ant_optimal_way_i));
        }

        pheromoneUpdate(antWay_i, ph_change_i);

    }
    shortestPath(antWay_i, ants[0]);
}

//----------------------------------------------------------------------------------------------------

int main()
{
    std::vector<float> dist_1 = {0, 10, 12, 11, 14};
    std::vector<float> dist_2 = {10, 0, 13, 15, 8};
    std::vector<float> dist_3 = {12, 13, 0, 90, 14};
    std::vector<float> dist_4 = {11, 15, 9, 0, 16};
    std::vector<float> dist_5 = {14, 8, 14, 16, 0};

    std::vector<std::vector<float>> node_distance = {dist_1, dist_2, dist_3, dist_4, dist_5};

    int n_ants = 5;
    std::vector<Ant> ants;

    for (int ii = 0; ii < n_ants; ii++)
    {
        Ant ant(node_distance);
        // Ant *p = new Ant(ii);
        ants.push_back(ant);
    }

    runAnts(ants);
}