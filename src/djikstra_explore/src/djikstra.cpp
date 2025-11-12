/*
 - Author: phoenixZ
 - Date: 2025-11-12 10:57:15
 - Description: 利用优先堆实现Djikstra
*/

#include <algorithm>
#include <iostream>
#include <limits>
#include <math.h>
#include <queue>
#include <vector>

using namespace std;

struct Edge {
  int to;
  long long weight;
};

using Graph = vector<vector<Edge>>;               // nxn的节点图
using PQElement = pair<int, long long>;           // 优先堆的类型
long long INF = numeric_limits<long long>::max(); //定义一个无穷大值

vector<int> DjikstraPath(int start, int end, int num_nodes,
                         const Graph &graph) {

  /**
   * 定义最小堆，初始化dist，prev
   */
  vector<int> path;
  vector<long long> dist(num_nodes, INF);
  vector<int> prev(num_nodes, -1);
  priority_queue<PQElement, vector<PQElement>, greater<PQElement>> pq;

  // 定义初始节点
  dist[start] = 0;
  pq.push({start, 0});

  while (!pq.empty()) {
    int U = pq.top().first;
    long long V = pq.top().second;
    pq.pop();

    if (V > dist[U]) {
      continue;
    }

    if (U == end) {
      int cur = end;
      while (cur != -1) {
        path.push_back(cur);
        if (cur == start)
          break;
        cur = prev[cur];
      }

      reverse(path.begin(), path.end());
      return path;
    }

    for (auto neighbor : graph[U]) {

      int X = neighbor.to;
      long long Y = neighbor.weight;

      if (dist[U] + Y < dist[X]) {
        dist[X] = dist[U] + Y;
        prev[X] = U;
        pq.push({X, dist[X]});
      }
    }
  }

  return {};
}

int main() {

  int N = 5;
  Graph graph(5); // 5x5

  graph[0].push_back({1, 2});
  graph[0].push_back({2, 5});
  graph[1].push_back({3, 6});
  graph[1].push_back({2, 2});
  graph[2].push_back({3, 7});
  graph[2].push_back({4, 1});
  graph[3].push_back({2, 1});
  graph[3].push_back({4, 1});

  int start = 0, end = 4;

  vector<int> res = DjikstraPath(start, end, N, graph);

  for (int i = 0; i < res.size(); i++) {
    cout << res[i] << ",";
  }
  cout << endl;

  return 0;
}
