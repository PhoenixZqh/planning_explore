/*
 - Author: phoenixZ
 - Date: 2025-11-11 09:48:33
 - Description: Djikstra 利用优先堆实现
 - 编写思路：边包括邻接点+权重
    1. 创建图，比如说5个节点，那么图就是5X5；
    2. 创建dist数组，保存到当前节点的距离
    3. 创建前置数组，保存当前节点的上一节点
    4. 创建最小堆，每次取出来距离最小的节点用来路径生成 {距离， 节点}

*/

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <vector>

using namespace std;

// 定义边， 包含邻接的节点和权重
struct Edge {
  int to;
  int weight;
};

// 定义图
using Graph = vector<vector<Edge>>;

// 初始化优先队列，权重，节点
using PQEelement = pair<long long, int>;

// 定义无穷大
const long long INF = numeric_limits<long long>::max();

/**
 * @brief djikstra找最短路径
 */
long long DjikstraOpt(const Graph &graph, int start_node, int end_node,
                      int num_nodes, vector<int> &path) {

  // 定义权重数组
  vector<long long> dist(num_nodes, INF);

  // 定义前置数组
  vector<int> prev(num_nodes, -1);

  // 定义最小堆
  priority_queue<PQEelement, vector<PQEelement>, greater<PQEelement>> pq;

  //初始化权重数组
  dist[start_node] = 0;

  // 将初始节点加入最小堆
  pq.push({0, start_node});

  while (!pq.empty()) {
    // 取出当前最小节点
    long long d = pq.top().first;
    int n = pq.top().second;
    pq.pop();

    // 如果当前权重大于之前找到到此节点的最小权重， 则跳过
    if (d > dist[n]) {
      continue;
    }

    // 已经是最后一个节点了， 退出
    if (n == end_node) {
      break;
    }

    // 遍历当前节点的所有邻接的节点
    for (const auto neighbor : graph[n]) {
      int v = neighbor.to;
      int d = neighbor.weight;

      if ((dist[n] + d) < dist[v]) {
        dist[v] = dist[n] + d;
        prev[v] = n;
        pq.push({dist[v], v});
      }
    }
  }

  // 路径检查
  if (dist[end_node] == INF) {
    path.clear();
    return INF;
  }

  // 回溯整个路径
  int cur_node = end_node;
  while (cur_node != -1) {

    path.push_back(cur_node);
    if (cur_node == start_node)
      break;
    cur_node = prev[cur_node];
  }

  reverse(path.begin(), path.end());

  return dist[end_node];
}

int main() {
  int N = 6;
  Graph graph(N);

  graph[1].push_back({2, 2});
  graph[1].push_back({3, 5});
  graph[2].push_back({3, 2});
  graph[2].push_back({4, 6});
  graph[3].push_back({5, 1});
  graph[4].push_back({3, 1});
  graph[4].push_back({5, 4});

  int start = 1, end = 4;

  vector<int> path;

  long long mindist = DjikstraOpt(graph, start, end, N, path);

  cout << "最短距离: " << mindist << endl;
  cout << "最短路径: ";
  for (int i = 0; i < path.size(); ++i) {
    cout << path[i];
    if (i != path.size() - 1)
      cout << " -> ";
  }

  cout << endl;

  return 0;
}