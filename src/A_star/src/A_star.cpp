/*
 - Author: phoenixZ
 - Date: 2025-11-12 13:47:42
 - Description: 手写ASTAR
*/

#include <algorithm>
#include <iostream>
#include <limits>
#include <math.h>
#include <queue>
#include <vector>

using namespace std;

//定义节点，包含坐标和当前花费与预计花费
struct Node {
  int x_, y_;

  long long g_score, f_score;

  Node *parent; //记录上一节点

  Node(int x, int y) : x_(x), y_(y) {
    g_score = numeric_limits<long long>::max();
    f_score = numeric_limits<long long>::max();
  }
};

using PQElement = pair<long long, Node *>;

// 预测未来代价
long long HFunction(Node *a, Node *b) {
  return abs(a->x_ - b->x_) + abs(a->y_ - b->y_);
}

void Astar(vector<vector<int>> &map, pair<int, int> start, pair<int, int> end,
           vector<pair<int, int>> &path) {
  int R = map.size();
  if (R == 0)
    return;
  int C = map[0].size();

  vector<vector<Node>> nodes(R);
  for (int i = 0; i < R; i++) {
    nodes[i].reserve(C);
    for (int j = 0; j < C; j++) {
      // 只创建一次对象，并放入 vector
      nodes[i].emplace_back(i, j);
    }
  }

  //创建最小堆
  priority_queue<PQElement, vector<PQElement>, greater<PQElement>> pq;

  //定义邻居点
  int dr[4] = {-1, 1, 0, 0};
  int dc[4] = {0, 0, -1, 1};

  //创建起始节点与目标节点
  auto start_node = &nodes[start.first][start.second];
  auto target_node = &nodes[end.first][end.second];

  // 定义起始节点
  start_node->g_score = 0;
  start_node->f_score = HFunction(start_node, target_node);

  // 起始节点入堆
  pq.push({start_node->f_score, start_node});

  while (!pq.empty()) {
    long long U = pq.top().first;
    auto V = pq.top().second;
    pq.pop();

    if (U > V->f_score)
      continue;

    // 建立路径
    if (V == target_node) {
      auto cur = target_node;
      while (cur != nullptr) {
        path.push_back({cur->x_, cur->y_});
        if (cur == start_node)
          break;
        cur = cur->parent;
      }
      reverse(path.begin(), path.end());
    }

    for (int i = 0; i < 4; i++) {
      int r = V->x_ + dr[i];
      int c = V->y_ + dc[i];

      //检查是否超过边界或者有障碍物
      if (r >= R || r < 0 || c >= C || c < 0 || map[r][c] == 1) {
        continue;
      }

      auto neighbor = &nodes[r][c];

      if (V->g_score + 1 < neighbor->g_score) {
        neighbor->g_score = V->g_score + 1;
        neighbor->f_score = V->g_score + 1 + HFunction(neighbor, target_node);
        neighbor->parent = V;
        pq.push({neighbor->f_score, neighbor});
      }
    }
  }
}

int main() {

  // 5x5地图，1代表障碍物
  vector<vector<int>> map = {{0, 1, 0, 1, 0},
                             {0, 0, 0, 0, 0},
                             {1, 0, 0, 1, 0},
                             {0, 0, 1, 1, 0},
                             {1, 0, 0, 0, 0}};

  pair<int, int> start = {0, 0};
  pair<int, int> target = {4, 4};

  vector<pair<int, int>> path;

  Astar(map, start, target, path);

  for (size_t i = 0; i < path.size(); ++i) {
    cout << "(" << path[i].first << "," << path[i].second << ")"
         << (i == path.size() - 1 ? "" : " -> ");
  }
  cout << endl;

  return 0;
}
