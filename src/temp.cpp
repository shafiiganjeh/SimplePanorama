
#include <vector>
#include <queue>
#include <utility>  // For std::pair

using namespace std;

vector<pair<int, vector<int>>> bfs_ordered_with_neighbors(const vector<vector<int>>& adj, int i) {
    vector<pair<int, vector<int>>> result;
    int n = adj.size();
    if (n == 0 || i < 0 || i >= n) {
        return result;
    }

    vector<bool> visited(n, false);
    queue<int> q;

    visited[i] = true;
    q.push(i);
    vector<int> traversal_order;

    // BFS
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        traversal_order.push_back(u);

        for (int v = 0; v < n; ++v) {
            if (adj[u][v] == 1 && !visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }

    // Collect neighbors for each node in BFS
    for (int u : traversal_order) {
        vector<int> neighbors;
        for (int v = 0; v < n; ++v) {
            if (adj[u][v] == 1) {
                neighbors.push_back(v);
            }
        }
        result.emplace_back(u, neighbors);
    }

    return result;
}
