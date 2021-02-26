////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Harsh Pandey, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Harsh Pandey
/// \author Andrew Dornbush

#include <smpl/bfs3d/bfs3d.h>

#include <smpl/console/console.h>

namespace smpl {

BFS_3D::BFS_3D(int width, int height, int length) :
    m_search_thread(),
    m_dim_x(),
    m_dim_y(),
    m_dim_z(),
    m_distance_grid(nullptr),
    m_queue(nullptr),
    m_queue_head(),
    m_queue_tail(),
    m_running(false),
    m_neighbor_offsets(),
    m_closed(),
    m_distances()
{
    if (width <= 0 || height <= 0 || length <= 0) {
        return;
    }

    m_dim_x = width + 2;
    m_dim_y = height + 2;
    m_dim_z = length + 2;

    m_dim_xy = m_dim_x * m_dim_y;
    m_dim_xyz = m_dim_xy * m_dim_z;

    m_neighbor_offsets[0] = -m_dim_x;
    m_neighbor_offsets[1] = 1;
    m_neighbor_offsets[2] = m_dim_x;
    m_neighbor_offsets[3] = -1;
    m_neighbor_offsets[4] = -m_dim_x-1;
    m_neighbor_offsets[5] = -m_dim_x+1;
    m_neighbor_offsets[6] = m_dim_x+1;
    m_neighbor_offsets[7] = m_dim_x-1;
    m_neighbor_offsets[8] = m_dim_xy;
    m_neighbor_offsets[9] = -m_dim_x+m_dim_xy;
    m_neighbor_offsets[10] = 1+m_dim_xy;
    m_neighbor_offsets[11] = m_dim_x+m_dim_xy;
    m_neighbor_offsets[12] = -1+m_dim_xy;
    m_neighbor_offsets[13] = -m_dim_x-1+m_dim_xy;
    m_neighbor_offsets[14] = -m_dim_x+1+m_dim_xy;
    m_neighbor_offsets[15] = m_dim_x+1+m_dim_xy;
    m_neighbor_offsets[16] = m_dim_x-1+m_dim_xy;
    m_neighbor_offsets[17] = -m_dim_xy;
    m_neighbor_offsets[18] = -m_dim_x-m_dim_xy;
    m_neighbor_offsets[19] = 1-m_dim_xy;
    m_neighbor_offsets[20] = m_dim_x-m_dim_xy;
    m_neighbor_offsets[21] = -1-m_dim_xy;
    m_neighbor_offsets[22] = -m_dim_x-1-m_dim_xy;
    m_neighbor_offsets[23] = -m_dim_x+1-m_dim_xy;
    m_neighbor_offsets[24] = m_dim_x+1-m_dim_xy;
    m_neighbor_offsets[25] = m_dim_x-1-m_dim_xy;

    m_distance_grid = new int[m_dim_xyz];
    m_queue = new int[width * height * length];

    for (int node = 0; node < m_dim_xyz; node++) {
        int x = node % m_dim_x;
        int y = node / m_dim_x % m_dim_y;
        int z = node / m_dim_xy;
        if (x == 0 || x == m_dim_x - 1 ||
            y == 0 || y == m_dim_y - 1 ||
            z == 0 || z == m_dim_z - 1)
        {
            m_distance_grid[node] = WALL;
        }
        else {
            m_distance_grid[node] = UNDISCOVERED;
        }
    }

    m_running = false;
}

BFS_3D::~BFS_3D()
{
    if (m_search_thread.joinable()) {
        m_search_thread.join();
    }

    if (m_distance_grid) {
        delete[] m_distance_grid;
    }
    if (m_queue) {
        delete[] m_queue;
    }
}

void BFS_3D::getDimensions(int* width, int* height, int* length)
{
    *width = m_dim_x - 2;
    *height = m_dim_y - 2;
    *length = m_dim_z - 2;
}

void BFS_3D::setWall(int x, int y, int z)
{
    if (m_running) {
        //error "Cannot modify grid while search is running"
        return;
    }

    int node = getNode(x, y, z);
    m_distance_grid[node] = WALL;
}

bool BFS_3D::isWall(int x, int y, int z) const
{
    int node = getNode(x, y, z);
    return m_distance_grid[node] == WALL;
}

bool BFS_3D::isUndiscovered(int x, int y, int z) const
{
    int node = getNode(x, y, z);
    while (m_running && m_distance_grid[node] < 0);
    return m_distance_grid[node] == UNDISCOVERED;
}

void BFS_3D::run(int x, int y, int z)
{
    if (m_running) {
        return;
    }

    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // get index of start coordinate
    int origin = getNode(x, y, z);

    // initialize the queue
    m_queue_head = 0;
    m_queue_tail = 1;
    m_queue[0] = origin;

    // initialize starting distance
    m_distance_grid[origin] = 0;

    m_running = true;

    if (m_search_thread.joinable()) {
        m_search_thread.join();
    }

    // fire off background thread to compute bfs
    m_search_thread = std::thread([&]()
    {
        this->search(m_dim_x, m_dim_xy, m_distance_grid, m_queue, m_queue_head, m_queue_tail);
    });
}

void BFS_3D::run_components(int gx, int gy, int gz)
{
    for (int i = 0; i < m_dim_xyz; i++) {
        if (m_distance_grid[i] != WALL) {
            m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // invert walls and free cells in an auxiliary bfs
    int length, width, height;
    getDimensions(&length, &width, &height);
    BFS_3D wall_bfs(length, width, height);
    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                if (!isWall(x, y, z)) {
                    wall_bfs.setWall(x, y, z);
                }
            }
        }
    }

    // initialize the distance grid of the wall bfs
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (wall_bfs.m_distance_grid[i] != WALL) {
            wall_bfs.m_distance_grid[i] = UNDISCOVERED;
        }
    }

    // initialize the distance grid queue
    wall_bfs.m_queue_head = 0;
    wall_bfs.m_queue_tail = 1;

    int volatile* curr_distance_grid = m_distance_grid;
    int* curr_queue = m_queue;
    int* curr_queue_head = &m_queue_head;
    int* curr_queue_tail = &m_queue_tail;

    int volatile* next_distance_grid = wall_bfs.m_distance_grid;
    int* next_queue = wall_bfs.m_queue;
    int* next_queue_head = &wall_bfs.m_queue_head;
    int* next_queue_tail = &wall_bfs.m_queue_tail;

    int gnode = getNode(gx, gy, gz);

    // seed the initial queue
    *curr_queue_head = 0;
    *curr_queue_tail = 1;
    curr_queue[0] = gnode;
    curr_distance_grid[gnode] = 0;

    *next_queue_head = 0;
    *next_queue_tail = 0;

    int num_iterations = 0;

    while (*curr_queue_head < *curr_queue_tail) {
        // next_queue and values of cells in next_queue via next_distance_grid
        // are initialized by this search call
        search(
                m_dim_x,
                m_dim_xy,
                curr_distance_grid,
                curr_queue,
                *curr_queue_head,
                *curr_queue_tail,
                next_distance_grid,
                next_queue,
                *next_queue_head,
                *next_queue_tail);

        std::swap(curr_distance_grid, next_distance_grid);
        std::swap(curr_queue, next_queue);
        std::swap(curr_queue_head, next_queue_head);
        std::swap(curr_queue_tail, next_queue_tail);
        ++num_iterations;
    }

    SMPL_INFO("Computed entire distance field in %d iterations", num_iterations);

    // combine distance fields
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (wall_bfs.m_distance_grid[i] != WALL) {
            m_distance_grid[i] = wall_bfs.m_distance_grid[i];
        }
    }
}

bool BFS_3D::escapeCell(int x, int y, int z)
{
    if (!inBounds(x, y, z)) {
        SMPL_ERROR("BFS goal is out of bounds");
        return false;
    }

    // clear cells until the goal connects to a free cell
    int escape_count = 0;
    std::queue<int> q;
    q.push(getNode(x, y, z));
    bool escaped = false;

    int length, width, height;
    getDimensions(&length, &width, &height);
    std::vector<bool> visited((length + 2) * (width + 2) * (height + 2), false);

    while (!q.empty()) {
        int n = q.front();
        q.pop();

        visited[n] = true;

        // goal condition
        if (!isWall(n)) {
            break;
        }

        unsetWall(n);

        for (int i = 0; i < 26; ++i) {
            int neighbor = this->neighbor(n, i);
            if (!visited[neighbor]) {
                q.push(neighbor);
            }
        }

        ++escape_count;
    }

    SMPL_INFO("Escaped goal cell in %d expansions", escape_count);

    // TODO: return false if no free cells (escape_count == width * height * depth?)
    return true;
}

template <typename Visitor>
void BFS_3D::visit_free_cells(int node, const Visitor& visitor)
{
    if (isWall(node)) {
        return;
    }

    int nx, ny, nz;
    getCoord(node, nx, ny, nz);

    std::vector<bool> visited(m_dim_xyz, false);
    std::queue<int> nodes;
    nodes.push(node);

    while (!nodes.empty()) {
        int n = nodes.front();
        nodes.pop();

        visitor(n);

        for (int i = 0; i < 26; ++i) {
            int nn = neighbor(n, i);
            if (!visited[nn] && !isWall(nn)) {
                nodes.push(nn);
                // mark visited here to avoid adding nodes to the queue multiple
                // times
                visited[nn] = true;
                if (nodes.size() >= m_dim_xyz) {
                    SMPL_ERROR("Wow queue is too damn big");
                    return;
                }
            }
        }
    }
}

int BFS_3D::getDistance(int x, int y, int z) const
{
    int node = getNode(x, y, z);
    while (m_running && m_distance_grid[node] < 0);
    return m_distance_grid[node];
}

int BFS_3D::getNearestFreeNodeDist(int x, int y, int z)
{
    // initialize closed set and distances
    m_closed.assign(m_dim_xyz, false);
    m_distances.assign(m_dim_xyz, -1);

    std::queue<std::tuple<int, int, int>> q;
    q.push(std::make_tuple(x, y, z));

    int n = getNode(x, y, z);
    m_distances[n] = 0;

    while (!q.empty()) {
        std::tuple<int, int, int> ncoords = q.front();
        q.pop();

        // extract the coordinates of this cell
        int nx = std::get<0>(ncoords);
        int ny = std::get<1>(ncoords);
        int nz = std::get<2>(ncoords);

        // extract the index of this cell
        n = getNode(nx, ny, nz);

        // mark as visited
        m_closed[n] = true;

        int dist = m_distances[n];

        // goal == found a free cell
        if (!isWall(n)) {
            int cell_dist = getDistance(nx, ny, nz);
            if (cell_dist < 0) {
                // TODO: mark as a wall, and move on
                setWall(nx, ny, nz);
                SMPL_INFO("Encountered isolated cell, m_running: %s", m_running ? "true" : "false");
            }
            else {
                return dist + cell_dist;
            }
        }


#define ADD_NEIGHBOR(xn, yn, zn) \
{\
if (inBounds(xn, yn, zn)) {\
    int nn = getNode(xn, yn, zn);\
    if (!m_closed[nn] && (m_distances[nn] == -1 || dist + 1 < m_distances[nn])) {\
        m_distances[nn] = dist + 1;\
        q.push(std::make_tuple(xn, yn, zn));\
    }\
}\
}

        ADD_NEIGHBOR(nx - 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx - 1, ny,     nz    );
        ADD_NEIGHBOR(nx - 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx - 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx - 1, ny + 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny - 1, nz    );
        ADD_NEIGHBOR(nx    , ny - 1, nz + 1);
        ADD_NEIGHBOR(nx    , ny,     nz - 1);
//            ADD_NEIGHBOR(nx    , ny,     nz    );
        ADD_NEIGHBOR(nx    , ny,     nz + 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz - 1);
        ADD_NEIGHBOR(nx    , ny + 1, nz    );
        ADD_NEIGHBOR(nx    , ny + 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny - 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny - 1, nz + 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz - 1);
        ADD_NEIGHBOR(nx + 1, ny,     nz    );
        ADD_NEIGHBOR(nx + 1, ny,     nz + 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz - 1);
        ADD_NEIGHBOR(nx + 1, ny + 1, nz    );
        ADD_NEIGHBOR(nx + 1, ny + 1, nz + 1);
#undef ADD_NEIGHBOR
    }

    fprintf(stderr, "Found no free neighbor\n");
    return -1;
}

int BFS_3D::countWalls() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == WALL) {
            ++count;
        }
    }
    return count;
}

int BFS_3D::countUndiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] == UNDISCOVERED) {
            ++count;
        }
    }
    return count;
}

int BFS_3D::countDiscovered() const
{
    int count = 0;
    for (int i = 0; i < m_dim_xyz; ++i) {
        if (m_distance_grid[i] != WALL && m_distance_grid[i] >= 0) {
            ++count;
        }
    }
    return count;
}

#define EXPAND_NEIGHBOR(offset)                            \
    if (distance_grid[currentNode + offset] < 0) {         \
        queue[queue_tail++] = currentNode + offset;        \
        distance_grid[currentNode + offset] = currentCost; \
    }

void BFS_3D::search(
    int width,
    int planeSize,
    int volatile* distance_grid,
    int* queue,
    int& queue_head,
    int& queue_tail)
{
    while (queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        EXPAND_NEIGHBOR(-width);
        EXPAND_NEIGHBOR(1);
        EXPAND_NEIGHBOR(width);
        EXPAND_NEIGHBOR(-1);
        EXPAND_NEIGHBOR(-width-1);
        EXPAND_NEIGHBOR(-width+1);
        EXPAND_NEIGHBOR(width+1);
        EXPAND_NEIGHBOR(width-1);
        EXPAND_NEIGHBOR(planeSize);
        EXPAND_NEIGHBOR(-width+planeSize);
        EXPAND_NEIGHBOR(1+planeSize);
        EXPAND_NEIGHBOR(width+planeSize);
        EXPAND_NEIGHBOR(-1+planeSize);
        EXPAND_NEIGHBOR(-width-1+planeSize);
        EXPAND_NEIGHBOR(-width+1+planeSize);
        EXPAND_NEIGHBOR(width+1+planeSize);
        EXPAND_NEIGHBOR(width-1+planeSize);
        EXPAND_NEIGHBOR(-planeSize);
        EXPAND_NEIGHBOR(-width-planeSize);
        EXPAND_NEIGHBOR(1-planeSize);
        EXPAND_NEIGHBOR(width-planeSize);
        EXPAND_NEIGHBOR(-1-planeSize);
        EXPAND_NEIGHBOR(-width-1-planeSize);
        EXPAND_NEIGHBOR(-width+1-planeSize);
        EXPAND_NEIGHBOR(width+1-planeSize);
        EXPAND_NEIGHBOR(width-1-planeSize);
    }
    m_running = false;
}

#undef EXPAND_NEIGHBOR

#define EXPAND_NEIGHBOR_FRONTIER(offset) \
{\
    if (distance_grid[currentNode + offset] < 0) {\
        queue[queue_tail++] = currentNode + offset;\
        distance_grid[currentNode + offset] = currentCost;\
    }\
    else if (distance_grid[currentNode + offset] == WALL) {\
        if (frontier_grid[currentNode + offset] < 0) {\
            frontier_queue[frontier_queue_tail++] = currentNode + offset;\
            frontier_grid[currentNode + offset] = currentCost;\
        }\
    }\
}

void BFS_3D::search(
    int width,
    int planeSize,
    int volatile* distance_grid,
    int* queue,
    int& queue_head,
    int& queue_tail,
    int volatile* frontier_grid,
    int* frontier_queue,
    int& frontier_queue_head,
    int& frontier_queue_tail)
{
    while (queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        EXPAND_NEIGHBOR_FRONTIER(-width);
        EXPAND_NEIGHBOR_FRONTIER(1);
        EXPAND_NEIGHBOR_FRONTIER(width);
        EXPAND_NEIGHBOR_FRONTIER(-1);
        EXPAND_NEIGHBOR_FRONTIER(-width-1);
        EXPAND_NEIGHBOR_FRONTIER(-width+1);
        EXPAND_NEIGHBOR_FRONTIER(width+1);
        EXPAND_NEIGHBOR_FRONTIER(width-1);
        EXPAND_NEIGHBOR_FRONTIER(planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-1+planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width-1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(-width+1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width+1-planeSize);
        EXPAND_NEIGHBOR_FRONTIER(width-1-planeSize);
    }
    m_running = false;
}

#undef EXPAND_NEIGHBOR_FRONTIER

} // namespace smpl
