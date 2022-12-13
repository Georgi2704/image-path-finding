"""
file: run.py
description:
This program finds a path between pixels provided in path file.

language: python3
"""

# imports
from collections import defaultdict
from queue import PriorityQueue
from math import sqrt, pow
from classes.PixelPriority import PixelPriority
from scripts.load import *
from classes.Pixel import *
import numpy as npy

def convert_pixels_to_metric():
    # Size in meters (m)
    image_size_x = 934
    # image_size_y = 204
    size_metric_x = 70.5
    # size_metric_y = 15

    return size_metric_x / image_size_x




def st(imagleFile: str, algorithm: str) -> None:
    landPixels, data = loadPixels(imagleFile)
    target = []
    with open("path", "r") as path:
        for line in path.readlines():
            target.append((line.split()))
    if len(target) > 1:
        for i in range(len(target) - 1):
            xSource, ySource = int(target[i][0]), int(target[i][1])
            xDestination, yDestination = int(target[i + 1][0]), int(target[i + 1][1])
            if algorithm == "dfs":
                path = dfs(landPixels[(xSource, ySource)], landPixels[(xDestination, yDestination)], data)
            elif algorithm == "bfs":
                path = bfs(landPixels[(xSource, ySource)], landPixels[(xDestination, yDestination)], data)
            elif algorithm == "ucs":
                path = ucs(landPixels[(xSource, ySource)], landPixels[(xDestination, yDestination)], data)
            elif algorithm == "astar":
                path = astar(landPixels[(xSource, ySource)], landPixels[(xDestination, yDestination)], data)
            if path:
                img = cv2.imread("tq42_floor_plan_small.png")
                img_copy = img.copy()

                for item in path:
                    # print(item)
                    img_copy[item.y][item.x] = npy.array([0, 0, 255])
                    # print(f"X:{item.x}, Y:{item.y}")

                # cv2.imshow("Image", img)
                # cv2.waitKey(1000)
                # print("Path", path)
                # thepath = path
                # img_copy[0][2] = npy.array([0, 0, 255])  # Red pixel
                prev_x = -1
                prev_y = -1
                prev_x_change = True
                prev_y_change = True

                nodes_list = []
                nodes_offsets = []
                nodes_offsets_metric = []

                for item in path:
                    node_x = prev_x
                    node_y = prev_y
                    current_x_has_changed = False
                    current_y_has_changed = False

                    # print(f"X:{item.x}, Y:{item.y}")

                    if prev_x != item.x:
                        current_x_has_changed = True
                        prev_x = item.x
                    else:
                        current_x_has_changed = False

                    if prev_y != item.y:
                        current_y_has_changed = True
                        prev_y = item.y
                    else:
                        current_y_has_changed = False

                    if (current_x_has_changed != prev_x_change) or (current_y_has_changed != prev_y_change):
                        img_copy[node_y][node_x] = npy.array([0, 255, 0])
                        nodes_list.append({"x": node_x, "y": node_y})
                        # print(f"Node Found! X:{node_x}, Y:{node_y}")

                    prev_x_change = current_x_has_changed
                    prev_y_change = current_y_has_changed

                last_item = nodes_list[0]
                for item in nodes_list:
                    x_offset = last_item['x'] - item['x']
                    y_offset = last_item['y'] - item['y']
                    last_item = item
                    nodes_offsets.append(({"x_offset": x_offset, "y_offset": y_offset}))
                    # print(item['x'], item['y'])


                    # nodes_list.append({"x": node_x, "y": node_y})

                    # print(item)

                for item in nodes_offsets:
                    print(item)
                    pixel_size = convert_pixels_to_metric()
                    print(pixel_size)
                    x_offset = item['x_offset'] * pixel_size
                    y_offset = item['y_offset'] * pixel_size

                    nodes_offsets_metric.append(({"x_offset": x_offset, "y_offset": y_offset}))

                for item in nodes_offsets_metric:
                    print(item)

                cv2.imwrite('path.png', img_copy)

                # [print(x) for x in path]


def bfs(st: Pixel, end: Pixel, data: np.ndarray):
    """
    Iterative method to find a bfs path, if one exists from current to end vertex
    :param st:       source vertex in image
    :param end:         destination vertex in image
    :param data:        numpy data for image display
    :return:            shortest path else None
    """
    queue = [st]
    predecessors = defaultdict(lambda: None)
    predecessors[st] = -1

    while queue:
        current = queue.pop(0)
        if current == end:
            break
        for neighbor in current.getNeighbors():  # Traverse nodes
            if predecessors[neighbor] is None:
                showImage(data, neighbor.y, neighbor.x)
                predecessors[neighbor] = current
                queue.append(neighbor)

    if predecessors[end] is not None:  # Backtrack to find st node
        path = []
        current = end
        while current != st:
            path.insert(0, current)
            current = predecessors[current]
        path.insert(0, st)
        return path
    else:
        return None


def dfs(st: Pixel, end: Pixel, data: np.ndarray):
    """
    Iterative method to find a dfs path, if one exists from current to end vertex
    :param st:       source vertex in image
    :param end:         destination vertex in image
    :param data:        numpy data for image display
    :return:            list of visited vertices else None
    """

    stack, path = [st], []

    while stack:
        vertex = stack.pop()
        if vertex in path:
            continue
        if vertex == end:
            return path
        showImage(data, vertex.y, vertex.x)
        path.append(vertex)
        for neighbor in vertex.getNeighbors():
            stack.append(neighbor)
    return None


def ucs(st: Pixel, end: Pixel, data: np.ndarray):
    """
    Iterative method to find a Dijkstra path, if one exists from current to end vertex
    :param startKey:        start pixel point key
    :param endKey:          end pixel point key
    :return:                path
    """
    q = PriorityQueue()
    startPriorityPixel = PixelPriority(st, 0, 0)  # start priority pixel with 0 priority
    q.put((0, startPriorityPixel))

    lowest = startPriorityPixel
    visited = dict()
    while lowest.pxl != end:
        if q.empty():  # No way to get to end
            return [], -1
        thisDistace = lowest.distance
        for u in lowest.pxl.getNeighbors():
            if u is not None and (u.x, u.y) not in visited:
                showImage(data, u.y, u.x)
                visited[(u.x, u.y)] = 1
                # distance travelled from start pixel to current pixel
                dist = sqrt(pow(u.x - lowest.pxl.x, 2) + pow(u.y - lowest.pxl.y, 2) + \
                            pow(u.elevation - lowest.pxl.elevation, 2))
                newDistance = thisDistace + dist
                priority = newDistance
                priorityPixel = PixelPriority(u, newDistance, priority)
                priorityPixel.predecessor = lowest
                q.put((priority, priorityPixel))

        lowest = q.get()[1]
    path = []
    if lowest.distance != 0:  # We found the end, but it never got connected.
        lst = lowest
        while lst is not None:
            path.insert(0, lst.pxl)
            lst = lst.predecessor
    return path


def astar(st: Pixel, end: Pixel, data: np.ndarray):
    """
    Compute the path through the graph from st to end vertex whose sum of edge weights is minimized
    :param startKey:        start pixel point key
    :param endKey:          end pixel point key
    :return:                path
    """
    q = PriorityQueue()
    startPriorityPixel = PixelPriority(st, 0, 0)  # start priority pixel with 0 priority
    q.put((0, startPriorityPixel))

    lowest = startPriorityPixel
    visited = dict()
    while lowest.pxl != end:
        if q.empty():  # No way to get to end
            return [], -1
        thisDistace = lowest.distance
        for u in lowest.pxl.getNeighbors():
            if u is not None and (u.x, u.y) not in visited:
                showImage(data, u.y, u.x)
                visited[(u.x, u.y)] = 1
                # heuristic function computed by calculating min. distance from end
                heuristic = sqrt(pow(u.x - end.x, 2) + pow(u.y - end.y, 2) + pow(u.elevation - end.elevation, 2))
                # distance travelled from start pixel to current pixel
                dist = sqrt(pow(u.x - lowest.pxl.x, 2) + pow(u.y - lowest.pxl.y, 2) + \
                            pow(u.elevation - lowest.pxl.elevation, 2))
                newDistance = thisDistace + dist
                priority = lowest.pxl.getCost(u) + heuristic
                priorityPixel = PixelPriority(u, newDistance, priority)
                priorityPixel.predecessor = lowest
                q.put((priority, priorityPixel))

        lowest = q.get()[1]
    path = []
    if lowest.distance != 0:  # found the end, but it never got connected
        lst = lowest
        while lst is not None:
            path.insert(0, lst.pxl)
            lst = lst.predecessor
    return path


if __name__ == '__main__':
    st("tq42_floor_plan_small.png", "ucs")
