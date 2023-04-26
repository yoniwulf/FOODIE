#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 25 19:15:02 2023

@author: kama
"""

import heapq

class PathwaySystem:
    def __init__(self, grid_size, obstacles=None):
        self.grid_size = grid_size
        self.obstacles = obstacles if obstacles is not None else []

    def add_obstacle(self, x, y):
        if (x, y) not in self.obstacles:
            self.obstacles.append((x, y))

    def remove_obstacle(self, x, y):
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))

    def can_move(self, position, direction):
        x, y = position
        if direction == "up":
            new_position = x, y + 1
        elif direction == "down":
            new_position = x, y - 1
        elif direction == "left":
            new_position = x - 1, y
        elif direction == "right":
            new_position = x + 1, y
        else:
            return False

        if (0 <= new_position[0] < self.grid_size and
            0 <= new_position[1] < self.grid_size and
            new_position not in self.obstacles):
            return True
        return False

    def get_next_position(self, position, direction):
        x, y = position
        if direction == "up":
            return x, y + 1
        elif direction == "down":
            return x, y - 1
        elif direction == "left":
            return x - 1, y
        elif direction == "right":
            return x + 1, y


class Order:
    def __init__(self, items, destination):
        self.items = items
        self.destination = destination
        self.is_delivered = False

class DeliveryRobot:
    def __init__(self, x, y, grid_size, warehouse_location, pathway, compartment_capacity, battery_capacity):
        self.x = x
        self.y = y
        self.grid_size = grid_size
        self.warehouse_location = warehouse_location
        self.pathway = pathway
        self.compartment_capacity = compartment_capacity
        self.compartment = []
        self.battery_capacity = battery_capacity
        self.battery_level = battery_capacity

    def at_warehouse(self):
        return (self.x, self.y) == self.warehouse_location

    def move(self, direction):
        if self.pathway.can_move((self.x, self.y), direction) and self.battery_level > 0:
            self.x, self.y = self.pathway.get_next_position((self.x, self.y), direction)
            self.battery_level -= self.energy_cost(direction)
            
    def recharge(self):
        if self.at_warehouse():
            self.battery_level = self.battery_capacity

    def load_order(self, order):
        items_to_load = [item for item in order.items if len(self.compartment) < self.compartment_capacity]
        self.compartment.extend(items_to_load)
        order.items = [item for item in order.items if item not in items_to_load]

    def deliver_order(self, order):
        if (self.x, self.y) == order.destination:
            order.is_delivered = True
            self.drop()

    def drop(self):
        self.compartment = []

    def energy_cost(self, direction):
        energy_costs = {
            "up": 1,
            "down": 1,
            "left": 1.2,
            "right": 1.2,
        }
        return energy_costs[direction]

    def get_neighbors(self, position):
        x, y = position
        possible_moves = ["up", "down", "left", "right"]
        neighbors = []
        for move in possible_moves:
            if self.pathway.can_move((x, y), move):
                neighbor = self.pathway.get_next_position((x, y), move)
                cost = self.energy_cost(move)
                neighbors.append((neighbor, cost))
        return neighbors

    def heuristic(self, position, destination):
        return abs(position[0] - destination[0]) + abs(position[1] - destination[1])

    def find_path(self, start, destination):
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == destination:
                break

            for neighbor, cost in self.get_neighbors(current):
                new_cost = cost_so_far[current] + cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, destination)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if current != destination:
            return None

        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def follow_path(self, path):
        for position in path[1:]:
            direction = None
            for move in ["up", "down", "left", "right"]:
                if self.pathway.get_next_position((self.x, self.y), move) == position:
                    direction = move
                    break
            if direction:
                self.move(direction)

class DeliverySystem:
    def __init__(self, grid_size, warehouse_location, pathway, compartment_capacity):
        self.grid_size = grid_size
        self.warehouse_location = warehouse_location
        self.pathway_system = pathway
        self.compartment_capacity = compartment_capacity
        self.robots = []
        self.orders = []

    def add_robot(self, x, y):
        robot = DeliveryRobot(x, y, self.grid_size, self.warehouse_location, self.pathway_system, self.compartment_capacity)
        self.robots.append(robot)

    def receive_order(self, items, destination):
        order = Order(items, destination)
        self.orders.append(order)

    def assign_order(self, order):
        min_path_length = float("inf")
        selected_robot = None
        selected_path = None

        for robot in self.robots:
            if not robot.at_warehouse():
                continue

            path = robot.find_path((robot.x, robot.y), order.destination)
            if path and len(path) < min_path_length:
                min_path_length = len(path)
                selected_robot = robot
                selected_path = path

        if not selected_robot:
            print("No available robot to assign the order")
            return

        print(f"Order assigned to robot at position ({selected_robot.x}, {selected_robot.y})")
        if selected_robot.at_warehouse():
            selected_robot.load_order(order)
        else:
            print("Robot not at warehouse, cannot load order")
            return

        selected_robot.follow_path(selected_path)
        selected_robot.deliver_order(order)
