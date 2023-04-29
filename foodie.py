#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 25 19:15:02 2023

@author: kama
"""

import heapq

class PathwaySystem:
    def __init__(self, width, height, obstacles=None):
        self.width = width
        self.height = height
        self.obstacles = obstacles if obstacles is not None else []

    def add_obstacle(self, x, y):
        if (x, y) not in self.obstacles:
            self.obstacles.append((x, y))

    def remove_obstacle(self, x, y):
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))
            
    def update_obstacle(self, x, y):
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.grid_size[x][y] = 1
            
    def can_move(self, start, direction):
        end = self.get_next_position(start, direction)
        if end is None or not self.in_bounds(end) or not self.passable(end):
            return False
        return True
    
    def in_bounds(self, position):
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, position):
        return position not in self.obstacles

    def get_next_position(self, position, direction):
        x, y = position
        dx, dy = direction
        new_position = (x + dx, y + dy)
        if self.in_bounds(new_position):
            return new_position
        return None
    
    def get_neighbors(self, position):
        x, y = position
        neighbors = [(x + dx, y + dy) for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]]
        return [neighbor for neighbor in neighbors if self.in_bounds(neighbor) and self.passable(neighbor)]

    def get_cost(self, current, next):
        return 1  # Assuming uniform cost for all moves

class Order:
    def __init__(self, order_id, destination):
        self.id = order_id
        self.destination = destination
        self.is_bagged = False

    def copy(self):
        new_order = Order(self.id, self.destination)
        new_order.is_bagged = self.is_bagged
        return new_order

class DeliveryRobot:
    def __init__(self, id, pathway, battery_capacity, arm, current_order, compartment_capacity, warehouse_location):
        self.id = id
        self.pathway = pathway
        self.battery_capacity = battery_capacity
        self.battery_level = battery_capacity
        self.battery = battery_capacity  # Add this line to initialize the battery
        self.arm = arm
        self.warehouse_location = warehouse_location
        self.x, self.y = warehouse_location
        self.current_order = current_order
        self.compartment_capacity = compartment_capacity
        self.compartment = []
        self.moves = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Add the moves attribute here
        
    def give_delivery_update(self, status, order=None):
        if order:
            print(f"Robot at ({self.x}, {self.y}): {status} for Order {order.id}")
        else:
            print(f"Robot at ({self.x}, {self.y}): {status}")

    def at_warehouse(self):
        return (self.x, self.y) == self.warehouse_location

    def move(self, direction):
        if self.pathway.can_move((self.x, self.y), direction) and self.battery_level > 0:
            self.x, self.y = self.pathway.get_next_position((self.x, self.y), direction)
            self.battery_level -= self.energy_cost(direction)
            # print(f"Robot moved to: {self.x}, {self.y}")  # Debugging print statement
        else:
            print(f"Robot {self.id} encountered an obstacle at {self.x}, {self.y} and could not move {direction}")

    def move_to_destination(self, path):
        for step in path:
            direction = (step[0] - self.x, step[1] - self.y)
            self.move(direction)
            print(f"Robot {self.id} moved to {self.x}, {self.y}")
        
    def recharge(self):
        if self.at_warehouse():
            self.battery_level = self.battery_capacity

    def charge_battery(self, charge_amount):
        self.battery = min(self.battery_capacity, self.battery + charge_amount)
        print(f"Robot {self.id} is charging. New battery level: {self.battery}/{self.battery_capacity}")

    def load_orders(self, orders):
        if self.at_warehouse():
            for order in orders:
                if len(self.compartment) < self.compartment_capacity:
                    bagged_order = self.pick_and_bag_items(order)
                    if bagged_order is not None:
                        self.compartment.append(bagged_order)
        else:
            print(f"Robot at ({self.x}, {self.y}): Cannot load orders outside the warehouse")


    def load_order(self, order):
        items_to_load = [item for item in order.items if len(self.compartment) < self.compartment_capacity]
        self.compartment.extend(items_to_load)
        order.items = [item for item in order.items if item not in items_to_load]
        
    def has_order(self):
        return len(self.compartment) > 0

    def pick_and_bag_items(self, order):
        if self.at_warehouse():
            # Simulate the process of picking and bagging items
            print(f"Robot at ({self.x}, {self.y}): Picking and bagging items for Order {order.id}")
            bagged_order = order.copy()
            bagged_order.is_bagged = True
            return bagged_order
        else:
            print(f"Robot at ({self.x}, {self.y}): Cannot pick and bag items outside the warehouse")
            return None
        
    def unload_orders(self, orders):
        for order in orders:
            if order in self.compartment:
                self.compartment.remove(order)
                self.give_delivery_update("Unloaded", order)

    def deliver_order(self, order):
        if (self.x, self.y) == order.destination:
            order.is_delivered = True
            self.drop()

    def deliver_orders(self, orders):
        self.assign_orders_to_robots(orders)
    
        for robot in self.robots:
            if robot.current_order:
                path_to_destination = robot.find_path((robot.x, robot.y), robot.current_order.destination)
                if path_to_destination:
                    print(f"Robot {robot.id} found a path to the destination")
                    robot.move_to_destination(path_to_destination)
                    robot.deliver_order(robot.current_order)
                else:
                    print(f"Robot {robot.id} could not find a path to the destination")
            else:
                print(f"Robot {robot.id} has no assigned order")

    # def deliver_orders(self):
    #     for order in self.compartment:
    #         if (self.x, self.y) == order.destination:
    #             order.is_delivered = True
    #             self.compartment.remove(order)

    def deliver_multiple_orders(self, orders):
        self.load_orders(orders)
        destinations = [order.destination for order in self.compartment]
        
        for destination in destinations:
            path = self.find_path((self.x, self.y), destination)
            if path:
                self.follow_path(path)
                self.deliver_orders()
                
        return_path = self.find_path((self.x, self.y), self.warehouse_location)
        if return_path:
            self.follow_path(return_path)
            self.recharge()

    def drop(self):
        self.compartment = []

    def energy_cost(self, direction):
        energy_costs = {
            (0, 1): 1,
            (0, -1): 1,
            (-1, 0): 1.2,
            (1, 0): 1.2,
        }
        return energy_costs.get(direction, 0)
    
    def get_neighbors(self, current):
        x, y = current
        neighbors = []
        for move in self.moves:
            new_position = self.pathway.get_next_position((x, y), move)  # Change this line
            if new_position is not None and self.pathway.can_move((x, y), move):  # Pass both start and end positions
                cost = self.energy_cost(move)
                neighbors.append((new_position, cost))
        return neighbors

    # def heuristic(self, position, destination):
    #     return abs(position[0] - destination[0]) + abs(position[1] - destination[1])

    def find_path(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
    
        while frontier:
            current = heapq.heappop(frontier)[1]
    
            if current == goal:
                break
    
            for next in self.pathway.get_neighbors(current):
                new_cost = cost_so_far[current] + self.pathway.get_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current
    
        return self.reconstruct_path(came_from, start, goal)
    
    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)
    
    def reconstruct_path(self, came_from, start, goal):
        if goal not in came_from:
            return None
    
        current = goal
        path = []
    
        while current != start:
            path.append(current)
            current = came_from[current]
    
        path.append(start)
        path.reverse()
        return path

    def follow_path(self, path, order):
        print(f"Path: {path}")  # Debugging print statement
        for i in range(len(path) - 1):
            start, end = path[i], path[i + 1]
            if self.pathway.can_move(start, end):
                self.move(end)
                self.battery_level -= 1
                self.give_delivery_update("Moving", order)
                if self.detect_obstacle(end):
                    self.pathway.update_obstacle(end[0], end[1])
                    new_path = self.find_path((self.x, self.y), path[-1])
                    if new_path:
                        self.follow_path(new_path, order)
                    break
            else:
                print(f"Cannot move from {start} to {end}")  # Debugging print statement
                break
        if order:
            if (self.x, self.y) == order.destination:
                self.give_delivery_update("Delivered", order)
            else:
                self.give_delivery_update("Returning to warehouse")
        else:
            self.give_delivery_update("Returning to warehouse")

    def detect_obstacle(self, end):
        # Implement obstacle detection logic here.
        pass

class DeliverySystem:
    def __init__(self, grid_size, warehouse_location, pathway, battery_capacity, compartment_capacity, num_robots):
        self.grid_size = grid_size
        self.warehouse_location = warehouse_location
        self.pathway_system = pathway
        self.battery_capacity = battery_capacity
        self.compartment_capacity = compartment_capacity
        self.num_robots = num_robots
        self.orders = []
        self.robots = [
            DeliveryRobot(i, pathway, battery_capacity, None, None, compartment_capacity, warehouse_location)
            for i in range(num_robots)
        ]  # Removed extra arguments here

    def add_robot(self, x, y):
        robot = DeliveryRobot(x, y, self.grid_size, self.warehouse_location, self.pathway_system, self.compartment_capacity)
        self.robots.append(robot)

    def receive_order(self, items, destination):
        order = Order(items, destination)
        self.orders.append(order)
        
    def assign_orders_to_robots(self, orders):
        orders_queue = orders[:]
        robots_queue = self.robots[:]
    
        while orders_queue and robots_queue:
            order = orders_queue.pop(0)
            robot = None
    
            for r in robots_queue:
                if len(r.compartment) < r.compartment_capacity and r.at_warehouse():
                    robot = r
                    break
    
            if robot is not None:
                if robot.battery_level < robot.battery_capacity * 0.2:
                    robot.recharge()
                else:
                    robot.compartment.append(order)
                    print(f"Order {order.id} assigned to Robot {robot.id}")
                    robots_queue.remove(robot)
            else:
                break

    def bag_items(self, items):
        sorted_items = sorted(items, key=lambda x: x[1], reverse=True)  # Sort items by size
        freezer_bags = [Bag("freezer", 3) for _ in range(self.num_robots)]  # Assuming 3 items max per freezer bag
        paper_bags = [Bag("paper", 3) for _ in range(self.num_robots)]  # Assuming 3 items max per paper bag

        for item in sorted_items:
            item_bagged = False
            for bag in freezer_bags + paper_bags:
                if bag.add_item(item):
                    item_bagged = True
                    break

            if not item_bagged:
                print(f"Unable to bag item {item[0]}")

        return freezer_bags, paper_bags
                        
    def deliver_orders(self, orders):
        self.assign_orders_to_robots(orders)
        
        while any(order for robot in self.robots for order in robot.compartment):
           for robot in self.robots:
               if not robot.current_order and robot.compartment:
                   robot.current_order = robot.compartment.pop(0)
               
               if robot.current_order is not None:
                   print(f"Robot {robot.id} has order {robot.current_order.id} assigned.")
                   path_to_destination = robot.find_path((robot.x, robot.y), robot.current_order.destination)
                   print(f"Robot {robot.id} found path to destination: {path_to_destination}")
                   robot.move_to_destination(path_to_destination)
                   print(f"Robot {robot.id} delivered order {robot.current_order.id} and is now at location {robot.x}, {robot.y}")
                   path_to_warehouse = robot.find_path((robot.x, robot.y), self.warehouse_location)
                   print(f"Robot {robot.id} found path back to warehouse: {path_to_warehouse}")
                   robot.move_to_destination(path_to_warehouse)
                   print(f"Robot {robot.id} returned to warehouse and is now at location {robot.x}, {robot.y}")
                   robot.current_order = None
            

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

    def charge_idle_robots(self, charge_amount):
        for robot in self.robots:
            if not robot.has_order():
                robot.charge_battery(charge_amount)


class Bag:
    def __init__(self, bag_type, max_items):
        self.bag_type = bag_type
        self.max_items = max_items
        self.items = []

    def can_add_item(self, item):
        if len(self.items) < self.max_items:
            if self.bag_type == "freezer" and item[1] == "frozen":
                return True
            elif self.bag_type == "paper" and item[1] != "frozen":
                return True
        return False

    def add_item(self, item):
        if self.can_add_item(item):
            self.items.append(item)
            return True
        return False



def main():
    # Parameters
    grid_size = (10, 10)
    warehouse_location = (0, 0)
    battery_capacity = 100
    compartment_capacity = 2
    num_robots = 3

    items1 = [("Item A", 2), ("Item B", 3)]
    items2 = [("Item C", 1), ("Item D", 4)]
    items3 = [("Item E", 2), ("Item F", 5)]
    items4 = [("Item G", 2), ("Item H", 3)]
    items5 = [("Item I", 1), ("Item J", 4)]
    items6 = [("Item K", 2), ("Item L", 5)]

    order1 = Order(items1, (3, 3))
    order2 = Order(items2, (1, 1))
    order3 = Order(items3, (2, 3))
    order4 = Order(items4, (0, 3))
    order5 = Order(items5, (1, 3))
    order6 = Order(items6, (2, 1))

    # Create the pathway system
    pathway = PathwaySystem(*grid_size)

    # Create the delivery system with 3 robots
    delivery_system = DeliverySystem(grid_size, warehouse_location, pathway, battery_capacity, compartment_capacity, num_robots)

    # Create some example orders with unique IDs and destinations
    orders = [order1, order2, order3, order4, order5, order6]

    # Assign and deliver the orders using the delivery system
    # delivery_system.deliver_orders(orders)
    
    delivery_system.receive_order(items1, (3, 3))
    delivery_system.receive_order(items2, (4, 4))
    delivery_system.receive_order(items3, (4, 9))
    delivery_system.receive_order(items4, (5, 7))
    delivery_system.receive_order(items5, (1, 8))
    delivery_system.receive_order(items6, (4, 6))

     # Bag items
    delivery_system.bag_items(items1 + items2 + items3)

    # Calculate paths for robots
    delivery_system.assign_orders_to_robots(orders)

    # Add a new obstacle after calculating the initial path
    pathway.add_obstacle(1, 2)

    # Deliver orders and check for obstacles
    delivery_system.deliver_orders(orders)
    
    # Call the charge_idle_robots method with a specific charge amount (e.g., 10)
    delivery_system.charge_idle_robots(10)

if __name__ == '__main__':
    main()