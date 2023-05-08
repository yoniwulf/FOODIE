import copy

class Item:
	def __init__(self, name, size, fragile, frozen):
		self.name = name
		self.size = size
		self.fragile = fragile
		self.frozen = frozen


class Order:
	def __init__(self, order_id):
		self.id = order_id
		self.regItemsSize = 0
		self.frozenItemsSize = 0
		self.regItemList = []
		self.frozenItemList = []
		self.is_bagged = False

	def addItem(self, item):
		tempItem = copy.deepcopy(item)
		
		if item.frozen:
			self.frozenItemsSize += item.size
			self.frozenItemList.append(tempItem)
		else:
			self.regItemsSize += item.size
			self.regItemList.append(tempItem)


class Bag:
	def __init__(self, name, frozen, items):
		self.name = name
		self.frozen = frozen
		self.order = None
		self.items = items

	def printBagItems(self):
		for item in self.items:
			if item.frozen:
				print("Rule 1: Pack frozen item: " + item.name + " in bag " + str(self.name))
			
			elif item.fragile:
				print("Rule 5: Pack fragile item: " + item.name + " in bag " + str(self.name))

			elif item.size == 3:
				print("Rule 2: Pack big item: " + item.name + " in bag " + str(self.name))

			elif item.size == 2:
				print("Rule 3: Pack medium item: " + item.name + " in bag " + str(self.name))
			
			elif item.size == 1:
				print("Rule 4: Pack small item: " + item.name + " in bag " + str(self.name))
				

class Robot:
	def __init__(self, id):
		self.id = id
		self.orders = []
		self.bags = []

	def addOrder(self, order):
		self.orders.append(order)

	def addBag(self, bag):
		tempBag = copy.deepcopy(bag)
		self.bags.append(tempBag)


class Bagger:
	def __init__(self, bagsize):
		self.bagsize = bagsize
		self.regList = []
		self.frozenList = []
		self.frag_queue = []
		self.med_queue = []
		self.small_queue = []
		self.numBags = 1

	def bagOrder(self, order, robot):
		# check order sizes
		self.regList = []
		self.frozenList = []
		self.frag_queue = []
		self.med_queue = []
		self.small_queue = []
		if (order.regItemsSize > self.bagsize) or (order.frozenItemsSize > self.bagsize):
			print("Rule 6: Order too big for bag size")
			return False
		
		elif len(robot.bags) == 6:
			print("Rule 7: Robot full")
			return False
		
		else:
			# sort order
			if order.frozenItemsSize > 0:
				self.frozenList = copy.deepcopy(order.frozenItemList)

			for item in order.regItemList:
				if item.fragile == True:
					self.frag_queue.append(item)
				
				elif item.size == 3:
					self.regList.append(item)
				
				elif item.size == 2:
					self.med_queue.append(item)

				else:
					self.small_queue.append(item)
			
			self.regList = self.regList + self.med_queue + self.small_queue + self.frag_queue

			tempBagName = self.numBags
			tempFrozenBag = Bag(tempBagName, True, self.frozenList)
			self.numBags += 1

			tempBagName = self.numBags
			tempRegBag = Bag(tempBagName, False, self.regList)
			self.numBags += 1

			robot.addOrder(order)
			robot.addBag(tempFrozenBag)
			robot.addBag(tempRegBag)
				

def main():
	bagSize = 10
	bagsPerBot = 6
	numBots = 3
	botList = []
	orderList = []

	robot1 = Robot("Robot 1")
	robot2 = Robot("Robot 2")
	robot3 = Robot("Robot 3")

	item1 = Item("Flowers", 1, True, False)
	item2 = Item("Watermelon", 3, False, False)
	item3 = Item("Water 1G", 3, False, False)
	item4 = Item("Grapes", 2, True, False)
	item5 = Item("Ice Cream", 1, False, True)
	item6 = Item("Bread", 2, True, False)
	item7 = Item("Butter", 1, False, False)
	item8 = Item("Candy Bar", 1, False, False)
	item9 = Item("Frozen Pizza", 3, False, True)
	item10 = Item("Pretzels", 2, False, False)
	item11 = Item("Water 1G", 3, False, False)
	item12 = Item("Eggs", 2, True, False)
	item13 = Item("Apples", 1, True, False)
	item14 = Item("Steak", 2, True, True)
	item15 = Item("Paper Towels", 2, False, False)
	item16 = Item("Chips", 2, False, False)
	item17 = Item("Icepack", 2, False, True)
	item18 = Item("Salt", 1, False, False)
	item19 = Item("Fish Food", 2, False, False)

	order1 = Order("Order 1")
	order1.addItem(item1)
	order1.addItem(item2)
	order1.addItem(item4)
	order1.addItem(item5)
	order1.addItem(item9)
	order2 = Order("Order 2")
	order2.addItem(item3)
	order2.addItem(item6)
	order2.addItem(item7)
	order2.addItem(item8)
	order2.addItem(item15)
	order3 = Order("Order 3")
	order3.addItem(item11)
	order3.addItem(item10)
	order3.addItem(item12)
	order3.addItem(item13)
	order3.addItem(item14)
	order4 = Order("Order 4")
	order4.addItem(item16)
	order4.addItem(item17)
	order4.addItem(item18)
	order4.addItem(item19)

	bagger = Bagger(10)
	
	bagger.bagOrder(order1, robot1)
	bagger.bagOrder(order2, robot1)
	bagger.bagOrder(order3, robot2)
	bagger.bagOrder(order3, robot3)

	print("\nRobot 1 orders:")
	for bag in robot1.bags:
		bag.printBagItems()
	
	print("\nRobot 2 orders:")
	for bag in robot2.bags:
		bag.printBagItems()
	
	print("\nRobot 3 orders")
	for bag in robot3.bags:
		bag.printBagItems()

if __name__ == '__main__':
	main()