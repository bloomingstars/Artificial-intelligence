import heapq
import sys
import math
from collections import deque
class City:
    def __init__(self, city_name, lon, lat):
        self.name = city_name
        self.lon = lon
        self.lat = lat
        self.f = 0
        self.bwdcost = 0
        # self.dist = dist
        self.adj_cities_dist = {}
        self.adj_cities = []
        self.visited = False

    def add_adj_cities(self, name, distance_diff):
        self.adj_cities_dist[name] = distance_diff
        self.adj_cities.append(name)
name_city_map = {}


# algo = sys.argv[0]
# heuristic = sys.argv[1]
# src = sys.argv[2]
# destn = sys.argv[3]

def route_in_dfs(name_city_map, src, destn):
    stack = deque()
    nodes_expanded = []
    nodes_along_route = []
    stack.append(src)
    size_of_stack = 1
    nodes_along_route.append(src)
    nodes_expanded.append(src)
    while stack:
        if size_of_stack < len(stack):
            size_of_stack = len(stack)
        current_city = stack[-1]
        print("current_city" + current_city)

        if current_city == destn:
            return [len(nodes_expanded), size_of_stack, len(nodes_along_route), nodes_along_route]
        next_city = get_next_node(current_city, nodes_expanded)
        if next_city is not None:
            print("next_city " + next_city)
        if next_city is None:
            stack.pop()
            nodes_along_route.remove(current_city)
        else:
            nodes_along_route.append(next_city)
            nodes_expanded.append(next_city)
            stack.append(next_city)
    return "No route found!"


def parse(filename):
    with open(filename, "r") as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()
            start = line.find('city(')
            if start != -1:
                start = start + 5
                end = line[start:].find(")") + start
                first_comma = line[start + 2:].find(",") + start + 2
                second_comma = line[first_comma + 2:].find(",") + first_comma + 2
                city_name = str(line[start:first_comma].strip())
                lat = float(line[first_comma + 1:second_comma].strip())
                lon = float(line[second_comma + 1:end].strip())
                name_city_map[city_name] = City(city_name, lon, lat)

        for line in lines:
            line = line.strip()
            startroad = line.find('road(')
            end = 0
            while startroad - end != -1:
                startroad = startroad + 5
                end = line[startroad:].find(")") + startroad
                first_comma = line[startroad + 2:].find(",") + startroad + 2
                second_comma = line[first_comma + 2:].find(",") + first_comma + 2
                city_name = str(line[startroad:first_comma].strip())
                adj_city_name = str(line[first_comma + 1:second_comma].strip())
                dist = str(line[second_comma + 1:end].strip())
                name_city_map[city_name].add_adj_cities(adj_city_name, dist)
                name_city_map[adj_city_name].add_adj_cities(city_name, dist)
                startroad = line[end:].find('road(') + end


def calc_fwd_cost(adjcity, destn, sph_heuristic):
    adjcity = name_city_map[adjcity]
    destncity = name_city_map[destn]
    adjlat = adjcity.lat
    adjlon = adjcity.lon
    destnlat = destncity.lat
    destnlon = destncity.lon

    if sph_heuristic is True:
        return heuristic(name_city_map, adjcity, destncity)
    return my_heuristic(adjcity, destncity)


def route_in_A_star(name_city_map, src, destn, spherical_heuristic):
    frontier = []
    expanded = []
    bwdcost = {}
    parent_map = {}
    heuristic = {}
    frontier.append(src)
    heuristic[src] = 0
    bwdcost[src] = 0
    max_length_q = 0
    while len(frontier) > 0:
        if max_length_q < len(frontier):
            max_length_q = len(frontier)
        print(frontier)
        min_cost = sys.maxsize
        next_city = None
        for i in range(0, len(frontier)):
            if heuristic[frontier[i]] < min_cost:
                min_cost = heuristic[frontier[i]]
                next_city = frontier[i]
        frontier.remove(next_city)
        if next_city != destn:
            expanded.append(next_city)
            parent_node = name_city_map[next_city]
            for adjcity in parent_node.adj_cities:
                cost_next_node = int(parent_node.adj_cities_dist[adjcity]) + bwdcost[next_city]
                bwdcost[adjcity] = cost_next_node
                heuristic[adjcity] = calc_fwd_cost(adjcity, destn, spherical_heuristic) + cost_next_node
                if adjcity not in frontier and adjcity not in expanded:
                    frontier.append(adjcity)
                    parent_map[adjcity] = next_city
                if adjcity in frontier and cost_next_node < bwdcost[adjcity]:
                    bwdcost[adjcity] = cost_next_node
        else:
            print("in else")
            parent_list = []
            current = destn
            while current != src:
                parent_list.insert(0, current)
                current = parent_map[current]
            parent_list.insert(0, src)
            return len(expanded), max_length_q, len(parent_list), parent_list


def remove_first_lowest_successor(successors, final_cost):
    lowest_f_in_successors = float('inf')
    lowest_successor = None
    for city in successors:
        if final_cost[city] <= lowest_f_in_successors:
            lowest_f_in_successors = final_cost[city]
            lowest_successor = city
    return lowest_successor
final_path = []



def get_next_node(current_city, nodes_along_route):
    adj_citi = name_city_map[current_city].adj_cities
    print("adj citites: ")
    print(adj_citi)
    print("adj citites end: ")

    for adj in adj_citi:
        if adj not in nodes_along_route:
            return adj
    return None




def new_RBFS_analysis(start,node, flimit, destn, heuristic, parent_node, path,current_path=set(),expandedcities=set(),successorlength=0):
    expandedcities.add(node)
    current_path.add(node)
    print("in rbfs with node : ", node)
    if node == destn:
        current_path.remove(node)
        return node, -1, expandedcities, path,0
    removelist = set()
    if parent_node is not None:
        removelist.add(parent_node)
        # print('removelist : ', removelist)
    immediate_successors = name_city_map[node].adj_cities
    successors = [successor for successor in immediate_successors if successor not in removelist and successor not in current_path]
    print(successors)
    if len(successors) == 0:
        # alreadyvisited.append(node)
        current_path.remove(node)
        return None, float('inf'),expandedcities, path ,0
    for c, v in name_city_map[node].adj_cities_dist.items():
        if c in successors:
            if c!= start:
                bwd_cost = name_city_map[node].bwdcost + int(v)
                name_city_map[c].bwdcost = bwd_cost
            print("vackword cost of ",c," is " ,name_city_map[c].bwdcost)
            h_cost=calc_fwd_cost(c, destn, heuristic)
            fcost=name_city_map[c].bwdcost+h_cost
            name_city_map[c].f = max(fcost,name_city_map[node].f)
            print(c, name_city_map[c].f)
    while True:
        print("successors  of", node, ": ", successors)
        successors.sort(key=lambda x: (name_city_map[x].f))  # Order by lowest f value
        best = successors[0]
        current_path.add(best)
        print('best= ', best)
        if name_city_map[best].f > flimit:
            print("flimit higher for ", best, " of ", name_city_map[best].f)
            print("current path : ",current_path)
            current_path.remove(best)
            return None, name_city_map[best].f,expandedcities, path ,0
        alternative_f = float('inf')
        if len(successors) > 1:
            alternative_f = name_city_map[successors[1]].f
        result, name_city_map[best].f,expandedcities, path,successorlength = new_RBFS_analysis(start,best, min(flimit, alternative_f), destn, heuristic, node,path,current_path)
        if result is not None:
            path.insert(1,best)
            successorlength += len(successors)
            return result,name_city_map[best].f,expandedcities, path,successorlength

def heuristic(dictionary_map, from_city, destn_city):
    heuristic_val = math.sqrt(math.pow(69.5 * (from_city.lat - destn_city.lat), 2) + pow(
        69.5 * math.cos((from_city.lat + destn_city.lat) / 360 * math.pi) * (from_city.lon - destn_city.lon), 2))
    return heuristic_val


def my_heuristic(from_city, destn_city):
    a = math.cos(destn_city.lat) * math.sin(abs(destn_city.lon - from_city.lon))
    b = math.cos(from_city.lat) * math.sin(destn_city.lat) - math.sin(from_city.lat) * math.cos(
        destn_city.lat) * math.cos(abs(destn_city.lon - from_city.lon))
    Angle = math.atan2(a, b)
    angle_in_degree = Angle * 180 / math.pi
    return angle_in_degree / 360 * 24901


# if algo == "astar":
#    route_to_destn = route_in_A_star(name_city_map, src, destn)
# elif algo == "DFS":
# route_to_destn = route_in_DFS(name_city_map, src, destn)

def analysis1_for_A_Star():
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            print(name, a_name, " : ")
            expanded_spherical_heuristic, q_length, len, parent_list_spherical_heuristic = route_in_A_star(name_city_map,
                                                                                                      name, a_name,
                                                                                                      True)
            expanded_own_heuristic, q_length,len, parent_list_own_heuristic = route_in_A_star(name_city_map, name, a_name,
                                                                                          False)
            if len(parent_list_own_heuristic) > len(parent_list_spherical_heuristic):
                print("final : ", parent_list_own_heuristic)
                print("final : ", parent_list_spherical_heuristic)
                return name, a_name

def old_RBFS_analysis(node, flimit, destn, heuristic, parent_node, path,expandedcities=set(),successorlength=0):
    expandedcities.add(node)
    print("in rbfs with node : ", node)
    if node == destn:
        return node, -1, expandedcities, path,0
    removelist = set()
    if parent_node is not None:
        removelist.add(parent_node)
        # print('removelist : ', removelist)
    immediate_successors = name_city_map[node].adj_cities
    successors = [successor for successor in immediate_successors if successor not in removelist]
    print(successors)
    if len(successors) == 0:
        # alreadyvisited.append(node)
        return None, float('inf'),expandedcities, path ,0
    for c, v in name_city_map[node].adj_cities_dist.items():
        if c in successors:
            if name_city_map[node].bwdcost==0:
                bwd_cost = name_city_map[node].bwdcost + int(v)
                name_city_map[c].bwdcost = bwd_cost
                print("vackword cost of ",c," is " ,bwd_cost)
            h_cost=calc_fwd_cost(c, destn, heuristic)
            fcost=name_city_map[c].bwdcost+h_cost
            name_city_map[c].f = max(fcost,name_city_map[node].f)
            print(c, name_city_map[c].f)
    while True:
        print("successors  of", node, ": ", successors)
        successors.sort(key=lambda x: (name_city_map[x].f))  # Order by lowest f value
        best = successors[0]
        print('best= ', best)
        if name_city_map[best].f > flimit:
            print("flimit higher for ", best, " of ", name_city_map[best].f)
            return None, name_city_map[best].f,expandedcities, path ,0
        alternative_f = float('inf')
        if len(successors) > 1:
            alternative_f = name_city_map[successors[1]].f
        result, name_city_map[best].f,expandedcities, path,successorlength = old_RBFS_analysis(best, min(flimit, alternative_f), destn, heuristic, node,path)
        if result is not None:
            path.insert(1,best)
            successorlength += len(successors)
            return result,name_city_map[best].f,expandedcities, path,successorlength



def analysis_2_for_dfs():
    smallest_cities_exp = 2147483647
    smallest_size_of_queue = 2147483647
    largest_cities_exp = 0
    largest_size_of_queue = 0
    average_cities_exp = 0
    average_size_of_queue = 0
    smallest_cities_exp_pair = []
    smallest_size_q_pair = []
    largest_cities_exp_pair = []
    largest_size_q_pair = []
    count = 0
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            if name != a_name:
                print(name, a_name, " : ")
                len_nodes_expanded, size_of_stack, len_nodes_along_route, nodes_along_route = route_in_dfs(
                    name_city_map, name, a_name)
                print(len_nodes_expanded, size_of_stack, len_nodes_along_route, nodes_along_route)
                if smallest_cities_exp > len_nodes_expanded:
                    smallest_cities_exp = len_nodes_expanded
                    smallest_cities_exp_pair = [name, a_name]
                if smallest_size_of_queue > size_of_stack:
                    smallest_size_of_queue = size_of_stack
                    smallest_size_q_pair = [name, a_name]
                if largest_cities_exp < len_nodes_expanded:
                    largest_cities_exp = len_nodes_expanded
                    largest_cities_exp_pair = [name, a_name]
                if largest_size_of_queue < size_of_stack:
                    largest_size_of_queue = size_of_stack
                    largest_size_q_pair = [name, a_name]
                average_cities_exp += len_nodes_expanded
                average_size_of_queue += size_of_stack
                count += 1
    print("smallest_cities_exp_pair : ", smallest_cities_exp_pair, 'of value', smallest_cities_exp)
    print("smallest_size_of_queue : ", smallest_size_q_pair, 'of value', smallest_size_of_queue)
    print("largest_cities_exp : ", largest_cities_exp_pair, 'of value', largest_cities_exp)
    print("largest_size_q_pair : ", largest_size_q_pair, 'of value', largest_size_of_queue)
    print("average_cities_exp", average_cities_exp / count)
    print("average_cities_exp", average_size_of_queue / count)


def analysis2_for_a_star():
    smallest_cities_exp = 2147483647
    smallest_size_of_queue = 2147483647
    largest_cities_exp = 0
    largest_size_of_queue = 0
    average_cities_exp = 0
    average_size_of_queue = 0
    smallest_cities_exp_pair = []
    smallest_size_q_pair = []
    largest_cities_exp_pair = []
    largest_size_q_pair = []
    count = 0
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            if name != a_name:
                print(name, a_name, " : ")
                expanded_len, qlength , len ,queue= route_in_A_star(
                    name_city_map, name, a_name, True)
                if smallest_cities_exp > expanded_len:
                    smallest_cities_exp = expanded_len
                    smallest_cities_exp_pair = [name, a_name]
                if smallest_size_of_queue > qlength:
                    smallest_size_of_queue = qlength
                    smallest_size_q_pair = [name, a_name]
                if largest_cities_exp < expanded_len:
                    largest_cities_exp = expanded_len
                    largest_cities_exp_pair = [name, a_name]
                if largest_size_of_queue < qlength:
                    largest_size_of_queue = qlength
                    largest_size_q_pair = [name, a_name]
                average_cities_exp += expanded_len
                average_size_of_queue += qlength
                count += 1
    print("smallest_cities_exp_pair : ", smallest_cities_exp_pair, 'of value', smallest_cities_exp)
    print("smallest_size_of_queue : ", smallest_size_q_pair, 'of value', smallest_size_of_queue)
    print("largest_cities_exp : ", largest_cities_exp_pair, 'of value', largest_cities_exp)
    print("largest_size_q_pair : ", largest_size_q_pair, 'of value', largest_size_of_queue)
    print("average_cities_exp", average_cities_exp / count)
    print("average_queue_size", average_size_of_queue / count)

def analysis2_for_rbfs():
    smallest_cities_exp = 2147483647
    smallest_size_of_queue = 2147483647
    largest_cities_exp = 0
    largest_size_of_queue = 0
    average_cities_exp = 0
    average_size_of_queue = 0
    smallest_cities_exp_pair = []
    smallest_size_q_pair = []
    largest_cities_exp_pair = []
    largest_size_q_pair = []
    count = 0
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            if name != a_name:
                print(name, a_name, " : ")
                x,y,expanded, path,qlength = callRBFSAnalysis(name,a_name,True)
                expanded_len=len(expanded)
                # qlength=len(queue)
                if smallest_cities_exp > expanded_len:
                    smallest_cities_exp = expanded_len
                    smallest_cities_exp_pair = [name, a_name]
                if smallest_size_of_queue > qlength:
                    smallest_size_of_queue = qlength
                    smallest_size_q_pair = [name, a_name]
                if largest_cities_exp < expanded_len:
                    largest_cities_exp = expanded_len
                    largest_cities_exp_pair = [name, a_name]
                if largest_size_of_queue < qlength:
                    largest_size_of_queue = qlength
                    largest_size_q_pair = [name, a_name]
                average_cities_exp += expanded_len
                average_size_of_queue += qlength
                count += 1
    print("smallest_cities_exp_pair : ", smallest_cities_exp_pair, 'of value', smallest_cities_exp)
    print("smallest_size_of_queue : ", smallest_size_q_pair, 'of value', smallest_size_of_queue)
    print("largest_cities_exp : ", largest_cities_exp_pair, 'of value', largest_cities_exp)
    print("largest_size_q_pair : ", largest_size_q_pair, 'of value', largest_size_of_queue)
    print("average_cities_exp", average_cities_exp / count)
    print("average_queue_size", average_size_of_queue / count)


def analysis4_for_A_star():
    largest_cities_exp = 0
    largest_size_of_queue = 0
    largest_cities_exp_pair = []
    largest_size_q_pair = []
    count = 0
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            if name != a_name:
                print(name, a_name, " : ")
                expanded_len, qlength, len, queue = route_in_A_star(
                    name_city_map, name, a_name, True)
                if largest_cities_exp < expanded_len:
                    largest_cities_exp = expanded_len
                    largest_cities_exp_q_size=qlength
                    largest_cities_exp_pair = [name, a_name]
                if largest_size_of_queue < qlength:
                    largest_size_of_queue = qlength
                    largest_size_q_pair = [name, a_name]
                    largest_size_of_queue_exp=expanded_len

    if(largest_size_of_queue_exp+largest_size_of_queue > largest_cities_exp+largest_cities_exp_q_size):
        print("largest_size_q_pair : ", largest_size_q_pair, 'of value qsize', largest_size_of_queue, ' and exp: ',
              largest_size_of_queue_exp)
    else:
        print("largest_cities_exp : ", largest_cities_exp_pair, 'of value exp:', largest_cities_exp,' and queue size ',largest_cities_exp_q_size)


def analysis4_for_dfs():
    largest_cities_exp = 0
    largest_size_of_queue = 0
    largest_cities_exp_pair = []
    largest_size_q_pair = []
    for (name, city) in name_city_map.items():
        for (a_name, a_city) in name_city_map.items():
            if name != a_name:
                print(name, a_name, " : ")
                expanded_len, qlength, len_nodes, nodes_along_route = route_in_dfs(
                    name_city_map, name, a_name)
                if largest_cities_exp < expanded_len:
                    largest_cities_exp = expanded_len
                    largest_cities_exp_q_size=qlength
                    largest_cities_exp_pair = [name, a_name]

                if largest_size_of_queue < qlength:
                    largest_size_of_queue = qlength
                    largest_size_q_pair = [name, a_name]
                    largest_size_of_queue_exp=expanded_len

    if(largest_size_of_queue_exp+largest_size_of_queue > largest_cities_exp+largest_cities_exp_q_size):
        print("largest_size_q_pair : ", largest_size_q_pair, 'of value qsize', largest_size_of_queue, ' and exp: ',
              largest_size_of_queue_exp)
    else:
        print("largest_cities_exp : ", largest_cities_exp_pair, 'of value exp:', largest_cities_exp,' and queue size ',largest_cities_exp_q_size)

parse("G:\\AI\\aI asssign 2\\usroads.pl")

def callAstar(src, destn , heuristic):
    if heuristic == 0:
        result = route_in_A_star(name_city_map, src, destn, True)
    else:
        result = route_in_A_star(name_city_map, src, destn, False)
    print("result:")
    print("The number of cities expanded : ", result[0],"\n● The maximum size of the queue during search : " , result[1],"\n● The final path length : ",result[2],"\n● The final path represented as a sequence of cities:",result[3])

def callDFS(src,destn):
    result= route_in_dfs(name_city_map,src,destn)
    print("result:")
    print("The number of cities expanded : ", result[0], "\n● The maximum size of the queue during search : ",
          result[1], "\n● The final path length : ", result[2],
          "\n● The final path represented as a sequence of cities:", result[3])

def callRBFSAnalysis(src, destn, heuristic):
    if heuristic == 0:
       # result= new_RBFS_analysis(src,src, float('inf'), destn, True, None, [src],current_path=set(),expandedcities=set())
         result= old_RBFS_analysis(src, float('inf'), destn, True, None, [src],expandedcities=set())

    else:
        result= old_RBFS_analysis(src,src, float('inf'), destn, False, None, [src],expandedcities=set())
    print("The number of cities expanded : ", len(result[2]), "\n● The maximum size of the queue during search : ",
          result[4], "\n● The final path length : ", len(result[3]),
          "\n● The final path represented as a sequence of cities:", result[3])
    return result


# result=get_next_node("albanyNY",name_city_map["albanyNY"].adj_cities)
# result = route_in_RBFS(name_city_map, "winnipeg", "japan", float('inf'), {"winnipeg": 0})
# name_city_map["winnipeg"].f= heuristic(name_city_map,name_city_map# "winnipeg"],name_city_map["japan"])
#result = new_RBFS_analysis("winnipeg",  float('inf'), "japan",[],None,['winnipeg'])
#result= callRBFSAnalysis('raleigh', 'boise',0)
#analysis2_for_rbfs()
#name_city_map['boston'].bwd_cost=0
# result = yesh_rbfs(name_city_map,"boston", "desMoines" ,  float('inf'),None)
#result=callDFS("philadelphia", "newYork")
#result= analysis2_for_a_star()
#result = route_in_A_star(name_city_map, "philadelphia", "newYork",True)
#print(result)
# print(name_city_map["providence"].adj_cities)
# print("result : ", result)
# for (name, city) in name_city_map.items():
#     for (a_name, a_city) in name_city_map.items():
#         print(name, a_name, " : ")
#         result = route_in_A_star(name_city_map, name, a_name)
#         print(result)
# print(analysis1_for_A_Star())
#analysis2_for_rbfs()
#analysis4_for_dfs()

a=sys.argv[2]
c=sys.argv[3]
d=sys.argv[4]
b=sys.argv[1]
if(b=="DFS"):
    callDFS(c,d)
elif(b=="A*"):
    callAstar(c,d,a)
elif(b=="RBFS"):
    callRBFSAnalysis(c,d,a)






























































































































































































































































































