# Grupo 01, Guilherme Ribeiro n 89454, Tiago Mamouros n 89548
import heapq
import copy
from itertools import permutations
import math


class SearchProblem:
    def __init__(self, goal, model, auxheur):
        self.goal = goal
        self.model = model
        self.aux = auxheur

    def search(self, *args, **kwargs):
        init = args[0]
        #
        limit_exp = math.inf
        if kwargs.get("limitexp") is not None:
            limit_exp = kwargs["limitexp"]
        limit_depth = math.inf
        if kwargs.get("limitdepth") is not None:
            limit_depth = kwargs["limitdepth"]
        ticket_list = None
        if kwargs.get("tickets") is not None:
            ticket_list = kwargs["tickets"]
        anyorder = False
        if kwargs.get("anyorder") is not None:
            anyorder = kwargs["anyorder"]
        #
        if len(init) == 1:
            if ticket_list is None:
                return bfs_first(init[0], self.goal[0], self.model, limit_exp)
            else:
                tickets = TicketList(ticket_list[0], ticket_list[1], ticket_list[2])
                limit_depth = ticket_list[0] + ticket_list[1] + ticket_list[2]
                if len(init) == 1:
                    heuristics, transitions = bfs(self.goal[0], limit_depth, self.model)
                    problem = OneAgentTickets(init[0], transitions, heuristics, self.goal[0], tickets, limit_exp)
                    return problem.search(limit_depth)

        else:
            if ticket_list is None:
                heuristics = []
                transitions = []
                for j in range(3):
                    heuristics_aux, transitions_aux = bfs(self.goal[j], math.inf, self.model)
                    heuristics += [heuristics_aux]
                    transitions += [transitions_aux]
                problem = ThreeAgents(init, transitions, heuristics, self.goal, limit_exp)
                return problem.search(math.inf)

            elif not anyorder:
                heuristics = []
                transitions = []
                for j in range(3):
                    heuristics_aux, transitions_aux = bfs(self.goal[j], limit_depth, self.model)
                    heuristics += [heuristics_aux]
                    transitions += [transitions_aux]
                    tickets = TicketList(ticket_list[0], ticket_list[1], ticket_list[2])
                problem = ThreeAgentsTickets(init, transitions, heuristics, self.goal, tickets, limit_exp)
                return problem.search(limit_depth)

            else:
                heuristics = []
                transitions = []
                for j in range(3):
                    heuristics_aux, transitions_aux = bfs(self.goal[j], limit_depth, self.model)
                    heuristics += [heuristics_aux]
                    transitions += [transitions_aux]
                    tickets = TicketList(ticket_list[0], ticket_list[1], ticket_list[2])

                problem = ThreeAgentsRandom(init, transitions, heuristics, self.goal, tickets, limit_exp)
                return problem.search(limit_depth)


def bfs(origin, limit, lis):
    count = 0
    paths = [None] * (len(lis))
    visited = [False] * (len(lis))
    queue = [origin]
    new = [None] * len(lis)
    for i in range(len(lis)):
        new[i] = {}
    visited[origin] = True
    paths[origin] = 0
    while queue:
        s = queue.pop(0)
        count += 1
        for a in lis[s]:
            c = a[0]
            b = a[1]
            if b not in new[s]:
                new[s][b] = 0
            new[s][b] += (2 ** c)
            if not visited[b]:
                queue.append(b)
                paths[b] = (paths[s] + 1)
                visited[b] = True
    return paths, new


def bfs_first(init, goal, list, limit):
    origin = init
    visited = [False] * (len(list))
    paths = [None] * len(list)
    queue = [origin]
    visited[origin] = True
    count = 0
    while queue:
        count += 1
        if count >= limit:
            return None
        s = queue.pop(0)
        count += 1
        for a in list[s]:

            c = a[0]
            b = a[1]
            if not visited[b]:
                queue.append(b)
                visited[b] = True
                paths[b] = [c, s]
                if b == goal:
                    new = []
                    for k in range(count):
                        test = paths[goal]
                        if goal != origin:
                            new.insert(0, [[test[0]], [goal]])
                            goal = test[1]
                        else:
                            new.insert(0, [[], [goal]])
                            return new


def get_prevision_solo(heuristics, p, tickets):
    return heuristics[p] + tickets.get_plays() + 1


class OneAgentTickets:
    def __init__(self, source_num, transitions, heuristics, objective, tickets, limit):
        self.source = Point(source_num, 0, tickets, "")
        self.transitions = transitions
        self.heuristics = heuristics
        self.objective = objective
        self.count = 0
        self.limit = limit

    def is_objective(self, point):
        return point.get_num() == self.objective

    def expand(self, father, lim):
        self.count += 1
        if self.count >= self.limit:
            return None
        if self.is_objective(father):
            return father
        n = father.get_num()
        transitions = self.transitions[n]
        tickets = father.get_tickets()
        queue = PriorityQueue()
        for t in transitions:
            neighbour = t
            prevision = get_prevision_solo(self.heuristics, neighbour, tickets)
            if prevision <= lim:  # It can be only smaller than
                queue.push((transitions[t], neighbour), prevision)
        length = queue.len()
        for i in range(length):
            n = queue.pop()
            curr_tickets = copy.deepcopy(tickets)
            path = get_path(n[0], curr_tickets)
            if path != "":
                p = Point(n[1], father, curr_tickets, path)
                destiny = self.expand(p, lim)
                if destiny is not None:
                    return destiny
        return None

    def search(self, lim_max):
        source = self.source
        lim = self.heuristics[source.get_num()]
        destiny = None
        while lim != lim_max:
            destiny = self.expand(source, lim)
            lim += 1
            if destiny is not None:
                lim = lim_max
        if destiny is None:
            return []
        return traceback(destiny)  # result


class Point:
    def __init__(self, num, father, ticket_list, path):
        self.num = num
        self.father = father
        self.ticket_list = ticket_list
        self.path_to_father = path

    def get_num(self):
        return self.num

    def has_father(self):
        return self.father != 0

    def get_father(self):
        return self.father

    def get_path_to_father(self):
        return self.path_to_father

    def get_number_plays(self):
        return self.ticket_list.get_plays()

    def get_priority(self, h):
        g = self.get_number_plays()
        return g + h

    def get_tickets(self):
        return self.ticket_list

    def tickets_allow(self):
        if self.path_to_father == "":
            return False
        return True


class TriplePoint:
    def __init__(self, n1, n2, n3, tickets):
        self.nodes = [n1, n2, n3]
        self.tickets = tickets

    def get_number_plays(self):
        return (self.tickets.get_plays()) // 3

    def get_tickets(self):
        return self.tickets

    def get_nodes(self):
        return self.nodes


class TripleNode:
    def __init__(self, n1, n2, n3, n_plays):
        self.nodes = [n1, n2, n3]
        self.plays = n_plays

    def get_number_plays(self):
        return self.plays

    def get_nodes(self):
        return self.nodes


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def push(self, point, priority):
        heapq.heappush(self.elements, (priority, point))

    def pop(self):
        element = heapq.heappop(self.elements)
        return element[1]

    def pop_element(self):
        element = heapq.heappop(self.elements)
        return element

    def len(self):
        return len(self.elements)


def add_metro(tickets):
    if tickets.add_metro() and tickets.add_metro_train() and tickets.add_metro_taxi() and tickets.add_metro_train_taxi():
        return "M"
    return ""


def add_train(tickets):
    if tickets.add_train() and tickets.add_metro_train() and tickets.add_train_taxi() and tickets.add_metro_train_taxi():
        return "C"
    return ""


def add_taxi(tickets):
    if tickets.add_taxi() and tickets.add_metro_taxi() and tickets.add_train_taxi() and tickets.add_metro_train_taxi():
        return "T"
    return ""


def add_metro_train(tickets):
    if tickets.add_metro_train_taxi() and tickets.add_metro_train_taxi():
        return "MC"
    return ""


def add_metro_taxi(tickets):
    if tickets.add_metro_taxi() and tickets.add_metro_train_taxi():
        return "MT"
    return ""


def add_train_taxi(tickets):
    if tickets.add_train_taxi() and tickets.add_metro_train_taxi():
        return "CT"
    return ""


def add_metro_train_taxi(tickets):
    if tickets.add_metro_train_taxi():
        return "MCT"
    return ""


path_translator = {
    1: add_taxi,
    2: add_train,
    3: add_train_taxi,
    4: add_metro,
    5: add_metro_taxi,
    6: add_metro_train,
    7: add_metro_train_taxi
}

path_translator2 = {
    1: "T",
    2: "C",
    3: "CT",
    4: "M",
    5: "MT",
    6: "MC",
    7: "MCT"
}


def get_only_path(path):
    return path_translator2.get(path)


def get_path(path, ticket_list):
    add_tickets = path_translator.get(path)
    return add_tickets(ticket_list)


def traceback(destiny):
    solution = []
    current = destiny
    tickets = current.get_tickets()
    decider = tickets.calculate_decider()
    while current.has_father():
        path = current.get_path_to_father()
        transport = decider.get_transport(path)
        transition = [[[transport], [current.get_num()]]]
        solution = transition + solution
        current = current.get_father()
    return [[[], [current.get_num()]]] + solution


simple_path_translator = {
    "M": 2,
    "C": 1,
    "T": 0,
    "MC": 2,
    "MT": 0,
    "CT": 1,
    "MCT": 0
}


def triple_node_traceback(triple_node):
    solution = []
    nodes = triple_node.get_nodes()
    current1 = nodes[0]
    current2 = nodes[1]
    current3 = nodes[2]
    while current1.has_father():
        path1 = current1.get_path_to_father()
        path2 = current2.get_path_to_father()
        path3 = current3.get_path_to_father()
        transport1 = simple_path_translator.get(path1)
        transport2 = simple_path_translator.get(path2)
        transport3 = simple_path_translator.get(path3)
        transition = [
            [[transport1, transport2, transport3], [current1.get_num(), current2.get_num(), current3.get_num()]]]
        solution = transition + solution
        current1 = current1.get_father()
        current2 = current2.get_father()
        current3 = current3.get_father()
    return [[[], [current1.get_num(), current2.get_num(), current3.get_num()]]] + solution


def triple_point_traceback(triple_point):
    solution = []
    nodes = triple_point.get_nodes()
    current1 = nodes[0]
    current2 = nodes[1]
    current3 = nodes[2]
    tickets = triple_point.get_tickets()
    decider = tickets.calculate_decider()
    while current1.has_father():
        path1 = current1.get_path_to_father()
        path2 = current2.get_path_to_father()
        path3 = current3.get_path_to_father()
        transport1 = decider.get_transport(path1)
        transport2 = decider.get_transport(path2)
        transport3 = decider.get_transport(path3)
        transition = [
            [[transport1, transport2, transport3], [current1.get_num(), current2.get_num(), current3.get_num()]]]
        solution = transition + solution
        current1 = current1.get_father()
        current2 = current2.get_father()
        current3 = current3.get_father()
    return [[[], [current1.get_num(), current2.get_num(), current3.get_num()]]] + solution


def get_values(m, c, t, mc, mt, ct):
    for m1 in range(m + 1):
        r = min(m - m1, mt)
        for m2 in range(r + 1):
            c2_max = min(c - mc + m1, c, ct)
            c2_min = mt + ct - t - m2
            if c2_min < 0:
                c2_min = 0
            if c2_max >= c2_min:
                c2 = c2_min
                c1 = mc - m1
                t1 = mt - m2
                t2 = ct - c2
                m3 = m - m1 - m2
                c3 = c - c1 - c2
                t3 = t - t1 - t2
                return m1, m2, c1, c2, t1, t2, m3, c3, t3
    print("Error calculating transports")


class Decider:
    def __init__(self, m, c, t):
        self.dict = {
            "M": [0, 0, m],
            "C": [0, c, 0],
            "T": [t, 0, 0]
        }

    def add(self, key, decisions):
        self.dict[key] = decisions

    def get_transport(self, key):
        connections = self.dict[key]
        length = (len(connections))
        for t in range(length):
            if connections[t] >= 1:
                connections[t] -= 1
                return t
        print("Error choosing transports")


class TicketList:
    def __init__(self, t, c, m):
        self.list = \
            [m, c, t, m + c, m + t, c + t, m + c + t]
        self.max = [m, c, t]
        self.total_plays = m + c + t

    def add_metro(self):
        if self.list[0] >= 1:
            self.list[0] -= 1
            return True
        return False

    def add_train(self):
        if self.list[1] >= 1:
            self.list[1] -= 1
            return True
        return False

    def add_taxi(self):
        if self.list[2] >= 1:
            self.list[2] -= 1
            return True
        return False

    def add_metro_train(self):
        if self.list[3] >= 1:
            self.list[3] -= 1
            return True
        return False

    def add_metro_taxi(self):
        if self.list[4] >= 1:
            self.list[4] -= 1
            return True
        return False

    def add_train_taxi(self):
        if self.list[5] >= 1:
            self.list[5] -= 1
            return True
        return False

    def add_metro_train_taxi(self):
        if self.list[6] >= 1:
            self.list[6] -= 1
            return True
        return False

    def get_plays(self):
        return self.total_plays - self.list[6]

    def calculate_decider(self):
        m = self.list[0]
        c = self.list[1]
        t = self.list[2]
        max_m = self.max[0]
        max_c = self.max[1]
        max_t = self.max[2]
        mc = - self.list[3] + m + c
        mt = - self.list[4] + m + t
        ct = - self.list[5] + c + t
        mct = self.list[6] - (max_m - m) - (max_c - c) - (max_t - t) - mc - mt - ct
        decider = Decider(max_m - m, max_c - c, max_t - t)
        m1, m2, c1, c2, t1, t2, m3, c3, t3 = get_values(m, c, t, mc, mt, ct)
        decider.add("MC", [0, c1, m1])
        decider.add("MT", [t1, 0, m2])
        decider.add("CT", [t2, c2, 0])
        decider.add("MCT", [t3, c3, m3])
        return decider


def get_prevision(heuristics, p, n_plays):
    return heuristics[p] + n_plays + 1


class ThreeAgents:
    def __init__(self, source_nums, transitions, heuristics, objective, limit):
        p1 = Point(source_nums[0], 0, None, "")
        p2 = Point(source_nums[1], 0, None, "")
        p3 = Point(source_nums[2], 0, None, "")
        self.source = TripleNode(p1, p2, p3, 0)
        self.transitions = transitions
        self.heuristics = heuristics
        self.objective = objective
        self.count = 0
        self.limit = limit

    def is_objective(self, point):
        nodes = point.get_nodes()
        for i in range(3):
            if nodes[i].get_num() != self.objective[i]:
                return False
        return True

    def expand(self, node_father, lim):
        self.count += 1
        if self.count == self.limit:
            self.count = 0
            return None
        if self.is_objective(node_father):
            return node_father
        queue = get_possibilities(self.transitions, self.heuristics, node_father, lim)
        queue_len = queue.len()
        n_plays = node_father.get_number_plays()
        for j in range(queue_len):
            node = queue.pop()
            new_node = make_node(node, node_father.get_nodes(), n_plays)
            if new_node is not None:
                destiny = self.expand(new_node, lim)
                if destiny is not None:
                    return destiny

        return None

    def search(self, lim_max):
        source = self.source
        h1 = self.heuristics[0][((source.get_nodes())[0]).get_num()]
        h2 = self.heuristics[1][((source.get_nodes())[1]).get_num()]
        h3 = self.heuristics[2][((source.get_nodes())[2]).get_num()]
        lim = max(h1, h2, h3)
        destiny = None
        while lim != lim_max:
            destiny = self.expand(source, lim)
            lim += 1
            if destiny is not None:
                lim = lim_max
        if destiny is None:
            return []
        return triple_node_traceback(destiny)


def make_node(node, father, n_plays):  # node: ((n1, transport), (n2, transport), (n3, transport))
    node_l = []
    for i in range(3):
        curr_node = node[i]
        path = get_only_path(curr_node[1])
        node_l += [Point(node[i][0], father[i], None, path)]
    return TripleNode(node_l[0], node_l[1], node_l[2], n_plays + 1)


def get_possibilities(transitions_total, heuristics, node_father, lim):
    possibilities = []
    queue = PriorityQueue()
    nodes = node_father.get_nodes()
    n_plays = node_father.get_number_plays()
    for i in range(3):
        p = nodes[i].get_num()
        transitions = transitions_total[i][p]
        neighbours = []
        for t in transitions:
            prevision = get_prevision(heuristics[i], t, n_plays)
            if prevision <= lim:
                neighbours += [(prevision, (t, transitions[t]))]
        possibilities += [neighbours]
    for option1 in possibilities[0]:
        n1 = option1[1][0]
        for option2 in possibilities[1]:
            n2 = option2[1][0]
            if n1 != n2:
                for option3 in possibilities[2]:
                    n3 = option3[1][0]
                    if n1 != n3 and n2 != n3:
                        priority = max(option1[0], option2[0], option3[0])
                        queue.push((option1[1], option2[1], option3[1]), priority)
    return queue


class ThreeAgentsTickets:
    def __init__(self, source_nums, transitions, heuristics, objective, tickets, limit):
        p1 = Point(source_nums[0], 0, None, "")
        p2 = Point(source_nums[1], 0, None, "")
        p3 = Point(source_nums[2], 0, None, "")
        self.source = TriplePoint(p1, p2, p3, tickets)
        self.transitions = transitions
        self.heuristics = heuristics
        self.objective = objective
        self.count = 0
        self.limit = limit

    def is_objective(self, point):
        nodes = point.get_nodes()
        for i in range(3):
            if nodes[i].get_num() != self.objective[i]:
                return False
        return True

    def expand(self, node_father, lim, count):
        self.count += 1
        if self.count == self.limit:
            self.count = 0
            return None

        if self.is_objective(node_father):
            return node_father

        queue = get_possibilities(self.transitions, self.heuristics, node_father, lim)
        queue_len = queue.len()
        for j in range(queue_len):
            node = queue.pop()
            curr_tickets = copy.deepcopy(node_father.get_tickets())
            new_node = make_node_1(node, node_father.get_nodes(), curr_tickets)
            if new_node is not None:
                destiny = self.expand(new_node, lim, count)
                if destiny is not None:
                    return destiny
                elif destiny is False:
                    return destiny
        return None

    def search(self, lim_max):
        source = self.source
        h1 = self.heuristics[0][((source.get_nodes())[0]).get_num()]
        h2 = self.heuristics[1][((source.get_nodes())[1]).get_num()]
        h3 = self.heuristics[2][((source.get_nodes())[2]).get_num()]
        lim = max(h1, h2, h3)
        destiny = None
        while lim != lim_max:
            destiny = self.expand(source, lim, 0)
            lim += 1
            if destiny is not None:
                lim = lim_max
        if destiny is None:
            return []
        return triple_point_traceback(destiny)


def make_node_1(node, father, tickets):  # node: ((n1, transport), (n2, transport), (n3, transport))
    node_l = []
    for i in range(3):
        curr_node = node[i]
        path = get_path(curr_node[1], tickets)
        if path == "":
            return None
        node_l += [Point(node[i][0], father[i], None, path)]
    return TriplePoint(node_l[0], node_l[1], node_l[2], tickets)


class ThreeAgentsRandom:
    def __init__(self, source_nums, transitions, heuristics, objective, tickets, limit):
        self.source = source_nums
        self.transitions = transitions
        self.heuristics = heuristics
        self.objective = objective
        self.tickets = tickets
        self.limit = limit

    def search(self, limit):
        perm = permutations([0, 1, 2])
        value = PriorityQueue()
        span = 0
        for p in perm:
            span += 1
            top = max(self.heuristics[p[0]][self.source[0]], self.heuristics[p[1]][self.source[1]],
                      self.heuristics[p[2]][self.source[2]])
            value.push(p, top)

        minim = math.inf
        best = []
        for i in range(span):
            src = value.pop_element()
            h = src[1]
            priority = src[0]
            if priority >= minim:
                return best
            finalheur = [self.heuristics[h[0]], self.heuristics[h[1]], self.heuristics[h[2]]]
            finalobjective = [self.objective[h[0]], self.objective[h[1]], self.objective[h[2]]]
            final = ThreeAgentsTickets(self.source, self.transitions, finalheur, finalobjective, self.tickets,
                                       self.limit)
            result = final.search(limit)

            if result is not None and len(result) - 1 < minim:
                minim = len(result) - 1
                best = result
        return best