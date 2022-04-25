class Node:
    def __init__(self, node_state, parent_node=None, cost_to_come=0.0, childs=None, action=None):
        if childs is None:
            childs = []
        self.node_state = node_state
        self.parent_node = parent_node
        self.cost_to_come = cost_to_come
        self.childs = childs
        self.action = action


def get_action_set(rpm1, rpm2):

    action_set = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    return action_set


def backtrack(final_node):
    path = []
    while final_node.parent_node is not None:
        path.append([final_node.node_state, final_node.action])
        final_node = final_node.parent_node

    path.reverse()

    return path

