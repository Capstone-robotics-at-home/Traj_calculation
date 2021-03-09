from JetbotPy import Jetbot
from Astar import Astar
from Path_Utils import plotting


def Astar_search(objects, jetbot):
    """ Searching conducting
    :return: path points """
    obstacle_ls = objects['Obstacle']
    s_start = jetbot.get_position()
    s_goal = objects['Target'][0]
    if type(obstacle_ls[0]) == type(()):  # if there is only one obstacle:
        obstacle_ls = [obstacle_ls]

    astar = Astar(s_start, s_goal, obstacle_ls)
    path_sol, visited = astar.searching()
    return path_sol


def get_obs_set(obstacle_list):
    if obstacle_list == []:
        raise ValueError('Obstacle list is empty')
    obs = set()
    for o in obstacle_list:
        # the 4 parameters of the obstacle 'box'
        left, right, top, bottom = o[-4:]
        for x in range(left, right+1):
            for y in range(bottom, top+1):
                obs.add((x, y))

    return obs

def traj_sim(objects):
    jetbot_pos = objects['Jetbot'][0]
    grab_pos = objects['Grabber'][0]
    bot = Jetbot(jetbot_pos, grab_pos)

    obstacle_ls = objects['Obstacle']
    s_start = objects['Jetbot'][0]
    s_goal = objects['Target'][0]
    if type(obstacle_ls[0]) == type(()):  # if there is only one obstacle:
        obstacle_ls = [obstacle_ls]
    astar = Astar(s_start, s_goal, obstacle_ls)
    Original_path, visited = astar.searching()

    plot = plotting.Plotting(s_start, s_goal, obstacle_ls)
    plot.animation(Original_path, visited, 'AStar')

    path = Original_path
    obs_set = get_obs_set(obstacle_ls)

    while len(path) > bot.Horizon:
        bot.jetbot_step(path, obs_set)
        path = Astar_search(objects, bot)


    trajectory = bot.get_trajectory()
    print('Terminate, Total number of movements is: %d' % len(trajectory))
    plot.plot_traj(Original_path, trajectory)


if __name__ == '__main__':
    objects = {'Jetbot': [(953, 461), 834, 1073, 636, 287], 
            'Obstacle': [(1100, 300), 1000,1200,800,200], 
            'Target': [(1342, 270), 1308, 1377, 249, 92], 
            'Grabber': [(1054, 626), 1003, 1106, 728, 525]}


    objects = {'Jetbot': [(210, 462), 107, 314, 577, 347],
               'Obstacle': [(758, 292), 693, 823, 388, 180],
               'Target': [(1070, 199), 1036, 1105, 256, 143],
               'Grabber': [(174, 591), 141, 207, 660, 523]}

    traj_sim(objects)
    