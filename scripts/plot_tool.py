# Plot and animation tools
import fire
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import os
import utils


def load_csv(input_csv):
    return pd.read_csv(input_csv)


def mode_to_color(mode):
    if mode == 0:
        return 'lime'
    elif mode == 1:
        return 'lime'
    elif mode == 2:
        return 'red'
    elif mode == 3:
        return 'cyan'


# TODO : 可変長引数に拡張したいね
def movie_new(input_csv, save=False, rate=100, *args):
    """ Movie from csv
        Usage: python3 plot_tools.py movei <input_csv> save=T/F x0 y0 x1 y1 x2 y2... 
    """
    assert len(args)%2 == 0, 'Arguments must be multiple (x,y)'
    df = load_csv(input_csv)
    point_num = int(len(args)/2)
    length = len(df[args[0]])

    x_mat = [df[args[2*i]] for i in range(point_num)]# xo, x1, x2, ...
    y_mat = [df[args[2*i+1]] for i in range(point_num)]# y0, y1, y2,...
 
    fig = plt.figure(figsize=(20, 2))
    fig.add_subplot(111, fc='0.8')

    anim = []
    for n in range(0, length, 1):
        for i in range(point_num):
            im =plt.plot(x_mat[i][n],y_mat[i][n], marker='o', markersize=10, aa=True)
        anim.append(im)
    
    anim = animation.ArtistAnimation(
        fig, anim, interval=rate, repeat=True)

    plt.xlabel('$x$[m]', fontsize=12)
    plt.ylabel('$y$[m]', fontsize=12)
    xmin, xmax = 0, 100
    plt.xlim(xmin, xmax)
    plt.ylim(-1, 4)
    plt.hlines([1.5, 1.5], xmin, xmax, "w", linestyles='dashed')

  
    plt.tight_layout()
    plt.show()

    if save:
        anim.save(os.path.splitext(input_csv)[
                  0] + '.gif', writer='imagemagick')



def movie_frenet(input_csv, save=False, rate=100, anim_repeat=True, multi_sim_mode=False):
    df = load_csv(input_csv)
    x_ego = df['x_f'].tolist()
    y_ego = df['y_f'].tolist()
    x_ped = df['ped_x_f'].tolist()
    y_ped = df['ped_y_f'].tolist()

    fig = plt.figure(figsize=(20, 2))
    fig.add_subplot(111, fc='0.8')
  
    anim = []
    length = len(x_ego)
    fps = 10

    for n in range(0, length, fps):
        im = plt.plot(x_ego[n], y_ego[n], 'red',
                      x_ped[n], y_ped[n], 'black',
                      marker='o', markersize=10, aa=True)
        anim.append(im)

    anim = animation.ArtistAnimation(
        fig, anim, interval=rate, repeat=anim_repeat)

    plt.xlabel('$x_F$[m]', fontsize=12)
    plt.ylabel('$y_F$[m]', fontsize=12)
    xmin, xmax = 0, 100
    plt.xlim(xmin, xmax)
    plt.ylim(-2, 2)
    plt.hlines([0, 0], xmin, xmax, "w", linestyles='dashed')


    plt.tight_layout()
    plt.show()

    if save:
        anim.save(os.path.splitext(input_csv)[
                  0] + '.mp4', writer='ffmpeg', fps = fps)

def movie_global(input_csv, save=False, rate=100, anim_repeat=True, multi_sim_mode=False):
    df = load_csv(input_csv)
    x_ego = df['x_g'].tolist()
    y_ego = df['y_g'].tolist()
    x_ped = df['ped_x_g'].tolist()
    y_ped = df['ped_y_g'].tolist()

    fig = plt.figure(figsize=(10, 10))
    fig.add_subplot(111, fc='0.8')
  
    anim = []
    length = len(x_ego)
    fps = 10

    for n in range(0, length, fps):
        im = plt.plot(x_ego[n], y_ego[n], 'red',
                      x_ped[n], y_ped[n], 'black',
                      marker='o', markersize=10, aa=True)
        anim.append(im)

    anim = animation.ArtistAnimation(
        fig, anim, interval=rate, repeat=anim_repeat)

    plt.xlabel('$x_G$[m]', fontsize=12)
    plt.ylabel('$y_G$[m]', fontsize=12)
    xmin, xmax = 0, 50
    ymin, ymax = 0, 100
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # plt.hlines([0, 0], xmin, xmax, "w", linestyles='dashed')


    plt.tight_layout()
    plt.show()

    if save:
        anim.save(os.path.splitext(input_csv)[
                  0] + '.mp4', writer='ffmpeg', fps = fps)


def plot(input_csv, x, y):
    df = load_csv(input_csv)
    x_elem = df[x]
    y_elem = df[y]
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111)
    ax.plot(x_elem, y_elem)
    ax.set_xlabel(x, fontsize=18)
    ax.set_ylabel(y, fontsize=18)
    ax.grid(c='gainsboro', zorder=9)
    name = x + ' - ' + y
    plt.title(name, fontsize=18)
    plt.tight_layout()
    plt.show()

def plot_with_ref(input_result, input_ref):
    fig, ax = plt.subplots()
    ax.set_xlabel('$X_G[m]$', fontsize=18)
    ax.set_ylabel('$Y_G[m]$', fontsize=18)
    ax.grid(c='gainsboro', zorder=9)
    # title = x + ' - ' + y
    # ax.set_title(title, fontsize=18)
    df_result = load_csv(input_result)
    df_ref = load_csv(input_ref)
    xg_result = df_result['x_g']
    yg_result = df_result['y_g']
    xg_ref = df_ref['reference.x']
    yg_ref = df_ref['reference.y']
    ax.plot(xg_ref, yg_ref, label="reference path", linestyle="dotted")
    ax.plot(xg_result, yg_result, label="result")
    
    fig.tight_layout()
    fig.legend()
    plt.show()

def plot_compare(x,y,*input_csvs):
    utils.plot_compare(x,y,*input_csvs)

def plot_all(input_folder, x, y):
    utils.plot_2d_folder(input_folder, x, y)


def plot_for_paper(input_csv):
    df = load_csv(input_csv)
    sampling_rate_for_path = 10
    sampling_rate_for_marker = 1000
    df_path = df[::sampling_rate_for_path]
    df_marker = df[::sampling_rate_for_marker]
    
    # Prepare Path data
    x_ego_mode_1 = df_path[(df['mode'] == 1) | (df['mode'] == 0)]['x'].tolist()
    y_ego_mode_1 = df_path[(df['mode'] == 1) | (df['mode'] == 0)]['y'].tolist()
    x_ego_mode_2 = df_path[df['mode'] == 2]['x'].tolist()
    y_ego_mode_2 = df_path[df['mode'] == 2]['y'].tolist()
    x_ego_mode_3 = df_path[df['mode'] == 3]['x'].tolist()
    y_ego_mode_3 = df_path[df['mode'] == 3]['y'].tolist()

    # Prepare marker data
    x_car1_mark = df_marker['car1_x'].tolist()
    y_car1_mark = df_marker['car1_y'].tolist()
    x_car2_mark = df_marker['car2_x'].tolist()
    y_car2_mark = df_marker['car2_y'].tolist()
    x_car3_mark = df_marker['car3_x'].tolist()
    y_car3_mark = df_marker['car3_y'].tolist()
    x_ego_mode_1_mark = df_marker[(df['mode'] == 1) | (df['mode'] == 0)]['x'].tolist()
    y_ego_mode_1_mark = df_marker[(df['mode'] == 1) | (df['mode'] == 0)]['y'].tolist()
    x_ego_mode_2_mark = df_marker[df['mode'] == 2]['x'].tolist()
    y_ego_mode_2_mark = df_marker[df['mode'] == 2]['y'].tolist()
    x_ego_mode_3_mark = df_marker[df['mode'] == 3]['x'].tolist()
    y_ego_mode_3_mark = df_marker[df['mode'] == 3]['y'].tolist()
    
    # Prepare fig
    fig = plt.figure(figsize=(20, 2))
    ax = fig.add_subplot(111, fc='0.8')

    #  Plot path
    path_size = 1
    ax.scatter(x_ego_mode_1, y_ego_mode_1, marker='>', s=path_size, c='green')
    ax.scatter(x_ego_mode_2, y_ego_mode_2, marker='>', s=path_size, c='blue')
    ax.scatter(x_ego_mode_3, y_ego_mode_3, marker='>', s=path_size, c='red')

    #  Plot marker
    path_size = 100
    ax.scatter(x_ego_mode_1_mark, y_ego_mode_1_mark, marker='>', s=path_size, c='green')
    ax.scatter(x_ego_mode_2_mark, y_ego_mode_2_mark, marker='>', s=path_size, c='blue')
    ax.scatter(x_ego_mode_3_mark, y_ego_mode_3_mark, marker='>', s=path_size, c='red')
    ax.scatter(x_car1_mark, y_car1_mark, marker='x', s=path_size, c='black')
    ax.scatter(x_car2_mark, y_car2_mark, marker='o', s=path_size, c='black')
    ax.scatter(x_car3_mark, y_car3_mark, marker='s', s=path_size, c='black')
    
    # plot center line
    x_ego = df_path['x'].tolist()
    mergin = 30
    centerline_x = np.linspace(0-mergin, math.floor(x_ego[-1])+mergin)
    centerline_y = 0*centerline_x + 1.5
    ax.plot(centerline_x, centerline_y, color='w', linestyle='--', linewidth='3.0')
    plt.xlim(0-mergin, math.floor(x_ego[-1])+mergin)
    plt.ylim(-1, 4)

    ax.set_xlabel('$x_G$[m]', fontsize=13)
    ax.set_ylabel('$y_G$[m]', fontsize=13)
    ax.tick_params(labelsize=12)
    plt.tight_layout()
    plt.show()

def plot_frenet_euclid(input_csv):
    df = load_csv(input_csv)
    # frenet coordinate
    x_ego_f = df['x_f'].tolist()
    y_ego_f = df['y_f'].tolist()
    x_ped_f = df['ped_x_f'].tolist()
    y_ped_f = df['ped_y_f'].tolist()
    # global coordinate 
    x_ego_g = df['x_g'].tolist()
    y_ego_g = df['y_g'].tolist()
    x_ped_g = df['ped_x_g'].tolist()
    y_ped_g = df['ped_y_g'].tolist()

    # dist
    dist_f = []
    dist_g = []

    for n in range(len(x_ego_f)):
        dist = math.sqrt((x_ego_f[n]-x_ped_f[n])**2 + (y_ego_f[n]-y_ped_f[n])**2)
        dist_f.append(dist)

    for n in range(len(x_ego_g)):
        dist = math.sqrt((x_ego_g[n]-x_ped_g[n])**2 + (y_ego_g[n]-y_ped_g[n])**2)
        dist_g.append(dist)

    assert(len(dist_f) == len(dist_g))
    
    time = df["t"].tolist()
    assert(len(time)==len(dist_f))

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111)
    ax.plot(time, dist_f, label="Frenet coordinate")
    ax.plot(time, dist_g, label="Euclid coordinate")
    ax.set_xlabel("time[s]", fontsize=18)
    ax.set_ylabel("distance [m]", fontsize=18)
    ax.grid(c='gainsboro', zorder=9)
    name = 'Compare distance between frenet and euclid coordinate'
    plt.title(name, fontsize=18)
    plt.legend()
    plt.tight_layout()
    plt.show()




# Calclation average calc_time[msec] for each mode
def ave_calc_time(input_csv, is_display=True):
    df = load_csv(input_csv)
    calc_time_list_acc = df[df['mode'] == 1]['calc_time[msec]']
    calc_time_list_acc2lc = df[df['mode'] == 2]['calc_time[msec]']
    calc_time_list_lc = df[df['mode'] == 3]['calc_time[msec]']

    if (is_display == True):
        print(
            'Average calclation time of ACC mode : %f [msec]' % calc_time_list_acc.mean())
        print(
            'Average calclation time of ACC2LC mode : %f [msec]' % calc_time_list_acc2lc.mean())
        print(
            'Average calclation time of LC mode : %f [msec]' % calc_time_list_lc.mean())

    mean_time = {'ACC': calc_time_list_acc.mean(
    ), 'ACC2LC': calc_time_list_acc2lc.mean(), 'LC': calc_time_list_lc.mean()}
    return mean_time

# Calclation average calc_time[msec] for each mode of all results in a folder


def ave_calc_time_folder(input_folder):
    input_folder = utils.backslash_remover(input_folder)
    result_csv_list = utils.get_files_list(input_folder, 'csv')
    all_num_result = len(result_csv_list)
    all_num_lc = len(result_csv_list)  # LCはnanの可能性があるので分母を調整する必要有る
    sum_acc_time = 0
    sum_acc2lc_time = 0
    sum_lc_time = 0
    for each_csv in result_csv_list:
        mean_time = ave_calc_time(each_csv, is_display=False)
        sum_acc_time += mean_time['ACC']
        sum_acc2lc_time += mean_time['ACC2LC']
        if (math.isnan(mean_time['LC']) == True):
            all_num_lc = all_num_lc - 1
        else:
            sum_lc_time += mean_time['LC']

    calc_tiem_acc = sum_acc_time / all_num_result
    calc_tiem_acc2lc = sum_acc2lc_time / all_num_result
    cal_time_lc = sum_lc_time / all_num_lc

    print(
        'Average calclation time of ACC mode : %f [msec]' % calc_tiem_acc)
    print(
        'Average calclation time of ACC2LC mode : %f [msec]' % calc_tiem_acc2lc)
    print('Average calclation time of LC mode : %f [msec]' % cal_time_lc)


if __name__ == "__main__":
    fire.Fire()
