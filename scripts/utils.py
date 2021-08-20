import yaml
import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import fire
import glob
import shutil


# フォルダ名の最後についている/を取り除く
def backslash_remover(folder):
    if (folder[-1:] == "/"):
        folder_name = folder[:-1]
    else:
        folder_name = folder
    return folder_name


#  フォルダ内の特定の拡張子のついたファイルパスをリストで返す
def get_files_list(folder, extention):
    file_list = glob.glob(folder + "/*." + extention)
    return file_list


# ファイルの拡張子を取り除く
def extention_remover(in_file):
    out_file = os.path.splitext(in_file)[0]
    return out_file


#  ファイルパスからファイル名のみを抽出
def extract_file_name(in_filepath):
    out_file_name = os.path.basename(in_filepath)
    return out_file_name


# yamlを読み込み
def read_yaml(yaml_path):
    try:
        with open(yaml_path, 'r') as file:
            obj = yaml.safe_load(file)
            return obj
    except Exception as e:
        print('Exception occurred while loading YAML...', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)


# yaml書き出し
def write_yaml(yaml_obj, output_path):
    with open(output_path, 'w') as file:
        yaml.dump(yaml_obj, file)


# csv読み込み
def load_csv(input_csv):
    return pd.read_csv(input_csv)


# csvを読み込んで2軸描画．x,yには描画したい列のインデックスを入力
def plot_2d(input_csv, x, y):
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
    plt.show()

# 複数のcsvファイルを読み込んで同一の列名のデータを比較する.行は同じ長さであることがのぞましい
def plot_compare(x, y, *input_csvs):
    fig, ax = plt.subplots()
    ax.set_xlabel(x, fontsize=18)
    ax.set_ylabel(y, fontsize=18)
    ax.grid(c='gainsboro', zorder=9)
    title = x + ' - ' + y
    ax.set_title(title, fontsize=18)
    for each_csv in input_csvs:
        df = load_csv(each_csv)
        x_elem = df[x]
        y_elem = df[y]
        name = extention_remover(extract_file_name(each_csv))
        ax.plot(x_elem, y_elem, label=name)
    fig.tight_layout()
    fig.legend()
    plt.show()



# フォルダ内のすべてのcsvの2軸グラフを描画する．すべてのcsvが同一のフォーマットで書かれていること
def plot_2d_folder(input_folder, x, y):
    inFolder = backslash_remover(input_folder)
    csv_list = get_files_list(inFolder, 'csv')
    fig, ax = plt.subplots()
    ax.set_xlabel(x, fontsize=18)
    ax.set_ylabel(y, fontsize=18)
    ax.grid(c='gainsboro', zorder=9)
    name = x + ' - ' + y
    ax.set_title(name, fontsize=18)
    for each_csv in csv_list:
        df = load_csv(each_csv)
        x_elem = df[x]
        y_elem = df[y]
        ax.plot(x_elem, y_elem)
    fig.tight_layout()
    plt.show()


# 該当のフォルダ(in_folder)，数字or文字のlist(list_num)と拡張子(extention)を読み込んで，フォルダ内のlist要素の名前と同じフアイルを抽出する
# 失敗したケースのyamlの抽出に使っている
# ex) python3 utils.py extract_list_file /home/honda/Desktop/multiple_simulation/non-const-speed_v2/yaml/ "yaml" `[1, 2, 3]`
#  注): CLI実行の場合，elem_listの引数を'[]' で与えること
def extract_list_file(in_folder, extention, elem_list):
    in_folder = backslash_remover(in_folder)
    extract_folder = in_folder + "/extract"
    if not os.path.exists(extract_folder):
        os.mkdir(extract_folder)

    extract_fileName_list = []
    for elem in elem_list:
        extract_fileName_list.append("/" + str(elem) + "." + extention)

    extract_filePath_list = [in_folder + s for s in extract_fileName_list]

    for file in get_files_list(in_folder, extention):
        if (file in extract_filePath_list):
            try:
                new_file_path = file.replace(in_folder, extract_folder)
                shutil.copy(file, new_file_path)
            except FileNotFoundError:
                pass
            except OSError:
                pass


def is_None_dict(f : dict):
    is_None = True
    for value in f.values():
        if value or value==0:
            continue
        else:
            is_None=False
    return is_None

if __name__ == "__main__":
    fire.Fire()
