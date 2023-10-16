#!/usr/bin/env python3

# import csv
# import matplotlib.pyplot as plt

# data = []

# # CSVファイルを読み込みます
# with open('pos_data.csv', 'r') as f:
#     reader = csv.reader(f)
#     for row in reader:
#         data = list(map(float, row))  # 文字列を浮動小数点数に変換します

# # データをプロットします
# plt.plot(data)
# plt.show()

import csv
import matplotlib.pyplot as plt

def read_csv(filename):
    data = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            data.append(list(map(float, row)))
    return data

# CSVファイルを読み込む
pos_data = read_csv('pos_data.csv')
vel_data = read_csv('vel_data.csv')

# データをtransposeする（列を行に変換する）
pos_data = list(map(list, zip(*pos_data)))
vel_data = list(map(list, zip(*vel_data)))

# 各列ごとにデータをプロットする
# for i in range(4):
#     plt.figure()  # 新しい図を作成する
#     plt.plot(pos_data[i], label='Position')
#     plt.plot(vel_data[i], label='Velocity')
#     plt.legend()  # 凡例を表示する
#     plt.show()

t = [i/30 for i in range(len(pos_data[0]))]

plt.subplot(2,2,1)  # 新しい図を作成する
plt.plot(t, pos_data[0], label='q')
plt.plot(t, vel_data[0], label='dq')
plt.legend()  # 凡例を表示する
plt.xlabel('time[s]') 
plt.ylabel('q[rad], dq[rad/s]') 
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 


plt.subplot(2,2,2)  # 新しい図を作成する
plt.plot(t, pos_data[1], label='q')
plt.plot(t, vel_data[1], label='dq')
plt.legend()  # 凡例を表示する
plt.xlabel('time[s]') 
plt.ylabel('q[rad], dq[rad/s]') 
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.subplot(2,2,4)  # 新しい図を作成する
plt.plot(t, pos_data[2], label='q')
plt.plot(t, vel_data[2], label='dq')
plt.legend()  # 凡例を表示する
plt.xlabel('time[s]') 
plt.ylabel('q[rad], dq[rad/s]') 
plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.subplot(2,2,3)  # 新しい図を作成する
plt.plot(t, pos_data[3], label='q')
plt.plot(t, vel_data[3], label='dq')
plt.legend()  # 凡例を表示する
plt.xlabel('time[s]') 
plt.ylabel('q[rad], dq[rad/s]') 

plt.xlim([-0, 30])  # x軸の範囲を0から最大時間までに設定します
plt.ylim([-2.5, 1.5]) 

plt.show()
