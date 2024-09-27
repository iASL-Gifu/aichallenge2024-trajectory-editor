import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backend_bases import MouseButton
import tkinter as tk
from tkinter import filedialog

class PlotTool:
    def __init__(self, master):
        self.master = master
        self.master.title("CSV Plot Tool")
        
        # CSVファイルのロードボタン
        self.load_button = tk.Button(master, text="Load CSV", command=self.load_csv)
        self.load_button.pack()

        # ラベル表示のチェックボタン
        self.show_labels_var = tk.BooleanVar(value=False)  # ラベルの表示・非表示を管理する変数
        self.show_labels_checkbutton = tk.Checkbutton(master, text="Show Labels", variable=self.show_labels_var, command=self.plot_data)
        self.show_labels_checkbutton.pack()

        # Matplotlibの図と軸を設定
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.patch.set_facecolor('white')  # 図の背景を白色に設定
        self.ax.set_facecolor('white')         # 軸の背景を白色に設定

        # Matplotlibの図をTkinterに埋め込む
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack()

        # イベントハンドラの設定
        self.cid_press = self.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_release = self.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cid_dblclick = self.canvas.mpl_connect('button_press_event', self.on_double_click)
        self.cid_scroll = self.canvas.mpl_connect('scroll_event', self.on_scroll)  # スクロールイベント

        self.x, self.y, self.labels = [], [], []
        self.texts = []  # ラベル表示用のテキスト
        self.selected_point = None
        self.selected_line = None
        self.active_label = None  # アクティブなラベルの保持

        # ズーム倍率の初期化
        self.zoom_scale = 1.1
        self.pan_active = False   # パン（画面移動）状態のフラグ
        self.pan_start = None     # パンの開始位置を記録する

    def load_csv(self):
        file_path = filedialog.askopenfilename()
        if not file_path:
            return
        self.x, self.y, self.labels = [], [], []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                self.x.append(float(row[0]))
                self.y.append(float(row[1]))
                self.labels.append(row[2])  # 3列目のデータをラベルとして保持
        
        # 軸の範囲をデータに合わせて調整
        self.ax.set_xlim(min(self.x) - 5, max(self.x) + 5)
        self.ax.set_ylim(min(self.y) - 5, max(self.y) + 5)
        
        self.plot_data()

    def plot_data(self):
        # 現在のxlimとylimを保存
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        self.ax.clear()  # 現在のプロットをクリア
        self.ax.set_facecolor('white')  # 軸の背景を白色で維持
        self.ax.plot(self.x, self.y, 'o', picker=5)  # 点をプロット
        self.ax.plot(self.x, self.y, '-')            # 点を線で接続

        # 既存のテキスト（ラベル）を削除
        for text in self.texts:
            text.remove()
        self.texts = []

        # チェックボタンの状態に応じてラベルを表示
        if self.show_labels_var.get():
            for i in range(len(self.x)):
                txt = self.ax.text(self.x[i], self.y[i], self.labels[i], fontsize=12, ha='right', color='blue')
                self.texts.append(txt)

        # 保存していたxlimとylimを再設定
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        self.canvas.draw()  # 描画を更新

    def on_click(self, event):
        if event.inaxes != self.ax: return
        if event.button == MouseButton.MIDDLE:
            self.selected_point = self.find_nearest_point(event.xdata, event.ydata)
        elif event.button == MouseButton.RIGHT:
            self.selected_line = self.find_nearest_line(event.xdata, event.ydata)
            if self.selected_line is None:
                # 左クリックで画面移動を開始
                self.pan_active = True
                self.pan_start = (event.xdata, event.ydata)

    def on_release(self, event):
        self.selected_point = None
        self.selected_line = None
        self.pan_active = False  # 画面移動を終了

    def on_motion(self, event):
        if self.selected_point is not None and event.inaxes == self.ax:
            # 点をドラッグして移動
            self.x[self.selected_point] = event.xdata
            self.y[self.selected_point] = event.ydata
            self.plot_data()
        elif self.pan_active and event.inaxes == self.ax:
            # 画面移動（パン）機能
            dx = self.pan_start[0] - event.xdata
            dy = self.pan_start[1] - event.ydata

            # 現在の軸範囲を取得して、ドラッグ量に基づいて新しい範囲を設定
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()

            self.ax.set_xlim(xlim[0] + dx, xlim[1] + dx)
            self.ax.set_ylim(ylim[0] + dy, ylim[1] + dy)

            self.canvas.draw()  # 描画を更新

    def on_scroll(self, event):
        # スクロールイベントでズームイン/ズームアウト
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        # ズームの中心はマウスの位置
        xdata, ydata = event.xdata, event.ydata

        if event.button == 'up':  # ズームイン
            scale_factor = self.zoom_scale
        elif event.button == 'down':  # ズームアウト
            scale_factor = 1 / self.zoom_scale
        else:
            return

        # 新しい範囲を計算
        new_xlim = [(x - xdata) * scale_factor + xdata for x in xlim]
        new_ylim = [(y - ydata) * scale_factor + ydata for y in ylim]

        # 新しい範囲をセット
        self.ax.set_xlim(new_xlim)
        self.ax.set_ylim(new_ylim)

        # 再描画
        self.canvas.draw()

    def find_nearest_point(self, x, y):
        distances = np.hypot(np.array(self.x) - x, np.array(self.y) - y)
        min_idx = np.argmin(distances)
        if distances[min_idx] < 1:
            return min_idx
        return None

    def find_nearest_line(self, x, y):
        for i in range(len(self.x) - 1):
            x0, y0 = self.x[i], self.y[i]
            x1, y1 = self.x[i + 1], self.y[i + 1]
            if self.is_near_line(x0, y0, x1, y1, x, y):
                new_x = (x0 + x1) / 2
                new_y = (y0 + y1) / 2
                self.x.insert(i + 1, new_x)
                self.y.insert(i + 1, new_y)
                self.labels.insert(i + 1, 0.0)  # 新しい点には空のラベルを付ける
                self.plot_data()
                return

    def is_near_line(self, x0, y0, x1, y1, x, y, tol=0.1):
        d = np.abs((y1 - y0) * x - (x1 - x0) * y + x1 * y0 - y1 * x0) / np.hypot(x1 - x0, y1 - y0)
        return d < tol
    

    def on_double_click(self, event):
        if event.dblclick and event.inaxes == self.ax:
            point_idx = self.find_nearest_point(event.xdata, event.ydata)
            if point_idx is not None:
                self.active_label = point_idx  # アクティブなラベルを設定
                self.edit_label(point_idx)     # ラベル編集
                self.plot_data()               # ラベルを表示



    def edit_label(self, point_idx):
        # ポイントのラベルをダブルクリックで編集
        new_label = tk.simpledialog.askstring("Edit Label", f"Edit label for point {point_idx}", initialvalue=self.labels[point_idx])
        if new_label is not None:
            self.labels[point_idx] = new_label
            self.plot_data()

if __name__ == "__main__":
    root = tk.Tk()
    plot_tool = PlotTool(root)
    root.mainloop()
