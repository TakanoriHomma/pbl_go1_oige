from pxr import Usd, UsdGeom

# USDファイルを読み込む
usd_file_path = "go1_camera.usd"
stage = Usd.Stage.Open(usd_file_path)

# ルートPrimを取得
root_prim = stage.GetPseudoRoot()

# すべてのPrimを格納するリスト
all_prims = []

# 再帰的にPrimを探索する関数
def find_prims(prim):
    for child_prim in prim.GetChildren():
        all_prims.append(child_prim)
        find_prims(child_prim)

# ルートPrimからPrimを探索
find_prims(root_prim)

# すべてのPrimの名前を表示
for prim in all_prims:
    print(prim.GetPath())
