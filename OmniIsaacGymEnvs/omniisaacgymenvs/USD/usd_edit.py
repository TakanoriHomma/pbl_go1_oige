from pxr import Usd, UsdGeom

# USDファイルを読み込む
usd_file_path = "go1_camera.usd"
stage = Usd.Stage.Open(usd_file_path)

# new_prim = stage.DefinePrim("/go1_description/camera_rearDown/camera")

camera_path = "/go1_description/camera_rearDown/camera"
camera = UsdGeom.Camera.Define(stage, camera_path)
# 視野を設定（50mmレンズ）
camera.GetFocalLengthAttr().Set(50.0)
# クリップ範囲を設定
# camera.GetClippingRangeAttr().Set(valueMin=1.0, valueMax=1000.0)
# camera.GetClippingRangeAttr().Set(1.0, 1000.0)

stage.GetRootLayer().Save()