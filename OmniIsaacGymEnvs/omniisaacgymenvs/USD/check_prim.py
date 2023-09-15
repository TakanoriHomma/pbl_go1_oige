from pxr import Usd, UsdGeom


"""
/go1_description/camera_chin
/go1_description/camera_chin/visuals
/go1_description/camera_chin/collisions
/go1_description/camera_chin/camera_optical_joint_chin
/go1_description/camera_optical_chin

"""
# USDファイルを読み込む
usd_file_path = "go1_camera.usd"
stage = Usd.Stage.Open(usd_file_path)

# Primを取得
prim_path = "/go1_description/camera_rearDown/camera"  # Primのパスを指定
prim = stage.GetPrimAtPath(prim_path)

if prim:
    # Primの基本情報を表示
    print(f"Primの名前: {prim.GetName()}")
    print(f"Primの種類: {prim.GetTypeName()}")
    print(f"Primのパス: {prim.GetPath()}")

    # Primの変換行列を表示（UsdGeomXformの場合）
    if prim.IsA(UsdGeom.Xform):
        xform = UsdGeom.Xform(prim)
        transform_matrix = xform.GetLocalTransformation()
        print(f"変換行列: {transform_matrix}")

    # 他のプロパティを表示
    custom_property = prim.GetAttribute("custom_property_name")
    if custom_property:
        custom_property_value = custom_property.Get()
        print(f"カスタムプロパティの値: {custom_property_value}")
else:
    print(f"指定したパスのPrimは見つかりませんでした: {prim_path}")