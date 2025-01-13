#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
脚本名称: parse_obj.py
功能描述: 读取并解析 .obj 文件，提取模型的详细信息 (顶点、面等) ，并计算模型的长、宽、高。
作者: ChatGPT
日期: 2025-01-13
"""

import sys
import os


class BoundingBox:
    def __init__(self):
        self.min_x = self.min_y = self.min_z = float("inf")
        self.max_x = self.max_y = self.max_z = float("-inf")

    def update(self, x, y, z):
        if x < self.min_x:
            self.min_x = x
        if x > self.max_x:
            self.max_x = x
        if y < self.min_y:
            self.min_y = y
        if y > self.max_y:
            self.max_y = y
        if z < self.min_z:
            self.min_z = z
        if z > self.max_z:
            self.max_z = z

    def get_dimensions(self):
        length = self.max_x - self.min_x
        width = self.max_y - self.min_y
        height = self.max_z - self.min_z
        return length, width, height


def parse_obj(file_path):
    """
    解析 .obj 文件，提取顶点信息并计算边界框。

    参数: 
        file_path (str): .obj 文件的路径。

    返回: 
        dict: 包含顶点列表和模型尺寸的信息。
    """
    if not os.path.isfile(file_path):
        print(f"错误: 文件不存在 - {file_path}")
        sys.exit(1)

    bbox = BoundingBox()
    vertices = []
    faces = []

    try:
        with open(file_path, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("v "):
                    parts = line.split()
                    if len(parts) < 4:
                        continue  # 跳过不完整的顶点定义
                    try:
                        x, y, z = map(float, parts[1:4])
                        vertices.append((x, y, z))
                        bbox.update(x, y, z)
                    except ValueError:
                        continue  # 跳过无法解析的顶点
                elif line.startswith("f "):
                    parts = line.split()
                    face = []
                    for part in parts[1:]:
                        # 处理可能的格式 v, v//vn, v/vt/vn
                        vertex_index = part.split("/")[0]
                        try:
                            vi = int(vertex_index)
                            face.append(vi)
                        except ValueError:
                            continue
                    if len(face) >= 3:
                        faces.append(face)
    except Exception as e:
        print(f"错误: 读取文件时出错 - {e}")
        sys.exit(1)

    length, width, height = bbox.get_dimensions()

    model_info = {
        "vertices_count": len(vertices),
        "faces_count": len(faces),
        "length": length,
        "width": width,
        "height": height,
        "bounding_box": {
            "min_x": bbox.min_x,
            "max_x": bbox.max_x,
            "min_y": bbox.min_y,
            "max_y": bbox.max_y,
            "min_z": bbox.min_z,
            "max_z": bbox.max_z,
        },
    }

    return model_info


def main():
    # if len(sys.argv) != 2:
    #     print("用法: python3 parse_obj.py <path_to_obj_file>")
    #     sys.exit(1)

    # obj_file_path = sys.argv[1]
    obj_file_path = "/home/czf/project_czf/20240901-MineSim/MineSim-3DVisualTool-Dev/src/minesim_visualizor_3d/core/common/materials/vehicle_model/Mining_Truck_v1_L1.123c6431771f-f361-4a1f-8727-590a5c800ce0/16747_Mining_Truck_v1.obj"
    model_info = parse_obj(obj_file_path)

    print("=== 模型信息 ===")
    print(f"顶点数量: {model_info['vertices_count']}")
    print(f"面数量: {model_info['faces_count']}")
    print(f"长度 (X轴) : {model_info['length']:.3f} 米")
    print(f"宽度 (Y轴) : {model_info['width']:.3f} 米")
    print(f"高度 (Z轴) : {model_info['height']:.3f} 米")
    print("\n=== 边界框 ===")
    bbox = model_info["bounding_box"]
    print(f"X轴范围: {bbox['min_x']} 至 {bbox['max_x']}")
    print(f"Y轴范围: {bbox['min_y']} 至 {bbox['max_y']}")
    print(f"Z轴范围: {bbox['min_z']} 至 {bbox['max_z']}")


#

if __name__ == "__main__":
    main()

# === 模型信息 ===
# 顶点数量: 81166
# 面数量: 81008
# 长度 (X轴) : 1169.141 米
# 宽度 (Y轴) : 997.701 米
# 高度 (Z轴) : 740.219 米

# === 边界框 ===
# X轴范围: -561.131409 至 608.009521
# Y轴范围: -505.750092 至 491.950745
# Z轴范围: -0.470459 至 739.748413
