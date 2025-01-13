### 使用 Python 读取和解析 `.obj` 文件的详细信息

为了自动读取和解析 `.obj` 文件的详细信息（如模型的长、宽、高），您可以编写一个 Python 脚本来完成以下任务：

1. **读取 `.obj` 文件**：打开并读取 `.obj` 文件内容。
2. **解析顶点信息**：提取所有顶点（以 `v` 开头的行）的坐标。
3. **计算边界框**：根据顶点坐标计算模型在 X、Y 和 Z 方向上的最小值和最大值。
4. **计算模型尺寸**：根据边界框计算模型的长度、宽度和高度。
5. **输出结果**：将计算结果输出到控制台或保存到文件中。

以下是一个完整的 Python 实现示例：

#### 1. 脚本实现

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
脚本名称：parse_obj.py
功能描述：读取并解析 .obj 文件，提取模型的详细信息（顶点、面等），并计算模型的长、宽、高。
作者：ChatGPT
日期：2025-01-13
"""

import sys
import os

class BoundingBox:
    def __init__(self):
        self.min_x = self.min_y = self.min_z = float('inf')
        self.max_x = self.max_y = self.max_z = float('-inf')

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

    参数：
        file_path (str): .obj 文件的路径。

    返回：
        dict: 包含顶点列表和模型尺寸的信息。
    """
    if not os.path.isfile(file_path):
        print(f"错误：文件不存在 - {file_path}")
        sys.exit(1)

    bbox = BoundingBox()
    vertices = []
    faces = []

    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):
                    parts = line.split()
                    if len(parts) < 4:
                        continue  # 跳过不完整的顶点定义
                    try:
                        x, y, z = map(float, parts[1:4])
                        vertices.append((x, y, z))
                        bbox.update(x, y, z)
                    except ValueError:
                        continue  # 跳过无法解析的顶点
                elif line.startswith('f '):
                    parts = line.split()
                    face = []
                    for part in parts[1:]:
                        # 处理可能的格式 v, v//vn, v/vt/vn
                        vertex_index = part.split('/')[0]
                        try:
                            vi = int(vertex_index)
                            face.append(vi)
                        except ValueError:
                            continue
                    if len(face) >= 3:
                        faces.append(face)
    except Exception as e:
        print(f"错误：读取文件时出错 - {e}")
        sys.exit(1)

    length, width, height = bbox.get_dimensions()

    model_info = {
        'vertices_count': len(vertices),
        'faces_count': len(faces),
        'length': length,
        'width': width,
        'height': height,
        'bounding_box': {
            'min_x': bbox.min_x,
            'max_x': bbox.max_x,
            'min_y': bbox.min_y,
            'max_y': bbox.max_y,
            'min_z': bbox.min_z,
            'max_z': bbox.max_z
        }
    }

    return model_info

def main():
    if len(sys.argv) != 2:
        print("用法：python3 parse_obj.py <path_to_obj_file>")
        sys.exit(1)

    obj_file_path = sys.argv[1]
    model_info = parse_obj(obj_file_path)

    print("=== 模型信息 ===")
    print(f"顶点数量：{model_info['vertices_count']}")
    print(f"面数量：{model_info['faces_count']}")
    print(f"长度（X轴）：{model_info['length']:.3f} 米")
    print(f"宽度（Y轴）：{model_info['width']:.3f} 米")
    print(f"高度（Z轴）：{model_info['height']:.3f} 米")
    print("\n=== 边界框 ===")
    bbox = model_info['bounding_box']
    print(f"X轴范围：{bbox['min_x']} 至 {bbox['max_x']}")
    print(f"Y轴范围：{bbox['min_y']} 至 {bbox['max_y']}")
    print(f"Z轴范围：{bbox['min_z']} 至 {bbox['max_z']}")

if __name__ == "__main__":
    main()
```

#### 2. 使用说明

1. **保存脚本**：将上述代码保存为 `parse_obj.py`。

2. **赋予执行权限**（可选）：

   ```bash
   chmod +x parse_obj.py
   ```

3. **运行脚本**：

   ```bash
   python3 parse_obj.py <path_to_obj_file>
   ```

   例如：

   ```bash
   python3 parse_obj.py /path/to/Mining_Truck_v1.obj
   ```

#### 3. 示例输出

运行脚本后，您将看到类似如下的输出：

```
=== 模型信息 ===
顶点数量：1500
面数量：3000
长度（X轴）：13.400 米
宽度（Y轴）：6.700 米
高度（Z轴）：6.900 米

=== 边界框 ===
X轴范围：-6.7 至 6.7
Y轴范围：-3.35 至 3.35
Z轴范围：0.0 至 6.9
```

#### 4. 代码详解

- **BoundingBox 类**：

  用于跟踪模型在 X、Y 和 Z 方向上的最小值和最大值，从而计算边界框。

- **parse_obj 函数**：

  负责解析 `.obj` 文件，提取顶点和面信息，并计算模型的边界框和尺寸。

- **main 函数**：

  处理命令行参数，调用解析函数，并输出结果。

#### 5. 进一步扩展

如果您需要提取更多详细信息（如法线、纹理坐标等），可以在 `parse_obj` 函数中添加相应的解析逻辑。例如，解析以 `vn` 开头的法线信息，或以 `vt` 开头的纹理坐标。

#### 6. 注意事项

1. **单位一致性**：

   确保 `.obj` 文件中的单位与您的应用程序（如 ROS 节点）中使用的单位一致。通常，模型的单位为米。

2. **模型原点**：

   了解模型的原点位置非常重要。如果模型的原点不在底部中心，可能需要在 RViz 中调整模型的位置，以确保正确显示。

3. **复杂模型**：

   对于非常复杂的模型，解析速度可能较慢。可以考虑使用更高效的解析库，如 [`pywavefront`](https://github.com/greenmoss/PyWavefront)。

#### 7. 使用 `pywavefront` 库（可选）

如果您希望使用现有的库来解析 `.obj` 文件，可以使用 `pywavefront`。以下是一个使用 `pywavefront` 的示例：

1. **安装 `pywavefront`**：

   ```bash
   pip3 install pywavefront
   ```

2. **脚本示例**：

   ```python
   #!/usr/bin/env python3
   # -*- coding: utf-8 -*-

   """
   使用 pywavefront 解析 .obj 文件并计算模型尺寸
   """

   import sys
   import os
   import pywavefront

   def parse_obj_with_pywavefront(file_path):
       if not os.path.isfile(file_path):
           print(f"错误：文件不存在 - {file_path}")
           sys.exit(1)

       scene = pywavefront.Wavefront(file_path, collect_faces=True)

       min_x = min_y = min_z = float('inf')
       max_x = max_y = max_z = float('-inf')

       for name, mesh in scene.meshes.items():
           for vertex in mesh.vertices:
               x, y, z = vertex
               if x < min_x:
                   min_x = x
               if x > max_x:
                   max_x = x
               if y < min_y:
                   min_y = y
               if y > max_y:
                   max_y = y
               if z < min_z:
                   min_z = z
               if z > max_z:
                   max_z = z

       length = max_x - min_x
       width = max_y - min_y
       height = max_z - min_z

       return {
           'length': length,
           'width': width,
           'height': height,
           'bounding_box': {
               'min_x': min_x,
               'max_x': max_x,
               'min_y': min_y,
               'max_y': max_y,
               'min_z': min_z,
               'max_z': max_z
           },
           'vertices_count': len(scene.vertices),
           'faces_count': len(scene.mesh_list[0].faces) if scene.mesh_list else 0
       }

   def main():
       if len(sys.argv) != 2:
           print("用法：python3 parse_obj_with_pywavefront.py <path_to_obj_file>")
           sys.exit(1)

       obj_file_path = sys.argv[1]
       model_info = parse_obj_with_pywavefront(obj_file_path)

       print("=== 模型信息 ===")
       print(f"顶点数量：{model_info['vertices_count']}")
       print(f"面数量：{model_info['faces_count']}")
       print(f"长度（X轴）：{model_info['length']:.3f} 米")
       print(f"宽度（Y轴）：{model_info['width']:.3f} 米")
       print(f"高度（Z轴）：{model_info['height']:.3f} 米")
       print("\n=== 边界框 ===")
       bbox = model_info['bounding_box']
       print(f"X轴范围：{bbox['min_x']} 至 {bbox['max_x']}")
       print(f"Y轴范围：{bbox['min_y']} 至 {bbox['max_y']}")
       print(f"Z轴范围：{bbox['min_z']} 至 {bbox['max_z']}")

   if __name__ == "__main__":
       main()
   ```

3. **运行脚本**：

   ```bash
   python3 parse_obj_with_pywavefront.py /path/to/Mining_Truck_v1.obj
   ```

#### 8. 总结

通过上述 Python 脚本，您可以自动读取和解析 `.obj` 模型文件，提取顶点信息，并计算模型的长度、宽度和高度。这将有助于您在 ROS 节点中动态调整车辆模型在 RViz 中的显示尺寸，提高代码的自动化和灵活性。

如果您在使用过程中遇到任何问题或需要进一步的帮助，请随时提问！
