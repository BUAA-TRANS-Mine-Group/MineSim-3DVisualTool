# 使用 VS Code 软件开发环境，全语言统一规范化

使用流程：

- 1）将全文件夹`.vscode/settings.json` 复制到 **VS Code 工程项目根目录** 下即可.
- 2）参考文档 `代码规范化(格式化)工具/.vscode/settings.json_VS Code 格式化方案的使用说明.md`

# 使用 VS Code 软件开发环境，github 的代码同步管理

- 1）将文件全部内容`demo.gitignore` 复制到 **github 项目根目录** 下 `.gitignore` 文件内即可.
- 2）根据具体需要，编写`.gitignore`文件内的忽略规则. `详细的 .gitignore 语法使用说明.md`
  注意：**VS Code 工程项目根目录** 和 **github 项目根目录** 不同，例如

'''
├── vscode_project # VS Code 工程项目根目录
│ ├── inputs
│ │ └── dapai_intersection_1_3_4
│ │ └── dapai_intersection_1_3_4.json
│ │ ├── 1.png
│ │ ├── 2.png
| |── outputs
│ │ ├── 0.png
│ │ ├── 0.png
| ├── MineSim-3DVisualTool-Dev # github 项目根目录
│ │ ├── devkit
│ │ | ├── main.cpp
│ │ | ├── plot_3D.cpp
│ │ ├── code_core
│ │ | ├── main2.cpp
│ │ | ├── plot_3D2.cpp

'''
