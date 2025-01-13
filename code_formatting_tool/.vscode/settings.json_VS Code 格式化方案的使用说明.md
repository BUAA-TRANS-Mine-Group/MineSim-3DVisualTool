# VS Code 格式化方案的使用说明

以下是对该 **VS Code 格式化方案**的详细使用说明和指导，可帮助你或你的团队了解如何正确安装、配置及使用。请务必在使用前查看并确认插件安装情况，保证各语言环境正常工作。

---

# 一、配置文件说明

这份 **`settings.json`** 配置示例中包含了以下关键点：

1. **不在保存时自动格式化（全局）**

   - 配置项：`"editor.formatOnSave": false`
   - 作用：当你保存文件时，VS Code **不会**自动触发格式化操作。
   - 场景：在多人协作或对自动格式化比较敏感的场合，避免因保存频繁而带来的代码变动。
   - 如需单个语言自动格式化，可以在对应的 `[language]` 设置块中添加或修改 `"editor.formatOnSave": true`。

2. **全局缩进大小 4 个空格**

   - 配置项：`"editor.tabSize": 4`
   - 作用：所有没有特定语言覆盖设置的地方，使用 4 个空格做缩进。
   - 场景：一般项目或团队对缩进大小都会有一致规定，常见是 2 或 4，这里统一为 4。

3. **Python 使用 Black 格式化**

   - 配置项：`"python.formatting.provider": "black"`，以及可能需要的 `black-formatter.args`。
   - 作用：通过 Black 保持 Python 代码的一致风格，包括指定行宽等参数。
   - 行宽：`--line-length 150`，适合横屏幕更宽的场景，减少换行次数。
   - 注意：
     - 需要在**VScode 软件中安装**专门的 **black-formatter** 插件。
     - 如果已在项目的 `pyproject.toml` 中进行配置，那么这里的命令行参数可省略。

4. **C/C++ 使用 Clang-Format**

   - 主要配置项：
     ```json
     "C_Cpp.clang_format_fallbackStyle": "{ BasedOnStyle: LLVM, IndentWidth: 4, ColumnLimit: 150 }"
     "C_Cpp.clang_format_style": "{ BasedOnStyle: LLVM, IndentWidth: 4, ColumnLimit: 150 }"
     ```
   - 作用：
     - **`.clang-format` 文件优先**：如果项目根目录存在 `.clang-format` 文件，则以该文件中的规则为准。
     - **`fallbackStyle`**：在没有 `.clang-format` 文件时，使用此作为后备配置。
   - 行宽：`ColumnLimit: 150`，与 Python Black 设置保持一致。
   - 注意：
     - 多人协作最好将 `.clang-format` 文件**提交到版本库**，以统一团队 C/C++ 代码风格。

5. **其它语言使用 Prettier**
   - 配置项：对常见前端语言 **JS、TS、HTML、CSS、JSON** 做了：
     ```jsonc
     "[javascript]": {
       "editor.defaultFormatter": "esbenp.prettier-vscode"
     },
     ...
     "[json]": {
       "editor.defaultFormatter": "esbenp.prettier-vscode"
     }
     ```
   - 作用：保证 JavaScript、TypeScript、HTML、CSS、JSON 等广泛语言使用 **Prettier** 统一处理。
   - 注意：
     - 需要安装 **Prettier** 插件（即 `esbenp.prettier-vscode`）。
     - 如果项目里包含 `.prettierrc` 配置文件，Prettier 会优先读取其中规则，否则使用其默认规则。

---

# 二、如何安装并启用这些工具

1. **安装 Python Black** (二选一,推荐安装 VS Code 插件)

   - 在你的 Python 环境（虚拟环境或全局）中安装 Black：
     ```bash
     pip install black
     ```
   - VS Code 插件市场中搜索并安装 **black-formatter** 插件。

2. **安装 Clang-Format** (二选一,推荐安装 VS Code 的**ms-vscode.cpptools** 插件)

   - 通过系统包管理器或从官网安装：
     - Windows：`choco install llvm` 或从 [LLVM 官网](https://releases.llvm.org/)下载安装包
     - macOS：`brew install clang-format`
     - Linux：在大多数发行版中，可以通过包管理器安装，如 `apt-get install clang-format`。
   - 如果你使用 **ms-vscode.cpptools** 插件，会内置对 Clang-Format 的调用支持。ms-vscode.cpptools 插件是 Microsoft 官方为 VS Code 提供的 C/C++ 扩展，常常在插件市场中标识为：C/C++ (by Microsoft)

3. **安装 Prettier**

   - 在 VS Code 插件市场中搜索 **Prettier - Code formatter (作者: esbenp)** 并安装；
   - 如果项目需要额外的 Prettier 插件配置（如对 Markdown、YAML 等格式化），可以在项目根目录下添加 `.prettierrc`、`prettier.config.js` 等自定义规则文件。

4. **验证安装及使用**
   - 打开 VS Code → 查看 **输出 (Output)** 面板 / 终端，有无报错。
   - 如有报错，检查是否已安装对应工具，以及路径配置是否正确。

---

# 三、如何手动触发格式化

> 因为在这个方案中，我们**关闭**了全局 **formatOnSave**。如果你希望在某些文件或语言手动进行格式化，通常有以下几种方式：

1. **快捷键** (推荐统一使用 `ctrl+shift+i` )

   - VS Code 默认：
     - **Windows/Linux**：`Shift + Alt + F`
     - **macOS**：`Shift + Option + F`
   - 如需改为你喜欢的 `Ctrl + Shift + I`，可以在 **Keyboard Shortcuts** 面板或 `keybindings.json` 中自行更改：
     ```jsonc
     [
       {
         "key": "ctrl+shift+i",
         "command": "editor.action.formatDocument",
         "when": "editorHasDocumentFormattingProvider && editorTextFocus && !editorReadonly"
       }
     ]
     ```

2. **命令面板**

   - 打开命令面板：
     - **Windows/Linux**：`Ctrl + Shift + P`
     - **macOS**：`Cmd + Shift + P`
   - 输入 “**Format Document**” 并回车。

3. **右键菜单**
   - 在编辑区右键 → 选择 “**Format Document**”。

---

# 四、常见问题 (FAQ)

1. **为什么保存时代码没有被自动格式化？**

   - 因为 `"editor.formatOnSave": false`。若你想针对某些语言在保存时格式化，可在对应的语言配置段 `[language]` 内加上 `"editor.formatOnSave": true`。
   - 或者暂时把全局设置改回：
     ```json
     "editor.formatOnSave": true
     ```

2. **Python 行宽或 C/C++ 行宽想要修改怎么办？**

   - Python：在你的 `pyproject.toml` 或 `settings.json` 里修改：
     ```json
     "black-formatter.args": ["--line-length", "100"]
     ```
   - C/C++：在 `.clang-format` 或 VS Code 设置中修改 `ColumnLimit: 100` 即可。

3. **如何与团队成员共享这套格式配置？** (推荐在 vscode 项目根目录使用 `.vscode/settings.json` )
   - **推荐**在项目仓库根目录建立 **`.clang-format`** 与 **`.prettierrc`** / `prettier.config.js` 等配置文件，让大家在同一个规则下工作。
   - 在 Python 项目中，也可将 `pyproject.toml` 提交到版本库，包含 Black 设置。
   - 对于 VS Code 的 `settings.json`，可使用 **工作区** (Workspace) 层级的 `.vscode/settings.json` 并提交到仓库，让团队共享。

---

# 五、总结

- 这套方案对于“**希望手动控制何时格式化**、并且在不同语言分别使用合适工具\*\*”的场景非常实用。
- **优点**：
  1. 避免保存时自动格式化引起的意外更改。
  2. 统一的 4 空格缩进和 150 行宽，改善横屏幕下的阅读体验。
  3. 通过不同插件（Black、Clang-Format、Prettier），在各自领域获得最优的格式化效果。
- **缺点**：
  1. 如果需要频繁格式化、希望一键保存即完成，就需要额外开启或修改 `"editor.formatOnSave": true`。
  2. 多语言多规则，会在各项目中需要更多协作和约束。

若你的团队中有明确的格式要求，也可依此基础进行微调。搭配 `.clang-format`、`.prettierrc`、`pyproject.toml` 等项目文件，能极大地减少不同开发者间的格式差异。祝你编码愉快，团队协作高效！
