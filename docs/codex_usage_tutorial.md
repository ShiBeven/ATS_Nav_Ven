# Codex 使用规范与教程

这份文档是给你自己和未来协作者用的操作手册。目标很明确：

- 提高 Codex 在这个仓库里的产出稳定性
- 减少无效 token 消耗
- 避免一个长对话里不断发生上下文冲突
- 让每一轮任务都能留下清晰 handoff

如果你只想看最短流程，先看第 2 章和第 4 章。如果你想建立稳定习惯，完整读一遍。

## 1. 为什么你会觉得 Codex “工程管理太糟糕”

大多数不是模型不行，而是输入协议失控。典型表现：

- 一个线程里连续做多个目标
- 没有显式告诉 Codex 允许改哪里
- 没有定义验证方式，导致它自己猜
- 把全仓背景、长日志、历史讨论一次性塞进去
- 任务切换后继续沿用旧线程

这样会直接带来四类问题：

- token 消耗快，因为每轮都在重复读旧上下文
- 搜索范围失控，因为没有包级边界
- 修改范围失控，因为没有 stop condition
- 推理偏移，因为旧线程里的假设会污染新任务

所以优化重点不是“写更花哨的 prompt”，而是建立固定协议。

## 2. 一套最小可执行流程

每轮任务固定按下面走：

1. 先判定这轮任务是不是只有一个目标
2. 确定只涉及哪个包或哪些文件
3. 运行 `./scripts/codex_brief.sh <package>`
4. 套用 `patterns/` 里的对应模板发起任务
5. 要求 Codex 只在约束范围内分析、修改、验证
6. 结束时要求输出 5 行 handoff
7. 下一轮如果目标变化，开新线程，只贴 handoff

这七步的意义分别是：

- 第 1 步防止线程目标混杂
- 第 2 步防止搜索范围爆炸
- 第 3 步防止你自己重复描述工程背景
- 第 4 步防止你每次临时组织 prompt
- 第 5 步防止 Codex 自行扩大任务
- 第 6 步防止结果无法继续接力
- 第 7 步防止旧上下文污染新任务

## 3. 你必须建立的五个规则

### 3.1 一个对话只做一个目标

一个目标的合理定义：

- 修一个 bug
- 实现一个 feature
- review 一组改动
- 重构一个明确切片
- 回答一个技术问题

不合理的定义：

- “顺手再看看另一个问题”
- “把视觉和导航一起整理一下”
- “先修 bug，再顺便优化架构”

你一旦说“顺便”，上下文就开始失真。

### 3.2 永远先说边界，再说细节

正确顺序：

1. 目标
2. 范围
3. 约束
4. 验证
5. 现象
6. 补充材料

错误顺序：

1. 先贴 200 行日志
2. 再贴 100 行代码
3. 最后说“帮我看下”

正确顺序会让 Codex 先建立解题边界。错误顺序会让它先被噪音淹没。

### 3.3 范围必须写成路径或包名

不要说：

- “视觉这块”
- “导航那边”
- “帮我看看行为树”

要说：

- `src/sp_vision25`
- `src/pb2025_sentry_nav`
- `src/pb2025_sentry_behavior`

如果更窄，直接写到文件层：

- `src/sp_vision25/src/auto_aim_debug_mpc.cpp`
- `src/pb2025_sentry_behavior/plugins/condition`

路径是最便宜、最稳定的上下文控制方式。

### 3.4 验证必须前置声明

如果你不写验证，Codex 会自己猜。它可能：

- 跑过大的验证
- 跑错验证
- 不敢验证
- 在无法验证时浪费很多 token 解释限制

所以要明确写：

- `colcon build --packages-select sp_vision25`
- `colcon build --packages-select pb2025_sentry_behavior`
- “先静态检查，不跑硬件”
- “只做 review，不改代码”

### 3.5 线程一旦变题，就开新对话

不要在旧线程里说：

- “算了我们改成做另一个”
- “先别修这个了，帮我看看配置”
- “刚才那个先放下，换导航”

这种做法对人看起来自然，对模型是高冲突输入。正确方式是：

- 要一个 handoff
- 开新线程
- 贴 handoff
- 明确新目标

## 4. 标准开场模板

### 4.1 通用任务模板

文件： [patterns/codex_task.md](/home/ats/ats_sentry_ws/patterns/codex_task.md)

推荐写法：

```md
按仓库里的 `AGENTS.md` 执行。

目标: 为 `sp_vision25` 修复串口录制逻辑
范围: 只允许改 `src/sp_vision25`
约束: 不改消息定义，不引入新依赖，不做无关重构
验证: `colcon build --packages-select sp_vision25`

现状:
录制文件为空，怀疑写入条件有问题。

补充:
- 先分析相关代码，再实施
- 做完后给我一个 5 行以内 handoff
```

### 4.2 Debug 模板

文件： [patterns/codex_debug.md](/home/ats/ats_sentry_ws/patterns/codex_debug.md)

推荐写法：

```md
按仓库里的 `AGENTS.md` 执行。

目标: 修复 `auto_aim_debug_mpc` 启动崩溃
范围: 只允许改 `src/sp_vision25`
约束: 不做超出 bug 修复范围的重构
复现: `ros2 run sp_vision25 auto_aim_debug_mpc`
期望: 启动不再崩溃
验证: `colcon build --packages-select sp_vision25`

错误签名:
`std::out_of_range` 出现在启动阶段

已知怀疑点:
配置读取逻辑
```

### 4.3 Review 模板

文件： [patterns/codex_review.md](/home/ats/ats_sentry_ws/patterns/codex_review.md)

推荐写法：

```md
请按 review 模式工作，不要直接改代码。

范围: `src/sp_vision25`
关注点: bug、行为回归、未覆盖边界条件
验证: 不需要改代码，先给 findings
```

### 4.4 Handoff 模板

文件： [patterns/codex_handoff.md](/home/ats/ats_sentry_ws/patterns/codex_handoff.md)

你在一轮任务结束时直接发：

```md
请基于当前结果，输出一个 handoff，限制在 5 行以内：

1. 已完成
2. 改动范围
3. 已验证
4. 未验证/风险
5. 下一轮从哪里继续
```

## 5. 任务类型对应教程

### 5.1 修 bug

#### 正确流程

1. 限定包范围
2. 给复现命令
3. 给错误签名
4. 指定验证命令
5. 等修复完成后要 handoff

#### 正确示例

```md
按仓库里的 `AGENTS.md` 执行。

目标: 修复 `sp_vision25` 中 recorder 写空文件的问题
范围: 只允许改 `src/sp_vision25`
约束: 不改外部接口，不做无关重构
复现: 运行测试后输出文件存在但内容为空
期望: 录制文件有实际数据
验证: `colcon build --packages-select sp_vision25`

错误签名:
无崩溃，但输出为空

已知怀疑点:
`tools/recorder.cpp`
```

#### 错误示例

```md
视觉最近很怪，你帮我全面看看，顺便优化一下 recorder，还有如果有必要把配置也一起清理。
```

这个错误示例有三个问题：

- 没有目标边界
- 没有路径范围
- 把 debug、优化、重构混成一件事

### 5.2 做 feature

#### 正确流程

1. 说清新增能力是什么
2. 说清兼容性约束
3. 说清入口在哪里
4. 说清验收标准

#### 正确示例

```md
按仓库里的 `AGENTS.md` 执行。

目标: 给 `pb2025_sentry_behavior` 增加一个新条件节点，用于判断视觉目标是否长时间无效
范围: 只允许改 `src/pb2025_sentry_behavior`
约束: 保持现有 BT XML 兼容，不改已有节点语义
验证: `colcon build --packages-select pb2025_sentry_behavior`

现状:
现有条件只能判断当前帧有效性，缺少持续失效判断
```

### 5.3 做重构

重构是最容易炸 context 的任务，因为它天然边界模糊。你必须显式限制切片。

#### 推荐写法

```md
按仓库里的 `AGENTS.md` 执行。

目标: 只重构 `recorder` 的内部实现，减少重复逻辑
范围: 只允许改 `src/sp_vision25/tools/recorder.*`
约束: 不改对外接口，不改调用方，不改变运行行为
验证: `colcon build --packages-select sp_vision25`
```

关键点是“切片”。不要说“重构 recorder 模块”，要说“只重构哪几个文件，且不改外部行为”。

### 5.4 做代码 review

review 时最常见错误是让 Codex去“review 并顺便修”。这样它会同时扮演审查者和作者，结论不稳定。

正确做法分两轮：

1. 第一轮只 review，要求 findings
2. 第二轮针对 findings 再要求修

#### review 开场

```md
请按 review 模式工作，不要直接改代码。

范围: `src/sp_vision25`
关注点: bug、行为回归、资源泄漏、线程安全
验证: 不需要改代码，先给 findings
```

#### 修 findings 开场

```md
按仓库里的 `AGENTS.md` 执行。

目标: 修复上一轮 review 提到的前两个高优先级问题
范围: 只允许改 `src/sp_vision25`
约束: 只修这两个问题，不做额外重构
验证: `colcon build --packages-select sp_vision25`
```

## 6. 如何节省 token

### 6.1 不要重复讲工作区背景

你已经有：

- [AGENTS.md](/home/ats/ats_sentry_ws/AGENTS.md)
- [docs/codex_workflow.md](/home/ats/ats_sentry_ws/docs/codex_workflow.md)
- [docs/codex_usage_tutorial.md](/home/ats/ats_sentry_ws/docs/codex_usage_tutorial.md)

以后直接说：

```md
按仓库里的 `AGENTS.md` 执行。
```

这句话比你自己重复解释工程结构更省。

### 6.2 不要贴整段日志

正确做法是贴：

- 错误签名
- 最关键的 10 到 30 行
- 复现命令

错误做法是：

- 整个终端输出
- 整个构建日志
- 整个 launch 日志

### 6.3 不要贴整文件源码

优先给：

- 文件路径
- 相关函数名
- 可疑代码片段

除非模型明确要求，否则不要先贴 300 行源码。

### 6.4 不要在一个线程里追多个方向

下面这种追问方式最浪费 token：

- “再看看这个”
- “顺便这个也解释下”
- “你觉得要不要顺手优化一下”

因为每一轮都要重新回忆旧线程状态，还要兼容新目标。

### 6.5 用脚本先生成摘要

你可以直接运行：

```bash
./scripts/codex_brief.sh
./scripts/codex_brief.sh sp_vision25
./scripts/codex_brief.sh pb2025_sentry_behavior
```

这个脚本会告诉你：

- 当前工作区分支
- 当前脏文件
- 包级构建命令
- 相关搜索根目录
- 关键文件列表

这样你开场时不需要手写背景。

## 7. 如何避免 context 冲突

### 7.1 最常见的冲突来源

- 同一线程里前后目标变化
- 旧线程里已经形成错误假设
- 你中途放宽或改变约束
- 你在后半段突然引入新日志或新模块

### 7.2 冲突出现时的标准处理

一旦出现下面迹象，直接切线程：

- 你发现自己在反复纠正 Codex 的任务理解
- 你要多次说“不是这个意思”
- 任务已经从修改代码变成讨论方案
- 包范围已经从单包变成跨包

正确操作：

1. 让 Codex 输出 handoff
2. 开新线程
3. 粘 handoff
4. 明确新目标和新边界

### 7.3 错误做法

```md
忽略前面所有内容，我们现在开始做导航。
```

这句话对人有效，对模型并不可靠，因为前面上下文依然存在。

### 7.4 正确做法

```md
按仓库里的 `AGENTS.md` 执行。

这是上一轮 handoff:
1. ...
2. ...
3. ...

新目标: 修复 `pb2025_sentry_nav` 中 xxx
范围: 只允许改 `src/pb2025_sentry_nav`
约束: 不改接口
验证: `colcon build --packages-select pb2025_sentry_nav`
```

## 8. 结合你这个仓库的专项规范

### 8.1 视觉任务

默认边界：

- `src/sp_vision25`

推荐验证：

- `colcon build --packages-select sp_vision25`

只有下面情况才允许跨包：

- topic / msg 变更
- bringup 接线变更
- 与串口或接口层联动变更

### 8.2 行为树任务

默认边界：

- `src/pb2025_sentry_behavior`

推荐验证：

- `colcon build --packages-select pb2025_sentry_behavior`

如果涉及 XML 和插件联动，仍然尽量限制在本包。

### 8.3 导航任务

默认边界：

- `src/pb2025_sentry_nav`

推荐验证：

- `colcon build --packages-select pb2025_sentry_nav`

除非明确涉及 bringup 或 interface，不要扩大到全仓。

### 8.4 接口变更任务

只有在下面情况才值得做跨包会话：

- 修改 `msg/srv/action`
- launch 启动链跨包重接线
- 一次改动明确影响多个包的构建和运行

这种任务一定要在开头明确写：

- 允许跨包
- 涉及哪些包
- 最终要跑什么验证

## 9. 你的每日标准操作

### 9.1 开工前

1. 先确认今天这轮要解决什么单一目标
2. 看看脏文件和分支状态
3. 跑 `./scripts/codex_brief.sh <package>`
4. 选一个模板发给 Codex

### 9.2 工作中

1. 只围绕当前目标追问
2. 不临时切模块
3. 不把大段无关日志塞进去
4. 如果边界变了，立即切线程

### 9.3 收工前

1. 要一个 5 行 handoff
2. 确认已验证和未验证项
3. 如果下一轮目标不同，不延续旧线程

## 10. 三种推荐强度

### 10.1 极简模式

适合小任务：

```md
按仓库里的 `AGENTS.md` 执行。
目标: ...
范围: ...
约束: ...
验证: ...
现象: ...
```

### 10.2 标准模式

适合绝大多数开发任务：

```md
按仓库里的 `AGENTS.md` 执行。

目标: ...
范围: ...
约束: ...
复现: ...
期望: ...
验证: ...

错误签名:

已知怀疑点:

补充:
- 先分析再改
- 只改必要文件
- 做完给 handoff
```

### 10.3 强约束模式

适合复杂任务、容易炸 context 的任务：

```md
按仓库里的 `AGENTS.md` 和 `docs/codex_usage_tutorial.md` 执行。

目标: ...
范围: 只允许改以下路径: ...
约束:
- 不允许跨包
- 不允许引入新依赖
- 不允许做无关重构
- 如果发现跨包影响，先停下说明
验证:
- 先静态分析
- 再执行: ...

现状:

补充:
- 先给 5 句内分析，再实施
- 完成后输出 5 行 handoff
```

## 11. 反模式清单

下面这些说法会显著降低稳定性：

- “帮我全面看看”
- “顺便优化一下”
- “你觉得哪里不顺眼就一起改”
- “忽略前面，我们换个任务”
- “这个模块和那个模块一起处理”
- “日志很多你自己看”

这些说法的问题不是不礼貌，而是任务边界失控。

## 12. 你现在就可以怎么用

### 12.1 修视觉问题

```bash
./scripts/codex_brief.sh sp_vision25
```

然后发：

```md
按仓库里的 `AGENTS.md` 和 `docs/codex_usage_tutorial.md` 执行。

目标: 修复 `sp_vision25` 中 xxx
范围: 只允许改 `src/sp_vision25`
约束: 不改接口，不做无关重构
验证: `colcon build --packages-select sp_vision25`
现象: ...
```

### 12.2 Review 当前视觉改动

```md
请按 review 模式工作，不要直接改代码。

范围: `src/sp_vision25`
关注点: bug、线程安全、资源释放、配置兼容性
验证: 不需要改代码，先给 findings
```

### 12.3 从视觉切到导航

先要 handoff，再开新线程发：

```md
按仓库里的 `AGENTS.md` 和 `docs/codex_usage_tutorial.md` 执行。

这是上一轮 handoff:
1. ...
2. ...
3. ...

新目标: 修复 `pb2025_sentry_nav` 中 xxx
范围: 只允许改 `src/pb2025_sentry_nav`
约束: 不改接口
验证: `colcon build --packages-select pb2025_sentry_nav`
```

## 13. 最后的执行纪律

你真正要坚持的不是“写更长的 prompt”，而是下面四条纪律：

1. 一个线程只做一个目标
2. 每个目标先写路径边界
3. 每个目标先写验证方式
4. 每轮结束都要 handoff

只要这四条坚持住，token、上下文冲突、工作流失控这三个问题会同时明显下降。
