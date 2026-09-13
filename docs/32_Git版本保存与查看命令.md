# 32 Git版本保存与查看命令

在开发电脑的工程根目录执行。以下路径为本机Ubuntu路径；Windows使用其实际工程路径。

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
```

## 1. 保存当前版本（中文提交说明）

先看本次改动，再暂存、检查并提交：

```bash
git status --short
git diff --stat
git add -A
git diff --cached --stat
git diff --cached --check
git commit -m "完善联合录包离线分析说明并统一文档编号"
git status --short
git log -1 --oneline
```

`git add -A`包含当前工程全部新增、修改和删除；检查暂存清单符合本次意图后再commit。只想保存部分文件时，改用 `git add 文件路径`。如果没有修改，Git会提示nothing to commit，无需重复提交。

`.gitignore`当前排除了 `data/`、构建、安装、日志等目录，因此原始录包、data内分析报告不会随代码保存。它们需单独备份。工具根目录的离线分析tar.gz已纳入版本管理。

## 2. 查看历史、版本差异

```bash
git log --oneline -10
git show --stat ea5ceb2
git diff ea5ceb2 HEAD -- docs tools
```

`ea5ceb2`是已保存的“保存三盒联合测试与离线分析工具版本”。命令中的版本号可替换成 `git log` 看到的其他提交。

查看旧版本某个文件，不改当前工作区：

```bash
git show ea5ceb2:tools/record_joint_time.py
```

文档改名后查看旧版本要使用当时的旧文件名。

## 3. 给已有提交加中文标签（可选）

提交说明是版本描述；tag用于给一个版本起固定名称。不需要每次提交都打标签。

```bash
git tag -a "三盒联合测试离线分析版_20260908" ea5ceb2 -m "保存三盒联合测试与离线分析工具版本"
git tag -n
```

同名标签已存在时不要重复创建或强制覆盖。若想标记最新提交，将 `ea5ceb2` 改成 `HEAD`。

## 4. 推送远端（有远端且需要共享时）

先查看远端和当前分支：

```bash
git remote -v
git branch --show-current
```

确认目标正确后执行；以下假定远端名为origin、分支为main：

```bash
git push -u origin main
```

如果创建了上述标签并希望共享，再执行：

```bash
git push origin "三盒联合测试离线分析版_20260908"
```

没有远端时，本地commit已经完成版本保存，无需push。不要使用force推送来处理普通推送失败。

## 5. 临时查看旧版工程，保留当前工作区

在另一个新目录检出历史版本：

```bash
git worktree add --detach ../协同定位_历史版_ea5ceb2 ea5ceb2
```

旧版目录不含被忽略的录包和构建产物，需要时自行提供数据并重新构建。避免直接执行 `git reset --hard` 来“查看旧版”。

## 6. 文档编号约定

工程说明文档使用编号前缀。29为录包说明，30为离线分析，31为本Git说明，32为根目录临时UDP协议。根目录README.md保留标准项目入口名；历史计划已有日期前缀；data中的自动生成报告保留分析工具约定名称。
