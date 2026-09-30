# 33 Codex 与 ChatGPT 连接器操作步骤

这份文件用于本工程的 `Code` workspace。以后优先照着本文件操作，不需要每次重新询问。

## 1. 本工程固定信息

| 项目 | 内容 |
|---|---|
| 本地工作区 | `/home/pdq/Code` |
| 连接工具目录 | `/home/pdq/codex-with-chatgpt` |
| ChatGPT 项目 | [Code 项目](https://chatgpt.com/g/g-p-6a953b661c508191827f7ded82f369a9/project) |
| 连接器名称 | `Codex with ChatGPT · Code` |
| 连接器设置 | [ChatGPT 连接器设置](https://chatgpt.com/plugins?view=personal) |
| 新建连接器 | [新建连接器](https://chatgpt.com/plugins#settings/Connectors?create-connector=true&redirectAfter=%2Fplugins) |

本机使用的是 Cloudflare 临时连接地址。这个地址不是固定网址，连接工具重启、电脑重启、进程异常退出或临时隧道被回收后都可能改变；不要保存或反复使用昨天的 `trycloudflare.com` 地址。Cloudflare 也明确说明 Quick Tunnel 适合测试和开发，没有持续在线保证。需要长期稳定时，应改用固定域名的 named tunnel。

## 2. 平时怎么使用

### 2.1 只使用 VS Code 里的 Codex

只让 VS Code 里的 Codex 读代码、改代码、运行测试时，不需要打开 ChatGPT 网页，也不需要选择连接器。

### 2.2 让 ChatGPT 网页读取当前工程

1. 打开 VS Code，并打开本工程。
2. 打开终端，复制执行下面一条命令：

   ```bash
   cd /home/pdq/codex-with-chatgpt
   node bin/c2c.js doctor -w /home/pdq/Code
   ```

3. 看到 `Everything looks good.` 后，打开上面的 [Code 项目](https://chatgpt.com/g/g-p-6a953b661c508191827f7ded82f369a9/project)。
4. 在 ChatGPT 项目中新建聊天；在输入框旁选择 `Codex with ChatGPT · Code` 连接器。
5. 第一句可以使用下面的固定测试语句：

   ```text
   请使用“Codex with ChatGPT · Code”连接器，先读取 workspace_info，只回复 workspace 名称；确认后再处理我的任务。
   ```

如果旧聊天提示“此项目无法使用工作模式”或“连接器在这一轮不可用”，先在同一个 `Code` 项目中新建聊天，再重新选择连接器。旧聊天记录仍然保留。

## 3. 看到“本轮连接器不可用”时

这不一定是周期性故障。常见情况是：当前聊天、当前模型或当前页面没有暴露该连接器；也可能是本地连接工具重启后换了临时地址。OpenAI 的排查说明也提到，应用可能显示为已连接，但在当前聊天或模型中暂时不可用。

按下面顺序处理：

1. 在同一个 `Code` 项目新建聊天，并重新选择 `Codex with ChatGPT · Code`。
2. 如果仍不可用，在终端执行：

   ```bash
   cd /home/pdq/codex-with-chatgpt
   node bin/c2c.js doctor -w /home/pdq/Code
   ```

3. 如果输出 `Everything looks good.`，回到第1步；不要删除项目，也不要新建第二个同名连接器。
4. 如果输出连接地址失效、`address_reclaimed`、`connectorAction: "update"`，按第4节删除并重新添加连接器。

## 4. 地址失效时，删除旧连接器并重新添加

### 4.1 先生成当前地址和配对码

执行：

```bash
cd /home/pdq/codex-with-chatgpt
node bin/c2c.js doctor -w /home/pdq/Code --json
```

在输出中找到：

- `chatgptRepair.mcpUrl`：当前服务器 URL，完整复制，通常以 `/mcp` 结尾；
- `chatgptRepair.pairingCode`：本次配对码。

配对码约 5 分钟过期。过期后重新执行：

```bash
node bin/c2c.js pair -w /home/pdq/Code --json
```

不要使用昨天的服务器 URL 或昨天的配对码。

### 4.2 在 ChatGPT 中删除旧的同名连接器

1. 打开[连接器设置](https://chatgpt.com/plugins?view=personal)。
2. 找到名称**完全等于** `Codex with ChatGPT · Code` 的连接器。
3. 打开右上角 `…`，选择“管理”或“删除”。
4. 只删除这个 `Code` workspace 的连接器，不要删除其他项目的连接器。

如果页面提示“连接器名称已存在”，说明旧条目还没有真正删除或页面尚未刷新。先刷新连接器列表，确认旧条目消失后再创建；不要连续创建多个同名连接器。

### 4.3 创建新连接器

打开[新建连接器](https://chatgpt.com/plugins#settings/Connectors?create-connector=true&redirectAfter=%2Fplugins)，填写：

- 名称：`Codex with ChatGPT · Code`
- 描述（可选）：`Securely connect ChatGPT to the current Codex workspace for planning and review.`
- 服务器 URL：第4.1节得到的最新 `mcpUrl`
- 身份验证：`OAuth`
- 勾选风险确认框，然后点击“创建”。

创建后点击“使用/登录此连接器”，在授权页面输入第4.1节刚生成的配对码，点击 `Connect`。只输入配对码，不要把密码、Token 或完整 JSON 粘贴到聊天框。

最后回到 [Code 项目](https://chatgpt.com/g/g-p-6a953b661c508191827f7ded82f369a9/project)，新建聊天并选择该连接器。若当前旧聊天仍显示不可用，继续使用新聊天即可。

## 5. 电脑重启、VS Code 重启后的处理

先运行：

```bash
cd /home/pdq/codex-with-chatgpt
node bin/c2c.js doctor -w /home/pdq/Code
```

- 看到 `Everything looks good.`：直接使用 ChatGPT 项目。
- 看到 `Tunnel start timed out`：等待约 10 秒后再运行一次；仍失败时执行下面两条，再运行 `doctor`：

  ```bash
  node bin/c2c.js restart -w /home/pdq/Code
  node bin/c2c.js doctor -w /home/pdq/Code
  ```

- 看到地址已更换或 `chatgptRepair.needed: true`：按第4节重建连接器。

只重启 VS Code 通常不需要重建连接器；只有 `doctor` 明确提示地址变化或授权失效时才重建。

## 6. 更换账户时怎么判断

- 只更换 VS Code/Codex 登录账户：先运行 `doctor`。如果连接地址和授权仍正常，可以继续使用；如果提示授权失效，按第4节重新授权。
- 更换 ChatGPT 网页账户：连接器授权属于网页账户，新账户通常需要在自己的连接器设置中重新创建或授权。
- 删除连接器不会删除 ChatGPT 项目或项目里的聊天记录；它只会删除该连接器的授权关系。可以参考 [OpenAI 的连接器管理说明](https://help.openai.com/en/articles/20001494-connecting-and-managing-app-accounts-in-chatgpt)。

## 7. 不要做的事情

- 不要把旧的 `trycloudflare.com` 地址当成永久地址。
- 不要在同一个 workspace 反复创建第二个连接器。
- 不要因为当前一轮不可用就删除整个 ChatGPT 项目。
- 不要点击旧地址连接器里的“重新连接”来修复已被回收的临时地址；先用 `doctor` 得到新地址，再删除并创建同名连接器。
- 不要把 ChatGPT 密码、OAuth Token、私钥或完整终端 JSON 发到聊天里。

## 8. 最短处理口诀

```text
先 doctor；新聊天重选连接器；仍失败看 doctor 的地址。
地址没变：新聊天。
地址变了：删旧的 Codex with ChatGPT · Code，再用新 mcpUrl 和新配对码创建。
```

## 9. 可选：想尽量避免以后重绑

当前 Quick Tunnel 的随机地址无法保证长期不变。若希望连接器地址稳定，应配置 Cloudflare 账户下的固定域名 named tunnel；这需要一个可用域名和 Cloudflare 登录授权，不要把临时 `trycloudflare.com` 地址改名冒充固定域名。没有这些条件时，继续使用第4节的“doctor → 更新连接器”流程即可。
