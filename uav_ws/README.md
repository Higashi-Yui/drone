# 调试通讯教程
1. 启动./start.sh 后，点击下方绿色条中开头为 Offboard 的模块跳转到该window

2. 点击左边的窗格，键盘敲击 ctrl+b 然后松手，敲击 - （减号）

3. 此时左边的窗格应该在下面又分出一个窗格，点击该窗格

4. 输入如下指令，输入后敲击回车执行：

```bash
rostopic pub -1 /land_done std_msgs/Bool "data: true"
```
5. 执行后，右边的tcp通信节点应有调试info打印，有的话就是成功发布了