
## 2026-09-07 追求优雅:四处清理落地

- sim 容器不再靠宿主拼 bash:ros 基础镜像的 entrypoint 负责 source ROS,桥就是容器命令;LIBERO 的设置文件由 LIBERO 装载器按已安装包的位置自己写,基准知识回到桥内;DDS peers 文件写在配置旁边由挂载带入。
- `sandbox.up` 只收参数:配置作为 dict 传入(用于播种工作区),镜像、网络、run_args 全是显式参数,文件只在命令行入口加载。
- 函数内 import 提到文件顶部(桥内为 rclpy 保留延迟 import;工作区模板文件不动,哈希不变)。
- 旧词汇清理:triallock、assembly。
- 证据:RoboCLI 提交(见 git log),193 测试 + 12 契约绿,三腿 `--operator none` 各一局 preflight 15/15、15/15、17/17。
